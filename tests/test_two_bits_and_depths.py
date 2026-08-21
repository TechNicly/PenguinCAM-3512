"""Tests for two-bit jobs (holes bit + main bit) and per-pocket partial depths."""

import unittest

from frc_cam_postprocessor import FRCPostProcessor, assemble_job_gcode
from team_config import TeamConfig


def make_pp(**kw):
    pp = FRCPostProcessor(material_thickness=0.25, tool_diameter=0.157,
                          config=TeamConfig({'machining': {'output': {'compression': False},
                                                          **kw.get('machining', {})}}))
    pp.apply_material_preset('plywood')
    pp.perimeter = [(0, 0), (10, 0), (10, 8), (0, 8)]
    pp.holes = []
    pp.pockets = []
    return pp


def gen(pp):
    r = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
    assert r.success, r.errors
    return r.gcode.split('\n')


class TestHolesBit(unittest.TestCase):

    def _with_holes(self, pp):
        pp.circles = [{'center': (1.0, 1.0), 'diameter': 0.201},
                      {'center': (9.0, 1.0), 'diameter': 0.140}]
        pp.classify_holes()
        pp.pockets = [[(4, 4), (6, 4), (6, 6), (4, 6)]]
        return pp

    def test_small_hole_needs_smaller_bit(self):
        pp = self._with_holes(make_pp())
        # 0.140" hole vs 0.157" main bit: classification error.
        self.assertTrue(any('too small' in e for e in pp.errors))

    def test_holes_bit_makes_small_holes_millable(self):
        pp = make_pp()
        pp.set_holes_tool(0.125)
        self._with_holes(pp)
        self.assertEqual(pp.errors, [])
        self.assertEqual(len(pp.holes), 2)

    def test_bit_change_pause_forced_between_phases(self):
        pp = make_pp()
        pp.set_holes_tool(0.125)
        self._with_holes(pp)
        lines = gen(pp)
        text = '\n'.join(lines)
        self.assertIn('PAUSE - CHANGE BIT', text)
        self.assertIn('CHANGE BIT: holes used the 0.1250" bit, install the 0.1570" bit now', text)
        self.assertIn('RE-ZERO Z to the sacrifice board with the new bit', text)
        self.assertIn('(Holes bit: 0.125" diam - START with this bit, change at the pause)', text)
        # Ordering: holes cut before the pause, pockets after.
        self.assertLess(text.index('Cleared holes'), text.index('PAUSE - CHANGE BIT'))
        self.assertLess(text.index('PAUSE - CHANGE BIT'), text.index('POCKETS'))

    def test_same_bit_means_no_change_pause(self):
        pp = make_pp()
        pp.set_holes_tool(0.157)   # same as main -> no-op
        self._with_holes(pp)
        # The 0.140 hole is too small again with the identical bit.
        pp.errors = []
        pp.circles = [{'center': (1.0, 1.0), 'diameter': 0.201}]
        pp.classify_holes()
        lines = gen(pp)
        self.assertNotIn('CHANGE BIT', '\n'.join(lines))

    def test_combined_bit_change_and_fixturing_pause(self):
        pp = make_pp(machining={'fixturing': {'pause_before_perimeter': False,
                                              'pause_after_holes': True}})
        pp.set_holes_tool(0.125)
        self._with_holes(pp)
        text = '\n'.join(gen(pp))
        self.assertIn('PAUSE - CHANGE BIT + FIXTURING', text)
        self.assertIn('Install screws', text)
        # One boundary stop, not two.
        self.assertEqual(text.count('M0  ; Program pause'), 1)

    def test_job_mode_splits_phases_for_bit_change(self):
        pp = make_pp()
        pp.set_holes_tool(0.125)
        self._with_holes(pp)
        phases = pp.generate_part_phases()
        self.assertEqual(phases['errors'], [])
        self.assertTrue(phases['holes'])       # split even with pause_after_holes off
        pj = {'name': 'p', 'place_x': 0, 'place_y': 0, 'rotation': 0,
              **{k: phases[k] for k in ('holes', 'interior', 'perimeter', 'tab_removal')}}
        result = assemble_job_gcode([pj], header_pp=pp, timestamp='2026-01-01 00:00:00')
        self.assertIn('PAUSE - CHANGE BIT', result.gcode)
        self.assertLess(result.gcode.index('PHASE: HOLES'),
                        result.gcode.index('PAUSE - CHANGE BIT'))
        self.assertLess(result.gcode.index('PAUSE - CHANGE BIT'),
                        result.gcode.index('PHASE: INTERIOR FEATURES'))

    def test_holes_toolpath_uses_holes_bit_radius(self):
        # The 0.201" bore's toolpath radius follows the bit in the spindle:
        # (0.201 - 0.125)/2 = 0.038 with the holes bit, vs (0.201 - 0.157)/2 = 0.022
        # with the main tool. The helix clamps to the bore, so 0.038 proves the
        # holes bit generated the toolpath.
        pp = make_pp()
        pp.set_holes_tool(0.125)
        pp.circles = [{'center': (1.0, 1.0), 'diameter': 0.201}]
        pp.classify_holes()
        text = '\n'.join(gen(pp))
        self.assertIn('helical entry at 0.0380" radius', text)
        self.assertNotIn('helical entry at 0.0220" radius', text)

    def test_bit_change_pause_fires_for_perimeter_only_parts(self):
        # Holes + perimeter, NO pockets: the bit must still be swapped before the
        # perimeter cut (this was a real bug - the pause was skipped).
        pp = make_pp()
        pp.set_holes_tool(0.125)
        pp.circles = [{'center': (1.0, 1.0), 'diameter': 0.201}]
        pp.classify_holes()
        text = '\n'.join(gen(pp))
        self.assertIn('PAUSE - CHANGE BIT', text)
        self.assertLess(text.index('PAUSE - CHANGE BIT'), text.index('PERIMETER'))


class TestPocketDepths(unittest.TestCase):

    def test_partial_pocket_cuts_to_its_depth(self):
        pp = make_pp()
        pp.pockets = [[(2, 2), (4, 2), (4, 4), (2, 4)], [(5, 5), (8, 5), (8, 7), (5, 7)]]
        pp.set_pocket_depths([((3, 3), 0.1)])
        lines = gen(pp)
        text = '\n'.join(lines)
        self.assertIn('partial depth 0.100 in, bottom at Z0.1500', text)
        self.assertIn('Z0.1500', text)                 # 0.25 top - 0.10 = 0.15
        self.assertIn('Z-0.0080', text)                # the through pocket still bottoms out

    def test_depth_at_thickness_is_through(self):
        pp = make_pp()
        pp.pockets = [[(2, 2), (4, 2), (4, 4), (2, 4)]]
        pp.set_pocket_depths([((3, 3), 0.25)])
        text = '\n'.join(gen(pp))
        self.assertNotIn('partial depth', text)

    def test_partial_pocket_never_contours(self):
        # Big enough to contour as a through-cut, but a partial depth forces clearing.
        pp = make_pp()
        pp.pockets = [[(1, 1), (9, 1), (9, 7), (1, 7)]]   # 48 sq in >> threshold
        text_through = '\n'.join(gen(pp))
        self.assertIn('will contour through-cut', text_through)
        pp2 = make_pp()
        pp2.pockets = [[(1, 1), (9, 1), (9, 7), (1, 7)]]
        pp2.set_pocket_depths([((5, 4), 0.05)])
        text_partial = '\n'.join(gen(pp2))
        self.assertNotIn('will contour through-cut', text_partial)
        self.assertIn('will fully clear - partial depth 0.050 in from top', text_partial)

    def test_cut_depth_restored_after_partial_pocket(self):
        pp = make_pp()
        pp.pockets = [[(2, 2), (4, 2), (4, 4), (2, 4)]]
        pp.set_pocket_depths([((3, 3), 0.1)])
        gen(pp)
        self.assertAlmostEqual(pp.cut_depth, -0.008)   # through-cut depth restored


if __name__ == '__main__':
    unittest.main()
