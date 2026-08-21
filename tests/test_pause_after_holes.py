"""Tests for machining.fixturing.pause_after_holes.

The option inserts one fixturing pause at the holes/contours boundary: after all
CLEARED circular holes (the ones screws go through) and before contoured holes,
pocket clearing, and pocket contours. It replaces the per-contour pauses, and in
multi-part job mode it splits the interior into a collated HOLES phase and an
INTERIOR phase with one shared pause between them.
"""

import unittest

from frc_cam_postprocessor import FRCPostProcessor, assemble_job_gcode
from team_config import TeamConfig


def make_pp(pause_after_holes=False, pause_before_perimeter=False):
    config = TeamConfig({
        'machining': {
            'fixturing': {
                'pause_before_perimeter': pause_before_perimeter,
                'pause_after_holes': pause_after_holes,
            },
            # Keep raw toolpaths: these tests assert on structure, not size.
            'output': {'compression': False},
        }
    })
    pp = FRCPostProcessor(material_thickness=0.25, tool_diameter=0.157, config=config)
    pp.apply_material_preset('plywood')
    return pp


def with_features(pp, holes=True, pockets=True, big_pocket=False):
    """Attach a standard feature set: screwable holes, a small cleared pocket, and
    optionally a large through-cut pocket (above the contour threshold, so it is
    contoured with tabs)."""
    pp.perimeter = [(0, 0), (12, 0), (12, 12), (0, 12)]
    pp.holes = [
        {'center': (1.0, 1.0), 'diameter': 0.201},
        {'center': (11.0, 1.0), 'diameter': 0.201},
    ] if holes else []
    pp.pockets = []
    if pockets:
        pp.pockets.append([(2, 2), (3, 2), (3, 3), (2, 3)])       # 1 sq in -> cleared
    if big_pocket:
        pp.pockets.append([(4, 4), (10, 4), (10, 10), (4, 10)])   # 36 sq in -> contoured
    return pp


def pause_indices(lines):
    return [i for i, l in enumerate(lines) if 'PAUSE FOR FIXTURING' in l]


class TestSinglePart(unittest.TestCase):

    def _gcode_lines(self, pp):
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success, result.errors)
        return result.gcode.split('\n')

    def test_pause_lands_between_holes_and_pockets(self):
        pp = with_features(make_pp(pause_after_holes=True))
        lines = self._gcode_lines(pp)
        pauses = pause_indices(lines)
        self.assertEqual(len(pauses), 1)
        holes_end = max(i for i, l in enumerate(lines) if '(--- Cleared holes ---)' in l)
        pockets_start = next(i for i, l in enumerate(lines) if '(===== POCKETS =====)' in l)
        self.assertGreater(pauses[0], holes_end)
        self.assertLess(pauses[0], pockets_start)
        # The pause parks with spindle off and waits for the operator.
        block = '\n'.join(lines[pauses[0] - 2:pauses[0] + 20])
        self.assertIn('M5', block)
        self.assertIn('M0', block)
        self.assertIn('screws', block)

    def test_default_off_emits_no_pause(self):
        pp = with_features(make_pp())
        lines = self._gcode_lines(pp)
        self.assertEqual(pause_indices(lines), [])

    def test_replaces_per_contour_pauses(self):
        # With pause_before_perimeter on, a contoured pocket normally gets its own
        # pause. pause_after_holes replaces it: exactly two pauses total - one at the
        # holes boundary, one before the perimeter - none between the pockets.
        pp = with_features(make_pp(pause_after_holes=True, pause_before_perimeter=True),
                           big_pocket=True)
        lines = self._gcode_lines(pp)
        pauses = pause_indices(lines)
        self.assertEqual(len(pauses), 2)
        contour_start = next(i for i, l in enumerate(lines)
                             if '(--- Contoured pockets - manual removal required ---)' in l)
        perimeter_start = next(i for i, l in enumerate(lines) if 'PERIMETER' in l)
        self.assertLess(pauses[0], contour_start)          # boundary pause before contours
        self.assertGreater(pauses[1], contour_start)       # perimeter pause after them
        self.assertLess(pauses[1], perimeter_start)

    def test_no_pause_without_holes(self):
        # Nothing to put screws through - the boundary pause must not fire.
        pp = with_features(make_pp(pause_after_holes=True), holes=False)
        lines = self._gcode_lines(pp)
        self.assertEqual(pause_indices(lines), [])

    def test_no_pause_when_nothing_follows_holes(self):
        # Holes only: the boundary pause would lead nowhere; leave fixturing to the
        # perimeter pause option.
        pp = with_features(make_pp(pause_after_holes=True), pockets=False)
        lines = self._gcode_lines(pp)
        self.assertEqual(pause_indices(lines), [])


class TestJobMode(unittest.TestCase):

    def _job(self, pause_after_holes):
        pps = []
        for dx in (0.0, 14.0):
            pp = with_features(make_pp(pause_after_holes=pause_after_holes))
            pp.perimeter = [(x + dx, y) for x, y in pp.perimeter]
            pp.holes = [{'center': (cx + dx, cy), 'diameter': d['diameter']}
                        for d, (cx, cy) in zip(pp.holes, [h['center'] for h in pp.holes])]
            pp.pockets = [[(x + dx, y) for x, y in p] for p in pp.pockets]
            pps.append(pp)
        part_jobs = []
        for i, pp in enumerate(pps):
            phases = pp.generate_part_phases()
            self.assertEqual(phases['errors'], [])
            part_jobs.append({'name': f'part{i}', 'place_x': 0, 'place_y': 0,
                              'rotation': 0, **{k: phases[k] for k in
                                                ('holes', 'interior', 'perimeter', 'tab_removal')}})
        result = assemble_job_gcode(part_jobs, header_pp=pps[0],
                                    timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success)
        return result.gcode.split('\n')

    def test_split_phases_with_shared_pause(self):
        lines = self._job(pause_after_holes=True)
        holes_phase = next(i for i, l in enumerate(lines) if '(===== PHASE: HOLES =====)' in l)
        interior_phase = next(i for i, l in enumerate(lines)
                              if '(===== PHASE: INTERIOR FEATURES =====)' in l)
        pauses = pause_indices(lines)
        self.assertEqual(len(pauses), 1)
        self.assertGreater(pauses[0], holes_phase)
        self.assertLess(pauses[0], interior_phase)
        # Both parts contribute to the HOLES phase and to the INTERIOR phase.
        holes_block = '\n'.join(lines[holes_phase:pauses[0]])
        interior_block = '\n'.join(lines[interior_phase:]).split('(===== PHASE: PERIMETERS')[0]
        self.assertEqual(holes_block.count('(--- PART'), 2)
        self.assertEqual(interior_block.count('(--- PART'), 2)
        # Hole cutting happens only in the HOLES phase; pockets only after the pause.
        self.assertIn('(--- Cleared holes ---)', holes_block)
        self.assertNotIn('(===== POCKETS =====)', holes_block)
        self.assertIn('(===== POCKETS =====)', interior_block)
        self.assertNotIn('(--- Cleared holes ---)', interior_block)

    def test_default_job_structure_unchanged(self):
        lines = self._job(pause_after_holes=False)
        self.assertFalse(any('(===== PHASE: HOLES =====)' in l for l in lines))
        self.assertEqual(pause_indices(lines), [])
        # Holes and pockets both live in the single interior phase, holes first.
        interior_phase = next(i for i, l in enumerate(lines)
                              if '(===== PHASE: INTERIOR FEATURES =====)' in l)
        block = '\n'.join(lines[interior_phase:])
        self.assertIn('(===== HOLES =====)', block)
        self.assertIn('(===== POCKETS =====)', block)
        self.assertLess(block.index('(===== HOLES =====)'), block.index('(===== POCKETS =====)'))


if __name__ == '__main__':
    unittest.main()
