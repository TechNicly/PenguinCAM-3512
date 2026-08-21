"""Tests for per-part perimeter tab overrides (the wizard's Tabs & Fixtures step).

Covers set_tab_overrides / compute_perimeter_tab_layout / _compute_tab_zones:
exclusion regions, custom positions, per-part disable, minimum-tab warnings, and
that the default (no overrides) layout is unchanged from the historical behavior.
"""

import math
import unittest

from frc_cam_postprocessor import FRCPostProcessor, assemble_job_gcode
from team_config import TeamConfig


def make_pp(**config_overrides):
    cfg = {'machining': {'output': {'compression': False}}}
    for k, v in config_overrides.items():
        cfg['machining'][k] = v
    pp = FRCPostProcessor(material_thickness=0.25, tool_diameter=0.157,
                          config=TeamConfig(cfg))
    pp.apply_material_preset('plywood')
    pp.perimeter = [(0, 0), (10, 0), (10, 8), (0, 8)]
    pp.holes = []
    pp.pockets = []
    return pp


def tab_lines(pp):
    result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
    assert result.success, result.errors
    lines = result.gcode.split('\n')
    return lines, [l for l in lines if 'Tab' in l and 'start' in l]


class TestTabLayout(unittest.TestCase):

    def test_default_layout_matches_historical_formula(self):
        pp = make_pp()
        layout = pp.compute_perimeter_tab_layout()
        # 10x8 rectangle, tool r=0.0785: contour ~36.5"; ramp for plywood ~1.18";
        # ceil(35.3/6) = 6 tabs.
        self.assertEqual(len(layout['tabs']), 6)
        self.assertTrue(layout['enabled'])
        self.assertEqual(layout['warnings'], [])
        lines, tabs = tab_lines(pp)
        self.assertEqual(len(tabs), 6)
        self.assertTrue(any('desired spacing' in l for l in lines))

    def test_layout_matches_generated_gcode(self):
        # The preview layout and the cut G-code must agree on tab count for every mode.
        for overrides in (
            {},
            {'exclusions': [((2.0, -0.0785), (8.0, -0.0785))]},
            {'positions': [(5.0, -0.0785), (10.0785, 4.0), (5.0, 8.0785)]},
        ):
            pp = make_pp()
            if overrides:
                pp.set_tab_overrides(**overrides)
            layout = pp.compute_perimeter_tab_layout()
            _, tabs = tab_lines(pp)
            self.assertEqual(len(layout['tabs']), len(tabs), overrides)

    def test_exclusion_keeps_tabs_off_the_edge(self):
        pp = make_pp()
        # Exclude the bottom edge (contour runs at y=-0.0785 along it).
        pp.set_tab_overrides(exclusions=[((1.0, -0.0785), (9.0, -0.0785))])
        layout = pp.compute_perimeter_tab_layout()
        self.assertGreaterEqual(len(layout['tabs']), 3)
        for t in layout['tabs']:
            self.assertGreater(t['y'], 0.0,
                               f"tab at ({t['x']:.2f}, {t['y']:.2f}) is on the excluded edge")
        self.assertEqual(len(layout['excluded']), 1)

    def test_exclusion_uses_shorter_arc(self):
        pp = make_pp()
        # Points near each other on the bottom edge: the short arc between them is
        # excluded, NOT the long way around the part.
        pp.set_tab_overrides(exclusions=[((4.0, -0.0785), (6.0, -0.0785))])
        layout = pp.compute_perimeter_tab_layout()
        # Excluding a 2" stretch of a 36" contour barely changes the count.
        self.assertGreaterEqual(len(layout['tabs']), 5)

    def test_custom_positions_project_onto_contour(self):
        pp = make_pp()
        # Click points slightly OFF the contour; tabs must land ON it.
        pp.set_tab_overrides(positions=[(5.0, -0.5), (10.5, 4.0), (5.0, 8.5), (-0.5, 4.0)])
        layout = pp.compute_perimeter_tab_layout()
        self.assertEqual(len(layout['tabs']), 4)
        r = pp.tool_radius
        for t in layout['tabs']:
            on_contour = (abs(t['y'] + r) < 0.01 or abs(t['y'] - 8 - r) < 0.01 or
                          abs(t['x'] + r) < 0.01 or abs(t['x'] - 10 - r) < 0.01)
            self.assertTrue(on_contour, f"tab not on contour: ({t['x']:.3f}, {t['y']:.3f})")

    def test_few_tabs_warns(self):
        pp = make_pp()
        pp.set_tab_overrides(positions=[(5.0, -0.0785)])
        layout = pp.compute_perimeter_tab_layout()
        self.assertEqual(len(layout['tabs']), 1)
        self.assertTrue(any('come loose' in w for w in layout['warnings']))
        lines, tabs = tab_lines(pp)
        self.assertEqual(len(tabs), 1)
        self.assertTrue(any('WARNING' in l and 'tabs only' in l for l in lines))

    def test_empty_custom_positions_means_zero_tabs_with_warning(self):
        pp = make_pp()
        pp.set_tab_overrides(positions=[])
        lines, tabs = tab_lines(pp)
        self.assertEqual(len(tabs), 0)
        self.assertTrue(pp.tab_warnings)

    def test_disable_per_part(self):
        pp = make_pp()
        pp.set_tab_overrides(enabled=False)
        layout = pp.compute_perimeter_tab_layout()
        self.assertFalse(layout['enabled'])
        self.assertEqual(layout['tabs'], [])
        lines, tabs = tab_lines(pp)
        self.assertEqual(len(tabs), 0)
        self.assertIn('(Tabs disabled - perimeter will be cut through completely)', lines)

    def test_force_enable_overrides_config_off(self):
        pp = make_pp(tabs={'enabled': False, 'width': 0.25, 'height': 0.1,
                           'spacing': 6.0, 'remove_tabs': True})
        self.assertFalse(pp.tabs_enabled)
        pp.set_tab_overrides(enabled=True)
        layout = pp.compute_perimeter_tab_layout()
        self.assertTrue(layout['enabled'])
        self.assertGreaterEqual(len(layout['tabs']), 3)

    def test_pocket_contours_unaffected_by_overrides(self):
        # A big through-cut pocket gets contoured with tabs; the part's custom
        # perimeter tabs must not leak into the pocket's tab layout.
        pp = make_pp()
        pp.pockets = [[(2, 2), (8, 2), (8, 6), (2, 6)]]   # 24 sq in -> contoured
        pp.set_tab_overrides(positions=[(5.0, -0.0785)])
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success)
        lines = result.gcode.split('\n')
        pocket_section = '\n'.join(lines[
            next(i for i, l in enumerate(lines) if 'POCKETS' in l):
            next(i for i, l in enumerate(lines) if 'PERIMETER' in l)])
        # Pocket still gets its auto-spaced tabs (comment carries 'desired spacing').
        self.assertIn('desired spacing', pocket_section)


class TestFixtureNotes(unittest.TestCase):

    def test_fixture_notes_reach_the_pause_block(self):
        pp = make_pp(fixturing={'pause_before_perimeter': True, 'pause_after_holes': False})
        pp.holes = [{'center': (1.0, 1.0), 'diameter': 0.201}]
        phases = pp.generate_part_phases()
        self.assertEqual(phases['errors'], [])
        pj = {'name': 'plate', 'place_x': 0, 'place_y': 0, 'rotation': 0,
              **{k: phases[k] for k in ('holes', 'interior', 'perimeter', 'tab_removal')}}
        result = assemble_job_gcode([pj], header_pp=pp, timestamp='2026-01-01 00:00:00',
                                    fixture_notes=['Screw: plate at X1.00 Y1.00'])
        self.assertTrue(result.success)
        self.assertIn('( Screw: plate at X1.00 Y1.00 )', result.gcode)

    def test_fixture_notes_sanitized(self):
        pp = make_pp(fixturing={'pause_before_perimeter': True, 'pause_after_holes': False})
        phases = pp.generate_part_phases()
        pj = {'name': 'plate', 'place_x': 0, 'place_y': 0, 'rotation': 0,
              **{k: phases[k] for k in ('holes', 'interior', 'perimeter', 'tab_removal')}}
        result = assemble_job_gcode([pj], header_pp=pp, timestamp='2026-01-01 00:00:00',
                                    fixture_notes=['Screw: weird (name) at X0 Y0'])
        self.assertIn('( Screw: weird [name] at X0 Y0 )', result.gcode)
        for line in result.gcode.split('\n'):
            self.assertNotRegex(line, r'\([^)]*\(')


if __name__ == '__main__':
    unittest.main()
