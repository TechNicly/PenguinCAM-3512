"""Tests for G53 park validation against machine soft limits.

Motivated by a real crash: a park_position of (1, 30, -0.5) on a Mach3 machine that
homes to 0 with negative travel drove the gantry 30" past the Y stop during the
after-holes fixturing pause, losing steps and corrupting position for the rest of
the job. Generation must now refuse to emit a park that violates configured soft
limits, warn loudly when a park cannot be validated, and allow pauses to skip
parking entirely.
"""

import unittest

from frc_cam_postprocessor import FRCPostProcessor
from team_config import TeamConfig


def make_pp(machine=None, fixturing=None):
    cfg = {'machining': {'output': {'compression': False}}}
    if machine:
        cfg['machine'] = machine
    if fixturing:
        cfg['machining']['fixturing'] = fixturing
    pp = FRCPostProcessor(material_thickness=0.25, tool_diameter=0.157,
                          config=TeamConfig(cfg))
    pp.apply_material_preset('plywood')
    pp.perimeter = [(0, 0), (10, 0), (10, 8), (0, 8)]
    pp.holes = [{'center': (1.0, 1.0), 'diameter': 0.201}]
    pp.pockets = []
    return pp


NEG_LIMITS = {'x': [-22.2, 0], 'y': [-30.3, 0], 'z': [-3.3, 0]}


class TestParkValidation(unittest.TestCase):

    def test_park_outside_soft_limits_blocks_generation(self):
        # The exact crash configuration: positive Y park on a negative-travel machine.
        pp = make_pp(machine={'park_position': {'x': 1, 'y': 30, 'z': -0.5},
                              'soft_limits': NEG_LIMITS})
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertFalse(result.success)
        self.assertTrue(any('soft limits' in e for e in result.errors))
        self.assertTrue(any('X1' in e or 'Y30' in e for e in result.errors))

    def test_park_inside_soft_limits_generates_and_notes_validation(self):
        pp = make_pp(machine={'park_position': {'x': -21, 'y': -0.3, 'z': -0.5},
                              'soft_limits': NEG_LIMITS})
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success, result.errors)
        self.assertIn('park validated', result.gcode)
        self.assertEqual(pp.park_warnings, [])

    def test_unvalidated_park_warns(self):
        pp = make_pp(machine={'park_position': {'x': 1, 'y': 30, 'z': -0.5}})
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success, result.errors)
        self.assertTrue(any('NOT validated' in w for w in result.warnings))
        self.assertIn('(WARNING: park NOT validated - no machine soft_limits configured)',
                      result.gcode)

    def test_no_park_no_g53_no_warnings(self):
        pp = make_pp()
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success)
        self.assertNotIn('G53', result.gcode)
        self.assertEqual(result.warnings, [])

    def test_generate_part_phases_also_blocks_bad_park(self):
        pp = make_pp(machine={'park_position': {'x': 1, 'y': 30, 'z': -0.5},
                              'soft_limits': NEG_LIMITS})
        phases = pp.generate_part_phases()
        self.assertTrue(phases['errors'])
        self.assertTrue(any('soft limits' in e for e in phases['errors']))

    def test_park_during_pause_off_keeps_pause_park_free(self):
        pp = make_pp(machine={'park_position': {'x': -21, 'y': -0.3, 'z': -0.5},
                              'soft_limits': NEG_LIMITS},
                     fixturing={'pause_before_perimeter': True,
                                'pause_after_holes': False,
                                'park_during_pause': False})
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        self.assertTrue(result.success, result.errors)
        lines = result.gcode.split('\n')
        pause_start = next(i for i, l in enumerate(lines) if 'PAUSE FOR FIXTURING' in l)
        pause_end = next(i for i, l in enumerate(lines[pause_start:], pause_start) if 'M0' in l)
        pause_block = '\n'.join(lines[pause_start:pause_end])
        self.assertNotIn('G53', pause_block)
        # The end-of-program park is unaffected.
        footer = '\n'.join(lines[next(i for i, l in enumerate(lines) if 'FINISH' in l):])
        self.assertIn('G53', footer)

    def test_park_during_pause_on_still_parks(self):
        pp = make_pp(machine={'park_position': {'x': -21, 'y': -0.3, 'z': -0.5},
                              'soft_limits': NEG_LIMITS},
                     fixturing={'pause_before_perimeter': True,
                                'pause_after_holes': False,
                                'park_during_pause': True})
        result = pp.generate_gcode(suggested_filename='t', timestamp='2026-01-01 00:00:00')
        lines = result.gcode.split('\n')
        pause_start = next(i for i, l in enumerate(lines) if 'PAUSE FOR FIXTURING' in l)
        pause_end = next(i for i, l in enumerate(lines[pause_start:], pause_start) if 'M0' in l)
        self.assertIn('G53', '\n'.join(lines[pause_start:pause_end]))


class TestSoftLimitsConfig(unittest.TestCase):

    def test_soft_limits_parse_and_order(self):
        cfg = TeamConfig({'machine': {'soft_limits': {'x': [0, -22.2], 'y': ['-30.3', '0']}}})
        limits = cfg.soft_limits
        self.assertEqual(limits['x'], (-22.2, 0.0))   # normalized to (min, max)
        self.assertEqual(limits['y'], (-30.3, 0.0))   # unit strings parsed
        self.assertNotIn('z', limits)                 # missing axis not validated

    def test_absent_soft_limits_is_none(self):
        self.assertIsNone(TeamConfig().soft_limits)
        self.assertIsNone(TeamConfig({'machine': {'soft_limits': {'x': [1]}}}).soft_limits)


if __name__ == '__main__':
    unittest.main()
