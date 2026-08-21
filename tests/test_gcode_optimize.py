"""Tests for gcode_optimize - output toolpath compression.

The optimizer's contract: the compressed program follows the same path as the raw
program to within the configured tolerance, preserves every non-motion line verbatim,
and never moves a run endpoint (so tabs, plunges, and ramps land exactly where the
generators put them).
"""

import math
import re
import unittest

from gcode_optimize import (
    optimize_gcode,
    path_stats,
    DEFAULT_TOLERANCE,
)


def make_arc_chords(cx, cy, r, a0_deg, a1_deg, n, feed="50.0", z=None):
    """Emit n chords approximating an arc, as the generators do."""
    lines = []
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    for k in range(1, n + 1):
        a = a0 + (a1 - a0) * k / n
        x, y = cx + r * math.cos(a), cy + r * math.sin(a)
        if z is None:
            lines.append(f"G1 X{x:.4f} Y{y:.4f} F{feed}")
        else:
            lines.append(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f} F{feed}")
    return lines


def arc_start_point(cx, cy, r, a0_deg):
    a0 = math.radians(a0_deg)
    return cx + r * math.cos(a0), cy + r * math.sin(a0)


def simulate_xy(lines):
    """Simulate a program into a dense XY polyline (arcs sampled finely)."""
    pts = [(0.0, 0.0)]
    pos = [0.0, 0.0]
    word = re.compile(r'([A-Za-z])(-?\d*\.?\d+)')
    for raw in lines:
        code = raw.split(';')[0].strip()
        m = re.match(r'^(G0|G1|G2|G3)\b', code)
        if not m:
            continue
        words = dict((l.upper(), float(v)) for l, v in word.findall(code))
        x = words.get('X', pos[0])
        y = words.get('Y', pos[1])
        if m.group(1) in ('G2', 'G3'):
            i, j = words.get('I', 0.0), words.get('J', 0.0)
            ccx, ccy = pos[0] + i, pos[1] + j
            r = math.hypot(i, j)
            sa = math.atan2(pos[1] - ccy, pos[0] - ccx)
            ea = math.atan2(y - ccy, x - ccx)
            sweep = ea - sa
            if m.group(1) == 'G3':
                if sweep <= 1e-9:
                    sweep += 2 * math.pi
            else:
                if sweep >= -1e-9:
                    sweep -= 2 * math.pi
            n = max(16, int(abs(sweep) * 64))
            for k in range(1, n + 1):
                a = sa + sweep * k / n
                pts.append((ccx + r * math.cos(a), ccy + r * math.sin(a)))
        else:
            pts.append((x, y))
        pos = [x, y]
    return pts


def max_deviation(src_lines, dst_lines):
    """Max distance from any vertex of src's polyline to dst's polyline."""
    src = simulate_xy(src_lines)
    dst = simulate_xy(dst_lines)

    def seg_dist(p, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0:
            return math.hypot(p[0] - ax, p[1] - ay)
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))

    worst = 0.0
    for p in src:
        best = min(seg_dist(p, dst[i], dst[i + 1]) for i in range(len(dst) - 1))
        worst = max(worst, best)
    return worst


PREAMBLE = [
    "G90 G94 G91.1 G40 G49 G17",
    "G20",
    "G0 Z0.6250",
    "G0 X0 Y0",
]


class TestCollinearMerge(unittest.TestCase):

    def test_collinear_run_becomes_one_move(self):
        lines = PREAMBLE + [f"G1 X{x / 10:.4f} Y0.0000 F50.0" for x in range(1, 21)]
        out = optimize_gcode(lines)
        moves = [l for l in out if l.startswith('G1')]
        self.assertEqual(len(moves), 1)
        self.assertIn("X2.0000", moves[0])
        self.assertIn("Y0.0000", moves[0])

    def test_endpoints_never_move(self):
        # A dogleg: two straight runs meeting at a corner. The corner and both ends
        # must survive exactly.
        lines = PREAMBLE + (
            [f"G1 X{x / 10:.4f} Y0.0000 F50.0" for x in range(1, 11)] +
            [f"G1 X1.0000 Y{y / 10:.4f} F50.0" for y in range(1, 11)]
        )
        out = optimize_gcode(lines)
        moves = [l for l in out if l.startswith('G1')]
        self.assertEqual(len(moves), 2)
        self.assertIn("X1.0000 Y0.0000", moves[0])
        self.assertIn("X1.0000 Y1.0000", moves[1])

    def test_non_collinear_points_kept(self):
        lines = PREAMBLE + [
            "G1 X1.0000 Y0.0000 F50.0",
            "G1 X1.0000 Y1.0000 F50.0",
            "G1 X0.0000 Y1.0000 F50.0",
        ]
        out = optimize_gcode(lines)
        moves = [l for l in out if l.startswith('G1')]
        self.assertEqual(len(moves), 3)

    def test_zero_length_moves_dropped(self):
        lines = PREAMBLE + [
            "G1 X1.0000 Y1.0000 F50.0",
            "G1 X1.0000 Y1.0000 F50.0",   # exact duplicate
            "G1 X1.0000 Y1.0000 F50.0",   # and again
            "G1 X2.0000 Y1.5000 F50.0",   # bends, so it cannot merge with the first
        ]
        out = optimize_gcode(lines)
        moves = [l for l in out if l.startswith('G1')]
        self.assertEqual(len(moves), 2)

    def test_deviation_within_tolerance(self):
        # A gentle S-curve of many short chords: whatever the optimizer does, the
        # path must stay within tolerance.
        lines = list(PREAMBLE)
        for k in range(200):
            x = k * 0.01
            y = 0.3 * math.sin(x * 2.0)
            lines.append(f"G1 X{x:.4f} Y{y:.4f} F50.0")
        out = optimize_gcode(lines)
        self.assertLess(len(out), len(lines))
        # tolerance + two 4-decimal roundings
        self.assertLess(max_deviation(out, lines), DEFAULT_TOLERANCE + 0.0002)
        self.assertLess(max_deviation(lines, out), DEFAULT_TOLERANCE + 0.0002)


class TestArcFitting(unittest.TestCase):

    def test_dense_chorded_arc_becomes_g3(self):
        sx, sy = arc_start_point(1.0, 1.0, 0.5, 0)
        lines = PREAMBLE + [f"G1 X{sx:.4f} Y{sy:.4f} F50.0"] + \
            make_arc_chords(1.0, 1.0, 0.5, 0, 90, 40)
        out = optimize_gcode(lines)
        arcs = [l for l in out if l.startswith('G3 ')]
        self.assertEqual(len(arcs), 1)
        # CCW quarter circle from (1.5, 1) to (1, 1.5) about (1, 1): I=-0.5 J=0,
        # within the noise of the chords' own 4-decimal rounding.
        i_val = float(re.search(r'I(-?[\d.]+)', arcs[0]).group(1))
        j_val = float(re.search(r'J(-?[\d.]+)', arcs[0]).group(1))
        self.assertAlmostEqual(i_val, -0.5, delta=0.0005)
        self.assertAlmostEqual(j_val, 0.0, delta=0.0005)
        self.assertLess(max_deviation(out, lines), DEFAULT_TOLERANCE + 0.0002)

    def test_clockwise_arc_becomes_g2(self):
        sx, sy = arc_start_point(1.0, 1.0, 0.5, 90)
        lines = PREAMBLE + [f"G1 X{sx:.4f} Y{sy:.4f} F50.0"] + \
            make_arc_chords(1.0, 1.0, 0.5, 90, 0, 40)
        out = optimize_gcode(lines)
        self.assertEqual(len([l for l in out if l.startswith('G2 ')]), 1)

    def test_sparse_cocircular_points_stay_chords(self):
        # Five points on a 1" circle, 40 degrees apart: they are exactly cocircular,
        # but the original path is the CHORDS between them - sagitta ~0.06". Refitting
        # an arc through them would bulge far outside the programmed path (this was a
        # real bug: the innermost pass of a cleared pocket is a handful of cocircular
        # vertices, and the fitted arc gouged 0.5" into the pocket wall).
        pts = [(2 + math.cos(math.radians(a)), 2 + math.sin(math.radians(a)))
               for a in range(0, 200, 40)]
        lines = PREAMBLE + [f"G1 X{x:.4f} Y{y:.4f} F50.0" for x, y in pts]
        out = optimize_gcode(lines)
        self.assertEqual(len([l for l in out if l.startswith(('G2 ', 'G3 '))]), 0)
        self.assertLess(max_deviation(out, lines), DEFAULT_TOLERANCE + 0.0002)

    def test_helical_chords_become_helical_arc(self):
        # Chords descending linearly in Z along the arc -> one helical G3 with Z.
        lines = PREAMBLE + ["G1 X1.5000 Y1.0000 F32.0"] + \
            make_arc_chords(1.0, 1.0, 0.5, 0, 180, 40)
        # rewrite the chords to descend in Z
        body = []
        for i, l in enumerate(lines[len(PREAMBLE) + 1:], 1):
            z = 0.25 - 0.005 * i
            body.append(l.replace(" F32.0", f" Z{z:.4f} F32.0") if " Z" not in l else l)
        lines = lines[:len(PREAMBLE) + 1] + \
            make_arc_chords(1.0, 1.0, 0.5, 0, 180, 40, feed="32.0")
        with_z = []
        for i, l in enumerate(lines):
            if i <= len(PREAMBLE):
                with_z.append(l)
            else:
                k = i - len(PREAMBLE)
                z = 0.25 - 0.005 * k
                with_z.append(l.replace(" F32.0", f" Z{z:.4f} F32.0"))
        out = optimize_gcode(with_z)
        arcs = [l for l in out if l.startswith('G3 ')]
        self.assertEqual(len(arcs), 1)
        self.assertIn("Z0.0500", arcs[0])

    def test_arc_fitting_can_be_disabled(self):
        sx, sy = arc_start_point(1.0, 1.0, 0.5, 0)
        lines = PREAMBLE + [f"G1 X{sx:.4f} Y{sy:.4f} F50.0"] + \
            make_arc_chords(1.0, 1.0, 0.5, 0, 90, 40)
        out = optimize_gcode(lines, arc_fitting=False)
        self.assertEqual(len([l for l in out if l.startswith(('G2 ', 'G3 '))]), 0)
        # Still compresses collinear-ish runs within tolerance, path preserved
        self.assertLess(max_deviation(out, lines), DEFAULT_TOLERANCE + 0.0002)

    def test_emitted_arc_radii_consistent_after_rounding(self):
        # Mach3 rejects arcs whose start and end radii disagree. Check every emitted
        # arc block at the printed 4-decimal coordinates.
        lines = list(PREAMBLE)
        # several arcs of awkward radii
        for r, cx, cy in ((0.3701, 3.1, 2.7), (1.2345, 7.3, 4.9), (0.0839, 1.05, 9.87)):
            sx, sy = arc_start_point(cx, cy, r, 10)
            lines.append(f"G1 X{sx:.4f} Y{sy:.4f} F200.0")
            lines += make_arc_chords(cx, cy, r, 10, 170, 60)
        out = optimize_gcode(lines)
        pos = [0.0, 0.0]
        word = re.compile(r'([A-Za-z])(-?\d*\.?\d+)')
        arc_count = 0
        for raw in out:
            code = raw.split(';')[0].strip()
            m = re.match(r'^(G0|G1|G2|G3)\b', code)
            if not m:
                continue
            words = dict((l.upper(), float(v)) for l, v in word.findall(code))
            x, y = words.get('X', pos[0]), words.get('Y', pos[1])
            if m.group(1) in ('G2', 'G3'):
                arc_count += 1
                i, j = words.get('I', 0.0), words.get('J', 0.0)
                r_start = math.hypot(i, j)
                r_end = math.hypot(x - (pos[0] + i), y - (pos[1] + j))
                self.assertLess(abs(r_start - r_end), 0.0005,
                                f"arc radius mismatch in: {raw}")
            pos = [x, y]
        self.assertGreater(arc_count, 0)


class TestSafety(unittest.TestCase):

    def test_comments_and_setup_pass_through(self):
        lines = [
            "(===== HOLES =====)",
            "G90 G94 G91.1 G40 G49 G17",
            "G20  ; Inches",
            "S18000 M3  ; Spindle on",
            "G4 P2  ; Wait",
            "G54  ; Use work coordinate system 1",
            "M0  ; Program pause",
            "(Tabs: 3 tabs - desired spacing: 6.00\")",
        ]
        out = optimize_gcode(lines)
        self.assertEqual(out, lines)

    def test_commented_moves_kept_verbatim(self):
        # Tab plunges and other annotated moves must not be merged or reworded.
        lines = PREAMBLE + [
            "G1 X1.0000 Y0.0000 F50.0",
            "G1 Z0.1420 F14.0  ; Tab 1 start",
            "G1 X1.2500 Y0.0000 F50.0",
            "G1 Z-0.0080 F14.0  ; Tab end",
            "G1 X2.0000 Y0.0000 F50.0",
        ]
        out = optimize_gcode(lines)
        self.assertIn("G1 Z0.1420 F14.0  ; Tab 1 start", out)
        self.assertIn("G1 Z-0.0080 F14.0  ; Tab end", out)
        # The three XY moves are separated by Z moves, so all three survive.
        self.assertEqual(len([l for l in out if l.startswith('G1 X')]), 3)

    def test_ramp_segment_comments_may_merge(self):
        # "; Ramp segment N" labels are generated noise - a straight XY ramp with
        # linear Z descent collapses to one move. Positioning moves precede the ramp
        # (as the generators emit them) and stay put because of their comments.
        lines = list(PREAMBLE)
        lines.append("G1 X0.0000 Y0.0000 F200.0  ; Move to perimeter start")
        lines.append("G1 Z0.1750 F50.0  ; Approach to ramp start height")
        for k in range(1, 21):
            x = k * 0.05
            z = 0.175 - k * 0.005
            lines.append(f"G1 X{x:.4f} Y0.0000 Z{z:.4f} F32.0  ; Ramp segment {k}")
        out = optimize_gcode(lines)
        ramp_moves = [l for l in out if l.startswith('G1 X') and 'F32.0' in l]
        self.assertEqual(len(ramp_moves), 1)
        self.assertIn("X1.0000", ramp_moves[0])
        self.assertIn("Z0.0750", ramp_moves[0])

    def test_feed_change_splits_runs(self):
        lines = PREAMBLE + (
            [f"G1 X{x / 10:.4f} Y0.0000 F50.0" for x in range(1, 11)] +
            [f"G1 X{x / 10:.4f} Y0.0000 F70.0" for x in range(11, 21)]
        )
        out = optimize_gcode(lines)
        moves = [l for l in out if l.startswith('G1')]
        self.assertEqual(len(moves), 2)
        self.assertIn("F50.0", moves[0])
        self.assertIn("F70.0", moves[1])

    def test_redundant_feed_omitted_but_first_kept(self):
        lines = PREAMBLE + [
            "G1 X0.5000 Y0.5000 F50.0",
            "G1 X1.0000 Y0.0000 F50.0",
            "G1 X2.0000 Y1.0000 F50.0",
            "G1 X3.0000 Y0.0000 F50.0",
        ]
        out = optimize_gcode(lines)
        moves = [l for l in out if l.startswith('G1')]
        self.assertEqual(len(moves), 4)
        self.assertIn("F50.0", moves[0])         # feed stated entering the run
        for m in moves[1:]:
            self.assertNotIn("F", m)             # modal thereafter

    def test_rapids_and_arcs_pass_through(self):
        lines = PREAMBLE + [
            "G0 Z0.6250  ; Retract",
            "G3 X1.0000 Y1.0000 I-0.5000 J0 F32.0  ; Helical pass 1/14",
            "G0 X5.0000 Y5.0000",
        ]
        out = optimize_gcode(lines)
        for l in lines[len(PREAMBLE):]:
            self.assertIn(l, out)

    def test_g53_park_blocks_compression_until_position_restated(self):
        # After a machine-coordinate park, tracked work position is stale. The XY-only
        # moves that follow must not merge (Z unknown) until every axis is restated.
        lines = PREAMBLE + [
            "G53 G0 Z-0.5000  ; Park: raise to safe machine Z",
            "G53 G0 X1 Y30  ; Park: move gantry",
            "M0  ; Program pause",
            "G1 X1.0000 Y1.0000 F50.0",
            "G1 X2.0000 Y2.0000 F50.0",
            "G1 X3.0000 Y3.0000 F50.0",
        ]
        out = optimize_gcode(lines)
        # All three moves survive verbatim (no merge, Z never restated).
        for l in lines[-3:]:
            self.assertIn(l, out)

    def test_incremental_mode_disables_compression(self):
        lines = PREAMBLE + [
            "G91",
            "G1 X0.1000 Y0.0000 F50.0",
            "G1 X0.1000 Y0.0000 F50.0",
            "G1 X0.1000 Y0.0000 F50.0",
            "G90",
        ]
        out = optimize_gcode(lines)
        self.assertEqual(out, lines)

    def test_tolerance_zero_disables_everything(self):
        lines = PREAMBLE + [f"G1 X{x / 10:.4f} Y0.0000 F50.0" for x in range(1, 21)]
        out = optimize_gcode(lines, tolerance=0)
        self.assertEqual(out, lines)

    def test_short_runs_reemitted_verbatim(self):
        lines = PREAMBLE + [
            "G1 X1.0000 Y1.0000 F50.0",
            "G0 Z0.6250",
            "G1 X2.0000 Y2.0000 F50.0",
        ]
        out = optimize_gcode(lines)
        self.assertEqual(out, lines)

    def test_no_unicode_or_nested_comments_introduced(self):
        sx, sy = arc_start_point(1.0, 1.0, 0.5, 0)
        lines = PREAMBLE + [f"G1 X{sx:.4f} Y{sy:.4f} F50.0"] + \
            make_arc_chords(1.0, 1.0, 0.5, 0, 90, 40)
        out = optimize_gcode(lines)
        for l in out:
            l.encode('ascii')  # raises if non-ASCII
            self.assertNotRegex(l, r'\([^)]*\(')

    def test_path_stats_counts(self):
        lines = PREAMBLE + [
            "G1 X1.0000 Y1.0000 F50.0",
            "G3 X1.0000 Y1.0000 I-0.5000 J0 F32.0",
            "(a comment)",
        ]
        stats = path_stats(lines)
        self.assertEqual(stats['rapid'], 2)
        self.assertEqual(stats['linear'], 1)
        self.assertEqual(stats['arc'], 1)


class TestEndToEnd(unittest.TestCase):
    """Compression wired into the generators."""

    def _make_pp(self, config=None):
        from frc_cam_postprocessor import FRCPostProcessor
        pp = FRCPostProcessor(material_thickness=0.25, tool_diameter=0.157,
                              config=config)
        pp.apply_material_preset('plywood')
        # A 3x3 square pocket polygon and a 6x6 perimeter, as load_dxf would produce.
        pp.perimeter = [(0, 0), (6, 0), (6, 6), (0, 6)]
        pp.pockets = [[(2, 2), (5, 2), (5, 5), (2, 5)]]
        pp.holes = []
        return pp

    def test_generate_gcode_is_compressed_by_default(self):
        pp = self._make_pp()
        result = pp.generate_gcode(suggested_filename="t", timestamp="2026-01-01 00:00:00")
        self.assertTrue(result.success)
        compressed_lines = result.gcode.count('\n')

        from team_config import TeamConfig
        raw_config = TeamConfig({'machining': {'output': {'compression': False}}})
        pp_raw = self._make_pp(config=raw_config)
        result_raw = pp_raw.generate_gcode(suggested_filename="t", timestamp="2026-01-01 00:00:00")
        self.assertTrue(result_raw.success)
        raw_lines = result_raw.gcode.count('\n')

        self.assertLess(compressed_lines, raw_lines)

    def test_compressed_output_follows_raw_path(self):
        pp = self._make_pp()
        result = pp.generate_gcode(suggested_filename="t", timestamp="2026-01-01 00:00:00")

        from team_config import TeamConfig
        raw_config = TeamConfig({'machining': {'output': {'compression': False}}})
        pp_raw = self._make_pp(config=raw_config)
        result_raw = pp_raw.generate_gcode(suggested_filename="t", timestamp="2026-01-01 00:00:00")

        opt_lines = result.gcode.split('\n')
        raw_lines = result_raw.gcode.split('\n')
        self.assertLess(max_deviation(opt_lines, raw_lines), DEFAULT_TOLERANCE + 0.0002)
        self.assertLess(max_deviation(raw_lines, opt_lines), DEFAULT_TOLERANCE + 0.0002)


if __name__ == '__main__':
    unittest.main()
