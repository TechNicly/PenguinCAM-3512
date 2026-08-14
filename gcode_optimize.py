"""Toolpath compression for generated G-code.

PenguinCAM builds every toolpath as a Shapely polyline and emits one G1 per vertex.
That is correct but extremely verbose: a DXF ARC is sampled into chords, Shapely's
buffer() adds its own round-join vertices on top, and the result is a program where a
0.09" corner fillet costs ~21 blocks and a whole bellypan costs ~25,000 lines - about
6x what Fusion 360 emits for the same part. Long programs of very short blocks are not
just big files; they starve a controller's look-ahead (Mach3 in particular stutters and
drops its toolpath preview), so the machine runs slower and rougher than it needs to.

This module runs as the LAST step of G-code generation and rewrites runs of consecutive
short G1 moves into the smallest set of moves that stays within `tolerance` of the
original path:

  * exact-duplicate vertices are dropped
  * collinear runs collapse into one G1
  * circular runs become a single G2/G3 (the arcs the geometry always was)
  * a redundant F word is omitted when the feed has not changed

Everything else passes through byte-for-byte. Because it works on the emitted text, it
covers every toolpath generator (holes, pockets, contours, perimeters, tube ops) without
any of them having to know about it, and it operates on the exact 4-decimal coordinates
that will be written to the file - so a fitted arc is verified against the numbers the
controller will actually see.

Safety properties (each verified by tests in tests/test_gcode_optimize.py):
  * every emitted point is within `tolerance` of the original path (default 0.0005",
    an order of magnitude tighter than a hobby router's positioning accuracy)
  * run endpoints are preserved exactly - compression never moves where a cut starts,
    stops, or changes Z, so tabs, ramps, and plunges land where they did before
  * a fitted arc's start and end radii are equal to within ARC_RADIUS_EPSILON after the
    I/J words are rounded for output, so controllers that cross-check arc radius
    (Mach3's "radius to end of arc differs from radius to start") accept them
  * arcs are only emitted in G91.1 (incremental I/J) mode, which the header always sets
"""

import math
import re
from typing import List, Optional, Tuple

# Maximum deviation, in inches, that a compressed move may introduce relative to the
# original polyline. Well below the accuracy of the machines PenguinCAM targets, and
# below the 0.001" grid that DXF stitching already snaps to.
DEFAULT_TOLERANCE = 0.0005

# An arc is only worth emitting when it replaces several linear blocks. Fewer points
# than this and a chord fit is just as good and less likely to surprise a controller.
MIN_ARC_POINTS = 5

# Never fold a near-full circle into one block: at >350 deg the start and end points sit
# close enough together that a controller can pick the wrong sweep direction.
MAX_ARC_SWEEP_DEG = 350.0

# Below this sweep an "arc" is indistinguishable from a straight line at our tolerance.
MIN_ARC_SWEEP_DEG = 5.0

# Plausible radius band. Outside it the three-point circle fit is numerically unstable
# (huge radius = collinear points, tiny radius = duplicate points).
MIN_ARC_RADIUS = 0.005
MAX_ARC_RADIUS = 1000.0

# Allowed mismatch between an arc's start radius and end radius once I/J have been
# rounded to the output precision. Mach3's default arc tolerance is 0.0005".
ARC_RADIUS_EPSILON = 0.0002

# Cap on how many source points one fitted arc may span, to bound the O(k^2) fit search.
MAX_ARC_POINTS = 256

# Inline comments that exist only to label one move in a long generated sequence. They
# would otherwise split a run and block compression, and 1,900 copies of
# "; Ramp segment 37" is exactly the kind of bulk this module exists to remove.
_DROPPABLE_COMMENT_RE = re.compile(r'^\s*Ramp segment \d+\s*$')

_MOTION_RE = re.compile(r'^(G0|G1|G2|G3)\b')
_WORD_RE = re.compile(r'([A-Za-z])\s*(-?\d*\.?\d+)')

# Blocks that move the tool in a frame we are not tracking (machine coordinates, or a
# redefined work origin). After one of these the modal position we have been following
# no longer describes where the tool is, so no run may start until every axis has been
# re-stated by a later block.
_FRAME_BREAK_RE = re.compile(r'\bG(?:28|30|53|92)\b')

# Distance-mode switches. PenguinCAM output is G90 throughout (G91.1 is the ARC-CENTER
# mode, not a distance mode), but the optimizer refuses to touch anything while G91
# incremental mode could be active, in case it is ever run over foreign G-code.
_G90_RE = re.compile(r'\bG90\b')
_G91_RE = re.compile(r'\bG91(?!\.)\b')

# Any coordinate word appearing on a line we did not parse as motion (bare "X1.5 Y2"
# continuation blocks, G10 offset writes, ...) may have moved the tool without our
# tracking it.
_COORD_WORD_RE = re.compile(r'\b[XYZIJ]\s*-?\d')


class _Move:
    """One parsed motion block."""

    __slots__ = ('code', 'x', 'y', 'z', 'feed_text', 'comment', 'raw', 'extra')

    def __init__(self, code, x, y, z, feed_text, comment, raw, extra):
        self.code = code            # 'G0' | 'G1' | 'G2' | 'G3'
        self.x = x                  # absolute position after this move
        self.y = y
        self.z = z
        self.feed_text = feed_text  # raw text of the F word, or None if absent
        self.comment = comment      # inline '; ...' text, or None
        self.raw = raw              # original line, for pass-through
        self.extra = extra          # True if the block carries words we do not model


def _split_comment(line: str) -> Tuple[str, Optional[str]]:
    """Split a line into its code portion and its inline ';' comment (if any)."""
    idx = line.find(';')
    if idx < 0:
        return line, None
    return line[:idx], line[idx + 1:]


def _parse_move(line: str, pos: Tuple[float, float, float]) -> Optional[_Move]:
    """Parse one line into a _Move, resolving omitted axes against `pos`.

    Returns None for anything that is not a G0/G1/G2/G3 block - comments, blank lines,
    modal setup, M-codes, dwells and G53/G43-style blocks all pass through untouched.
    """
    code_part, comment = _split_comment(line)
    stripped = code_part.strip()
    m = _MOTION_RE.match(stripped)
    if not m:
        return None
    code = m.group(1)

    words = {}
    duplicated = False
    for letter, value in _WORD_RE.findall(stripped):
        letter = letter.upper()
        if letter in words:
            duplicated = True
        words[letter] = value

    # The leading G word is in `words` too; drop it after checking nothing else G-ish
    # rides along (e.g. "G0 G53 X..."), which we would rather not rewrite.
    modelled = {'G', 'X', 'Y', 'Z', 'I', 'J', 'F'}
    extra = duplicated or bool(set(words) - modelled)

    def coord(letter, current):
        return float(words[letter]) if letter in words else current

    return _Move(
        code=code,
        x=coord('X', pos[0]),
        y=coord('Y', pos[1]),
        z=coord('Z', pos[2]),
        feed_text=words.get('F'),
        comment=comment,
        raw=line,
        extra=extra,
    )


def _is_compressible(move: _Move) -> bool:
    """True when a move may be merged with its neighbours.

    Only plain G1 cutting blocks qualify. Rapids, arcs already in the program, blocks
    carrying unmodelled words, and blocks whose inline comment says something a person
    would want to keep are all left exactly as they are.
    """
    if move.code != 'G1' or move.extra:
        return False
    if move.comment is not None and not _DROPPABLE_COMMENT_RE.match(move.comment):
        return False
    return True


# --------------------------------------------------------------------------- geometry


def _point_seg_distance(p, a, b) -> float:
    """Distance from 3D point `p` to the segment a-b."""
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length_sq = dx * dx + dy * dy + dz * dz
    if length_sq <= 0.0:
        return math.dist(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy + (p[2] - az) * dz) / length_sq
    t = max(0.0, min(1.0, t))
    return math.dist(p, (ax + t * dx, ay + t * dy, az + t * dz))


def _circle_through(p0, p1, p2):
    """Circumcircle of three 2D points, or None if they are (near) collinear."""
    ax, ay = p0
    bx, by = p1
    cx, cy = p2
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    a_sq = ax * ax + ay * ay
    b_sq = bx * bx + by * by
    c_sq = cx * cx + cy * cy
    ux = (a_sq * (by - cy) + b_sq * (cy - ay) + c_sq * (ay - by)) / d
    uy = (a_sq * (cx - bx) + b_sq * (ax - cx) + c_sq * (bx - ax)) / d
    return ux, uy


def _recentre_on_bisector(center, start, end):
    """Slide `center` onto the perpendicular bisector of start-end.

    A three-point circle fit puts the centre wherever the residuals fall, which leaves
    the start and end radii very slightly unequal. Controllers that validate arcs
    compare exactly those two radii, so we project the centre onto the bisector - the
    locus where they are equal by construction - keeping the component along the chord
    direction fixed only insofar as it stays the nearest such point.
    """
    mx = (start[0] + end[0]) / 2.0
    my = (start[1] + end[1]) / 2.0
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    chord_sq = dx * dx + dy * dy
    if chord_sq <= 0.0:
        return None
    # Remove the component of (center - midpoint) that lies along the chord.
    t = ((center[0] - mx) * dx + (center[1] - my) * dy) / chord_sq
    return center[0] - t * dx, center[1] - t * dy


def _sweep_steps(center, points) -> Optional[List[float]]:
    """Per-step signed sweeps in radians if `points` turn monotonically about `center`.

    Returns None when any step reverses direction (the points do not describe a single
    arc) or repeats an angle exactly.
    """
    angles = [math.atan2(p[1] - center[1], p[0] - center[0]) for p in points]
    steps = []
    sign = 0
    for i in range(1, len(angles)):
        delta = angles[i] - angles[i - 1]
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        if delta == 0.0:
            return None
        step_sign = 1 if delta > 0 else -1
        if sign == 0:
            sign = step_sign
        elif step_sign != sign:
            return None  # direction reversed: not a single arc
        steps.append(delta)
    return steps


def _fit_arc(points, tolerance: float, precision: int):
    """Fit one circular arc through `points`, or return None.

    Returns (center, ccw) with the centre already validated at output precision.
    """
    n = len(points)
    if n < MIN_ARC_POINTS:
        return None

    xy = [(p[0], p[1]) for p in points]
    start, end = xy[0], xy[-1]

    center = _circle_through(xy[0], xy[n // 2], xy[-1])
    if center is None:
        return None
    center = _recentre_on_bisector(center, start, end)
    if center is None:
        return None

    radius = math.hypot(start[0] - center[0], start[1] - center[1])
    if not (MIN_ARC_RADIUS <= radius <= MAX_ARC_RADIUS):
        return None

    for p in xy:
        if abs(math.hypot(p[0] - center[0], p[1] - center[1]) - radius) > tolerance:
            return None

    steps = _sweep_steps(center, xy)
    if steps is None:
        return None
    sweep = sum(steps)
    if not (math.radians(MIN_ARC_SWEEP_DEG) <= abs(sweep) <= math.radians(MAX_ARC_SWEEP_DEG)):
        return None

    # The vertices lying ON the circle is necessary but not sufficient: the original
    # path is the CHORDS between them, and the arc bulges outside each chord by its
    # sagitta r*(1-cos(step/2)). Sparse vertices that happen to be cocircular (e.g. the
    # 5-point innermost pass of a pocket) must stay chords, not become a fat arc.
    for step in steps:
        sagitta = radius * (1.0 - math.cos(abs(step) / 2.0))
        if sagitta > tolerance:
            return None

    # Z must advance linearly with sweep for a helical arc to reproduce the path.
    z_start, z_end = points[0][2], points[-1][2]
    if abs(z_end - z_start) > 1e-9:
        angles_from_start = 0.0
        for idx in range(1, n - 1):
            angles_from_start += steps[idx - 1]
            expected_z = z_start + (z_end - z_start) * (angles_from_start / sweep)
            if abs(points[idx][2] - expected_z) > tolerance:
                return None

    # Round I/J exactly as they will be written, then re-check that the controller's
    # own start-radius/end-radius comparison will pass.
    i_word = round(center[0] - start[0], precision)
    j_word = round(center[1] - start[1], precision)
    rounded_center = (start[0] + i_word, start[1] + j_word)
    r_start = math.hypot(start[0] - rounded_center[0], start[1] - rounded_center[1])
    r_end = math.hypot(end[0] - rounded_center[0], end[1] - rounded_center[1])
    if abs(r_start - r_end) > ARC_RADIUS_EPSILON:
        return None
    for p in xy:
        if abs(math.hypot(p[0] - rounded_center[0], p[1] - rounded_center[1]) - r_start) > tolerance:
            return None

    return (i_word, j_word), sweep > 0


def _is_straight(points, tolerance: float) -> bool:
    a, b = points[0], points[-1]
    return all(_point_seg_distance(p, a, b) <= tolerance for p in points[1:-1])


def _compress_run(points, tolerance: float, arc_fitting: bool, precision: int):
    """Greedily rewrite a polyline as the fewest line/arc moves within `tolerance`.

    `points` includes the starting position at index 0. Returns a list of
    ('line', point) / ('arc', point, (i, j), ccw) instructions covering points[1:].
    """
    out = []
    i = 0
    n = len(points)
    while i < n - 1:
        # Longest straight run starting at i.
        j_line = i + 1
        while j_line + 1 < n and _is_straight(points[i:j_line + 2], tolerance):
            j_line += 1

        j_arc = i
        arc = None
        if arc_fitting:
            j = i + MIN_ARC_POINTS - 1
            limit = min(n, i + MAX_ARC_POINTS)
            while j < limit:
                candidate = _fit_arc(points[i:j + 1], tolerance, precision)
                if candidate is None:
                    break
                arc, j_arc = candidate, j
                j += 1

        if arc is not None and (j_arc - i) > (j_line - i):
            out.append(('arc', points[j_arc], arc[0], arc[1]))
            i = j_arc
        else:
            out.append(('line', points[j_line]))
            i = j_line
    return out


# ------------------------------------------------------------------------------ emit


def _format(value: float, precision: int) -> str:
    text = f"{value:.{precision}f}"
    # Avoid "-0.0000", which some controllers' parsers dislike and which reads as a bug.
    if float(text) == 0.0:
        text = f"{0.0:.{precision}f}"
    return text


def _emit(instructions, start, run_feed, prev_feed, precision, drop_redundant_feed):
    """Turn compression instructions into G-code lines.

    Returns (lines, feed_in_force_afterwards) so the caller keeps tracking modal F.
    """
    lines = []
    prev = start
    active_feed = prev_feed
    for item in instructions:
        if item[0] == 'line':
            target = item[1]
            words = [f"G1 X{_format(target[0], precision)} Y{_format(target[1], precision)}"]
        else:
            _, target, (i_word, j_word), ccw = item
            words = [
                f"{'G3' if ccw else 'G2'}"
                f" X{_format(target[0], precision)} Y{_format(target[1], precision)}"
                f" I{_format(i_word, precision)} J{_format(j_word, precision)}"
            ]
        if abs(target[2] - prev[2]) > 0.5 * 10 ** -precision:
            words.append(f"Z{_format(target[2], precision)}")
        if run_feed is not None and (not drop_redundant_feed or run_feed != active_feed):
            words.append(f"F{run_feed}")
            active_feed = run_feed
        lines.append(' '.join(words))
        prev = target
    return lines, active_feed


# ------------------------------------------------------------------------------- api


def optimize_gcode(lines: List[str],
                   tolerance: float = DEFAULT_TOLERANCE,
                   arc_fitting: bool = True,
                   drop_redundant_feed: bool = True,
                   precision: int = 4) -> List[str]:
    """Compress runs of short linear moves in an already-generated program.

    Args:
        lines: the generated G-code, one block per element.
        tolerance: maximum path deviation in inches. 0 disables compression entirely.
        arc_fitting: emit G2/G3 for circular runs. Turn off for a controller that
            cannot do arcs (see docs/ASSUMPTIONS.md - Easel is the known case, and it
            is already unsupported for other reasons).
        drop_redundant_feed: omit the F word when the feed rate has not changed.
        precision: decimal places for coordinates. Must match the generators' own
            formatting so compression never re-rounds a coordinate.

    Returns:
        A new list of lines. Non-motion blocks are passed through unchanged.
    """
    if tolerance <= 0:
        return list(lines)

    out: List[str] = []
    pos = (0.0, 0.0, 0.0)
    feed_text: Optional[str] = None
    absolute_mode = True
    # Axes whose tracked value is not trustworthy - at program start, and again after any
    # machine-coordinate move. A run may only begin once all three have been re-stated.
    unknown_axes = {'X', 'Y', 'Z'}

    # A pending run of compressible G1 moves: the point it starts from plus every point
    # it passes through, the feed in force during it, the feed in force before it, and
    # the original lines (so a run too short to compress is re-emitted verbatim).
    run_points: List[Tuple[float, float, float]] = []
    run_raw: List[str] = []
    run_feed: Optional[str] = None
    run_prev_feed: Optional[str] = None

    def flush():
        nonlocal run_points, run_raw, run_feed, run_prev_feed, feed_text
        if run_points:
            if len(run_points) < 3:
                # Nothing to gain; re-emit the originals rather than reformatting them.
                out.extend(run_raw)
            else:
                instructions = _compress_run(run_points, tolerance, arc_fitting, precision)
                emitted, feed_text = _emit(instructions, run_points[0], run_feed,
                                           run_prev_feed, precision, drop_redundant_feed)
                out.extend(emitted)
        run_points = []
        run_raw = []
        run_feed = None
        run_prev_feed = None

    def start_run():
        nonlocal run_points, run_raw, run_feed, run_prev_feed
        run_points = [pos]
        run_raw = []
        run_prev_feed = feed_text

    for line in lines:
        code_part = _split_comment(line)[0]
        if _G91_RE.search(code_part):
            absolute_mode = False
        if _G90_RE.search(code_part):
            absolute_mode = True

        move = _parse_move(line, pos) if absolute_mode else None

        if move is None:
            flush()
            out.append(line)
            # Anything that may have moved the tool outside our tracking poisons the
            # modal position until every axis is re-stated: machine-coordinate or
            # offset-changing codes, incremental-mode motion, and any unparsed block
            # that carries coordinate words. Pure comment lines are exempt (their text
            # may mention X/Y/Z, e.g. a part-placement label).
            stripped = line.lstrip()
            is_comment = (not stripped) or stripped.startswith('(') or stripped.startswith(';')
            if not is_comment and (not absolute_mode or _FRAME_BREAK_RE.search(code_part)
                                   or _COORD_WORD_RE.search(code_part)):
                unknown_axes = {'X', 'Y', 'Z'}
            continue

        target = (move.x, move.y, move.z)
        line_feed = move.feed_text if move.feed_text is not None else feed_text
        unknown_axes -= {letter.upper() for letter, _ in _WORD_RE.findall(code_part)}

        if not unknown_axes and _is_compressible(move):
            if run_points and line_feed != run_feed:
                flush()  # feed changed mid-run: close it and open a new one
            if not run_points:
                start_run()
                run_feed = line_feed
            if math.dist(target, pos) > 0.5 * 10 ** -precision:
                run_points.append(target)
                run_raw.append(line)
        else:
            flush()
            out.append(line)

        pos = target
        feed_text = line_feed
        if _FRAME_BREAK_RE.search(code_part):
            unknown_axes = {'X', 'Y', 'Z'}

    flush()
    return out


def path_stats(lines: List[str]) -> dict:
    """Summarise a program: block count and motion-block breakdown. Used by tests and
    by the generation log so the effect of compression is visible."""
    counts = {'total': len(lines), 'rapid': 0, 'linear': 0, 'arc': 0, 'other': 0}
    pos = (0.0, 0.0, 0.0)
    for line in lines:
        move = _parse_move(line, pos)
        if move is None:
            counts['other'] += 1
            continue
        pos = (move.x, move.y, move.z)
        if move.code == 'G0':
            counts['rapid'] += 1
        elif move.code == 'G1':
            counts['linear'] += 1
        else:
            counts['arc'] += 1
    return counts
