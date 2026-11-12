"""Usage:  python layout/meander.py in out NET x1 y1 x2 y2 LAYER ADD [--amp 1.2] [--width 0.5]
       [--gap 0.5] [--side both|left|right] [--margin 0.8]
Lengthen one straight segment of NET (the one from (x1, y1) to (x2, y2) on LAYER) by ADD mm with
rectangular bumps to one side: each bump goes out by the amplitude, along by --width and back,
--gap after the previous one, so it adds twice the amplitude. Bumps are placed greedily along the
segment wherever they clear every other net (geo.Board, netclass clearances); the last one is cut
down to hit ADD. For skew matching a pair's short leg (the bumps of one leg are --gap apart, far
more than the 3 track widths that keep their self-coupling small)."""
import os, sys, re, math, uuid, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import Board, CU
from shapely.geometry import LineString

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("net")
for k in ("x1", "y1", "x2", "y2"): ap.add_argument(k, type=float)
ap.add_argument("layer"); ap.add_argument("add", type=float)
ap.add_argument("--amp", type=float, default=1.2)
ap.add_argument("--width", type=float, default=0.5)
ap.add_argument("--gap", type=float, default=0.5)
ap.add_argument("--side", default="both")
ap.add_argument("--margin", type=float, default=0.8)
a = ap.parse_args()
B = Board(a.inp)
t = B.t

# the segment to replace
seg = None
for m in re.finditer(r"\n\t\(segment\n.*?\n\t\)", t, re.S):
    b = m.group(0)
    if f'(net "{a.net}")' not in b or f'(layer "{a.layer}")' not in b: continue
    s = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups()))
    e = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()))
    for p, q in ((s, e), (e, s)):
        if math.dist(p, (a.x1, a.y1)) < 0.01 and math.dist(q, (a.x2, a.y2)) < 0.01:
            seg = (m.group(0), p, q, float(re.search(r"\(width ([\d.]+)\)", b).group(1)))
if not seg: sys.exit("segment not found")
blk, P0, P1, W = seg
L = math.dist(P0, P1)
u = ((P1[0] - P0[0]) / L, (P1[1] - P0[1]) / L)


def pt(s, n, side):   # s along the segment, n out to the side
    nx, ny = -u[1] * side, u[0] * side
    return (P0[0] + u[0] * s + nx * n, P0[1] + u[1] * s + ny * n)


def clear(pts):
    g = LineString(pts).buffer(W / 2, 8)
    return not (B.conflicts(a.layer, g, a.net) or B.hole_conflicts(g, a.net)
                or B.keepout_hit(a.layer, g, "tracks") or not B.edge_ok(g))


best = None
for side in ((1, -1) if a.side == "both" else ((1,) if a.side == "left" else (-1,))):
    bumps, need, s = [], a.add, a.margin
    while need > 1e-6 and s + a.width <= L - a.margin:
        A = min(a.amp, need / 2)
        poly = [pt(s, 0, side), pt(s, A, side), pt(s + a.width, A, side), pt(s + a.width, 0, side)]
        if clear(poly):
            bumps.append((s, A)); need -= 2 * A; s += a.width + a.gap
        else:
            s += 0.1
    got = a.add - need
    print(f"side {side:+d}: {len(bumps)} bumps, +{got:.2f} mm of {a.add}")
    if best is None or got > best[0]: best = (got, side, bumps)
got, side, bumps = best
if not bumps: sys.exit("no room")
pts = [P0]
for s, A in bumps:
    pts += [pt(s, 0, side), pt(s, A, side), pt(s + a.width, A, side), pt(s + a.width, 0, side)]
pts.append(P1)
txt = ""
for p, q in zip(pts, pts[1:]):
    if math.dist(p, q) < 1e-6: continue
    txt += (f'\n\t(segment\n\t\t(start {p[0]:.6f} {p[1]:.6f})\n\t\t(end {q[0]:.6f} {q[1]:.6f})\n\t\t(width {W})\n'
            f'\t\t(layer "{a.layer}")\n\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
t = t.replace(blk, txt, 1)
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"+{got:.2f} mm in {len(bumps)} bumps on side {side:+d}; written {a.out}")
