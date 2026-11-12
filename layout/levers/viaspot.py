"""viaspot.py board NET x y [--r 1.2] [--clear 0.15] [--n 8] [--skip REGEX]
Through-via spots for NET near (x, y): the 0.45 mm via clears every other net's copper by
--clear on all signal layers (In1/In4 are GND pours and In3 the power pour: their fill clears
round a via, so they are not obstacles) and other holes by 0.25. Nearest first.
--skip: nets whose copper is ignored (it is about to be ripped)."""
import os, sys, re, math, argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board, CU
from shapely.geometry import Point

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("net"); ap.add_argument("x", type=float); ap.add_argument("y", type=float)
ap.add_argument("--r", type=float, default=1.2); ap.add_argument("--clear", type=float, default=0.15)
ap.add_argument("--n", type=int, default=8); ap.add_argument("--skip", default=None)
ap.add_argument("--step", type=float, default=0.05)
a = ap.parse_args()
B = Board(a.board, ignore=a.skip)
skip = re.compile(a.skip) if a.skip else None


def ok(x, y):
    v = Point(x, y).buffer(0.225, 16)
    for l in CU:
        if l in ("In1.Cu", "In4.Cu", "In6.Cu"): continue
        for k in B.tree[l].query(v.buffer(a.clear + 0.05)):
            it = B.by_layer[l][k]
            if it.net == a.net or (skip and skip.search(it.net or "")): continue
            if it.geom.distance(v) < a.clear: return False
    for p, r, n in B.holes:
        if n == a.net: continue
        if p.distance(Point(x, y)) < r + 0.1 + 0.25: return False
    return True


res = []
n = int(a.r / a.step)
for i in range(-n, n + 1):
    for j in range(-n, n + 1):
        x, y = a.x + i * a.step, a.y + j * a.step
        d = math.hypot(x - a.x, y - a.y)
        if d <= a.r and ok(x, y): res.append((round(d, 3), round(x, 3), round(y, 3)))
res.sort()
for r in res[:a.n]: print(*r)
print(len(res), "spots")
