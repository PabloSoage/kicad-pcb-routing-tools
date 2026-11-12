"""waypoints.py board LAYER x0,y0 x1,y1 [--net NET] [--soft REGEX] [--half 0.175] [--margin 6]
[--soft-cost 8] [--grid 0.05] [--eps 0.15] [--png out.png]

A global path for a pair (or a track) on one layer, for pns_drive to follow: a grid A* from x0,y0
to x1,y1 with 8 directions. Hard obstacles are every pad, via and track on the layer of other nets,
each grown by the clearance plus HALF (half the pair's width: 0.175 for 0.10/0.15). Tracks of nets
matching --soft (default: the LVDS pairs) are not obstacles but cost --soft-cost per cell crossed:
KiCad's router shoves them aside. Prints "move X Y" lines (the path simplified to EPS) and the
path's length; exit 1 if no path."""
import sys, os, re, math, heapq, argparse
import numpy as np
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union
import shapely

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("layer"); ap.add_argument("a"); ap.add_argument("b")
ap.add_argument("--net", default="")
ap.add_argument("--soft", default=r"(ADC[RS]_|LINK_|FPGA_CLK|CLK_ADC|CLK_FPGA).*_[PN]$")
ap.add_argument("--half", type=float, default=0.175)
ap.add_argument("--margin", type=float, default=6.0)
ap.add_argument("--soft-cost", type=float, default=8.0)
ap.add_argument("--grid", type=float, default=0.05)
ap.add_argument("--eps", type=float, default=0.15)
ap.add_argument("--free", type=float, default=0.6, help="radius kept free round each end")
ap.add_argument("--png", default="")
A = ap.parse_args()

a = tuple(map(float, A.a.split(","))); b = tuple(map(float, A.b.split(",")))
B = Board(A.board)
own = re.sub(r"_[PN]$", "", A.net.split("/")[-1]) if A.net else None
x0, x1 = min(a[0], b[0]) - A.margin, max(a[0], b[0]) + A.margin
y0, y1 = min(a[1], b[1]) - A.margin, max(a[1], b[1]) + A.margin
G = A.grid
NX, NY = int((x1 - x0) / G) + 1, int((y1 - y0) / G) + 1
xs = x0 + np.arange(NX) * G; ys = y0 + np.arange(NY) * G
XX, YY = np.meshgrid(xs, ys, indexing="ij")

hard, soft = [], []
win = box(x0, y0, x1, y1)
for it in B.by_layer[A.layer]:
    if not it.geom.intersects(win): continue
    base = it.net.split("/")[-1]
    if own and re.sub(r"_[PN]$", "", base) == own: continue
    clr = B.clr(it.net, A.net or it.net)
    if it.kind == "seg" and re.search(A.soft, base):
        soft.append(it.geom.buffer(A.half + 0.05))
    else:
        hard.append(it.geom.buffer(clr + A.half))
for h, r, n in B.holes:                       # holes of other nets (vias are in hard already)
    if win.contains(h) and n.split("/")[-1] and not (own and re.sub(r"_[PN]$", "", n.split("/")[-1]) == own):
        hard.append(h.buffer(r + 0.2 + A.half))
for l, poly, f in B.keepouts:
    if l == A.layer and f.get("tracks") == "not_allowed": hard.append(poly.buffer(A.half))
for e in B.edge:
    hard.append(e.buffer(0.3 + A.half))

H = unary_union(hard) if hard else None
S = unary_union(soft) if soft else None
blocked = shapely.contains_xy(H, XX, YY) if H is not None else np.zeros_like(XX, bool)
softm = shapely.contains_xy(S, XX, YY) if S is not None else np.zeros_like(XX, bool)
for p in (a, b):                              # the ends sit on the pair's own vias/pads
    blocked &= ~((XX - p[0]) ** 2 + (YY - p[1]) ** 2 <= A.free ** 2)

cost = 1.0 + A.soft_cost * softm
cell = lambda p: (int(round((p[0] - x0) / G)), int(round((p[1] - y0) / G)))
s, t = cell(a), cell(b)
D = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
dist = {s: 0.0}; prev = {}; pq = [(0.0, s)]
h = lambda c: math.hypot(c[0] - t[0], c[1] - t[1]) * 0.9
while pq:
    f, c = heapq.heappop(pq)
    if c == t: break
    dc = dist[c]
    if f - h(c) > dc + 1e-9: continue
    for dx, dy, w in D:
        n = (c[0] + dx, c[1] + dy)
        if not (0 <= n[0] < NX and 0 <= n[1] < NY) or blocked[n]: continue
        nd = dc + w * cost[n]
        if nd < dist.get(n, 1e18):
            dist[n] = nd; prev[n] = c; heapq.heappush(pq, (nd + h(n), n))


def draw(pts):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 12 * NY / NX))
    img = np.zeros((NY, NX, 3)) + 1
    img[blocked.T] = (0.6, 0.6, 0.6); img[softm.T & ~blocked.T] = (1, 0.8, 0.5)
    ax.imshow(img, origin="lower", extent=(x0, x1, y0, y1))
    if pts: ax.plot(*zip(*pts), "r-", lw=1.5)
    ax.plot(*zip(a, b), "b+", ms=12); ax.invert_yaxis(); ax.set_aspect("equal")
    fig.savefig(A.png, dpi=110, bbox_inches="tight")


if t not in prev and t != s:
    print("no path")
    if A.png: draw(None)                      # the blocked map, to see what is in the way
    sys.exit(1)
path = [t]
while path[-1] != s: path.append(prev[path[-1]])
path = [(x0 + i * G, y0 + j * G) for i, j in reversed(path)]
line = LineString(path).simplify(A.eps)
pts = list(line.coords)
geo_len = LineString(path).length
crossed = sum(softm[cell(p)] for p in path) * G
print(f"# {A.layer}: path {geo_len:.2f} mm (straight {math.dist(a, b):.2f}), "
      f"{crossed:.2f} mm over soft tracks, {len(pts)} points")
for p in pts[1:]:
    print(f"move {p[0]:.3f} {p[1]:.3f}")

if A.png:
    draw(pts)
