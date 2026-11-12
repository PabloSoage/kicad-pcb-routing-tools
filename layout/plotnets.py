"""plotnets.py board x0 y0 x1 y1 out.png [--layers F.Cu,In2.Cu] [--nets REGEX] [--before OLD.kicad_pcb]
                [--title TEXT]
A picture of a board's copper in a box (text parse, no pcbnew): every pad, track and via on the
chosen layers in light grey, the nets matching --nets in colour (one colour per layer), and with
--before the same nets as they were on another board, as dashed centre lines. For reviews,
before/after figures and checking what a script did."""
import os, sys, re, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import Board
from shapely.geometry import box
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("x0", type=float); ap.add_argument("y0", type=float)
ap.add_argument("x1", type=float); ap.add_argument("y1", type=float); ap.add_argument("out")
ap.add_argument("--layers", default="F.Cu,In2.Cu,In5.Cu,B.Cu")
ap.add_argument("--nets", default="")
ap.add_argument("--before", default="")
ap.add_argument("--title", default="")
A = ap.parse_args()

COL = {"F.Cu": "#d6452f", "In1.Cu": "#7a5195", "In2.Cu": "#e8a33d", "In3.Cu": "#2a9d8f",
       "In4.Cu": "#8d6e63", "In5.Cu": "#2f9e44", "In6.Cu": "#5c7cfa", "B.Cu": "#2a78d6"}
win = box(A.x0, A.y0, A.x1, A.y1)
layers = A.layers.split(",")
fig, ax = plt.subplots(figsize=(10, 10 * (A.y1 - A.y0) / (A.x1 - A.x0)))


def polys(g):
    g = g.intersection(win)
    return [p for p in getattr(g, "geoms", [g]) if p.geom_type == "Polygon" and not p.is_empty]


B = Board(A.board)
for hl in (False, True):                       # the background first, the highlighted nets on top
    for l in layers:
        for it in B.by_layer.get(l, []):
            if not it.geom.intersects(win):
                continue
            mine = bool(A.nets) and bool(re.search(A.nets, it.net or ""))
            if mine != hl:
                continue
            for p in polys(it.geom):
                xs, ys = p.exterior.xy
                ax.fill(xs, ys, color=COL.get(l, "k") if hl else "#cfcdc8", alpha=0.85 if hl else 0.6, lw=0)

if A.before and A.nets:
    t = open(A.before, encoding="utf-8").read()
    for m in re.finditer(r"\n\t\(segment\n.*?\n\t\)", t, re.S):
        b = m.group(0)
        net = re.search(r'\(net "([^"]*)"\)', b).group(1)
        if not re.search(A.nets, net) or re.search(r'\(layer "([^"]+)"', b).group(1) not in layers:
            continue
        s = list(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups()))
        e = list(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()))
        ax.plot([s[0], e[0]], [s[1], e[1]], "--", color="#3b3a37", lw=1)
    ax.plot([], [], "--", color="#3b3a37", lw=1, label="before")

for l in layers:
    if A.nets:
        ax.fill([], [], color=COL.get(l, "k"), label=l)
ax.set_xlim(A.x0, A.x1); ax.set_ylim(A.y1, A.y0); ax.set_aspect("equal")
ax.set_xlabel("x, mm"); ax.set_ylabel("y, mm")
if A.nets:
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
if A.title:
    ax.set_title(A.title, fontsize=10)
fig.savefig(A.out, dpi=120, bbox_inches="tight")
print("written", A.out)
