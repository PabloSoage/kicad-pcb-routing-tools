"""greedyplace.py board SPEC.txt [--box x0 y0 x1 y1] [--keep x0 y0 x1 y1]... > specs
Place small parts one by one, each at the free spot nearest its target point: its courtyard
(w x h, both orientations tried) clear of the courtyards on that side, of the parts placed before
it and of the keep-out boxes, and its body clear of that side's copper by 0.15 mm (any net: a pad
of one net would short a track of the other). Round courtyards (holes) are not read: keep-out them. SPEC lines: REF w h tx ty side [NET1,NET2]. Prints addfp.py specs
REF:x:y:rot:side, in SPEC order; a part with no spot is reported on stderr."""
import os, sys, re, math, argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import box
from shapely.ops import unary_union
ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("spec")
ap.add_argument("--box", nargs=4, type=float, required=True)
ap.add_argument("--keep", nargs=4, type=float, action="append", default=[])
ap.add_argument("--step", type=float, default=0.25)
ap.add_argument("--clear", type=float, default=0.15, help="body to other copper (0.2 next to LVDS)")
a = ap.parse_args()
B = Board(a.board); t = B.t
crt = {"F": [], "B": []}
for fp in re.finditer(r"\n\t\(footprint .*?\n\t\)", t, re.S):
    f = fp.group(0); side = "B" if '\n\t\t(layer "B.Cu")' in f else "F"
    at = re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", f)
    X, Y, A = float(at[1]), float(at[2]), math.radians(float(at[3] or 0)); pts = []
    for g in re.finditer(r"\(fp_(?:line|rect)\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\(end ([-\d.]+) ([-\d.]+)\)(?:(?!\(fp_).)*?\(layer \"" + side + r"\.CrtYd\"\)", f, re.S):
        for px, py in ((float(g[1]), float(g[2])), (float(g[3]), float(g[4]))):
            pts.append((X + px * math.cos(A) + py * math.sin(A), Y - px * math.sin(A) + py * math.cos(A)))
    if pts:
        xs, ys = zip(*pts); crt[side].append(box(min(xs), min(ys), max(xs), max(ys)))
keep = [box(*k) for k in a.keep]
placed = {"F": [], "B": []}
x0, y0, x1, y1 = a.box
for line in open(a.spec):
    p = line.split()
    if not p or p[0].startswith("#"): continue
    ref, w, h, tx, ty, side = p[0], float(p[1]), float(p[2]), float(p[3]), float(p[4]), p[5]
    own = set(p[6].split(",")) if len(p) > 6 else set()
    lay = side + ".Cu"
    obst = unary_union(crt[side] + placed[side] + keep)
    best = None
    pts = []
    x = x0
    while x <= x1:
        y = y0
        while y <= y1:
            pts.append((math.hypot(x - tx, y - ty), x, y)); y += a.step
        x += a.step
    for d, x, y in sorted(pts):
        if best and d > best[0]: break
        for rot, (ww, hh) in ((0, (w, h)), (90, (h, w))):
            c = box(x - ww / 2, y - hh / 2, x + ww / 2, y + hh / 2)
            if c.intersects(obst): continue
            body = c.buffer(-0.2)
            bad = False
            for k in B.tree[lay].query(body.buffer(a.clear)):
                it = B.by_layer[lay][k]
                if it.geom.distance(body) < a.clear: bad = True; break
            if not bad:
                best = (d, x, y, rot, c); break
        if best: break
    if not best:
        print(f"no spot for {ref}", file=sys.stderr); continue
    d, x, y, rot, c = best
    placed[side].append(c)
    print(f"{ref}:{x:g}:{y:g}:{rot}:{side}")
