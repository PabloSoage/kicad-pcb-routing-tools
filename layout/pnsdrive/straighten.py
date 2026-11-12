"""straighten.py in out NET LAYER x0 y0 x1 y1 [--tol 0.04]
Replace the chain of NET's tracks on LAYER inside the box by the same chain simplified to TOL
(Douglas-Peucker; the ends stay put), when the result is clear of other copper: removes the
few-hundredths-of-a-millimetre jogs a walkaround leaves on a leg, which KiCad's DRC no longer
counts as coupled to the other leg. Text edit; DRC after."""
import sys, os, re, math, uuid, argparse
from shapely.geometry import LineString, box
from shapely.ops import linemerge
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("net"); ap.add_argument("layer")
ap.add_argument("box", nargs=4, type=float)
ap.add_argument("--tol", type=float, default=0.04)
A = ap.parse_args()

B = Board(A.inp)
t = B.t
win = box(*A.box)
segs = [i for i in B.by_layer[A.layer] if i.kind == "seg" and i.net == A.net and win.contains(LineString(i.data[:2]))]
if not segs:
    sys.exit("no segments in the box")
w = segs[0].data[2]
merged = linemerge([LineString(s.data[:2]) for s in segs])
chains = [merged] if merged.geom_type == "LineString" else list(merged.geoms)
pair = re.sub(r"_P$", "_N", A.net) if A.net.endswith("_P") else re.sub(r"_N$", "_P", A.net)
new, drop = [], set(s.uuid for s in segs)
for c in chains:
    s = c.simplify(A.tol, preserve_topology=False)
    g = s.buffer(w / 2, 8)
    bad = [x for x in B.conflicts(A.layer, g, A.net) if x.net != pair and x.uuid not in drop]
    bad += [x for x in B.by_layer[A.layer] if x.net == pair and x.kind == "seg" and g.distance(x.geom) < 0.1]
    if bad:
        print(f"chain of {len(c.coords)} points: simplified one collides with {sorted(set(b.net for b in bad))}; kept")
        drop -= set(x.uuid for x in segs if c.buffer(1e-3).contains(LineString(x.data[:2])))
        continue
    pts = list(s.coords)
    print(f"chain {len(c.coords)} -> {len(pts)} points")
    for p0, p1 in zip(pts, pts[1:]):
        new.append(f'\t(segment\n\t\t(start {p0[0]:.6f} {p0[1]:.6f})\n\t\t(end {p1[0]:.6f} {p1[1]:.6f})\n'
                   f'\t\t(width {w})\n\t\t(layer "{A.layer}")\n\t\t(net "{A.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
for u in drop:
    t = re.sub(r'\n\t\(segment\n(?:(?!\n\t\)).)*?\(uuid "%s"\)\n\t\)' % re.escape(u), "", t, count=1, flags=re.S)
end = t.rstrip().rfind(")")
t = t[:end].rstrip("\n") + "\n" + "\n".join(new) + "\n)\n"
open(A.out, "w", encoding="utf-8", newline="").write(t)
print(f"{len(drop)} segments replaced by {len(new)}")
