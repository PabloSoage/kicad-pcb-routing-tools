"""rfstitch.py in out [--cls RF50,LVDS100] [--extra REGEX] [--fence-box x0 y0 x1 y1] [--pitch 1.0] [--off 0.8]
Ground the high-speed lines' return path:
  * every via of an RF50 net gets two GND vias beside it (0.75-1.8 mm away, on opposite sides
    of the line where possible), every via of the other classes (LVDS) and --extra nets
    (clocks) one within 2 mm (up to 2.9 when nothing nearer is free): the return current
    changes reference plane with the signal. GND vias already that near count;
  * inside --fence-box, every RF50 track on L1 gets a GND via fence on both sides, --off mm from
    its centre every --pitch mm (ties the L1 band pour to L2).
A spot is used only if the via clears every other net by 0.2 mm on the signal layers and other
holes by 0.25 mm (the pours on In1/In3/In4 clear round it). Text edit, no pcbnew."""
import os, sys, re, math, uuid, argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board, CU
from shapely.geometry import Point, LineString

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out")
ap.add_argument("--cls", default="RF50")
ap.add_argument("--extra", default=None)
ap.add_argument("--fence-box", nargs=4, type=float, default=None)
ap.add_argument("--pitch", type=float, default=1.0); ap.add_argument("--off", type=float, default=0.8)
a = ap.parse_args()
B = Board(a.inp)
t = B.t
CLS = a.cls.split(",")
xre = re.compile(a.extra) if a.extra else None
rf = {i.net for i in B.items if i.net and (B.netclass(i.net) in CLS or (xre and xre.search(i.net) and not i.net.startswith("+")))}
gvias = [it.data[:2] for it in B.items if it.kind == "via" and it.layer == "F.Cu" and it.net == "GND"]
placed = []          # (x, y) of the GND vias added here
SIG = [l for l in CU if l not in ("In1.Cu", "In4.Cu")]


def ok(x, y):
    v = Point(x, y).buffer(0.225, 16)
    if not B.edge_ok(v): return False
    for l in SIG:
        for k in B.tree[l].query(v.buffer(0.45)):
            it = B.by_layer[l][k]
            if it.net == "GND": continue
            need = 0.2 if it.net in rf else 0.15
            if it.geom.distance(v) < need: return False
    for p, r, n in B.holes:
        if p.distance(Point(x, y)) < r + 0.1 + 0.25: return False
    for q in placed:
        if math.dist(q, (x, y)) < 0.45 + 0.25: return False
    return True


# 1. return vias beside each RF via
segs = {}
for it in B.items:
    if it.kind == "seg" and it.net in rf:
        (sx, sy), (ex, ey), w = it.data
        segs.setdefault(it.net, []).append(((sx, sy), (ex, ey)))
rv = 0
seen = set()
for it in B.items:
    if it.kind != "via" or it.net not in rf: continue
    x, y, sz = it.data
    if (x, y) in seen: continue
    seen.add((x, y))
    # direction of the tracks at the via
    dirs = []
    for (s, e) in segs.get(it.net, []):
        for p, q in ((s, e), (e, s)):
            if math.dist(p, (x, y)) < 0.05 and math.dist(p, q) > 1e-3:
                dirs.append(math.atan2(q[1] - p[1], q[0] - p[0]))
    base = (dirs[0] + math.pi / 2) if dirs else 0.0
    isrf = B.netclass(it.net) == "RF50"
    want = 2 if isrf else 1             # LVDS/clocks: lambda/20 at 1.2 GHz is 6 mm; 2 mm is plenty
    got = sum(1 for q in gvias + placed if math.dist(q, (x, y)) <= (1.0 if isrf else 2.0))
    have = got
    for r in (0.6, 0.75, 0.85, 1.0, 1.1, 1.3, 1.5, 1.8) + (() if isrf else (2.0, 2.3, 2.6, 2.9)):
        if got >= want: break
        for k in range(24):
            if got >= want: break
            ang = base + (math.pi if got == 1 else 0) + (k // 2) * (math.pi / 12) * (1 if k % 2 == 0 else -1)
            px, py = x + r * math.cos(ang), y + r * math.sin(ang)
            if ok(px, py):
                placed.append((round(px, 3), round(py, 3))); got += 1
    rv += got - have
    if got < want: print(f"{it.net} via ({x}, {y}): only {got} return vias")

# 2. fence along L1 RF tracks inside the box
fv = 0
if a.fence_box:
    x0, y0, x1, y1 = a.fence_box
    for net, ss in segs.items():
        for (s, e) in ss:
            L = math.dist(s, e)
            if L < 0.3: continue
            ux, uy = (e[0] - s[0]) / L, (e[1] - s[1]) / L
            n = int(L / a.pitch)
            for j in range(n + 1):
                cx, cy = s[0] + ux * j * a.pitch, s[1] + uy * j * a.pitch
                if not (x0 <= cx <= x1 and y0 <= cy <= y1): continue
                for sgn in (1, -1):
                    px, py = cx - uy * a.off * sgn, cy + ux * a.off * sgn
                    if ok(px, py):
                        placed.append((round(px, 3), round(py, 3))); fv += 1

new = []
for x, y in placed:
    new.append(f'\t(via\n\t\t(at {x} {y})\n\t\t(size 0.45)\n\t\t(drill 0.2)\n\t\t(layers "F.Cu" "B.Cu")\n'
               f'\t\t(capping yes)\n\t\t(filling yes)\n\t\t(net "GND")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
end = t.rstrip().rfind(")")
t = t[:end].rstrip("\n") + "\n" + "\n".join(new) + "\n)\n"
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"RF nets {len(rf)}; return vias {rv}; fence vias {fv}")
