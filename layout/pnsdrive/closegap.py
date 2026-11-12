"""closegap.py in out NET [NET...] [--max 2.0] [--trim 1.5]
Join each dangling track end of NET to the nearest via or pad of NET on the same layer, for the
last fraction of a millimetre that KiCad's router leaves between a coupled pair's head and its
target vias. The join is a straight track, or an L (horizontal then vertical, or the other way),
of the leg's width, clear of other nets (their clearance) and 0.1 mm from the pair's other leg,
including joins already added. When no join fits, the leg is cut back one segment at a time, up
to TRIM mm, and the joins are tried again from the new end. Text edit; DRC after."""
import sys, os, re, math, uuid, argparse
from shapely.geometry import LineString, Point
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("nets", nargs="+")
ap.add_argument("--max", type=float, default=2.0)
ap.add_argument("--trim", type=float, default=1.5)
A = ap.parse_args()

B = Board(A.inp)
t = B.t
new, added, removed = [], [], set()   # added: (net, layer, geom)


def seg_text(p0, p1, w, layer, net):
    return (f'\t(segment\n\t\t(start {p0[0]:.6f} {p0[1]:.6f})\n\t\t(end {p1[0]:.6f} {p1[1]:.6f})\n'
            f'\t\t(width {w})\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')


def clear(layer, g, net, pair):
    if [c for c in B.conflicts(layer, g, net) if c.net != pair and c.uuid not in removed]:
        return False
    for c in B.by_layer[layer]:
        if c.net == pair and c.kind == "seg" and c.uuid not in removed and g.distance(c.geom) < 0.1:
            return False
    for n, l, ag in added:
        if l == layer and n != net and g.distance(ag) < (0.1 if n == pair else 0.15):
            return False
    return True


for net in A.nets:
    pair = re.sub(r"_P$", "_N", net) if net.endswith("_P") else re.sub(r"_N$", "_P", net)
    segs = [i for i in B.items if i.kind == "seg" and i.net == net]
    vias = [i for i in B.items if i.kind == "via" and i.net == net and i.layer == "F.Cu"]
    pads = [i for i in B.items if i.kind == "pad" and i.net == net]

    def attached(end, layer, skip):
        n = [o for o in segs if o is not skip and o.uuid not in removed and o.layer == layer
             and (math.dist(o.data[0], end) < 0.005 or math.dist(o.data[1], end) < 0.005)]
        v = [x for x in vias if math.dist(x.data[:2], end) <= x.data[2] / 2]
        p = [x for x in pads if x.layer == layer and x.geom.contains(Point(end))]
        return n, v + p

    for s in segs:
        for end in (s.data[0], s.data[1]):
            if s.uuid in removed:
                break
            others, anchors = attached(end, s.layer, s)
            if others or anchors:
                continue
            w, layer = s.data[2], s.layer
            cur, cut, done = s, 0.0, False
            while not done:
                cands = [(math.dist(v.data[:2], end), v.data[:2]) for v in vias]
                cands += [(math.dist(p.data[2:4], end), p.data[2:4]) for p in pads if p.layer == layer]
                for d, q in sorted(c for c in cands if c[0] <= A.max + cut):
                    for path in ([end, q], [end, (q[0], end[1]), q], [end, (end[0], q[1]), q]):
                        path = [p for k, p in enumerate(path) if k == 0 or math.dist(p, path[k - 1]) > 1e-4]
                        g = LineString(path).buffer(w / 2, 8)
                        if not clear(layer, g, net, pair):
                            continue
                        for p0, p1 in zip(path, path[1:]):
                            new.append(seg_text(p0, p1, w, layer, net))
                        added.append((net, layer, g))
                        print(f"{net} {layer}: cut {cut:.2f} mm, {end[0]:.3f},{end[1]:.3f} -> "
                              f"{q[0]:.3f},{q[1]:.3f} ({len(path) - 1} segments)")
                        done = True
                        break
                    if done:
                        break
                if done:
                    break
                # cut back one segment
                a, e = cur.data[0], cur.data[1]
                other = e if math.dist(a, end) < 0.005 else a
                seglen = math.dist(a, e)
                if cut + seglen > A.trim:
                    print(f"{net} {layer}: {end[0]:.3f},{end[1]:.3f} dangling, no join within {A.trim} mm")
                    break
                removed.add(cur.uuid)
                cut += seglen
                end = other
                nxt, anchors = attached(end, layer, cur)
                if anchors or len(nxt) != 1:
                    print(f"{net} {layer}: cut back to a branch or anchor at {end[0]:.3f},{end[1]:.3f}; stop")
                    break
                cur = nxt[0]

for u in removed:
    t = re.sub(r'\n\t\(segment\n(?:(?!\n\t\)).)*?\(uuid "%s"\)\n\t\)' % re.escape(u), "", t, count=1, flags=re.S)
end = t.rstrip().rfind(")")
t = t[:end].rstrip("\n") + "\n" + "\n".join(new) + "\n)\n"
open(A.out, "w", encoding="utf-8", newline="").write(t)
print(f"{len(added)} joins, {len(removed)} segments cut back")
