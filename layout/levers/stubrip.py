"""stubrip.py in out LEG [LEG...] [--min-ps 3]
Delete the copper of an LVDS leg (PAIR_P or PAIR_N, as lvds_skew.py names them) that is off its
pin-to-pin path: every branch of more than --min-ps ps (the branch to a termination resistor
that is not passed through, say). The resistor is then free to be moved next to the path and
joined to it with a short stub (segadd.py, fastroute.py). Uses ../lvds_skew.py."""
import sys, os, re, io, runpy, contextlib, argparse
from shapely.geometry import Point, LineString

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("legs", nargs="+")
ap.add_argument("--min-ps", type=float, default=3.0)
a = ap.parse_args()
here = os.path.dirname(os.path.abspath(__file__))
skew = os.path.join(here, "..", "lvds_skew.py")
argv, sys.argv = sys.argv, ["lvds_skew.py", a.inp]
with contextlib.redirect_stdout(io.StringIO()):
    g = runpy.run_path(skew)
sys.argv = argv
legs, branches, leg_nets = g["legs"], g["branches"], g["leg_nets"]
t = open(a.inp, encoding="utf-8").read()
drop_segs, drop_vias = [], set()   # (layer, LineString) of the path to keep; vias to drop
n = 0
for name in a.legs:
    k = (name[:-2], name[-1])
    if "path" not in legs[k]: sys.exit(f"{name}: no path")
    path = legs[k]["path"]
    on = set(path)
    keep = {}
    for u, v in zip(path, path[1:]):   # path edges between track nodes on one layer
        if u[0] == "p" and v[0] == "p" and u[1] == v[1]:
            keep.setdefault(u[1], []).append(LineString([u[2:], v[2:]]))
    G = legs[k]["graph"]
    nodes = set()
    for u in G:
        if u in on: continue
        nodes.add(u)
    # keep only the components that belong to a big branch
    comp_of = {}
    for u in nodes:
        if u in comp_of: continue
        st, c = [u], set([u])
        while st:
            x = st.pop()
            for v, w in G[x]:
                if v not in on and v not in c: c.add(v); st.append(v)
        for x in c: comp_of[x] = frozenset(c)
    bad = set()
    for c in set(comp_of.values()):
        ps = sum(w for x in c for v, w in G[x] if v in c) / 2 + sum(w for x in c for v, w in G[x] if v in on)
        if ps > a.min_ps: bad |= c
    pts = {}
    for x in bad:
        if x[0] == "p": pts.setdefault(x[1], []).append(Point(x[2], x[3]))
        if x[0] == "v": drop_vias.add((round(x[1], 4), round(x[2], 4)))
    for l, ps in pts.items():
        drop_segs.append((set(leg_nets[k]), l, ps, keep.get(l, [])))


def kill(m):
    global n
    b = m.group(0)
    net = re.search(r'\(net "([^"]*)"\)', b)
    if not net: return b
    net = net.group(1)
    if m.group(1) == "via":
        x, y = map(float, re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b).groups())
        if (round(x, 4), round(y, 4)) in drop_vias and any(net in d[0] for d in drop_segs):
            n += 1; return ""
        return b
    L = re.search(r'\(layer "([^"]+)"\)', b).group(1)
    s = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups()))
    e = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()))
    seg = LineString([s, e])
    mid = seg.interpolate(0.5, normalized=True)
    for nets, l, ps, keep in drop_segs:
        if net not in nets or l != L: continue
        if any(k.distance(mid) < 0.01 for k in keep): continue   # on the path
        if any(p.distance(Point(s)) < 0.07 or p.distance(Point(e)) < 0.07 for p in ps):
            n += 1; return ""
    return b


t = re.sub(r"\n\t\((segment|via|arc)\n.*?\n\t\)", kill, t, flags=re.S)
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"{n} items deleted")
