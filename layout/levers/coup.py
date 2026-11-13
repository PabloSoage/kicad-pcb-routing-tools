import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import LineString
from shapely.ops import unary_union
B = Board(sys.argv[1]); pair = sys.argv[2]
segs = {n: [it for it in B.items if it.kind == "seg" and it.net == pair + s] for s, n in (("_P", "P"), ("_N", "N"))}
segs = {"P": [it for it in B.items if it.kind == "seg" and it.net == pair + "_P"], "N": [it for it in B.items if it.kind == "seg" and it.net == pair + "_N"]}
for k, o in (("P", "N"), ("N", "P")):
    tot = cp = 0
    other = {}
    for it in segs[o]: other.setdefault(it.layer, []).append(LineString(it.data[:2]))
    for it in segs[k]:
        L = LineString(it.data[:2]); n = max(1, int(L.length / 0.05))
        ou = unary_union(other.get(it.layer, [])) if other.get(it.layer) else None
        for i in range(n):
            p = L.interpolate((i + 0.5) / n, normalized=True)
            tot += L.length / n
            if ou is not None and ou.distance(p) <= 0.4: cp += L.length / n
    print(f"{pair}{'_'+k}: {tot:.1f} mm, coupled (partner within 0.4 mm, same layer) {cp:.1f} mm = {100*cp/max(tot,1e-9):.0f} %")
