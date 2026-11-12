"""pairlib.py: coupled differential pairs on one layer, routed as a centre line (A* on a 0.05 mm
grid, 45 degree moves) and then split into two parallel legs.

Two geometries: FULL outside the BGA (w 0.10, gap 0.15: 100 ohm on L3/L6 of JLC08161H-2116C;
0.15 mm to other nets, PAIR_CLR to other pairs) and NECK inside the BGA's box (w 0.09, gap 0.12:
98.5 ohm; 0.1 mm to everything: what passes between two vias at 1 mm pitch).

A terminal is two via centres (P, N). The centre line starts at their mid point pushed out at right
angles to their axis, far enough for each via to reach its leg with a 45 degree connector, so the
two legs are the same length at the ends. Geometry from geo.Board; text edit helpers."""
import math, heapq, uuid, re
import numpy as np
import shapely
from shapely.geometry import LineString, Point, box as sbox
from shapely.ops import unary_union

G = 0.05
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
DLEN = [1.0, math.sqrt(2)] * 4
FULL = dict(w=0.10, s=0.15, clr=0.15)
NECK = dict(w=0.09, s=0.12, clr=0.10)
PAIR_CLR = dict(full=0.25, neck=0.12)
# L1 (F.Cu) microstrip over L2 on 2116C: 0.117 / 0.15 mm is 100 ohm, 0.10 / 0.12 mm 99 ohm
GEOMS = {"F.Cu": (dict(w=0.117, s=0.15, clr=0.15), dict(w=0.10, s=0.12, clr=0.10))}


def geoms(layer):
    return GEOMS.get(layer, (FULL, NECK))
BGA = None                                     # (x0, y0, x1, y1)


def inside(x, y):
    return BGA is not None and BGA[0] <= x <= BGA[2] and BGA[1] <= y <= BGA[3]


def pitch(g):
    return g["w"] + g["s"]


def half(g):                                    # centre line to a leg's outer edge
    return g["w"] + g["s"] / 2


class Field:
    def __init__(s, B, layer, own, box, ispair, extra=()):
        """own: the pair's two nets (not obstacles). ispair(net): another pair's leg.
        extra: [(geometry, net)] copper not yet on the board (this run's earlier pairs)."""
        s.box = box = tuple(round(v / G) * G for v in box)
        x0, y0, x1, y1 = box
        win = sbox(x0 - 1, y0 - 1, x1 + 1, y1 + 1)
        obs = [(it.geom, it.net) for it in B.by_layer[layer] if it.net not in own and it.geom.intersects(win)]
        obs += [(g, n) for g, n in extra if n not in own and g.intersects(win)]
        holes = [p.buffer(r, 12) for p, r, n in B.holes if n not in own and win.contains(p)]
        s.nx = int(round((x1 - x0) / G)) + 1; s.ny = int(round((y1 - y0) / G)) + 1
        X, Y = np.meshgrid(x0 + np.arange(s.nx) * G, y0 + np.arange(s.ny) * G, indexing="ij")
        ins = np.zeros_like(X, dtype=bool)
        if BGA: ins = (X >= BGA[0]) & (X <= BGA[2]) & (Y >= BGA[1]) & (Y <= BGA[3])
        blocked = np.zeros_like(X, dtype=bool)
        full, neck = geoms(layer)
        for g, key, mask in ((full, "full", ~ins), (neck, "neck", ins)):
            if not mask.any(): continue
            parts = []
            for geom, n in obs:
                c = PAIR_CLR[key] if ispair(n) else g["clr"]
                if not ispair(n) and (n.startswith("+") or n in ("GND", "-3V3")):
                    c = max(c, 0.15)        # the Power netclass's 0.15 holds even under the FPGA (KiCad takes it)
                parts.append(geom.buffer(half(g) + c + 0.004, 8))
            parts += [h.buffer(half(g) + 0.2 + 0.004, 8) for h in holes]
            # rule areas that forbid tracks on this layer, and the board edge (0.3 mm)
            if B is not None:
                parts += [poly.buffer(half(g) + 0.004, 8) for l, poly, fl in B.keepouts
                          if l == layer and fl.get("tracks") == "not_allowed" and poly.intersects(win)]
                parts += [e.buffer(half(g) + 0.3 + 0.004, 8) for e in B.edge if e.intersects(win)]
            u = unary_union(parts)
            blocked |= mask & shapely.contains_xy(u, X, Y)
        s.blocked = blocked

    def cell(s, x, y):
        return int(round((x - s.box[0]) / G)), int(round((y - s.box[1]) / G))

    def xy(s, i, j):
        return round(s.box[0] + i * G, 4), round(s.box[1] + j * G, 4)

    def free(s, i, j):
        return 0 <= i < s.nx and 0 <= j < s.ny and not s.blocked[i, j]


def astar(F, starts, goals, lead=4, minseg=4, turn=3.0, weight=1.4, limit=4_000_000, pen=None):
    """Centre line on F's grid. starts: [(i, j, d)] leave cell (i, j) in direction d; goals:
    {(i, j): set of arrival directions}. The first and last 'lead' cells are straight, and every
    segment between two turns is at least 'minseg' cells (a turn is a macro move of minseg cells),
    so the offset legs never fold. State (i, j, d); costs in a numpy array."""
    nx, ny = F.nx, F.ny
    free = ~F.blocked
    P = pen if pen is not None else np.zeros((nx, ny), dtype=np.float32)
    INF = np.float32(1e30)
    cost = np.full((nx, ny, 8), INF, dtype=np.float32)
    came = {}
    # goals moved back 'lead' cells against their arrival direction
    gset = {}
    for (gi, gj), ds in goals.items():
        for d in ds:
            di, dj = DIRS[d]
            ok = all(0 <= gi - k * di < nx and 0 <= gj - k * dj < ny and free[gi - k * di, gj - k * dj] for k in range(lead + 1))
            if ok: gset.setdefault((gi - lead * di, gj - lead * dj, d), ((gi, gj), float(sum(P[gi - k * di, gj - k * dj] for k in range(lead)))))
    if not gset: return None, None
    gx = np.array([k[0] for k in gset]); gy = np.array([k[1] for k in gset])
    cx, cy = gx.mean(), gy.mean(); rad = float(np.max(np.hypot(gx - cx, gy - cy)))

    def h(i, j):
        dx, dy = abs(i - cx), abs(j - cy)
        return max(0.0, max(dx, dy) + 0.4142 * min(dx, dy) - rad) * weight
    pq = []
    for i, j, d in starts:
        di, dj = DIRS[d]
        if not all(0 <= i + k * di < nx and 0 <= j + k * dj < ny and free[i + k * di, j + k * dj] for k in range(lead + 1)):
            continue
        si, sj = i + lead * di, j + lead * dj
        c0 = lead * DLEN[d] + float(sum(P[i + k * di, j + k * dj] for k in range(lead + 1)))
        if c0 < cost[si, sj, d]:
            cost[si, sj, d] = c0; came[(si, sj, d)] = ("start", i, j)
            heapq.heappush(pq, (c0 + h(si, sj), c0, si, sj, d))
    n = 0
    while pq:
        f, c, i, j, d = heapq.heappop(pq)
        if c > cost[i, j, d] + 1e-6: continue
        if (i, j, d) in gset:
            (gi, gj), gpen = gset[(i, j, d)]
            if not getattr(F, "_final", False) and gpen > 0 and (i, j, d, "g") not in came:
                # the goal's own lead is penalised: push it back with that cost once
                came[(i, j, d, "g")] = 1
                heapq.heappush(pq, (f + gpen, c + gpen, i, j, d)); cost[i, j, d] = c + gpen
                continue
            path = [(gi, gj), (i, j)]; st = (i, j, d)
            while True:
                pr = came[st]
                if pr[0] == "start": path.append((pr[1], pr[2])); break
                path.append((pr[0], pr[1])); st = pr
            path.reverse()
            return path, c + lead * DLEN[d]
        n += 1
        if n > limit: break
        di, dj = DIRS[d]
        # straight one cell
        ni, nj = i + di, j + dj
        if 0 <= ni < nx and 0 <= nj < ny and free[ni, nj] and (d % 2 == 0 or (free[i + di, j] and free[i, j + dj])):
            nc = c + DLEN[d] + P[ni, nj]
            if nc < cost[ni, nj, d]:
                cost[ni, nj, d] = nc; came[(ni, nj, d)] = (i, j, d); heapq.heappush(pq, (nc + h(ni, nj), nc, ni, nj, d))
        # turn 45 degrees and run minseg cells
        for nd in ((d + 1) % 8, (d - 1) % 8):
            ei, ej = DIRS[nd]; ok = True; pi, pj = i, j; pp = 0.0
            for k in range(minseg):
                qi, qj = pi + ei, pj + ej
                if not (0 <= qi < nx and 0 <= qj < ny and free[qi, qj]) or (nd % 2 and not (free[pi + ei, pj] and free[pi, pj + ej])):
                    ok = False; break
                pi, pj = qi, qj; pp += P[qi, qj]
            if not ok: continue
            nc = c + turn + minseg * DLEN[nd] + pp
            if nc < cost[pi, pj, nd]:
                cost[pi, pj, nd] = nc; came[(pi, pj, nd)] = (i, j, d); heapq.heappush(pq, (nc + h(pi, pj), nc, pi, pj, nd))
    return None, None


def simplify(pts):
    out = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        if abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) > 1e-9: out.append(b)
    out.append(pts[-1]); return out


def offset(pts, d):
    """Parallel polyline at signed distance d to the left of travel (normal (-dy, dx)), mitred."""
    nor = []
    for a, b in zip(pts, pts[1:]):
        L = math.dist(a, b); nor.append((-(b[1] - a[1]) / L, (b[0] - a[0]) / L))
    out = []
    for k in range(len(pts)):
        if k == 0: nx, ny = nor[0]; m = 1.0
        elif k == len(pts) - 1: nx, ny = nor[-1]; m = 1.0
        else:
            ax, ay = nor[k - 1]; bx, by = nor[k]
            nx, ny = ax + bx, ay + by; L = math.hypot(nx, ny); nx, ny = nx / L, ny / L
            m = 1.0 / (nx * ax + ny * ay)
        out.append((round(pts[k][0] + nx * d * m, 4), round(pts[k][1] + ny * d * m, 4)))
    return out


def zones(pts):
    """The centre line cut where it crosses the BGA box: [(inside, pts)]."""
    out = []; cur = [pts[0]]; cin = inside(*pts[0])
    for a, b in zip(pts, pts[1:]):
        L = math.dist(a, b); n = max(1, int(L / 0.02))
        for k in range(1, n + 1):
            q = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            if inside(*q) != cin:
                q = (round(q[0], 4), round(q[1], 4)); cur.append(q); out.append((cin, cur)); cur = [q]; cin = not cin
        cur.append(b)
    out.append((cin, cur))
    return [(z, simplify(p)) for z, p in out if len(p) > 1 and LineString(p).length > 1e-6]


def legs(centre, p_left, layer=None):
    """-> {'P': [(pts, w)], 'N': [(pts, w)]}; p_left: P is the left leg."""
    out = {"P": [], "N": []}
    full, neck = geoms(layer)
    for zin, pts in zones(centre):
        g = neck if zin else full; d = pitch(g) / 2
        out["P"].append((offset(pts, d if p_left else -d), g["w"]))
        out["N"].append((offset(pts, -d if p_left else d), g["w"]))
    for k in out:
        j = []
        for pts, w in out[k]:
            if j and math.dist(j[-1][0][-1], pts[0]) > 1e-6: j.append(([j[-1][0][-1], pts[0]], min(w, j[-1][1])))
            j.append((pts, w))
        out[k] = j
    return out


def left_of(a, b, p):
    """p left of travel a -> b (normal (-dy, dx))?"""
    return (-(b[1] - a[1])) * (p[0] - a[0]) + (b[0] - a[0]) * (p[1] - a[1]) > 0


def seg_text(net, layer, w, pts):
    out = []
    for a, b in zip(pts, pts[1:]):
        if math.dist(a, b) < 1e-4: continue
        out.append(f'\t(segment\n\t\t(start {a[0]:g} {a[1]:g})\n\t\t(end {b[0]:g} {b[1]:g})\n\t\t(width {w:g})\n'
                   f'\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
    return out


def via_text(net, x, y):
    return (f'\t(via\n\t\t(at {x:g} {y:g})\n\t\t(size 0.45)\n\t\t(drill 0.2)\n\t\t(layers "F.Cu" "B.Cu")\n'
            f'\t\t(capping yes)\n\t\t(filling yes)\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')


def append(t, items):
    end = t.rstrip().rfind(")")
    return t[:end].rstrip("\n") + "\n" + "\n".join(items) + "\n)\n"


def delete_uuids(t, kill):
    def k(m):
        u = re.search(r'\(uuid "([^"]+)"\)', m.group(0))
        return "" if u and u.group(1) in kill else m.group(0)
    return re.sub(r"\n\t\((segment|via|arc)\n.*?\n\t\)", k, t, flags=re.S)
