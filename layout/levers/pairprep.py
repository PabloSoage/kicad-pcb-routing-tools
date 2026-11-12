"""pairprep.py in out
Prepares the coupled re-route of the FPGA's LVDS pairs on the 8-layer board:
  1. rips every track and via of the pairs' nets (ADCR_*, ADCS_*, LINK_RX_*, LINK_TX_* legs);
  2. puts a via in pad (0.45/0.2, filled and capped) at each of their FPGA balls, deleting the
     segments and vias of OTHER nets in the way (slow nets; they are re-routed afterwards);
  3. moves the 100 ohm terminations R1101-R1114 to the bottom face, each straddling its pair's
     two ball vias (0402 pads 1.02 mm apart on balls 1.0 mm apart): the termination sits at the
     receiver's balls, no stub. LINK_TX's 100 ohm (R105, R106) likewise; their 10k bias resistors to +1V2 (R101-R104) stay.
Prints the nets whose copper was cut, to re-route. Text edit, no pcbnew."""
import os, sys, re, math, uuid
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board, CU
from shapely.geometry import Point
import pairlib as PL

inp, out = sys.argv[1:3]
PAIRS = re.compile(r"^/(ADCR_(D\d|DCO|OR)|ADCS_(D\d|DCO|FCO)|LINK_RX_(D\d|CLK)|LINK_TX_(D\d|CLK))_[PN]$")
TERM = {"R1101": "ADCR_D0", "R1102": "ADCR_D1", "R1103": "ADCR_D2", "R1104": "ADCR_D3", "R1105": "ADCR_D4",
        "R1106": "ADCR_D5", "R1107": "ADCR_D6", "R1108": "ADCR_D7", "R1109": "ADCR_DCO", "R1110": "ADCR_OR",
        "R1111": "ADCS_D0", "R1112": "ADCS_D1", "R1113": "ADCS_DCO", "R1114": "ADCS_FCO", "R105": "LINK_TX_D0", "R106": "LINK_TX_CLK"}

t = open(inp, encoding="utf-8").read()


# 1. rip
def rip(m):
    n = re.search(r'\(net "([^"]*)"\)', m.group(0))
    return "" if n and PAIRS.match(n.group(1)) else m.group(0)


t = re.sub(r"\n\t\((segment|via|arc)\n.*?\n\t\)", rip, t, flags=re.S)
open(out, "w", encoding="utf-8", newline="\n").write(t)

# 2. ball vias
B = Board(out)
balls = {}
for it in B.by_layer["F.Cu"]:
    if it.kind == "pad" and it.data[0] == "U1101" and PAIRS.match(it.net):
        balls[it.net] = (round(it.data[2], 4), round(it.data[3], 4))
kill = set(); cut = set(); new = []
for net, (x, y) in sorted(balls.items()):
    v = Point(x, y).buffer(0.225, 16)
    for l in CU:
        if l in ("In1.Cu", "In4.Cu", "In6.Cu"): continue
        for k in B.tree[l].query(v.buffer(0.4)):
            it = B.by_layer[l][k]
            if it.net == net or it.kind == "pad" or it.uuid in kill: continue
            if it.geom.distance(v) < B.clr(net, it.net) + 0.005:
                kill.add(it.uuid); cut.add(it.net)
    for p, r, n in B.holes:
        if n != net and p.distance(Point(x, y)) < r + 0.1 + 0.25:
            for it in B.items:
                if it.kind == "via" and it.net == n and math.dist(it.data[:2], (p.x, p.y)) < 0.01:
                    kill.add(it.uuid); cut.add(n)
    new.append(PL.via_text(net, x, y))
t = PL.delete_uuids(B.t, kill)
t = PL.append(t, new)
print("ball vias:", len(new), "; items cut:", len(kill), "; nets cut:", " ".join(sorted(cut)))

# 3. terminations to the bottom, on the ball vias (rotation chosen so pad 1 lands on P)
import subprocess
EDIT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "edit.py")
open(out, "w", encoding="utf-8", newline="\n").write(t)


def padpos(ref):
    B = Board(out)
    return {it.data[1]: (it.data[2], it.data[3]) for it in B.items if it.kind == "pad" and it.data[0] == ref and it.layer in ("F.Cu", "B.Cu")}


for ref, pair in TERM.items():
    (px, py), (nx, ny) = balls[f"/{pair}_P"], balls[f"/{pair}_N"]
    rot0 = 0 if abs(py - ny) < 1e-6 else 90
    for rot in (rot0, rot0 + 180):
        subprocess.run([sys.executable, EDIT, out, out, f"place:{ref}:{(px + nx) / 2:g}:{(py + ny) / 2:g}:{rot}:B"], check=True)
        pp = padpos(ref)
        d1 = math.dist(pp["1"], (px, py)); d2 = math.dist(pp["2"], (nx, ny))
        if d1 < 0.05 and d2 < 0.05: break
    print(f"{ref} {pair}: rot {rot}, pad1-P {d1:.3f} mm, pad2-N {d2:.3f} mm")

# 4. what the bottom-side terminations now touch: other nets' B.Cu tracks and vias. Slow nets are
#    cut (re-routed later); a supply via is a ball's dogbone: it is moved into its ball's pad when
#    that is free, else reported.
B = Board(out)
rpads = [it for it in B.by_layer["B.Cu"] if it.kind == "pad" and it.data[0] in TERM]
kill = set(); cut = set(); new = []; moved = []
sup = lambda n: n.startswith("+") or n == "GND"
upads = {(round(it.data[2], 3), round(it.data[3], 3)): it.net for it in B.by_layer["F.Cu"] if it.kind == "pad" and it.data[0] == "U1101"}
for pd in rpads:
    for k in B.tree["B.Cu"].query(pd.geom.buffer(0.4)):
        it = B.by_layer["B.Cu"][k]
        if it.net == pd.net or it.uuid in kill or it.kind == "pad": continue
        if it.geom.distance(pd.geom) >= B.clr(pd.net, it.net) + 0.005: continue
        if it.kind == "seg" and not sup(it.net):
            kill.add(it.uuid); cut.add(it.net)
        elif it.kind == "via":
            x, y = it.data[:2]
            if not sup(it.net):
                kill.add(it.uuid); cut.add(it.net); continue
            # dogbone of a supply ball: the ball is the nearest U1101 pad of that net
            ball = min(((math.dist((x, y), q), q) for q, n in upads.items() if n == it.net), default=None)
            if ball and ball[0] < 0.75:
                bx, by = ball[1]
                kill.add(it.uuid)
                for s in B.items:   # its dogbone track
                    if s.kind == "seg" and s.net == it.net and s.layer == "F.Cu":
                        a, e, w = s.data
                        if {(round(a[0], 3), round(a[1], 3)), (round(e[0], 3), round(e[1], 3))} >= set() and \
                           min(math.dist(a, (x, y)), math.dist(e, (x, y))) < 0.02 and min(math.dist(a, (bx, by)), math.dist(e, (bx, by))) < 0.02:
                            kill.add(s.uuid)
                has = any(v.kind == "via" and v.layer == "F.Cu" and v.net == it.net and math.dist(v.data[:2], (bx, by)) < 0.01 for v in B.items)
                if not has and f"{it.net}@({bx},{by})" not in moved:
                    new.append(PL.via_text(it.net, bx, by)); moved.append(f"{it.net}@({bx},{by})")
            else:
                print("supply via in the way, not a dogbone:", it.net, x, y)
        elif it.kind == "seg":
            a, e, w = it.data       # a supply stub off a via that ends in nothing: drop it
            ends = [q for q in (a, e) if not any(v.kind == "via" and v.layer == "F.Cu" and math.dist(v.data[:2], q) < 0.02 for v in B.items)]
            if math.dist(a, e) < 0.5: kill.add(it.uuid); print("supply stub removed:", it.net, it.data)
            else: print("supply track in the way:", it.net, it.data)
t = PL.delete_uuids(open(out, encoding="utf-8").read(), kill)
t = PL.append(t, new) if new else t
open(out, "w", encoding="utf-8", newline="\n").write(t)
print("under the terminations: items cut", len(kill), "nets cut:", " ".join(sorted(cut)), "; supply dogbones moved into the ball:", " ".join(moved))
