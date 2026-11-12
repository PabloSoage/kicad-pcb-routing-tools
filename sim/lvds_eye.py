"""lvds_eye.py board [--pair /LINK_RX_D1] [--rate 1e9]   (python with numpy, scipy, shapely;
ngspice on PATH, or in WSL with LVDS_EYE_WSL=1)
Eye of one LVDS pair as routed, against the original 0.117 mm legs and an ideal 100-ohm pair:
  * each leg's copper by (layer, width) from the board; Z0 and eps_eff of each from
    sim/field_solver.py (L1/L6 microstrip over L2/L5, L3 and L4 striplines);
  * a leg = its sections in series as lossless lines (the non-50-ohm sections put together at
    the receiver end, and again at the driver end: the larger eye closure is reported);
  * the legs are taken as uncoupled (89 % of their length runs > 1 mm apart);
  * driver: LVDS current steering, 3.5 mA, 300 ps edges, 10 kohm to the 1.2 V common mode;
    receiver: 100 ohm across, 2 pF per input;
  * PRBS7 at --rate; eye height at the UI centre and the zero-crossing spread (jitter).
Appends to out/lvds_eye.csv (pair, rate, case, eye opening mV, jitter ps, P-N delay ps) and
writes each case's folded eye to out/eye_<pair>_<rate>_<case>.csv."""
import os, sys, re, math, argparse, subprocess, collections, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "layout"))
import numpy as np
import field_solver as fs
from geo import Board

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("--pair", default="/LINK_RX_D1"); ap.add_argument("--rate", type=float, default=1e9)
ap.add_argument("--term", choices=("receiver", "source"), default="receiver",
                help="where the 100 ohm is: 'source' = 2 x 49.9 ohm to the common mode at the driver end, the receiver open "
                     "(LINK_TX on the 6-layer board, R101-R104 next to the connector)")
ap.add_argument("--save", help="write the eye CSVs to this folder instead of out/, and do not append to out/lvds_eye.csv")
a = ap.parse_args()
OUTD = a.save or os.path.join(HERE, "out")


def stripline_l4(w, s=None):
    def build(odd):
        y_t = fs.CORE_THIN
        y_l2 = y_t + fs.PP_2116 + fs.CORE_MID + fs.PP_2116 + fs.CORE_THIN
        g = fs.Grid(fs.BOX_W, y_l2)
        g.layer(0, y_t, fs.ER_CORE_THIN); g.layer(y_t, y_t + fs.PP_2116, fs.ER_2116)
        g.layer(y_t + fs.PP_2116, y_t + fs.PP_2116 + fs.CORE_MID, fs.ER_CORE_MID)
        g.layer(y_t + fs.PP_2116 + fs.CORE_MID, y_l2 - fs.CORE_THIN, fs.ER_2116)
        g.layer(y_l2 - fs.CORE_THIN, y_l2, fs.ER_CORE_THIN)
        g.conductor((fs.BOX_W - w) / 2, (fs.BOX_W + w) / 2, y_t, y_t + fs.T_IN, 1.0)
        return g
    return build


GEOM = {"F.Cu": fs.microstrip, "B.Cu": fs.microstrip, "In2.Cu": fs.stripline_l3, "In3.Cu": stripline_l4}
_cache = {}
EIGHT = False          # set below from the board: JLC08161H-2116C, pairs routed coupled
COUPLED = {0.1: 0.15, 0.09: 0.12}       # 8 layers, L3/L6: leg width -> gap of a coupled pair


def line(layer, w):
    """(Z0 ohm, delay s/mm) seen by one leg. On the 8-layer board a coupled leg on L3/L6 (In2/In5)
    is taken in the odd mode (what a differential signal sees): Z_odd = Z_diff / 2."""
    k = (layer, round(w, 3))
    if k not in _cache:
        if EIGHT and layer in ("In2.Cu", "In5.Cu"):
            import field_solver8 as f8
            s = COUPLED.get(round(w, 3))
            g = f8.stripline_in(w, s)(odd=bool(s))
            c = g.energy_c(); c0 = g.energy_c(np.ones_like(g.eps))
            if s: c, c0 = c / 2, c0 / 2
        elif EIGHT:                         # 8 layers, outer: 2116C prepreg under L1/L8
            import field_solver8 as f8
            fs.PP_2116, fs.ER_2116 = f8.PP_OUT, f8.ER_OUT
            g = fs.microstrip(w)(odd=False)
            c = g.energy_c(); c0 = g.energy_c(np.ones_like(g.eps))
        else:
            g = GEOM[layer](w)(odd=False)
            c = g.energy_c(); c0 = g.energy_c(np.ones_like(g.eps))
        _cache[k] = (1 / (fs.C_LIGHT * math.sqrt(c * c0)), math.sqrt(c / c0) / fs.C_LIGHT * 1e-3)
    return _cache[k]


B = Board(a.board)
EIGHT = '"In6.Cu"' in B.t[:3000]
comp = {leg: collections.Counter() for leg in ("P", "N")}
for it in B.items:
    if it.kind == "seg" and it.net in (a.pair + "_P", a.pair + "_N"):
        comp[it.net[-1]][(it.layer, round(it.data[2], 3))] += math.dist(it.data[0], it.data[1])


def sections(leg, mode):
    s = [(L, w, mm) + line(L, w) for (L, w), mm in comp[leg].items()]
    if mode == "orig": s = [(L, 0.117, mm) + line(L, 0.117) for L, w, mm, *_ in s]
    if mode == "ideal": return [("ideal", 0, sum(x[2] for x in s), 50.0, sum(x[2] * x[4] for x in s) / sum(x[2] for x in s))]
    # sections under 3 mm are lumped into one of their length-weighted Z0 and delay (ngspice's step
    # would otherwise follow the shortest line's delay down to femtoseconds)
    big = [x for x in s if x[2] >= 3]; small = [x for x in s if x[2] < 3]
    if small:
        mm = sum(x[2] for x in small)
        lump = ("lumped", 0, mm, sum(x[2] * x[3] for x in small) / mm, sum(x[2] * x[4] for x in small) / mm)
        if mm < 1.0 and big:     # a lump under 1 mm joins the longest section (its delay would set ngspice's step)
            k = max(range(len(big)), key=lambda i: big[i][2]); L0, w0, m0, z0, d0 = big[k]
            big[k] = (L0, w0, m0 + mm, (m0 * z0 + mm * lump[3]) / (m0 + mm), (m0 * d0 + mm * lump[4]) / (m0 + mm))
        else:
            big.append(lump)
    good = [x for x in big if abs(x[3] - 50) < 2.5]; bad = [x for x in big if abs(x[3] - 50) >= 2.5]
    good.sort(key=lambda x: (x[3], x[2])); bad.sort(key=lambda x: (x[3], x[2]))   # by Z0: not file order (KiCad reorders on save)
    return good + bad          # mismatched sections at the receiver end (reversed for the other run)


UI = 1 / a.rate; TR = 300e-12
prbs = []; r = 0x7F
for _ in range(127 * 2):
    b = ((r >> 6) ^ (r >> 5)) & 1; r = ((r << 1) | b) & 0x7F; prbs.append(b)


def run(tag, secP, secN):
    lines = [f"* {tag}", ".param ui=" + repr(UI)]
    pwl = []
    prev = prbs[0]
    pwl.append(f"0 {3.5e-3 if prev else -3.5e-3}")
    for k, b in enumerate(prbs[1:], 1):
        if b != prev:
            t0 = k * UI
            pwl.append(f"{t0:.4e} {3.5e-3 if prev else -3.5e-3}"); pwl.append(f"{t0 + TR:.4e} {3.5e-3 if b else -3.5e-3}")
            prev = b
    lines.append("Idp 0 sp PWL(" + " ".join(pwl) + ")")
    neg = [f"{p.split()[0]} {-float(p.split()[1]):.4e}" for p in pwl]
    lines.append("Idn 0 sn PWL(" + " ".join(neg) + ")")      # N leg: the opposite current
    lines.append("Rcp sp cm 10k\nRcn sn cm 10k\nVcm cm 0 1.2")
    for leg, secs, src in (("p", secP, "sp"), ("n", secN, "sn")):
        node = src
        for k, (L, w, mm, z, d) in enumerate(secs):
            nxt = f"{leg}{k}"
            lines.append(f"T{leg}{k} {node} 0 {nxt} 0 Z0={z:.3f} TD={mm * d:.4e}")
            node = nxt
        lines.append(f"Cin{leg} {node} 0 2p")
        lines.append(f"Vprobe{leg} {node} r{leg} 0")
    if a.term == "receiver":
        lines.append("Rterm rp rn 100")
    else:
        lines += ["Rsp sp ct 49.9", "Rsn sn ct 49.9", "Vct ct 0 1.2", "Ropen rp rn 1meg"]
    T = len(prbs) * UI
    name = f"lvds_{tag}_{os.getpid()}.txt"          # per process: two runs may go at once
    out = "/tmp/" + name if os.environ.get("LVDS_EYE_WSL") == "1" else os.path.join(tempfile.gettempdir(), name)
    lines += [f".tran 2p {T:.4e}", ".control", "run", f"wrdata {out} v(rp)-v(rn)", ".endc", ".end"]
    net = os.path.join(HERE, f"_lvds_{tag}_{os.getpid()}.cir")
    open(net, "w", newline="\n").write("\n".join(lines) + "\n")
    # ngspice: on PATH by default; LVDS_EYE_WSL=1 runs it inside WSL instead (Windows host),
    # through $LVDS_EYE_NGSPICE there (default "ngspice", e.g. a wrapper that activates an env)
    wsl = os.environ.get("LVDS_EYE_WSL") == "1"
    spice = os.environ.get("LVDS_EYE_NGSPICE", "ngspice")
    try:
        if wsl:
            wsl_net = "/mnt/" + net[0].lower() + net[2:].replace("\\", "/")
            subprocess.run(["wsl", "-e", "bash", "-lc", f"timeout 900 {spice} -b {wsl_net} >/dev/null 2>&1; cat {out}"],
                           capture_output=True, text=True, check=True)
        else:
            subprocess.run(spice.split() + ["-b", net], capture_output=True, text=True, check=True, timeout=900)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        print(f"{a.pair} {tag}: ngspice did not finish (15 min)"); os.remove(net); return 0, 0, 0
    if wsl:
        txt = subprocess.run(["wsl", "-e", "bash", "-lc", f"cat {out}"], capture_output=True, text=True).stdout
    else:
        txt = open(out).read(); os.remove(out)
    d = np.array([[float(x) for x in l.split()[:2]] for l in txt.splitlines() if l.strip()])
    os.remove(net)
    t, v = d[:, 0], d[:, 1]
    delay = sum(mm * dd for *_, mm, z, dd in secP)
    keep = t > 127 * UI           # second PRBS period: settled
    t, v = t[keep], v[keep]
    ph = ((t - delay) % UI) / UI
    np.savetxt(os.path.join(OUTD, f"eye_{a.pair.strip('/').replace('/', '_')}_{a.rate/1e9:g}G_{tag}.csv"),
               np.c_[ph, v * 1e3], delimiter=",", header="phase_UI,vdiff_mV", comments="", fmt="%.4f")
    # eye height at the centre +-5 % of the UI
    c = (ph > 0.45) & (ph < 0.55)
    hi = v[c & (v > 0)]; lo = v[c & (v < 0)]
    eh = (hi.min() - lo.max()) * 1e3 if len(hi) and len(lo) else 0
    # zero crossings
    zc = []
    for k in np.where(np.sign(v[:-1]) != np.sign(v[1:]))[0]:
        tz = t[k] - v[k] * (t[k + 1] - t[k]) / (v[k + 1] - v[k]); zc.append(((tz - delay) % UI) / UI)
    zc = np.array(zc); zc = np.where(zc > 0.5, zc - 1, zc)
    jit = (zc.max() - zc.min()) * UI * 1e12 if len(zc) else 0
    swing = np.percentile(np.abs(v), 95) * 1e3
    return eh, jit, swing


rows = []
for mode in ("ideal", "orig", "routed"):
    P, N = sections("P", mode), sections("N", mode)
    dP = sum(x[2] * x[4] for x in P); dN = sum(x[2] * x[4] for x in N)
    runs = [run(mode, P, N)] + ([run(mode + "_rev", P[::-1], N[::-1])] if mode != "ideal" else [])
    eh, jit, sw = min(runs, key=lambda r: r[0])
    rows.append((mode, eh, jit, (dP - dN) * 1e12, sw))
    zs = sorted({round(x[3], 1) for x in P + N})
    print(f"{a.pair} {mode:6s}: eye opening {eh:5.0f} mV (peak +-{sw:3.0f} mV), jitter {jit:4.0f} ps, P-N {1e12*(dP-dN):+5.1f} ps; leg Z0 {zs}")
csv_ = os.path.join(OUTD, "lvds_eye.csv"); new_ = not os.path.exists(csv_)
with open(csv_, "a") as f:
    if new_: f.write("pair,rate_bps,case,eye_opening_mV,zero_crossing_spread_ps,P_minus_N_ps\n")
    for r_ in rows: f.write(f"{a.pair},{a.rate:.3g},{r_[0]},{r_[1]:.0f},{r_[2]:.0f},{r_[3]:.1f}\n")
