"""stubsim.py board --pair /LINK_RX_D0 [--rate 1.25e9] [--stub 0,3,6] [--eth MM] [--term receiver|source]
(python with shapely + scipy; ngspice as lvds_eye.py finds it: on PATH, or LVDS_EYE_WSL=1)

LVDS pairs that carry an assembly-option branch (a DNP capacitor to a second device): each leg of
the pair as routed (lvds_eye.py's model: sections by layer and width, field-solved Z0 and delay,
LVDS driver 3.5 mA / 300 ps / 10 k to 1.2 V, receiver 100 ohm across and 2 pF a leg, PRBS7) with,
at the receiver end, an open branch of --stub mm of 0.1 mm L1 track ending on an unfitted 0402
pad (0.1 pF): what the IAS1 link sees with the Ethernet capacitors not fitted.
--eth MM instead models the Ethernet mode: the same leg, then the branch (MM of 0.1 mm L1 to the
Ethernet capacitor, taken as a short), then 45 mm of 100 ohm coupled pair on L3 (50 ohm a leg) to the
PHY's input (100 ohm, 1.5 pF a leg); the IAS1 capacitor removed leaves the J101 side open.
Prints the eye opening and the zero-crossing spread for each case."""
import os, sys, math, argparse, subprocess, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
argv = sys.argv[1:]
ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("--pair", default="/LINK_RX_D0"); ap.add_argument("--rate", type=float, default=1.25e9)
ap.add_argument("--stub", default="0,3,6"); ap.add_argument("--eth", type=float, default=None)
ap.add_argument("--term", choices=("receiver", "source"), default="receiver")
ap.add_argument("--at", type=float, default=None, help="IAS1 mode: the branch leaves the leg this many mm from the driver "
                "(the sections are in lvds_eye's order, so the position is approximate); default: at the receiver")
a = opt = ap.parse_args(argv)
# reuse lvds_eye.py's board reading, line models and sections (everything above its PRBS)
src = open(os.path.join(HERE, "lvds_eye.py"), encoding="utf-8").read()
sys.argv = [sys.argv[0], a.board, "--pair", a.pair, "--rate", str(a.rate), "--save", os.path.join(HERE, "out")]
exec(src[:src.index("UI = 1 / a.rate")])
import numpy as np
UI = 1 / opt.rate; TR = 300e-12
prbs = []; r = 0x7F
for _ in range(127 * 2):
    b = ((r >> 6) ^ (r >> 5)) & 1; r = ((r << 1) | b) & 0x7F; prbs.append(b)
zs, ds = line("F.Cu", 0.1)              # the branch: 0.1 mm on L1
zc, dc = line("In2.Cu", 0.1)            # the Ethernet run: a coupled leg on L3 (odd mode)


def run(tag, secP, secN, stub_mm, eth_mm=None):
    L = [f"* {tag}"]
    pwl = []; prev = prbs[0]; pwl.append(f"0 {3.5e-3 if prev else -3.5e-3}")
    for k, b in enumerate(prbs[1:], 1):
        if b != prev:
            t0 = k * UI; pwl.append(f"{t0:.4e} {3.5e-3 if prev else -3.5e-3}"); pwl.append(f"{t0 + TR:.4e} {3.5e-3 if b else -3.5e-3}"); prev = b
    L.append("Idp 0 sp PWL(" + " ".join(pwl) + ")")
    L.append("Idn 0 sn PWL(" + " ".join(f"{p.split()[0]} {-float(p.split()[1]):.4e}" for p in pwl) + ")")
    L.append("Rcp sp cm 10k\nRcn sn cm 10k\nVcm cm 0 1.2")
    for leg, secs, s in (("p", secP, "sp"), ("n", secN, "sn")):
        node = s; run_mm = 0; tapped = opt.at is None
        for k, (Ly, w, mm, z, d) in enumerate(secs):
            if not tapped and eth_mm is None and run_mm + mm > opt.at:     # split this section at the tap
                m1 = opt.at - run_mm
                L.append(f"T{leg}{k}a {node} 0 {leg}{k}t 0 Z0={z:.3f} TD={max(m1, 0.01) * d:.4e}"); node = f"{leg}{k}t"
                if stub_mm > 0:
                    L.append(f"Ts{leg} {node} 0 b{leg} 0 Z0={zs:.3f} TD={stub_mm * ds:.4e}"); L.append(f"Cs{leg} b{leg} 0 0.1p")
                else:
                    L.append(f"Cs{leg} {node} 0 0.1p")
                mm -= m1; tapped = True
            L.append(f"T{leg}{k} {node} 0 {leg}{k} 0 Z0={z:.3f} TD={max(mm, 0.01) * d:.4e}"); node = f"{leg}{k}"; run_mm += mm
        if eth_mm is None:
            if stub_mm > 0 and opt.at is None:      # the open branch to the unfitted Ethernet capacitor
                L.append(f"Ts{leg} {node} 0 b{leg} 0 Z0={zs:.3f} TD={stub_mm * ds:.4e}"); L.append(f"Cs{leg} b{leg} 0 0.1p")
            L.append(f"Cin{leg} {node} 0 2p"); L.append(f"V{leg} {node} r{leg} 0")
        else:                    # Ethernet mode: branch, then 45 mm of coupled leg to the PHY
            L.append(f"Ts{leg} {node} 0 b{leg} 0 Z0={zs:.3f} TD={max(eth_mm, 0.2) * ds:.4e}")
            L.append(f"Te{leg} b{leg} 0 e{leg} 0 Z0={zc:.3f} TD={45 * dc:.4e}")
            L.append(f"Cin{leg} e{leg} 0 1.5p"); L.append(f"V{leg} e{leg} r{leg} 0")
    L.append("Rterm rp rn 100")
    T = len(prbs) * UI; name = f"stub_{tag}_{os.getpid()}.txt"
    wsl = os.environ.get("LVDS_EYE_WSL") == "1"; spice = os.environ.get("LVDS_EYE_NGSPICE", "ngspice")
    out = "/tmp/" + name if wsl else os.path.join(tempfile.gettempdir(), name)
    L += [f".tran 2p {T:.4e}", ".control", "run", f"wrdata {out} v(rp)-v(rn)", ".endc", ".end"]
    net = os.path.join(HERE, f"_stub_{tag}_{os.getpid()}.cir"); open(net, "w", newline="\n").write("\n".join(L) + "\n")
    if wsl:
        w = "/mnt/" + net[0].lower() + net[2:].replace("\\", "/")
        txt = subprocess.run(["wsl", "-e", "bash", "-lc", f"timeout 900 {spice} -b {w} >/dev/null 2>&1; cat {out}; rm -f {out}"],
                             capture_output=True, text=True).stdout
    else:
        subprocess.run(spice.split() + ["-b", net], capture_output=True, text=True, timeout=900)
        txt = open(out).read(); os.remove(out)
    os.remove(net)
    d = np.array([[float(x) for x in l.split()[:2]] for l in txt.splitlines() if l.strip()])
    t, v = d[:, 0], d[:, 1]
    delay = sum(mm * dd for *_, mm, z, dd in secP)
    keep = t > 127 * UI; t, v = t[keep], v[keep]
    ph = ((t - delay) % UI) / UI
    best = (0, 0)
    for c0 in np.arange(0.3, 0.71, 0.02):          # the eye's best sampling point, not a fixed centre
        c = (ph > c0 - 0.05) & (ph < c0 + 0.05)
        hi = v[c & (v > 0)]; lo = v[c & (v < 0)]
        eh = (hi.min() - lo.max()) * 1e3 if len(hi) and len(lo) else 0
        if eh > best[0]: best = (eh, c0)
    zc_ = []
    for k in np.where(np.sign(v[:-1]) != np.sign(v[1:]))[0]:
        tz = t[k] - v[k] * (t[k + 1] - t[k]) / (v[k + 1] - v[k]); zc_.append(((tz - delay) % UI) / UI)
    zc_ = np.array(zc_); zc_ = np.where(zc_ > 0.5, zc_ - 1, zc_)
    jit = (zc_.max() - zc_.min()) * UI * 1e12 if len(zc_) else 0
    return best[0], jit


P, N = sections("P", "routed"), sections("N", "routed")
if opt.eth is None:
    for s_ in [float(x) for x in opt.stub.split(",")]:
        eh, j = run(f"s{s_:g}", P, N, s_)
        print(f"{opt.pair} {opt.rate/1e9:g} Gb/s, IAS1 mode, branch {s_:g} mm: eye {eh:4.0f} mV, zero-crossing spread {j:4.0f} ps")
else:
    eh, j = run("eth", P, N, 0, opt.eth)
    print(f"{opt.pair} {opt.rate/1e9:g} Gb/s, Ethernet mode (branch {opt.eth:g} mm + 45 mm to the PHY): eye {eh:4.0f} mV, zero-crossing spread {j:4.0f} ps")
