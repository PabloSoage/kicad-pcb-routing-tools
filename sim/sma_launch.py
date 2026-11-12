"""sma_launch.py CASE [--fmax 6e9]  (openEMS 0.37, run with openEMS and its python bindings installed)
Edge-SMA signal pad on the 2116A stack, as a through line: 183 um microstrip on L1 over L2,
the SMA's 1.27 x 3.2 mm signal pad, then the line again (standing in for the connector's
50-ohm coax). CASE:
  plain    the pad over L2 (0.116 mm below it): the board before 11 Nov
  cutout   L2, L3 and L4 cleared under the pad (+0.3 mm), so it references L5 (1.355 mm)
  line     no pad, the 183 um line straight through: the set-up's own check (S11 should be tiny)
  tvs      cutout plus an ESD diode on the line 2.15 mm before the pad (its pad and 0.5 pF)
  tvsnNN   tvs with the line narrowed to 0.1 mm for NN/10 mm on each side of the diode's pad
  tvsnAA_BB  the same, AA/10 mm on the source side and BB/10 mm on the SMA side
           (series inductance against the diode's shunt capacitance), e.g. tvsn10
Both with the board's L1 GND pour (0.5 mm clearance), the via fence (0.8 mm off the line,
1 mm pitch) and three stitching vias per SMA ground leg. L4 is a PEC plane not tied to GND.
FR-4 loss tan d = 0.02 (as conductivity at 3 GHz). Stops at -40 dB energy or 400k steps. Excitation: the derivative of a Gaussian (no DC).
STACK=8 in the environment: the same on JLC08161H-2116C (L1 | 0.1194 er 4.29 | L2 GND | 0.13 | L3 |
0.3658 | L4 PWR | 0.13 | L5 GND | 0.3658 | L6 | 0.13 | L7 GND | ...): the cut-out clears L2, L4
and L5, so the pad references L7; writes sma8_<CASE>.csv.
Writes sma_<CASE>.csv: f, |S11| dB, |S21| dB, and prints |S11| at 0.5, 1, 2, 3 GHz."""
import os, sys, numpy as np
from CSXCAD import ContinuousStructure
from openEMS import openEMS
from openEMS.physical_constants import C0

case = sys.argv[1]; fmax = 6e9
here = os.path.dirname(os.path.abspath(__file__))
sim = os.path.join("/tmp", ("sma8_" if os.environ.get("STACK") == "8" else "sma_") + case)
# stack, um, L5 at z = 0 (STACK=8: L7 at z = 0)
EIGHT = os.environ.get("STACK") == "8"
zL5, zL4, zL3, zL2, zL1 = 0.0, 145.0, 1093.0, 1238.0, 1355.0
if EIGHT:   # copper 15.2 um between the dielectrics; zB is the reference plane (L7), zM the extra GND plane (L5)
    zB, z6, zM, zL4, zL3, zL2, zL1 = 0.0, 145.2, 526.2, 671.4, 1052.4, 1197.6, 1317.0
    zL5 = zB
W, PW, PL = 183.0, 1270.0, 3200.0
X0, XP0 = 0.0, 10000.0; XP1 = XP0 + PL; X1 = XP1 + 10000.0
CLR, YB = 500.0, 5000.0

FDTD = openEMS(EndCriteria=1e-4, NrTS=400000)     # -40 dB energy, or 400k steps
TAU = 1 / (np.pi * 2e9 * np.sqrt(2)); T0 = 5 * TAU   # derivative of a Gaussian, peak 2 GHz: exactly no
FDTD.SetCustomExcite(f"-(t-{T0})/{TAU}*exp(-((t-{T0})/{TAU})^2)", 2e9, fmax)   # DC (a DC part stays circling in the PEC via/plane loops)
FDTD.SetBoundaryCond(['PML_8', 'PML_8', 'PML_8', 'PML_8', 'PEC', 'PML_8'])   # MUR went unstable with the layered dielectric
CSX = ContinuousStructure(); FDTD.SetCSX(CSX)
mesh = CSX.GetGrid(); mesh.SetDeltaUnit(1e-6)

EPS0, TAND, F0 = 8.854e-12, 0.02, 3e9        # FR-4 loss, fitted at 3 GHz: without it the
def kappa(er): return 2 * np.pi * F0 * EPS0 * er * TAND   # L2/L4/L5 plane cavity rings for ever
def diel(name, er, z0, z1):
    CSX.AddMaterial(name, epsilon=er, kappa=kappa(er)).AddBox([X0, -YB, z0], [X1, YB, z1], priority=1)
if EIGHT:
    for k, (z0, z1, er) in enumerate(((zB, z6, 4.17), (z6, zM, 4.29), (zM, zL4, 4.17), (zL4, zL3, 4.29), (zL3, zL2, 4.17), (zL2, zL1, 4.29))):
        diel(f"d8_{k}", er, z0, z1)
else:
    diel("core_thin_b", 4.17, zL5, zL4)
    diel("pp_b", 4.16, zL4, zL4 + 116)
    diel("core_mid", 4.53, zL4 + 116, zL3 - 132)
    diel("pp_c", 4.16, zL3 - 132, zL3)
    diel("core_thin_t", 4.17, zL3, zL2)
    diel("pp_t", 4.16, zL2, zL1)
gnd = CSX.AddMetal("gnd"); l4 = CSX.AddMetal("l4"); sig = CSX.AddMetal("sig")
gnd.AddBox([X0, -YB, zL5], [X1, YB, zL5], priority=10)              # L5
gnd.AddBox([X0, -YB, zL2], [X1, YB, zL2], priority=10)              # L2
l4.AddBox([X0, -YB, zL4], [X1, YB, zL4], priority=10)               # L4 (+3V3_D)
if EIGHT: gnd.AddBox([X0, -YB, zM], [X1, YB, zM], priority=10)      # L5 (GND) of the 8-layer stack
cut = [XP0 - 300, XP1 + 500]
if case in ("cutout", "tvs") or case.startswith("tvsn"):
    for z, er in ((zL2, 4.17), (zL4, 4.17)) + (((zM, 4.17),) if EIGHT else ()):
        CSX.AddMaterial(f"hole{z}", epsilon=er, kappa=kappa(er)).AddBox([cut[0], -1050, z], [cut[1], 1050, z], priority=20)
# L1 pour, clear of the line and the pad by 0.5 mm
for sgn in (1, -1):
    gnd.AddBox([X0, sgn * (W / 2 + CLR), zL1], [XP0 - CLR, sgn * YB, zL1], priority=10)
    gnd.AddBox([XP0 - CLR, sgn * (PW / 2 + CLR), zL1], [XP1 + CLR, sgn * YB, zL1], priority=10)
    gnd.AddBox([XP1 + CLR, sgn * (W / 2 + CLR), zL1], [X1, sgn * YB, zL1], priority=10)
# vias: fence along the line, stitching beside the pad (through L4 in a 0.35 mm antipad)
vias = [(x, s * 800.0) for x in np.arange(5000, XP0 - 600, 1000) for s in (1, -1)]
vias += [(x, s * 800.0) for x in np.arange(XP1 + 1000, X1 - 4000, 1000) for s in (1, -1)]
vias += [(XP0 + d, s * 2825.0) for d in (200, 1200, 2200) for s in (1, -1)]
for k, (x, y) in enumerate(vias):
    CSX.AddMaterial(f"ap{k}", epsilon=4.17, kappa=kappa(4.17)).AddBox([x - 350, y - 350, zL4], [x + 350, y + 350, zL4], priority=25)
    gnd.AddCylinder([x, y, zL5], [x, y, zL1], 100, priority=30)
# tvs: an ESD diode (PESD5V0F1BL, DFN1006-2) on the line 2.15 mm before the SMA pad (as placed): its pad on the
# line (0.5 x 0.5 mm) and its capacitance, 0.5 pF (datasheet maximum), from the line to L2
XT = XP0 - 2150.0
# um of 0.1 mm line on the source side (LA) and the SMA side (LB): tvsnNN or tvsnAA_BB
LA, LB = ([float(v) * 100.0 for v in case[4:].split("_")] * 2)[:2] if case.startswith("tvsn") else (0.0, 0.0)
LN = LA + LB
if case == "tvs" or LN:
    sig.AddBox([XT - 250, -250, zL1], [XT + 250, 250, zL1], priority=16)
    CSX.AddLumpedElement("esd", ny="z", caps=True, C=0.5e-12).AddBox([XT - 50, -50, zL2], [XT + 50, 50, zL1], priority=40)
if LN:
    air = CSX.AddMaterial("narrow", epsilon=1.0)
    for xa, xb in ((XT - 250 - LA, XT - 250), (XT + 250, XT + 250 + LB)):
        for sgn in (1, -1):
            air.AddBox([xa, sgn * 50, zL1], [xb, sgn * W / 2, zL1], priority=17)
# the pad
sig.AddBox([XP0, -(W if case == 'line' else PW) / 2, zL1], [XP1, (W if case == 'line' else PW) / 2, zL1], priority=15)
# mesh
res = C0 / (fmax * np.sqrt(4.3)) / 1e-6 / 40
xl = [X0, X1, XP0, XP1, cut[0], cut[1], XP0 - CLR, XP1 + CLR, XT - 250, XT - 50, XT + 50, XT + 250] + ([XT - 250 - LA, XT + 250 + LB] if LN else []) + [v[0] + d for v in vias for d in (-100, 100)]
mesh.AddLine('x', xl); mesh.SmoothMeshLines('x', min(res, 250))
yl = [0, 50, -50, 250, -250, W / 2, -W / 2, W / 2 + CLR, -W / 2 - CLR, PW / 2, -PW / 2, PW / 2 + CLR, -PW / 2 - CLR, 1050, -1050, YB, -YB]
yl += [s * v for v in (700, 900, 2725, 2925) for s in (1, -1)]
yl += list(np.linspace(-W / 2, W / 2, 5))
mesh.AddLine('y', yl); mesh.SmoothMeshLines('y', 60)
zl = list(np.linspace(zL2, zL1, 5)) + list(np.linspace(zL3, zL2, 4)) + ([zB, z6, zM, zL4] if EIGHT else [zL5, zL4, zL4 + 116, zL3 - 132])
zl += list(np.linspace(0, zL4, 3)) + [zL1 + 200, zL1 + 600, zL1 + 3000]
mesh.AddLine('z', zl); mesh.SmoothMeshLines('z', 120)
# ports (microstrip over L2)
p1 = FDTD.AddMSLPort(1, sig, [X0, -W / 2, zL1], [XP0, W / 2, zL2], 'x', 'z', excite=-1, Feed_R=50,
                     FeedShift=1000, MeasPlaneShift=3000, priority=15)
p2 = FDTD.AddMSLPort(2, sig, [X1, -W / 2, zL1], [XP1, W / 2, zL2], 'x', 'z', Feed_R=50,
                     MeasPlaneShift=3000, priority=15)
FDTD.Run(sim, cleanup=True, verbose=0)
f = np.linspace(0.3e9, fmax, 571)
for p in (p1, p2): p.CalcPort(sim, f, ref_impedance=50)
s11 = p1.uf_ref / p1.uf_inc; s21 = p2.uf_ref / p1.uf_inc
out = os.path.join(here, f"sma{'8' if EIGHT else ''}_{case}.csv")
np.savetxt(out, np.c_[f, 20 * np.log10(abs(s11)), 20 * np.log10(abs(s21))], delimiter=",", header="f_Hz,S11_dB,S21_dB", comments="")
for fq in (0.5e9, 1e9, 2e9, 3e9, 4.5e9):
    k = np.argmin(abs(f - fq)); print(f"{case}: {fq/1e9:.1f} GHz  S11 {20*np.log10(abs(s11[k])):6.1f} dB  S21 {20*np.log10(abs(s21[k])):5.2f} dB")
