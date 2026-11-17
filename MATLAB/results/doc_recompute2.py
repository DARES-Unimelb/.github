#!/usr/bin/env python3
"""Independent recompute round 2 for the UACS twin, 2026-10-10 frozen parameters.

Every formula is implemented here from scratch; no project function is called.
Constants are transcribed from MATLAB/params/uacs_twin_params.m at the revision of
2026-10-10 09:20; the source line is named for each block.

Output: results/doc_recompute2.txt
"""

import math

RHO = 1.225            # env.rho_sl, :29
G = 9.80665            # env.g, :30

# geometry, :67-103
SPAN_LIMIT = 3.50
B = 3.40
AR = 5.80
S = B * B / AR
CBAR = S / B
SH = 0.270
LH = 1.95
SV = 0.600
LV = 1.95
ELE_CHORD = 0.35       # geom.elevator.chord_frac, :102
RUD_CHORD = 0.50       # geom.rudder.chord_frac, :103
VH = SH * LH / (S * CBAR)
VV = SV * LV / (S * B)

# aerodynamics, :122-188
CL0 = 0.22
CL_ALPHA = 4.65
TAU_ELE = 0.60
TAU_RUD = 0.50
CL_MAX = 1.33
CD0 = 0.0300
E_OSWALD = 0.78
K_IND = 1.0 / (math.pi * AR * E_OSWALD)
CM0 = 0.020
CM_ALPHA_WB = 0.020
CY_BETA_TAIL = -0.750
K_BODY_RUDDER = 0.85
CL_DA = 0.0412
CL_P = -CL_ALPHA / 12.0

CL_DE = TAU_ELE * CL_ALPHA * SH / S
CM_ALPHA = CM_ALPHA_WB - VH * CL_ALPHA
CM_Q = -2 * CL_ALPHA * VH * (LH / CBAR)
CM_DE = -CL_DE * VH * (LH / CBAR)
CY_DR = K_BODY_RUDDER * TAU_RUD * CL_ALPHA * SV / S
CN_BETA = -VV * CY_BETA_TAIL
CN_R = 2 * VV * CY_BETA_TAIL
CN_DR = -VV * CY_DR * RUD_CHORD

# propulsion, :200-250
D_PROP = 0.4064
PEAK_ELEC = 3500.0
ETA_TOTAL = 0.72 * 0.88 * 0.95
FM = 0.55
DISK_A = math.pi * D_PROP ** 2 / 4
P_SHAFT_PEAK = PEAK_ELEC * 0.88 * 0.95
T_STATIC = (P_SHAFT_PEAK * FM * math.sqrt(2 * RHO * DISK_A)) ** (2.0 / 3.0)
P_THR = 2.0 / 3.0
SPEED_V = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]
Y_MOTOR = 0.90
DIFF_LIM = 0.60

MASS_M = 20.0
V_CRUISE = 25.0
STALL_MARGIN = 1.20
V_MAX = 40.0
DR_MAX = math.radians(25.0)

LEAD = {
    "Sv": 0.600, "Vh": 0.4506, "Vv": 0.1578,
    "Cm_alpha": -2.0754, "Cm_q": -13.941, "Cm_de": -0.5291,
    "Cn_beta": 0.0904, "Cn_r": -0.1808, "Cn_dr": -0.0320,
    "Cl_p": -0.3875, "Cl_da": 0.0412,
    "T_static": 93.7157, "TWR": 0.956,
    "CL_cruise": 0.2571, "CD_cruise": 0.03465, "LD": 7.42, "q": 383.0,
    "T_req": 26.44, "P_elec": 1098.0, "throttle": 0.102,
    "V_stall": 13.9, "V_min": 16.7,
    "rudder_Nm": 58.13, "aileron_Nm": 46.63, "elevator_Nm": 110.57,
    "diff_Nm": 17.19, "engine_out_Nm": 84.34, "V_mc": 30.11,
    "ratio_Cn_dr_over_Cn_beta": 0.355,
}

out = []
def p(s=""):
    out.append(s)

def vdist(V, P_ideal):
    lo, hi = 0.0, (P_ideal / (2 * RHO * DISK_A)) ** (1.0 / 3.0) + 1.0
    while 2 * RHO * DISK_A * hi * (V + hi) ** 2 < P_ideal:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if 2 * RHO * DISK_A * mid * (V + mid) ** 2 < P_ideal:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

def thrust(throttle, V):
    x = min(max(throttle, 0.0), 1.0) ** P_THR
    P_ideal = x * P_SHAFT_PEAK * FM
    if P_ideal <= 0:
        return 0.0, 0.0
    vi = vdist(V, P_ideal)
    return 2 * RHO * DISK_A * vi * (V + vi), vi

def solve_throttle(T_target, V):
    lo, hi = 0.0, 1.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if thrust(mid, V)[0] < T_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

def ratio_for(Sv, chord, cy_dr=None):
    """|Cn_dr|/Cn_beta for one fin configuration."""
    vv = Sv * LV / (S * B)
    cy = (K_BODY_RUDDER * TAU_RUD * CL_ALPHA * Sv / S) if cy_dr is None else cy_dr
    cn_beta = -vv * CY_BETA_TAIL
    cn_dr = -vv * cy * chord
    return abs(cn_dr) / cn_beta, cn_dr, cn_beta, cy

def vmc(cn_dr_abs, t_static=T_STATIC):
    return math.sqrt(2 * t_static * Y_MOTOR / (RHO * S * B * cn_dr_abs * DR_MAX))

# ---------------------------------------------------------------- 1 geometry
p("== 1. geometry and derived aerodynamics (my own implementation) ==")
p(f"S {S:.6f} m2   cbar {CBAR:.6f} m   Vh {VH:.6f}   Vv {VV:.6f}")
p(f"CL_de {CL_DE:.6f}   Cm_alpha {CM_ALPHA:.6f}   Cm_q {CM_Q:.6f}   Cm_de {CM_DE:.6f}")
p(f"CY_dr {CY_DR:.6f}   Cn_beta {CN_BETA:+.6f}   Cn_r {CN_R:+.6f}   Cn_dr {CN_DR:+.6f}")
p(f"Cl_p {CL_P:.6f}   Cl_da {CL_DA:.6f}")
p(f"|Cn_dr|/Cn_beta {abs(CN_DR)/CN_BETA:.4f}")
p()

# ---------------------------------------------------------------- 2 propulsion
p("== 2. propulsion invariants ==")
p(f"P_shaft_peak {P_SHAFT_PEAK:.3f} W   P_ideal full {P_SHAFT_PEAK*FM:.4f} W   T_static {T_STATIC:.5f} N")
p(f"static thrust/weight {2*T_STATIC/(MASS_M*G):.5f}")
tab = []
for V in SPEED_V:
    T, vi = thrust(1.0, V)
    tab.append((V, T, 2 * RHO * DISK_A * vi * (V + vi) ** 2, T * (V + vi) / P_SHAFT_PEAK))
p("  V     T_full      P_balance      FM_implied")
for V, T, bal, fm in tab:
    p(f"{V:3d}  {T:10.6f}  {bal:12.6f}  {fm:.6f}")
p(f"power balance spread {max(r[2] for r in tab)-min(r[2] for r in tab):.3e} W")
p(f"implied FM spread    {max(r[3] for r in tab)-min(r[3] for r in tab):.3e}")
t0 = thrust(1.0, 0.0)[0]
p(f"V=0 throttle exponent {math.log(thrust(0.25,0.0)[0]/t0)/math.log(0.25):.6f} (4/9 = {4/9:.6f})")
p(f"half-power thrust ratio at V=0 {thrust(0.5**(3/2),0.0)[0]/t0:.9f} vs 2^(-2/3) "
  f"{2**(-2/3):.9f} diff {abs(thrust(0.5**(3/2),0.0)[0]/t0-2**(-2/3)):.2e}")
p()

# ---------------------------------------------------------------- 3 yaw criterion
p("== 3. single-engine criterion |Cn_dr|/Cn_beta ==")
p("claim under test: enlarging the fin by itself makes single-engine control worse")
p("  config            Sv     chord   CY_dr    Cn_beta   Cn_dr      ratio    V_mc")
configs = [
    ("0.34 m2, 0.35 chord, derived", 0.34, 0.35, None),
    ("0.55 m2, 0.35 chord, derived", 0.55, 0.35, None),
    ("0.60 m2, 0.50 chord, derived", 0.60, 0.50, None),
    ("0.55 m2, rudder area held at 0.34*0.35", 0.55, 0.34 * 0.35 / 0.55, None),
    ("0.90 m2, rudder area held at 0.34*0.35", 0.90, 0.34 * 0.35 / 0.90, None),
    ("0.34 m2, 0.35 chord, hand CY_dr=0.18", 0.34, 0.35, 0.18),
    ("0.55 m2, 0.35 chord, hand CY_dr=0.18", 0.55, 0.35, 0.18),
    ("0.60 m2, 0.50 chord, hand CY_dr=0.18", 0.60, 0.50, 0.18),
]
for name, sv, ch, cy in configs:
    r, cn_dr, cn_beta, cy_used = ratio_for(sv, ch, cy)
    p(f"  {name:36s} {sv:.2f}  {ch:.2f}  {cy_used:.4f}  {cn_beta:+.5f}  {cn_dr:+.6f}  {r:.4f}  {vmc(abs(cn_dr)):6.2f}")
p("algebra: with CY_dr derived, ratio = k_body*tau_rud*CL_alpha*Sv*chord/(0.75*S), so it is")
p(f"strictly proportional to Sv*chord; check {abs(CN_DR)/CN_BETA:.6f} vs "
  f"{K_BODY_RUDDER*TAU_RUD*CL_ALPHA*SV*RUD_CHORD/(0.75*S):.6f}")
p("with CY_dr hand entered the ratio is 0.18*chord/0.75, independent of Sv.")
p()

# ---------------------------------------------------------------- 4 authority
p("== 4. authority at cruise ==")
q = 0.5 * RHO * V_CRUISE ** 2
CL = MASS_M * G / (q * S)
CD = CD0 + K_IND * CL ** 2
T_req = q * S * CD
thr = solve_throttle(T_req / 2, V_CRUISE)
Sb = q * S * B
Sc = q * S * CBAR
N_rud = Sb * abs(CN_DR) * DR_MAX
N_ail = Sb * CL_DA * DR_MAX
N_ele = Sc * abs(CM_DE) * DR_MAX
N_eo = T_STATIC * Y_MOTOR
d_eff = min(DIFF_LIM, min(thr, 1 - thr))
T_hi, _ = thrust(thr + d_eff, V_CRUISE)
T_lo, _ = thrust(thr - d_eff, V_CRUISE)
N_diff = (T_hi - T_lo) * Y_MOTOR
N_diff_lin = 2 * d_eff * T_STATIC * Y_MOTOR
p(f"q {q:.4f}   CL {CL:.6f}   CD {CD:.6f}   L/D {CL/CD:.4f}")
p(f"T_req {T_req:.4f} N   P_elec {T_req*V_CRUISE/ETA_TOTAL:.3f} W   throttle {thr:.6f}")
p(f"rudder {N_rud:.4f} Nm   aileron {N_ail:.4f} Nm   elevator {N_ele:.4f} Nm")
p(f"engine out {N_eo:.4f} Nm = {N_eo/N_rud:.4f} x rudder   V_mc {vmc(abs(CN_DR)):.4f} m/s")
p(f"diff thrust, nonlinear map {N_diff:.4f} Nm = {N_diff/N_rud:.4f} x rudder")
p(f"diff thrust, retired linear law {N_diff_lin:.4f} Nm = {N_diff_lin/N_rud:.4f} x rudder")
p(f"V_stall {math.sqrt(2*MASS_M*G/(RHO*S*CL_MAX)):.4f} m/s   V_min {STALL_MARGIN*math.sqrt(2*MASS_M*G/(RHO*S*CL_MAX)):.4f} m/s")
p()

# ---------------------------------------------------------------- 5 comparison
p("== 5. comparison against the frozen parameter table ==")
mine = {
    "Sv": SV, "Vh": VH, "Vv": VV,
    "Cm_alpha": CM_ALPHA, "Cm_q": CM_Q, "Cm_de": CM_DE,
    "Cn_beta": CN_BETA, "Cn_r": CN_R, "Cn_dr": CN_DR,
    "Cl_p": CL_P, "Cl_da": CL_DA,
    "T_static": T_STATIC, "TWR": 2 * T_STATIC / (MASS_M * G),
    "CL_cruise": CL, "CD_cruise": CD, "LD": CL / CD, "q": q,
    "T_req": T_req, "P_elec": T_req * V_CRUISE / ETA_TOTAL, "throttle": thr,
    "V_stall": math.sqrt(2 * MASS_M * G / (RHO * S * CL_MAX)),
    "V_min": STALL_MARGIN * math.sqrt(2 * MASS_M * G / (RHO * S * CL_MAX)),
    "rudder_Nm": N_rud, "aileron_Nm": N_ail, "elevator_Nm": N_ele,
    "diff_Nm": N_diff, "engine_out_Nm": N_eo, "V_mc": vmc(abs(CN_DR)),
    "ratio_Cn_dr_over_Cn_beta": abs(CN_DR) / CN_BETA,
}
bad = []
for k, ref in LEAD.items():
    val = mine[k]
    rel = abs(val - ref) / abs(ref) if ref else abs(val)
    flag = "OK  " if rel <= 0.02 else "DIFF"
    if rel > 0.02:
        bad.append(k)
    p(f"{flag} {k:26s} mine {val:12.6f}   lead {ref:12.6f}   rel {rel*100:7.3f} %")
p(f"mismatches over 2 percent: {len(bad)} -> {', '.join(bad)}")
p()
p("RECOMPUTE2_DONE")
text = "\n".join(out)
print(text)
open(r"C:\Ash\Projects\DARES_UACS\MATLAB\results\doc_recompute2.txt", "w", encoding="utf-8").write(text + "\n")
