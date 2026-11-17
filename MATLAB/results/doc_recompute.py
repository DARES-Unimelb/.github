#!/usr/bin/env python3
"""Independent recomputation of the UACS sizing, cruise and authority numbers.

Every formula here is implemented from scratch. No MATLAB function of the project
is called. The input numbers are transcribed from params/uacs_twin_params.m at the
revision of 2026-10-10 02:07; each block names the line it came from.

Output: results/doc_recompute.txt
"""

import math

RHO = 1.225          # env.rho_sl, params:29
G = 9.80665          # env.g, params:30
MU = 1.789e-5        # env.mu_sl

# mass.items, params:38-49 : name, m, x, y, z
ITEMS = [
    ("wing", 3.20, -0.05, 0.00, 0.00),
    ("fuselage_boom", 2.40, 0.00, 0.00, 0.00),
    ("payload_avionics", 3.00, -0.25, 0.00, 0.02),
    ("battery", 4.20, -0.15, 0.00, 0.03),
    ("tail", 0.85, 1.95, 0.00, 0.08),
    ("motor_esc_left", 0.70, -0.05, 0.90, -0.10),
    ("motor_esc_right", 0.70, -0.05, -0.90, -0.10),
    ("prop_left", 0.12, -0.05, 0.90, -0.10),
    ("prop_right", 0.12, -0.05, -0.90, -0.10),
    ("landing_gear", 1.30, -0.10, 0.00, 0.18),
    ("servos_links", 0.90, -0.10, 0.00, 0.05),
    ("margin", 2.51, 0.00, 0.00, 0.00),
]
MASS_M = 20.0        # params:35

B = 3.40             # params:68
AR = 5.80            # params:69
SPAN_LIMIT = 3.50    # params:67
S = B * B / AR       # params:70
CBAR = S / B         # params:71
SH = 0.270           # params:78
LH = 1.95            # params:79
SV = 0.550           # params:92
LV = 1.95            # params:93
VH = SH * LH / (S * CBAR)   # params:80
VV = SV * LV / (S * B)      # params:94

CL0 = 0.22           # params:115
CL_ALPHA = 4.65      # params:116
CL_DE = 0.353        # params:121
CL_MAX = 1.33        # params:123
CD0 = 0.0300         # params:126
E_OSWALD = 0.78      # params:127
K_IND = 1.0 / (math.pi * AR * E_OSWALD)   # params:128
CM0 = 0.020          # params:133
CM_ALPHA_WB = 0.020  # params:136
CM_ALPHA = CM_ALPHA_WB - VH * CL_ALPHA    # params:137
CM_DE = -CL_DE * VH * (LH / CBAR)         # params:139
CY_BETA_TAIL = -0.750   # params:162
CY_DR = 0.180           # params:143
CN_BETA = -VV * CY_BETA_TAIL   # params:163
CN_R = 2 * VV * CY_BETA_TAIL   # params:165
CN_DR = -VV * CY_DR            # params:167
CL_DA = 0.0412       # params:148
CL_P = -CL_ALPHA / 12.0   # params:146

AIL_MAX = math.radians(25.0)   # params:171
ELE_MAX = math.radians(25.0)   # params:176
RUD_MAX = math.radians(25.0)   # params:181

D_PROP = 0.4064      # params:194
PEAK_ELEC = 3500.0   # params:200
ETA_PROP = 0.72      # params:201
ETA_MOTOR = 0.88     # params:202
ETA_ESC = 0.95       # params:203
ETA_TOTAL = ETA_PROP * ETA_MOTOR * ETA_ESC   # params:204
FM = 0.55            # params:206
DISK_A = math.pi * D_PROP ** 2 / 4           # params:215
P_SHAFT_PEAK = PEAK_ELEC * ETA_MOTOR * ETA_ESC   # params:216
T_STATIC = (P_SHAFT_PEAK * FM * math.sqrt(2 * RHO * DISK_A)) ** (2.0 / 3.0)  # params:218
P_THR = 2.0 / 3.0    # params:223
SPEED_V = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]   # params:229
Y_MOTOR = 0.90       # params:232
Z_MOTOR = -0.10      # params:234
DIFF_LIM = 0.60      # params:244

V_CRUISE = 25.0      # params:254
H_CRUISE = 150.0     # params:253
STALL_MARGIN = 1.20  # params:342
V_MAX = 40.0         # params:346

LEAD = {   # (value, tolerance_rel) : Lead's numbers, for comparison
    "CL_cruise": (0.257, 0.005),
    "CD_cruise": (0.03465, 0.005),
    "LD_cruise": (7.42, 0.005),
    "q_cruise": (383.0, 0.005),
    "T_req": (26.44, 0.005),
    "P_elec": (1098.0, 0.005),
    "throttle_cruise": (0.102, 0.02),
    "T_static": (93.716, 0.002),
    "T_full_25": (52.836, 0.002),
    "twr_static": (0.956, 0.005),
    "rudder_Nm": (32.25, 0.005),
    "aileron_Nm": (46.63, 0.005),
    "elevator_Nm": (103.27, 0.005),
    "diff_Nm": (17.19, 0.01),
    "engine_out_Nm": (84.34, 0.005),
    "V_mc": (40.43, 0.005),
    "P_ideal_full": (1609.30, 0.001),
    "exp_throttle_V0": (0.444444, 0.001),
    "exp_power_V0": (0.666667, 0.001),
    "exp_power_V25": (0.89, 0.03),
    "exp_power_V45": (0.97, 0.03),
    "V_stall": (13.9, 0.005),
    "V_min": (16.7, 0.005),
}

lines = []
def out(s=""):
    lines.append(s)

def vdist(V, P_ideal):
    """Momentum theory: solve P_ideal = 2*rho*A*v_i*(V+v_i)^2 for v_i by bisection."""
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
    """Independent prop map: throttle is a share of peak shaft power."""
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

# ---------------------------------------------------------------- mass, inertia
mt = sum(i[1] for i in ITEMS)
xcg = sum(i[1] * i[2] for i in ITEMS) / mt
ycg = sum(i[1] * i[3] for i in ITEMS) / mt
zcg = sum(i[1] * i[4] for i in ITEMS) / mt
r2 = [i[2] ** 2 + i[3] ** 2 + i[4] ** 2 for i in ITEMS]
Lc = 0.55 * sum(math.sqrt(r) for r in r2) / len(ITEMS)
scale = Lc ** 2
Ixx = sum(i[1] * (i[3] ** 2 + i[4] ** 2) for i in ITEMS) + mt * scale * 0.35
Iyy = sum(i[1] * (i[2] ** 2 + i[4] ** 2) for i in ITEMS) + mt * scale * 0.55
Izz = sum(i[1] * (i[2] ** 2 + i[3] ** 2) for i in ITEMS) + mt * scale * 0.55
Ixz = -sum(i[1] * i[2] * i[4] for i in ITEMS)
Ixx_cg = Ixx - mt * (ycg ** 2 + zcg ** 2)
Iyy_cg = Iyy - mt * (xcg ** 2 + zcg ** 2)
Izz_cg = Izz - mt * (xcg ** 2 + ycg ** 2)
Ixz_cg = Ixz + mt * xcg * zcg

out("== 1. mass, geometry, inertia (independent recompute) ==")
out(f"mass sum            {mt:.6f} kg   P.mass.m {MASS_M:.6f}   diff {abs(mt-MASS_M):.2e}")
out(f"x_cg {xcg:+.6f}  y_cg {ycg:+.6f}  z_cg {zcg:+.6f} m")
out(f"b {B:.4f} m (limit {SPAN_LIMIT:.2f})  S {S:.6f} m2  cbar {CBAR:.6f} m  AR {AR:.3f}")
out(f"Vh {VH:.6f}  Vv {VV:.6f}")
out(f"Ixx_cg {Ixx_cg:.4f}  Iyy_cg {Iyy_cg:.4f}  Izz_cg {Izz_cg:.4f}  Ixz_cg {Ixz_cg:+.5f} kg m2")
out(f"z_motor - z_cg {Z_MOTOR - zcg:+.6f} m  (thrust line above CG)")
out()

# ---------------------------------------------------------------- propulsion
out("== 2. propulsion, momentum theory ==")
P_ideal_full = P_SHAFT_PEAK * FM
out(f"P_shaft_peak {P_SHAFT_PEAK:.3f} W   disk area {DISK_A:.6f} m2   P_ideal full {P_ideal_full:.4f} W")
out(f"T_static_max {T_STATIC:.5f} N per motor   thrust/weight {2*T_STATIC/(MASS_M*G):.5f}")
rows = []
for V in SPEED_V:
    T, vi = thrust(1.0, V)
    bal = 2 * RHO * DISK_A * vi * (V + vi) ** 2
    fm_impl = T * (V + vi) / P_SHAFT_PEAK
    rows.append((V, T, vi, bal, fm_impl))
out("  V     T_full      v_i     P_bal      FM_impl")
for V, T, vi, bal, fm_impl in rows:
    out(f"{V:3d}  {T:9.5f}  {vi:7.4f}  {bal:9.4f}  {fm_impl:.6f}")
out(f"power balance spread {max(r[3] for r in rows)-min(r[3] for r in rows):.3e} W")
out(f"implied FM spread    {max(r[4] for r in rows)-min(r[4] for r in rows):.3e}")
# exponent checks
t_lo, _ = thrust(1.0, 0.0)
# Exponent of thrust against THROTTLE at V=0 (throttle is a share of shaft power).
exp_thr_V0 = math.log(thrust(0.25, 0.0)[0] / t_lo) / math.log(0.25)
# Exponent of thrust against SHAFT POWER at V=0: should be 2/3 exactly.
exp_pow_V0 = exp_thr_V0 / P_THR
out(f"V=0 thrust vs throttle exponent {exp_thr_V0:.6f}  (contract 4/9 = {4/9:.6f})")
out(f"V=0 thrust vs shaft power exponent {exp_pow_V0:.6f}  (2/3 = {2/3:.6f})")
eps = 1e-6
def expo(V):
    a = thrust(1.0, V)[0]
    b = thrust(1.0 - eps, V)[0]
    e_thr = math.log(a / b) / math.log(1.0 / (1.0 - eps))
    return e_thr, e_thr / P_THR      # divide by 2/3 to get the power exponent
e25t, e25p = expo(25.0)
e45t, e45p = expo(45.0)
out(f"V=25 thrust vs throttle {e25t:.4f}, vs shaft power {e25p:.4f}")
out(f"V=45 thrust vs throttle {e45t:.4f}, vs shaft power {e45p:.4f}")
out()

# ---------------------------------------------------------------- cruise
out("== 3. cruise operating point ==")
q = 0.5 * RHO * V_CRUISE ** 2
CL = MASS_M * G / (q * S)
CD = CD0 + K_IND * CL ** 2
D = q * S * CD
T_req = D
P_shaft = T_req * V_CRUISE
P_elec = P_shaft / ETA_TOTAL
T_per_motor = T_req / 2
thr = solve_throttle(T_per_motor, V_CRUISE)
out(f"q {q:.4f} Pa   CL {CL:.6f}   CD {CD:.6f}   L/D {CL/CD:.4f}")
out(f"drag {D:.4f} N   P_shaft {P_shaft:.3f} W   P_elec {P_elec:.3f} W   per motor {P_elec/2:.3f} W")
out(f"throttle to hold cruise {thr:.6f}")
Vstall = math.sqrt(2 * MASS_M * G / (RHO * S * CL_MAX))
out(f"V_stall {Vstall:.4f} m/s   V_min {STALL_MARGIN*Vstall:.4f} m/s   V_max {V_MAX:.1f}")
alpha_pred = (CL - CL0) / CL_ALPHA
out(f"predicted trim alpha {math.degrees(alpha_pred):.4f} deg (CL/CL_alpha linearisation)")
out()

# ---------------------------------------------------------------- authority
out("== 4. control authority at cruise ==")
Sb = q * S * B
Sc = q * S * CBAR
N_rud = Sb * abs(CN_DR) * RUD_MAX
L_ail = Sb * CL_DA * AIL_MAX
M_ele = Sc * abs(CM_DE) * ELE_MAX
out(f"Cn_beta {CN_BETA:.6f}  Cn_r {CN_R:.6f}  Cn_dr {CN_DR:.6f}")
out(f"Cl_p {CL_P:.6f}  Cl_da {CL_DA:.6f}")
out(f"rudder  yaw   {N_rud:.4f} N m")
out(f"aileron roll  {L_ail:.4f} N m")
out(f"elevator pitch {M_ele:.4f} N m")
side_force = q * S * abs(CN_DR) * RUD_MAX
out(f"rudder side-force cross-check: {side_force:.4f} N x {LV:.2f} m = {side_force*LV:.4f} N m")
d_head = min(thr, 1.0 - thr)
d_eff = min(DIFF_LIM, d_head)
T_hi, _ = thrust(thr + d_eff, V_CRUISE)
T_lo, _ = thrust(thr - d_eff, V_CRUISE)
dT = T_hi - T_lo
N_diff = dT * Y_MOTOR
out(f"d_cap {DIFF_LIM:.3f}  d_headroom {d_head:.6f}  d_eff {d_eff:.6f}")
out(f"differential: T_hi {T_hi:.4f} T_lo {T_lo:.4f} dT {dT:.4f} N  yaw {N_diff:.4f} N m"
    f"  ratio to rudder {N_diff/N_rud:.4f}")
# The retired linear thrust law T = T_static_max*throttle gives a different number.
dT_lin = 2 * d_eff * T_STATIC
N_diff_lin = dT_lin * Y_MOTOR
out(f"linear law T=T_static*throttle would give dT {dT_lin:.4f} N, yaw {N_diff_lin:.4f} N m,"
    f" ratio {N_diff_lin/N_rud:.4f}  (rel diff {(N_diff_lin-N_diff)/N_diff*100:+.2f} %)")
out(f"total thrust with the split: {T_hi+T_lo:.4f} N vs commanded {2*T_per_motor:.4f} N")
N_eo = T_STATIC * Y_MOTOR
V_mc = math.sqrt(2 * N_eo / (RHO * S * B * abs(CN_DR) * RUD_MAX))
out(f"engine out: T_asym {T_STATIC:.4f} N  yaw {N_eo:.4f} N m  ratio to rudder {N_eo/N_rud:.4f}")
out(f"V_mc {V_mc:.4f} m/s  = {V_mc/V_CRUISE:.3f} x cruise, {V_mc/Vstall:.3f} x stall,"
    f" {V_mc/V_MAX:.3f} x V_max")
out()

# ---------------------------------------------------------------- derived checks
out("== 5. derived-value consistency checks ==")
cl_de_formula = 0.6 * CL_ALPHA * SH / S
out(f"CL_de stored {CL_DE:.6f} vs stated formula 0.6*CL_alpha*Sh/S = {cl_de_formula:.6f}"
    f"  rel diff {(CL_DE-cl_de_formula)/cl_de_formula*100:+.2f} %")
cm_alpha_nodw = CM_ALPHA
cm_alpha_dw = CM_ALPHA_WB - VH * CL_ALPHA * 0.6   # with a typical downwash factor 0.6
out(f"Cm_alpha {CM_ALPHA:.6f} (no downwash) vs {cm_alpha_dw:.6f} (downwash factor 0.6)")
out(f"Cm_de {CM_DE:.6f}   CL_de/Vh/lh/cbar chain consistent: "
    f"{abs(CM_DE - (-CL_DE*VH*(LH/CBAR))) < 1e-12}")
# analytic mode estimates
sp = math.sqrt(q * S * CBAR * (-CM_ALPHA) / Iyy_cg)
tau_roll = Ixx_cg / (q * S * B * (B / (2 * V_CRUISE)) * abs(CL_P))
dr = math.sqrt(q * S * B * CN_BETA / Izz_cg)
out(f"analytic short-period {sp:.4f} rad/s (V4 window 2..8)")
out(f"analytic roll subsidence tau {tau_roll:.4f} s (V4 window < 1 s)")
out(f"analytic Dutch roll {dr:.4f} rad/s")
out()

# ---------------------------------------------------------------- lead compare
out("== 6. comparison against the numbers reported by Lead ==")
mine = {
    "CL_cruise": CL, "CD_cruise": CD, "LD_cruise": CL / CD, "q_cruise": q,
    "T_req": T_req, "P_elec": P_elec, "throttle_cruise": thr,
    "T_static": T_STATIC, "T_full_25": thrust(1.0, 25.0)[0],
    "twr_static": 2 * T_STATIC / (MASS_M * G),
    "rudder_Nm": N_rud, "aileron_Nm": L_ail, "elevator_Nm": M_ele,
    "diff_Nm": N_diff, "engine_out_Nm": N_eo, "V_mc": V_mc,
    "P_ideal_full": P_ideal_full,
    "exp_throttle_V0": exp_thr_V0, "exp_power_V0": exp_pow_V0,
    "exp_power_V25": e25p, "exp_power_V45": e45p,
    "V_stall": Vstall, "V_min": STALL_MARGIN * Vstall,
}
bad = 0
for k, (ref, tol) in LEAD.items():
    val = mine[k]
    rel = abs(val - ref) / abs(ref)
    flag = "OK " if rel <= tol else "DIFF"
    if rel > tol:
        bad += 1
    out(f"{flag} {k:18s} mine {val:12.6f}  lead {ref:12.6f}  rel {rel*100:7.3f} %")
out(f"mismatches: {bad}")
out()
out("RECOMPUTE_DONE")
print("\n".join(lines))
open(r"C:\Ash\Projects\DARES_UACS\MATLAB\results\doc_recompute.txt", "w", encoding="utf-8").write("\n".join(lines) + "\n")
