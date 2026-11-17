#!/usr/bin/env python3
"""Independent recompute round 3: frozen parameter set rev 10.

Implements the longitudinal trim from scratch (Newton on alpha, elevator, throttle),
the authority budget, the V_min definitions and the AR x tail-ratio design search.
No project function is called; constants are transcribed from
MATLAB/params/uacs_twin_params.m at revision 10 (meta.revision = 10).

Output: results/doc_recompute3.txt
"""

import math

RHO = 1.225
G = 9.80665
B = 3.40
LH = 1.95
AR_REF = 5.00
SH_OVER_S = 0.16
SV = 0.600
LV = 1.95
ELE_TRAVEL_DEG = 25.0
AIL_TRAVEL_DEG = 25.0
RUD_TRAVEL_DEG = 25.0
ELE_TRIM_FRAC = 0.80

CL0 = 0.22
CL_ALPHA = 4.65
TAU_ELE = 0.65
TAU_RUD = 0.50
CL_MAX = 1.33
CD0 = 0.0300
E_OSWALD = 0.78
CD_ALPHA = 0.10
CD_DE = 0.010
CM0_WING = -0.012
I_TAIL_REL_WING_DEG = 1.5
CM_ALPHA_WB = 0.020
CY_BETA_TAIL = -0.750
K_BODY_RUDDER = 0.85
RUD_CHORD = 0.50
CL_DA = 0.0412
CL_P = -CL_ALPHA / 12.0
CN_DR_CHORD = RUD_CHORD

ITEMS = [
    ("wing", 1.10, -0.05, 0.00, 0.00),
    ("fuselage_boom", 0.55, 0.00, 0.00, 0.00),
    ("payload", 12.00, -0.22, 0.00, 0.02),
    ("battery", 2.10, -0.15, 0.00, 0.03),
    ("tail", 0.20, 1.95, 0.00, 0.08),
    ("motor_esc_left", 0.17, -0.05, 0.90, -0.10),
    ("motor_esc_right", 0.17, -0.05, -0.90, -0.10),
    ("prop_left", 0.03, -0.05, 0.90, -0.10),
    ("prop_right", 0.03, -0.05, -0.90, -0.10),
    ("landing_gear", 0.20, -0.10, 0.00, 0.18),
    ("servos_links", 0.20, -0.10, 0.00, 0.05),
    ("margin", 0.25, 0.00, 0.00, 0.00),
]
MASS = sum(i[1] for i in ITEMS)
PEAK_ELEC = 1500.0
ETA_MOTOR, ETA_ESC, ETA_PROP = 0.88, 0.95, 0.72
ETA_TOTAL = ETA_PROP * ETA_MOTOR * ETA_ESC
FM = 0.55
D_PROP = 0.4064
DISK_A = math.pi * D_PROP ** 2 / 4
P_SHAFT_PEAK = PEAK_ELEC * ETA_MOTOR * ETA_ESC
T_STATIC = (P_SHAFT_PEAK * FM * math.sqrt(2 * RHO * DISK_A)) ** (2.0 / 3.0)
P_THR = 2.0 / 3.0
Y_MOTOR = 0.90
V_CRUISE = 25.0
V_MAX = 30.0
RUD_MAX = math.radians(RUD_TRAVEL_DEG)
ELE_MAX = math.radians(ELE_TRAVEL_DEG)

out = []
def p(s=""):
    out.append(s)

def geom_of(ar, ratio, xcg):
    S = B * B / ar
    cbar = S / B
    Sh = ratio * S
    Vh = Sh * LH / (S * cbar)
    return S, cbar, Sh, Vh

def aero_of(ar, ratio, xcg):
    S, cbar, Sh, Vh = geom_of(ar, ratio, xcg)
    Cm_alpha = CM_ALPHA_WB - Vh * CL_ALPHA
    CL_de = TAU_ELE * CL_ALPHA * Sh / S
    Cm_de = -CL_de * Vh * (LH / cbar)          # as written in the parameter file
    Cm_de_std = -Vh * CL_ALPHA * TAU_ELE       # standard tail-volume form
    k = 1.0 / (math.pi * ar * E_OSWALD)
    return S, cbar, Sh, Vh, Cm_alpha, CL_de, Cm_de, Cm_de_std, k

# --- CG from the item table (independent of AR) ---
x_cg = sum(i[1] * i[2] for i in ITEMS) / MASS
y_cg = sum(i[1] * i[3] for i in ITEMS) / MASS
z_cg = sum(i[1] * i[4] for i in ITEMS) / MASS

# ---------------------------------------------------------------- propulsion
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

def thrust_1(t, V):
    x = min(max(t, 0.0), 1.0) ** P_THR
    Pi = x * P_SHAFT_PEAK * FM
    if Pi <= 0:
        return 0.0
    vi = vdist(V, Pi)
    return 2 * RHO * DISK_A * vi * (V + vi)

# ---------------------------------------------------------------- trim solver
def residuals(z, V, ar, ratio, gamma=0.0):
    alpha, de, t = z
    S, cbar, Sh, Vh, Cm_alpha, CL_de, Cm_de, _, k = aero_of(ar, ratio, x_cg)
    q = 0.5 * RHO * V * V
    alpha0 = -CL0 / CL_ALPHA
    CL = CL0 + CL_ALPHA * alpha + CL_de * de
    CD = CD0 + k * CL * CL + CD_ALPHA * (alpha - alpha0) ** 2 + CD_DE * abs(de)
    L = q * S * CL
    D = q * S * CD
    T = 2.0 * thrust_1(t, V)
    theta = alpha + gamma
    Cm = CM0_LOCAL + Cm_alpha * alpha + Cm_de * de
    f1 = L * math.sin(alpha) - D * math.cos(alpha) + T - MASS * G * math.sin(theta)
    f2 = -L * math.cos(alpha) - D * math.sin(alpha) + MASS * G * math.cos(theta)
    f3 = q * S * cbar * Cm + (Z_MOTOR - z_cg) * T
    return [f1, f2, f3]

def trim(V, ar=AR_REF, ratio=SH_OVER_S, gamma=0.0):
    """Damped Newton on (alpha, de, throttle)."""
    S, cbar, Sh, Vh = geom_of(ar, ratio, x_cg)
    q = 0.5 * RHO * V * V
    CL_req = MASS * G * math.cos(gamma) / (q * S)
    alpha = (CL_req - CL0) / CL_ALPHA
    de = 0.0
    t = 0.4
    for _ in range(200):
        f = residuals([alpha, de, t], V, ar, ratio, gamma)
        nrm = max(abs(v) for v in f)
        if nrm < 1e-12:
            break
        J = [[0.0] * 3 for _ in range(3)]
        h = [1e-7, 1e-7, 1e-7]
        for j in range(3):
            zp = [alpha, de, t]
            zm = [alpha, de, t]
            zp[j] += h[j]
            zm[j] -= h[j]
            fp = residuals(zp, V, ar, ratio, gamma)
            fm = residuals(zm, V, ar, ratio, gamma)
            for i in range(3):
                J[i][j] = (fp[i] - fm[i]) / (2 * h[j])
        # solve 3x3 by Cramer
        def det3(M):
            return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                    - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
        D0 = det3(J)
        if abs(D0) < 1e-18:
            break
        step = []
        for c in range(3):
            M = [row[:] for row in J]
            for r in range(3):
                M[r][c] = -f[r]
            step.append(det3(M) / D0)
        lam = 1.0
        for _ in range(40):
            cand = [alpha + lam * step[0], de + lam * step[1], t + lam * step[2]]
            if max(abs(v) for v in residuals(cand, V, ar, ratio, gamma)) < nrm:
                break
            lam *= 0.5
        alpha, de, t = alpha + lam * step[0], de + lam * step[1], t + lam * step[2]
    S, cbar, Sh, Vh, Cm_alpha, CL_de, Cm_de, Cm_de_std, k = aero_of(ar, ratio, x_cg)
    q = 0.5 * RHO * V * V
    CL = CL0 + CL_ALPHA * alpha + CL_de * de
    CD = CD0 + k * CL * CL + CD_ALPHA * (alpha + CL0 / CL_ALPHA) ** 2 + CD_DE * abs(de)
    return dict(alpha=alpha, de=de, t=t, CL=CL, CD=CD, S=S, cbar=cbar, Vh=Vh,
                Cm_alpha=Cm_alpha, CL_de=CL_de, Cm_de=Cm_de, k=k,
                res=max(abs(v) for v in residuals([alpha, de, t], V, ar, ratio, gamma)))

Z_MOTOR = -0.10
CM0_LOCAL = None

def set_cm0(ar=AR_REF, ratio=SH_OVER_S):
    """Cm0 chain exactly as the parameter file derives it."""
    global CM0_LOCAL
    S, cbar, Sh, Vh = geom_of(ar, ratio, x_cg)
    Cm_alpha = CM_ALPHA_WB - Vh * CL_ALPHA
    q_d = 0.5 * RHO * V_CRUISE ** 2
    CL_d = MASS * G / (q_d * S)
    a_trim = (CL_d - CL0) / CL_ALPHA
    a_t = (CM0_WING + Cm_alpha * a_trim) / (Vh * CL_ALPHA)
    eps_dw = I_TAIL_REL_WING_DEG - math.degrees(a_t)
    CM0_LOCAL = CM0_WING - Vh * CL_ALPHA * a_t
    return dict(CL_d=CL_d, a_trim=a_trim, a_t=a_t, eps_dw=eps_dw, Cm0=CM0_LOCAL)

# ---------------------------------------------------------------- 1 rev 10 numbers
cm0 = set_cm0()
S, cbar, Sh, Vh, Cm_alpha, CL_de, Cm_de, Cm_de_std, k = aero_of(AR_REF, SH_OVER_S, x_cg)
Vh_geo = Sh * LH / (S * cbar)
Vv = SV * LV / (S * B)
CY_dr = K_BODY_RUDDER * TAU_RUD * CL_ALPHA * SV / S
Cn_beta = -Vv * CY_BETA_TAIL
Cn_r = 2 * Vv * CY_BETA_TAIL
Cn_dr = -Vv * CY_dr * RUD_CHORD

p("== 1. rev 10 geometry, mass, derived aerodynamics ==")
p(f"mass {MASS:.4f} kg   x_cg {x_cg:+.6f}  z_cg {z_cg:+.6f} m   span {B} (limit 3.50)")
p(f"S {S:.6f} m2  cbar {cbar:.6f} m  Sh {Sh:.6f} m2 ({(Sh/S)*100:.2f}% S)  Vh {Vh_geo:.6f}")
p(f"Sv {SV:.3f} m2 ({(SV/S)*100:.2f}% S)  Vv {Vv:.6f}  CY_dr {CY_dr:.6f}")
p(f"Cm_alpha {Cm_alpha:.6f}  CL_de {CL_de:.6f}  Cm_de {Cm_de:.6f}  Cm_de_standard_form {Cm_de_std:.6f}")
p(f"ratio Cm_de/Cm_de_std {Cm_de/Cm_de_std:.6f}  (= Vh if the file double counts)")
p(f"alpha_trim_design {math.degrees(cm0['a_trim']):.6f} deg  alpha_tail_zerolift {math.degrees(cm0['a_t']):.6f} deg")
p(f"implied downwash {cm0['eps_dw']:.4f} deg   Cm0 {cm0['Cm0']:+.6f}")
p(f"Cn_beta {Cn_beta:+.6f}  Cn_r {Cn_r:+.6f}  Cn_dr {Cn_dr:+.6f}  ratio {abs(Cn_dr)/Cn_beta:.6f}")
p()

# ---------------------------------------------------------------- 2 cruise trim
p("== 2. my own trim solver ==")
tr = trim(V_CRUISE)
q = 0.5 * RHO * V_CRUISE ** 2
CLw = tr["CL"]
CDw = tr["CD"]
T_req = 2.0 * thrust_1(tr["t"], V_CRUISE)
p(f"cruise V=25: alpha {math.degrees(tr['alpha']):+.6f} deg  de {math.degrees(tr['de']):+.6f} deg  throttle {tr['t']:.6f}")
p(f"             CL {CLw:.6f}  CD {CDw:.6f}  L/D {CLw/CDw:.4f}  q {q:.4f} Pa  residual {tr['res']:.3e}")
p(f"             T_total {T_req:.4f} N  P_shaft {T_req*V_CRUISE:.3f} W  P_elec {T_req*V_CRUISE/ETA_TOTAL:.3f} W")
p(f"             elevator demand {abs(math.degrees(tr['de']))/ELE_TRAVEL_DEG*100:.2f} % of travel")
for V in (16.0, 20.0, 25.0, 30.0):
    t2 = trim(V)
    p(f"  V={V:5.1f}: alpha {math.degrees(t2['alpha']):+8.4f} deg  de {math.degrees(t2['de']):+9.4f} deg"
      f"  ({abs(math.degrees(t2['de']))/ELE_TRAVEL_DEG*100:5.1f} % travel)  throttle {t2['t']:.4f}")
p()

# ---------------------------------------------------------------- 3 V_min
p("== 3. V_min: stall limit vs trim limit ==")
V_stall = math.sqrt(2 * MASS * G / (RHO * S * CL_MAX))
V_min_stall = 1.20 * V_stall
lo, hi = 1.0, V_CRUISE
for _ in range(60):
    mid = 0.5 * (lo + hi)
    d = abs(trim(mid)["de"])
    if d > math.radians(ELE_TRIM_FRAC * ELE_TRAVEL_DEG):
        lo = mid
    else:
        hi = mid
V_min_trim = 0.5 * (lo + hi)
p(f"V_stall {V_stall:.4f} m/s   V_min_stall (1.2x) {V_min_stall:.4f} m/s")
p(f"V_min_trim (my solver, |de| = {ELE_TRIM_FRAC:.0%} travel) {V_min_trim:.9f} m/s")
p(f"V_min = max(both) = {max(V_min_stall, V_min_trim):.4f} m/s  -> trim binds: {V_min_trim > V_min_stall}")
# closed-form check quoted in the architecture doc
def de_closed(V):
    CLr = MASS * G / (0.5 * RHO * V * V * S)
    a = (CLr - CL0) / CL_ALPHA
    return -(CM0_LOCAL + Cm_alpha * a) / Cm_de
Vq = 15.41
p(f"closed-form de at {Vq} m/s: {math.degrees(de_closed(Vq)):+.4f} deg ; "
  f"true trim de: {math.degrees(trim(Vq)['de']):+.4f} deg")
p()

# ---------------------------------------------------------------- 4 authority
p("== 4. cruise authority ==")
Sb = q * S * B
Sc = q * S * cbar
N_rud = Sb * abs(Cn_dr) * RUD_MAX
N_ail = Sb * CL_DA * math.radians(AIL_TRAVEL_DEG)
N_ele_file = Sc * abs(Cm_de) * ELE_MAX
N_ele_std = Sc * abs(Cm_de_std) * ELE_MAX
N_eo = T_STATIC * Y_MOTOR
V_mc = math.sqrt(2 * N_eo / (RHO * S * B * abs(Cn_dr) * RUD_MAX))
p(f"rudder  {N_rud:.4f} Nm   aileron {N_ail:.4f} Nm")
p(f"elevator with the file's Cm_de {N_ele_file:.4f} Nm ; with the standard form {N_ele_std:.4f} Nm")
p(f"engine out yaw {N_eo:.4f} Nm = {N_eo/N_rud:.4f} x rudder   V_mc {V_mc:.6f} m/s")
d_head = min(tr["t"], 1 - tr["t"])
d_eff = min(0.60, d_head)
T_hi, T_lo = thrust_1(tr["t"] + d_eff, V_CRUISE), thrust_1(tr["t"] - d_eff, V_CRUISE)
N_diff_nonlin = (T_hi - T_lo) * Y_MOTOR
N_diff_lin = 2 * d_eff * T_STATIC * Y_MOTOR
p(f"d_headroom {d_head:.4f}  d_eff {d_eff:.4f}")
p(f"diff thrust: nonlinear map {N_diff_nonlin:.4f} Nm ({N_diff_nonlin/N_rud:.4%} of rudder);"
  f" linear law {N_diff_lin:.4f} Nm ({N_diff_lin/N_rud:.4%})")
margin = -Cm_alpha / CL_ALPHA - x_cg / cbar
p(f"static margin {margin:.6f} ({margin*100:.2f}% cbar)")
p()

# ---------------------------------------------------------------- 5 design search
p("== 5. design search: AR x tail area ratio ==")
p("criteria: static margin <= 0.30 AND trim at 16 m/s within half travel (12.5 deg)")
ars = [4.2 + 0.1 * i for i in range(34)]
ratios = [0.06 + 0.01 * i for i in range(25)]
best = None
n_margin_ok = 0
n_both_ok = 0
for ar in ars:
    for rt in ratios:
        Sx, cbx, Shx, Vhx = geom_of(ar, rt, x_cg)
        Cma = CM_ALPHA_WB - Vhx * CL_ALPHA
        m = -Cma / CL_ALPHA - x_cg / cbx
        if m > 0.30:
            continue
        n_margin_ok += 1
        set_cm0(ar, rt)
        d = trim(16.0, ar, rt)["de"]
        if abs(math.degrees(d)) <= 12.5:
            n_both_ok += 1
            if best is None:
                best = (ar, rt, m, math.degrees(d))
min_margin = min(-(CM_ALPHA_WB - geom_of(ar, rt, x_cg)[3] * CL_ALPHA) / CL_ALPHA
                 - x_cg / geom_of(ar, rt, x_cg)[1] for ar in ars for rt in ratios)
p(f"grid {len(ars)} x {len(ratios)} = {len(ars)*len(ratios)} points")
p(f"cells with margin <= 30%: {n_margin_ok}")
p(f"cells satisfying both:    {n_both_ok}")
p(f"minimum static margin anywhere on the grid: {min_margin:.6f} ({min_margin*100:.2f}% cbar)")
p(f"margin <= 0.30 requires ratio <= {(0.3043011-0.0463676*4.2)/(0.573529*4.2):.4f} at AR 4.2"
  f" (grid starts at 0.06)")
if best:
    p(f"first feasible cell: AR {best[0]:.2f} ratio {best[1]:.2f} margin {best[2]:.4f} de {best[3]:.2f} deg")
# current design
set_cm0()
cur = trim(16.0)
p(f"current design (AR 5.0, ratio 0.16): margin {margin:.4f}, de at 16 m/s {math.degrees(cur['de']):+.4f} deg"
  f" ({abs(math.degrees(cur['de']))/ELE_TRAVEL_DEG*100:.1f} % travel)")
p()
p("RECOMPUTE3_DONE")

# ---------------------------------------------------------------- 6 vs MATLAB job
p()
p("== 6. my solver against the MATLAB plant baseline (results/atmo_regress_report.txt) ==")
ref = {
    25.0: dict(alpha=-0.303253629813, de=-0.811542051406, t=0.430065225287, CL=0.188538849981, CD=0.033219464752),
    18.0: dict(alpha=2.971291074235, de=-11.854496125419, t=0.152463984128, CL=0.361086642640, CD=0.043694116668),
}
ref_climb = dict(alpha=0.369364117762, de=-3.438764899248, t=0.729613896065, CL=0.220952189547, CD=0.034873750996)
set_cm0()
for V in (25.0, 18.0):
    m = trim(V)
    r = ref[V]
    p(f"V={V:5.1f}  alpha mine {math.degrees(m['alpha']):+.12f} vs matlab {r['alpha']:+.12f}"
      f"  diff {abs(math.degrees(m['alpha'])-r['alpha']):.2e}")
    p(f"        de    mine {math.degrees(m['de']):+.12f} vs matlab {r['de']:+.12f}"
      f"  diff {abs(math.degrees(m['de'])-r['de']):.2e}")
    p(f"        thrott mine {m['t']:.12f} vs matlab {r['t']:.12f}  diff {abs(m['t']-r['t']):.2e}")
    p(f"        CL    mine {m['CL']:.12f} vs matlab {r['CL']:.12f}  CD mine {m['CD']:.12f} vs matlab {r['CD']:.12f}")
mc = trim(23.0, gamma=math.radians(6.0))
p(f"climb V=23 gamma=6  alpha mine {math.degrees(mc['alpha']):+.12f} vs matlab {ref_climb['alpha']:+.12f}"
  f"  diff {abs(math.degrees(mc['alpha'])-ref_climb['alpha']):.2e}")
p(f"                    de    mine {math.degrees(mc['de']):+.12f} vs matlab {ref_climb['de']:+.12f}"
  f"  diff {abs(math.degrees(mc['de'])-ref_climb['de']):.2e}")
p(f"                    thrott mine {mc['t']:.12f} vs matlab {ref_climb['t']:.12f}  diff {abs(mc['t']-ref_climb['t']):.2e}")
p(f"                    CL mine {mc['CL']:.12f} vs matlab {ref_climb['CL']:.12f}  CD mine {mc['CD']:.12f} vs matlab {ref_climb['CD']:.12f}")
p("MATCH_DONE")
text = "\n".join(out)
print(text)
open(r"C:\Ash\Projects\DARES_UACS\MATLAB\results\doc_recompute3.txt", "w", encoding="utf-8").write(text + "\n")
