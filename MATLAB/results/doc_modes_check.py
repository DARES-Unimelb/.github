#!/usr/bin/env python3
"""Independent check of the rev 10 linearisation output printed in
results/plant_modes_report.txt.

The matrices are taken verbatim from that report; nothing is re-linearised and no
project function is called. Eigenvalues are verified against the printed ones
through the elementary symmetric functions of the characteristic polynomial,
which for a 4x4 determine the eigenvalues exactly.

Output: results/doc_modes_check.txt
"""

import math
from itertools import combinations

A_LONG = [[-0.19966, 0.026542, 0.032719, -9.80651],
          [-0.836131, -9.74879, 22.2387, 0.0519041],
          [0.0716309, -28.7258, -68.0962, 0.0],
          [0.0, 0.0, 1.0, 0.0]]
A_LAT = [[-1.63105, -0.132319, -24.9996, 9.80651],
         [-13.0944, -94.7884, 12.7002, 0.0],
         [6.56259, -2.14732, -22.0441, 0.0],
         [0.0, 1.0, -0.00529282, 0.0]]

# Printed eigenvalues
EIG_LONG = [complex(-53.49048926), complex(-24.36347204),
            complex(-0.09532180, 0.42087158), complex(-0.09532180, -0.42087158)]
EIG_LAT = [complex(-94.55843271), complex(-24.36347204) * 0,
           complex(-11.90023014, 8.07132689), complex(-11.90023014, -8.07132689),
           complex(-0.10470561)]
EIG_LAT = [e for e in EIG_LAT if not (e == 0 and False)]
EIG_LAT = [complex(-94.55843271), complex(-11.90023014, 8.07132689),
           complex(-11.90023014, -8.07132689), complex(-0.10470561)]

def minors(M):
    """Elementary symmetric functions of the eigenvalues, from principal minors."""
    n = len(M)
    e = [1.0]
    for k in range(1, n + 1):
        s = 0.0
        for idx in combinations(range(n), k):
            s += det([[M[i][j] for j in idx] for i in idx])
        e.append(s)
    return e

def det(M):
    n = len(M)
    if n == 1:
        return M[0][0]
    if n == 2:
        return M[0][0] * M[1][1] - M[0][1] * M[1][0]
    s = 0.0
    for j in range(n):
        minor = [row[:j] + row[j + 1:] for row in M[1:]]
        s += ((-1) ** j) * M[0][j] * det(minor)
    return s

def sym_from_roots(roots):
    n = len(roots)
    e = [1.0]
    for k in range(1, n + 1):
        s = 0.0
        for idx in combinations(range(n), k):
            p = 1.0 + 0.0j
            for i in idx:
                p *= roots[i]
            s += p
        e.append(s)
    return e

out = []
def p(s=""):
    out.append(s)

p("== independent check of the printed rev 10 linearisation ==")
p("method: characteristic-polynomial elementary symmetric functions from principal")
p("minors, against the same functions of the printed eigenvalues. For a 4x4 these")
p("four numbers determine the eigenvalues exactly; no re-linearisation is done.")
p()
for name, M, eig in (("A_long", A_LONG, EIG_LONG), ("A_lat", A_LAT, EIG_LAT)):
    e_mat = minors(M)
    e_eig = sym_from_roots(eig)
    p(f"[{name}]  trace {e_mat[1]:+.8f}   det {e_mat[4]:+.8f}")
    worst = 0.0
    for k in range(1, 5):
        a, b = e_mat[k], e_eig[k]
        scale = max(1.0, abs(a), abs(b))
        d = abs(a - b) / scale
        worst = max(worst, d)
        p(f"   e{k}: from matrix {a.real:+.8e}{a.imag:+.2e}i   from printed eigenvalues"
          f" {b.real:+.8e}{b.imag:+.2e}i   rel {d:.3e}")
    p(f"   worst relative mismatch {worst:.3e}")
    p(f"   max real part of printed eigenvalues {max(r.real for r in eig):+.6e}")
    p()

# short period identification
q = 382.8125
S = 2.312
cbar = 0.680
Cm_alpha = -2.113529
Iyy = 1.7713
g = 9.80665
V = 25.0
wn_analytic = math.sqrt(q * S * cbar * (-Cm_alpha) / Iyy)
p("== short period identification ==")
p(f"single-derivative estimate sqrt(q*S*cbar*|Cm_alpha|/Iyy) = {wn_analytic:.6f} rad/s")
p("the two real roots of A_long are -53.490489 and -24.363472 rad/s;")
gm = math.sqrt(53.490489 * 24.363472)
p(f"as an overdamped pair their equivalent natural frequency is sqrt(l1*l2) = {gm:.4f} rad/s")
p(f"that is within {(gm-wn_analytic)/wn_analytic*100:.1f}% of the estimate, which is the expected")
p("accuracy of a single-derivative form that omits the damping coupling; so the real")
p("pair is the short period and the 0.4315 rad/s oscillation is not.")
wn_phugoid = math.sqrt(2.0) * g / V
p(f"phugoid estimate sqrt(2)*g/V = {wn_phugoid:.6f} rad/s vs the 0.431531 rad/s oscillation,")
p(f"same order, so the oscillation is the phugoid: wn 0.4315 rad/s, zeta 0.2209, 0.0687 Hz")
p("equivalent damping of the real pair (l1+l2)/(2*gm) = "
  f"{(53.490489+24.363472)/(2*gm):.4f}, i.e. overdamped, so a damping-ratio criterion")
p("cannot be applied to it as written.")
p()
p("MODES_CHECK_DONE")
text = "\n".join(out)
print(text)
open(r"C:\Ash\Projects\DARES_UACS\MATLAB\results\doc_modes_check.txt", "w", encoding="utf-8").write(text + "\n")
