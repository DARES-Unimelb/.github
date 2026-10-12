# Control law design

For readers who have had no contact at all with flight control. Each section first explains what this layer is doing, then gives the method and the numbers.

The facts in this document fall into three classes, and the wording distinguishes them strictly:

1. Measured values. From `MATLAB\results\gains_baseline.mat`, `MATLAB\results\control_pid_report.txt`, `MATLAB\results\control_lqr_report.txt`, `MATLAB\results\control_allocation_report.txt`, `MATLAB\results\plant_modes_report.txt`. These numbers are the set that was actually flying in this bench test.
2. Source-code facts. Checked line by line against the source, with the source line numbers cited afterwards.
3. Derived values. Numbers recomputed by hand from the source formulas. This task did not allow running MATLAB, so this class is weaker than measured values, and every place where it disagrees with a measured value is listed separately.

## 0. What problem this section solves

For an aircraft to fly as a person intends, someone must continuously do three things: know what the current state is, decide how it should move, and turn that decision into control surface and throttle actions. Control law design is the design of the middle step. This project designs and delivers two control laws at the same time, one is a cascaded PID and one is an LQR. Both run in parallel in the same bench, switched by one switch.

The input to control law design is the linearization of the aircraft model at an operating point, and the output is a gain matrix. The design itself changes neither the aircraft nor the actuators.

## 1. Coordinate and sign conventions

### 1.1 Body frame

The origin of the body frame is fixed to the aircraft, and the three axes rotate with the aircraft. The x axis points forward out of the nose, the y axis points to the right wing, and the z axis points downward out of the belly. The name of this convention is the body frame, also called body axes (source: params/uacs_twin_params.m:13).

The three axes are defined this way because aerodynamic forces and moments are best described in them: the oncoming airflow always arrives from the nose direction relative to the aircraft, lift is along the negative z axis, and drag is along the negative x axis.

The origin is not at the center of gravity, it is taken at the intersection of the wing aerodynamic center and the fuselage centerline. `x_cg` is negative, which means the center of gravity is ahead of the origin (source: params/uacs_twin_params.m:16-17). This must be remembered when doing moment balance, otherwise the center of gravity position will be computed wrongly.

### 1.2 NED and absolute position

NED is the North-East-Down coordinate frame. It does not rotate with the aircraft, it is fixed to the ground. North is x, east is y, and down is z. Down is positive, and this is the point that most easily leads people astray: when the aircraft climbs, the quantity "down position" is getting smaller.

The position of the aircraft is expressed in NED, so the last three of the twelve states are north position, east position and down position. Altitude and down position differ by a minus sign, and the altitude seen from outside equals the negated down position (source: plant/trim_cruise.m:75).

### 1.3 State vector, 12 entries

The order is fixed and defined in `plant/linearize_at.m:73`, and the names are written on the same line.

| Index | Symbol | Meaning | Unit |
|---|---|---|---|
| 1 | u | Body frame forward velocity | m/s |
| 2 | v | Body frame right velocity | m/s |
| 3 | w | Body frame downward velocity | m/s |
| 4 | p | Roll rate | rad/s |
| 5 | q | Pitch rate | rad/s |
| 6 | r | Yaw rate | rad/s |
| 7 | phi | Roll angle | rad |
| 8 | theta | Pitch angle | rad |
| 9 | psi | Yaw angle | rad |
| 10 | pn | North position | m |
| 11 | pe | East position | m |
| 12 | pd | Down position, positive downward | m |

The first three are velocity components, the middle three are angular rates, the next three are attitude angles, and the last three are position. Among the angular rates, p, q and r are the rotation rates about the x, y and z body axes, and the three words roll, pitch and yaw are defined by those three axes. The unit convention is angles in radians, position in meters, attitude in radians, angular rates in radians per second, and velocities in meters per second (source: params/uacs_twin_params.m:14-15).

Of the nine states, only a part actually enters the control law. The three position states are not used in the control law at all, and section 3 explains this clearly later.

### 1.4 Input vector, 6 entries

The order is fixed and defined in `plant/linearize_at.m:74`.

| Index | Symbol | Meaning | Unit |
|---|---|---|---|
| 1 | da | Aileron deflection, controls roll | rad |
| 2 | de | Elevator deflection, controls pitch | rad |
| 3 | dr | Rudder deflection, controls yaw | rad |
| 4 | dth_l | Left throttle | 0 to 1 |
| 5 | dth_r | Right throttle | 0 to 1 |
| 6 | dT | Total thrust increment command | N |

Entry 6 and entries 4 and 5 are physically the same set of actuators. Entry 6 is the total thrust command used by the LQR and the energy loop, and it converts back and forth with the two throttles, so the two cannot be treated as independent degrees of freedom at the same time (source: docs/architecture.md:60).

Throttle is a fraction of peak shaft power, not a fraction of thrust, and thrust follows throttle to the power three quarters (source: params/uacs_twin_params.m:368-371). This must be known when designing the thrust loop, otherwise the thrust loop gain will be understood as if it were a linear loop.

### 1.5 Output vector

Eight words: every state is an output. `lin.C = eye(12)`, `lin.D = zeros(12, 6)` (source: plant/linearize_at.m:69-70). The project defines no separate output selection matrix, whoever wants a state takes that row itself.

### 1.6 Control surface polarity, and why it is easy to get wrong

The polarity convention is written in the parameter file and enforced by assertions inside `init_uacs_twin` (source: params/uacs_twin_params.m:227-233). The three rules are:

1. Positive elevator deflection means trailing edge down, which produces a nose-down moment. This corresponds to `Cm_de < 0` (source: params/init_uacs_twin.m:249).
2. Positive aileron deflection means the right aileron goes down, which produces a rolling moment that drops the right wing. This corresponds to `Cl_da > 0` (source: params/init_uacs_twin.m:250).
3. Positive rudder deflection means trailing edge left, which produces a yawing moment that turns the nose left. This corresponds to `Cn_dr < 0` (source: params/init_uacs_twin.m:251).

Each of the three sentences is reasonable on its own, but taken together they are easy to get wrong, for three reasons.

The first reason is that a positive sign does not mean an intuitive direction such as nose up or roll right. Positive elevator is nose down, positive rudder is yaw left. When the stick is pushed forward, the actual elevator deflection moves in the positive direction.

The second reason is that the signs of the aerodynamic derivatives come from geometry and moment arms, with no uniform rule. Static stability requires `Cm_alpha < 0`, `Cn_beta > 0`, `Cl_p < 0`, `Cn_r < 0`, `Cl_beta < 0`, each with a different origin of its sign, and the four assertions are written one by one in `init_uacs_twin` (source: params/init_uacs_twin.m:127, 239-246).

The third reason is that controller polarity cannot be derived directly from control surface polarity. In the model the error is always defined as command minus measurement, so whether the closed loop is negative feedback or positive feedback depends on the sign of the plant at the frequency where that loop acts. The pitch rate loop is the counterexample: a positive gain splits the originally overdamped short period into an underdamped complex pair (source: control/design_pid.m:44-47). For this reason the polarity in the design is searched for, not assumed (source: control/design_pid.m:314).

The polarity measured in this run is aileron positive, elevator negative, rudder negative. The three are written loop by loop in the PID report (source: results/control_pid_report.txt:9-11). The bench self-check records this item without judging it, with signs aileron positive, elevator negative, rudder negative (source: results/bench_check.txt:41).

### 1.7 Error and limiting conventions

The error is always command minus measurement, consistently across the seven loops (source: control/design_pid.m:208-209). This determines what the positive and negative signs of the gains mean, and with a different error definition every delivered gain sign would have to be flipped.

Attitude angles and attitude commands are always in radians inside the controller. Degree to radian conversion is done only at two boundaries, the parameter file and the stick mapping (source: control/pilot_to_command.m:38-39, params/uacs_twin_params.m:479-480).

## 2. How trim is solved

### 2.1 What problem trim solves

When the aircraft flies level at some speed, the forces and moments in every direction must cancel each other. The lift produced by the wing must exactly equal the weight, the propeller thrust must exactly equal the drag, and the moment produced by the elevator must exactly push back against the pitching moment of the wing. A flight state satisfying these three conditions is called a trim point.

The trim point is the foundation of the whole design. Linearization is done at the trim point, gains are computed at the trim point, and the aircraft in normal flight also stays near it. Get the trim wrong and everything after it is wrong.

### 2.2 Unknowns and constraints

Three unknowns: angle of attack, elevator deflection, throttle. The two motors share one throttle (source: plant/trim_cruise.m:8, 59).

Three constraints: the sum of forces along the body x direction is zero, the sum along z is zero, and the moment about the y axis is zero (source: plant/trim_cruise.m:10).

The remaining quantities are fixed: sideslip angle zero, aileron zero, rudder zero, roll angle zero, and pitch angle equal to angle of attack plus flight path angle. In level flight the flight path angle is zero, so the pitch angle equals the angle of attack (source: plant/trim_cruise.m:9-10).

Trim with bank angle is not implemented in v1, and a nonzero bank angle fails an assertion directly rather than quietly returning an unbalanced point (source: plant/trim_cruise.m:30).

### 2.3 How the residual is computed

The residual is taken from the six-degree-of-freedom equations of motion, not from a separate simplified formula. Forces are divided by weight and moments by dynamic pressure times reference area times mean aerodynamic chord, as a nondimensionalization, and `fsolve` looks only at the three rows sum of forces x, sum of forces z, and pitching moment (source: plant/trim_cruise.m:11-13, 154-158).

The nondimensionalization is there to make the three residuals of the system comparable in magnitude, otherwise the solver would be dominated by whichever equation has the largest magnitude.

### 2.4 Initial values and solver settings

`fsolve` needs an initial value, and the initial value is given by five analytic expressions (source: plant/trim_cruise.m:39-49).

```
CL_guess    = m*g*cos(gamma) / (qbar0*S)
alpha_guess = (CL_guess - CL0) / CL_alpha
de_guess    = -(Cm0 + Cm_alpha*alpha_guess) / Cm_de
CD_guess    = CD0 + k_induced*CL_guess^2 + CD_alpha*(alpha_guess - alpha_zl)^2 + CD_de*|de_guess|
T_guess     = qbar0*S*CD_guess + m*g*sin(gamma)
```

The initial value of the throttle does not use an analytic expression, it is instead inverted from the authoritative propulsion model by bisection. The propulsion model is monotonic in throttle, so the inversion does not need an initial value of its own (source: plant/trim_cruise.m:46-49, 178-200).

The solver settings are six items, written one by one in the source (source: plant/trim_cruise.m:54-57):

```
Algorithm           levenberg-marquardt
Function tolerance  1e-14
Step tolerance      1e-14
Max iterations      200
Max function evals  4000
Display             off
```

When the first solve fails, it restarts once from an angle of attack of 5 degrees, with the throttle clamped between 0.05 and 0.9 (source: plant/trim_cruise.m:61-65).

### 2.5 Convergence criteria

There are four criteria, and all four must hold for the trim to count as successful (source: plant/trim_cruise.m:139-145):

1. The maximum absolute value of the normalized residual is below 1e-6.
2. `exitflag` is greater than zero.
3. The angle of attack falls inside the permitted window. The frozen criterion is 2 to 10 degrees, the recalibrated window is 0 to 8 degrees, and both are computed and both are kept.
4. The absolute value of the elevator deflection does not exceed 15 degrees.

The combined criterion comes in two booleans, one for the frozen version and one for the recalibrated version.

### 2.6 The cruise point solution

Cruise altitude 150 m, cruise speed 25 m/s, flight path angle zero degrees, bank angle zero degrees (source: params/uacs_twin_params.m:101-104).

Measured trim result:

```
Angle of attack        -0.303254 deg
Pitch angle            -0.303254 deg
Elevator deflection    -0.811542 deg
Throttle per motor      0.430065
Max abs normalized residual 1.705e-16
exitflag                4
```

Source: results/plant_trim_report.txt:22-25, results/plant_modes_report.txt:6, results/control_lqr_report.txt:5-6.

The full form of the trim output is two vectors (source: plant/trim_cruise.m:72-76):

```
x_star = [V*cos(alpha); 0; V*sin(alpha); 0; 0; 0; 0; theta; 0; 0; 0; -h_m]
u_star = [0; de; 0; th; th; 0]
```

Altitude is represented by the down-positive down position, hence the negative cruise altitude. Entry 6, the total thrust increment, is always zero in trim.

### 2.7 Why the minimum speed is decided by trim

There are two candidate answers for how slowly the aircraft can fly. One is the stall speed, the speed at which the wing lift coefficient reaches its ceiling. The other is the minimum trimmable speed, the lowest speed that can still be trimmed when the elevator has used up its full travel.

This aircraft takes the larger of the two as its minimum usable speed. The criterion is written in the source: `limits.V_min_mps = max(V_min_stall_mps, V_min_trim_mps)` (source: params/init_uacs_twin.m:200).

The source also explains why trim must win here: at this wing loading the trim requirement reaches its limit first, and stall has not (source: params/uacs_twin_params.m:539-541). That is to say, looking only at the stall speed would promise a speed the aircraft simply cannot trim at.

The three numbers are as follows. The stall speed is computed from weight, density, wing area and maximum lift coefficient, then multiplied by a stall margin factor of 1.20 (source: params/uacs_twin_params.m:550-554). The minimum trimmable speed is bisected out with the real trim solver, with the criterion that the elevator uses 80% of its travel budget (source: params/uacs_twin_params.m:193, params/init_uacs_twin.m:175-192).

```
V_stall        9.408319 m/s
V_min_stall    11.290 m/s   (1.20 times the stall speed)
V_min_trim    15.466467 m/s
V_min         15.466467 m/s (the larger of the two)
V_max         30.0 m/s
```

Source: results/atmo_regress_report.txt:47-52, results/plant_trim_report.txt:157.

The trim solver cannot be replaced by a closed-form linear relation, and the source records the cost: a closed-form pitching moment relation predicts an elevator of -20.0 degrees at 15.41 m/s, while the true trim needs -33.5 degrees, so it underestimates the minimum speed by 2.6 m/s (source: params/init_uacs_twin.m:169-174).

## 3. How the state space equations are obtained

### 3.1 What a state space equation is

A state space equation is a pair of linear equations describing how the state changes over time and how the inputs affect the state:

```
x_dot = A*x + B*u
y     = C*x + D*u
```

x is the state vector, u is the input vector, and x_dot is the derivative of the state. A is the state matrix and B is the input matrix. These two matrices are the entire input to control law design.

Linearization means this: near the trim point the aircraft is nonlinear, and its equations of motion are expanded to first order at the operating point to obtain this pair of linear equations. The closer to the operating point, the more accurate the approximation.

### 3.2 Numerical central differences

This project does not derive an analytic Jacobian, it uses numerical central differences (source: plant/linearize_at.m:51, 57). The method is to add a little to one state and subtract a little from it in turn, and watch how much the output of the equations of motion changes:

```
A(:, i) = (eom_6dof(0, x+h_i*e_i, u0, P) - eom_6dof(0, x-h_i*e_i, u0, P)) / (2*h_i)
B(:, j) = (eom_6dof(0, x0, u+h_j*e_j, P) - eom_6dof(0, x0, u-h_j*e_j, P)) / (2*h_j)
```

The first argument of the equations of motion is time, and here it is always passed as 0, taking the steady point (source: plant/linearize_at.m:51).

Central differences are used rather than forward differences because their error in the step size is second order, so they are more accurate for the same step size.

### 3.3 Step size scaled per component

The default step size is 1e-6 (source: plant/linearize_at.m:28-30). The example in the header comment writes 1e-5, which disagrees with the actual default value, and both coexist (source: plant/linearize_at.m:5). Every call site passes 1e-6 explicitly, and no usage relies on the default (source: control/uacs_job_control_design.m:30).

There are two scaling rules (source: plant/linearize_at.m:41-42):

```
State step  hx(i) = step * max(1, |x0(i)|)
Input step  hu(j) = step * max(1, |u0(j)|)
```

At the trim point, any component with magnitude below 1 receives an actual perturbation of 1e-6 itself, while the velocity components are scaled up by their own magnitude. In this way angles and angular rates both receive an absolute 1e-6 radians or radians per second (source: plant/linearize_at.m:21-22).

The measured step vectors agree with what is computed here: entry 1 of the state step vector is 2.5e-05, entry 12 is 0.00015, and the rest are 1e-06; all six input steps are 1e-06 (source: results/plant_modes_report.txt:8-9).

### 3.4 Number of evaluations

The number of Jacobian evaluations is 2 times 12 plus 6, which equals 36 (source: the loops in plant/linearize_at.m:46-59, the counter field is in plant/linearize_at.m:95). The measurement reports the same 36 (source: results/plant_modes_report.txt:10).

### 3.5 Shapes of the four matrices

A is 12 by 12, B is 12 by 6, C is the 12 by 12 identity matrix, and D is the 12 by 6 zero matrix (source: plant/linearize_at.m:67-70). The outputs are simply all twelve states, with no feedthrough term.

### 3.6 How longitudinal and lateral-directional are split

The full model of twelve states is unnecessary and inconvenient for design. At the trim point the longitudinal motion and the lateral-directional motion are essentially decoupled, so they can be split into two fourth-order submodels and designed separately.

The index sets are hardcoded in `linearize_at.m` and printed at that run, and re-deriving them elsewhere is not allowed (source: plant/linearize_at.m:61-64):

| Submodel | State index | State name | Input index | Input name |
|---|---|---|---|---|
| Longitudinal | 1 3 5 8 | u w q theta | 2 6 | de dT |
| Lateral-directional | 2 4 6 7 | v p r phi | 1 3 | da dr |

The submatrices are taken directly from the elements of A and B, without re-linearizing (source: plant/linearize_at.m:79-83).

This split has a cost: the longitudinal velocity states obtained are the body frame forward velocity u and downward velocity w, not airspeed and angle of attack. Section 3.7 deals specifically with that difference.

There is measured corroboration for the validity of the split: the eigenvalues of the full A against the union of the eigenvalues of the two submodels plus four zeros have a maximum mismatch of 4.263e-14 (source: results/plant_modes_report.txt:41). The four zeros correspond to the four kinematic states yaw angle, north, east and down, which are not coupled to any force.

### 3.7 Contract coordinate transform

`lin_contract` moves the submodels to the frozen ordering (source: control/lin_contract.m:5-7):

```
Longitudinal      [dV dalpha dq dtheta] paired with [dde dT]
Lateral-directional [dbeta dp dr dphi]  paired with [dda ddr]
```

The reason the change is mandatory is written in the source header comment: what `linearize_at` extracts are the body components u, w, v, whereas the Bryson weights are defined in terms of airspeed and angle of attack, so extracting directly would be off by a factor of V0 (source: control/lin_contract.m:8-12).

There are three transform formulas (source: control/lin_contract.m:14-16, implemented at control/lin_contract.m:39-52):

```
dV     =  cos(a0)*du + sin(a0)*dw
dalpha = -sin(a0)*du/V0 + cos(a0)*dw/V0
dbeta  =  dv/V0
```

Here V0 is the magnitude of the three velocity components at the trim point, and a0 is the arctangent of the downward component over the forward component at the trim point (source: control/lin_contract.m:29-30).

Matrix form and how it is applied:

```
T2      = [cos(a0) sin(a0); -sin(a0)/V0 cos(a0)/V0]
T_long  = blkdiag(T2, eye(2))
T_lat   = diag([1/V0 1 1 1])
A_long  = T_long * A_long / T_long
B_long  = T_long * B_long
A_lat   = T_lat  * A_lat  / T_lat
B_lat   = T_lat  * B_lat
```

Source: control/lin_contract.m:39-43, 50-52.

Whether the transform is applied is decided by the state names of the submodel: if w or u appears among the state names it is transformed as longitudinal, if v appears it is transformed as lateral-directional (source: control/lin_contract.m:36-37). When the states are already airspeed states it is skipped and the mode is recorded as identity (source: control/lin_contract.m:22, 25).

This run measured the branch "body components to airspeed and angle of attack, and the sideslip component divided by speed", and the transform matrix together with V0 and a0 is recorded in the report (source: results/control_lqr_report.txt:7-9):

```
T_long = [0.999986 -0.00529275 0 0
          0.00021171 0.0399994 0 0
          0 0 1 0
          0 0 0 1]
V0 = 25.000000 m/s
a0 = -0.005293 rad
```

### 3.8 The linearization point is the trim point

The trim result is used directly as the linearization point, with no further processing (source: plant/linearize_at.m:35-36). So A and B are the Jacobian with the trim point as origin, the origin is an equilibrium point, and in deviation coordinates the feedforward is zero by construction (source: control/design_lqr.m:23-25).

This brings a consequence that must be remembered: the gains work in deviation coordinates, and the operating point must be subtracted before use. In measurement, feeding the absolute trim state directly in as the reference, the unaugmented closed loop settles at a speed of 40.3573 m/s, an angle of attack of -0.0831 rad and a pitch angle of -0.3340 rad, while the intent was 25 m/s and -0.0053 rad (source: results/control_lqr_report.txt:87-94).

### 3.9 Magnitude check of the full aircraft A matrix

The four delivered rows of the longitudinal A matrix are (source: results/control_lqr_report.txt:19):

```
A = [-0.19564    1.9276    -0.084986  -9.8066
     -0.031423  -9.7528     0.88954    2.1138e-12
      0.22367  -718.12   -68.096       0
      0            0        1          0]
```

Row 1, column 4 is -9.8066, exactly the gravitational acceleration negated. Row 4, column 3 is 1, exactly the pitch rate integrated into pitch angle. These two entries are a shape check on the longitudinal dynamics equations, and if they match, the physical meaning of the matrix is sound.

Magnitude check for the lateral-directional part: row 2, column 2 is -94.788, on the order of a hundred, because roll damping is very strong and roll inertia is small; row 3, column 1 is 164.06, on the order of a hundred, because the yawing moment produced by sideslip acts on a very long tail moment arm (source: results/control_lqr_report.txt:40).

## 4. How transfer functions are obtained

### 4.1 One fact to state first

There are no per-channel analytic transfer functions in this project. Across the whole MATLAB directory `zpk(` and `bode(` have zero hits (source: docs/report_sources/lqr_facts.md:500; a spot check back against the source in this task supports that conclusion: none of control/design_lqr.m, control/lqr_reference.m, control/lin_contract.m, control/allocate.m, plant/linearize_at.m, plant/trim_cruise.m, sim/sim_push_design_vars.m, sim/sim_lqr_refmap.m contains `zpk(` or `bode`).

The LQR path only ever manipulates numeric matrices: the four matrices A, B, C and D, plus the mode table derived from the eigenvalues, plus the per-loop margins derived from `allmargin`. The PID path uses `ss` and `tf` to wrap the matrices into objects and then cascades the actuator delay, but it does not write any single channel as a transfer function on paper.

This does not affect design quality, but there are three consequences to know. First, the deliverables contain no transfer function that can be checked by hand, checking can only be done through matrices and poles. Second, the frequency-domain shape of any channel is computed on the spot from the matrices, so a reader of the report does not see the expression. Third, changing the operating point requires re-running the linearization, because the expression does not exist.

### 4.2 How to get a transfer function from A and B yourself

To get the transfer function from one input to one output, three steps.

Step one, choose the state equations. Take the A and B matrices of a submodel, then write one row of output matrix C to pick out the output you want, and take D as zero.

Step two, use `ss` to wrap the four matrices into a state space object.

Step three, use `tf` to convert it to a transfer function, or use `minreal` to cancel pole-zero pairs first and then convert.

The two examples below are both executable and indented by three spaces. The first is longitudinal, elevator to pitch angle; the second is lateral-directional, aileron to roll angle. Both snippets assume `lc` is the return value of `lin_contract(lin)`, that is, the submodels already moved to contract coordinates.

Example one, elevator to pitch angle:

```matlab
   % Longitudinal submodel: states [dV dalpha dq dtheta], inputs [dde dT]
   A = lc.A_long;              % 4x4
   B = lc.B_long;              % 4x2
   % Take state 4 as the output, that is the pitch angle dtheta
   C = [0 0 0 1];
   D = [0 0];
   % Take input 1 only, that is the elevator dde
   Gq_theta = ss(A, B(:, 1), C, D);
   % Convert to a transfer function. This step inverts (sI-A) and multiplies it out
   Gq_theta_tf = tf(Gq_theta);
   % Cancel numerically almost-cancelling pole-zero pairs, tolerance 1e-9, the same one used in the design
   Gq_theta_min = minreal(Gq_theta_tf, 1e-9);
```

What each step does: `ss` merely packs the four matrices into an object and does no computation; `tf` converts it to numerator and denominator polynomials, which means inverting and collecting like terms; `minreal` cancels the zeros and poles in the numerator and denominator that are numerically almost identical, and too loose a tolerance will cancel poles that decide stability, so in this project that number is 1e-9 (source: control/design_pid.m:19, 630).

This is how it is done in the design, except that the designer computes the poles of the closed loop directly rather than writing the open-loop transfer function first (source: control/design_pid.m:104-106).

Example two, aileron to roll angle:

```matlab
   % Lateral-directional submodel: states [dbeta dp dr dphi], inputs [dda ddr]
   A = lc.A_lat;               % 4x4
   B = lc.B_lat;               % 4x2
   % Take state 4 as the output, that is the roll angle dphi
   C = [0 0 0 1];
   D = [0 0];
   % Take input 1 only, that is the aileron dda
   Gp_phi = ss(A, B(:, 1), C, D);
   Gp_phi_min = minreal(tf(Gp_phi), 1e-9);
```

There is one difference between example two and the design that must be stated. In the design the plant of the roll attitude loop is not "aileron directly to roll angle", it is "the already closed roll rate loop with one integrator cascaded after it". The source is written exactly that way: take the roll rate channel first, close it, then cascade a 1 over s (source: control/design_pid.m:104, 132-133; the integrator cascade is implemented at control/design_pid.m:917-922). To see that one, the rate loop must first be closed:

```matlab
   % The closed roll rate loop with a 1/s cascaded after it is the plant the roll attitude loop sees in the design
   Gp   = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * tf(1, [0.045 1]);
   Gphi = feedback(0.760746 * Gp, 1) * tf(1, [1 0]);
```

Here 0.045 is the first-order delay time constant of the aileron servo (source: params/uacs_twin_params.m:130), and 0.760746 is the roll rate loop gain delivered in this run (source: results/control_pid_report.txt:9).

### 4.3 The substitutes actually used in the project

The transfer functions are absent, and there are four substitutes, each answering a different question.

The first substitute, per-loop margins. The method is to break one input at a time while the other loop stays closed (source: control/design_lqr.m:186-189, 194-196):

```
k   = setdiff(1:n_in, j)
A_k = A - B(:, k)*K(k, :)
L_j = ss(A_k, B(:, j), K(j, :), 0)
S   = allmargin(L_j)
```

The question it answers is: by how much more can this channel's gain be amplified before instability, and by how many more degrees can the phase be delayed before instability. In the LQR report the longitudinal elevator path has infinite gain margin, phase margin 107.90 degrees, delay margin 0.00859 s, and crossover frequency 219.181 rad/s; the lateral-directional aileron path has phase margin 87.27 degrees and delay margin 0.66456 s (source: results/control_lqr_report.txt:32-33, 53-54).

Note that the PID side deliberately does not use `allmargin`, for the reason written in the comment: it reports crossovers caused by numerical noise near nearly cancelled zeros (source: control/design_pid.m:996-1000).

The second substitute, closed-loop poles. `eig(A_aug - B_aug*K)` gives all closed-loop poles at once (source: control/design_lqr.m:88, 149). The question it answers is: is the closed loop stable, and what are the fastest and slowest time scales. The measured longitudinal augmented closed-loop poles are -294.096, -8.52176, -0.701065 plus or minus 0.676227i, and -0.997207; the lateral-directional ones are -107.291, -38.9309, -9.89975, and -1.06523 plus or minus 0.449168i (source: results/control_lqr_report.txt:25, 46). All are in the left half plane.

The third substitute, the mode table at the linearization point. Eigenvalues are turned into named modes, each mode carrying natural frequency, damped frequency, damping ratio and time constant (source: plant/linearize_at.m:88-89, 132-164). The question it answers is: which natural motion modes does this aircraft have, how fast is each and how hard is it to damp. The measured modes at the cruise point are as follows (source: results/plant_modes_report.txt:43-59).

| Mode | Plain-language explanation | Real part | Natural frequency | Damping ratio |
|---|---|---|---|---|
| Short period | Oscillation in which the angle of attack quickly returns to equilibrium after a disturbance, usually the fastest | -24.3635 | 24.3635 | 1.0000 (overdamped) |
| Phugoid | Oscillation in which speed and altitude slowly interchange, with a period of seconds to tens of seconds | -0.0953 | 0.4315 | 0.2209 |
| Roll subsidence | Roll rate decays monotonically after a disturbance | -94.5584 | 94.5584 | 1.0000 |
| Dutch roll | Wobble in which sideslip and yaw are coupled | -11.9002 | 14.3792 | 0.8276 |
| Spiral | Slow diverging or converging turning tendency | -0.1047 | 0.1047 | 1.0000 |

The naming of short period and phugoid uses two analytic estimates rather than "take the fastest oscillatory mode", because when the short period is overdamped the old rule would attach the name to the phugoid (source: plant/linearize_at.m:171-189).

The fourth substitute, two comparisons of steady-state error. The closed-loop equilibrium is solved once without integral and once with integral, and where it settles is measured (source: control/design_lqr.m:223-252). In measurement, a 2.0 m/s step in the longitudinal channel settles at 1.225568 m/s without integral, an error of -0.774432 m/s, which is -38.72% of the command; with integral it settles at 2.000000000 m/s, an error of -4.441e-16 (source: results/control_lqr_report.txt:68-69).

### 4.4 Division of labour among the four

What answers "is this gain acceptable" at the design stage is the margins and the closed-loop poles. What answers "what is this aircraft itself like" in the delivery report is the mode table. What answers "can it stop where we want" in acceptance is the steady-state error. The three cannot replace one another: good margins do not imply zero steady-state error, and all poles in the left half plane do not imply it can track.

## 5. How the PID is tuned

### 5.1 Structure of the seven loops

The seven loops are fixed as roll rate, pitch rate, yaw rate, roll angle, pitch angle, airspeed and height, and the order is hardcoded in the code (source: control/design_pid.m:76). The structure has three layers, also hardcoded (source: control/design_pid.m:12-15):

1. The three inner rate loops are pure proportional.
2. The outer roll angle and pitch angle loops are PIDs with a filtered derivative, and their output is a rate command.
3. The outermost airspeed loop outputs a total thrust command, and the height loop outputs a pitch angle command.

The unified control law is `u = Kp*e + Ki*int(e) + Kd*Df(e)`, and the error is always command minus measurement (source: control/design_pid.m:95-96, 208-209). The derivative term is defined as the first-order filtered derivative `Df(e) = Kd*s/(1 + Tf*s) e` (source: control/design_pid.m:207).

The gains actually flying in this test are as follows (source: results/control_pid_report.txt:9-15, digit for digit identical to results/gains_baseline.mat).

| Loop | What it does | Controlled quantity | Actuator | Kp | Ki | Kd | Tf | Limit target |
|---|---|---|---|---|---|---|---|---|
| p | Provides damping for the roll attitude loop, not tracking | Roll rate | Aileron | +0.760746 | 0 | 0 | 0.0100 | surface 0.43633 rad |
| q | Provides damping for the pitch attitude loop | Pitch rate | Elevator | -0.863454 | 0 | 0 | 0.0100 | surface 0.43633 rad |
| r | Provides damping for the yaw channel | Yaw rate | Rudder | -0.896236 | 0 | 0 | 0.0100 | surface 0.43633 rad |
| phi | Turns the roll angle command into a roll rate command | Roll angle | Aileron | +5.938286 | 0 | +0.098971 | 0.0300 | error 0.26180, integral clamp 0.69813 |
| theta | Turns the pitch angle command into a pitch rate command | Pitch angle | Elevator | +4.198453 | +0.748481 | 0 | 0.0100 | error 0.17453, integral clamp 0.55851 |
| V | Drives airspeed to the command with total thrust | Airspeed | Both throttles | +16.959350 | +12.093735 | +30.234337 | 0.0100 | error 5.0, integral clamp 50.0 |
| h | Turns the height command into a pitch angle command | Height | Elevator | +0.016092 | 0 | 0 | 0.0100 | error 20.0, integral clamp 400.0 |

The two limit columns in the table need explaining. The "error" column is the error-domain bound in the parameter file, and its unit varies by loop, radians for the attitude loops, meters per second for airspeed and meters for height (source: params/uacs_twin_params.m:479-480). The "integral clamp" column is the clamp value actually carried by the integrator in the model, with units of radians times seconds, meters per second times seconds, and meters times seconds (source: params/uacs_twin_params.m:501-504). Treating these two columns as the same thing is why the model and the report were once off by a factor of 2.5 (source: control/design_pid.m:54-59).

A control surface travel of 0.43633 rad is 25 degrees (source: params/uacs_twin_params.m:128, 133, 138). The output limit of the height loop is the pitch envelope of 30 degrees, equivalent to 0.523599 rad (source: params/uacs_twin_params.m:563).

### 5.2 How the rate loop gains are set

The rule in one sentence: make the loop magnitude exactly equal to 1 at the target crossover frequency.

The crossover frequency is the frequency at which the loop gain crosses 1, and it roughly determines how fast the closed loop responds. The nominal gain is simply `Kp_nom = 1/|G(j*wc)|` (source: control/design_pid.m:305, 864-866).

There is an important implementation fact here: the crossover frequency targets of the three rate loops are not the three numbers in the parameter file. The parameter file writes roll 12.0, pitch 8.0, yaw 5.0 rad/s (source: params/uacs_twin_params.m:470-472), but the design uses a different quantity, taken from the structural modes of the airframe itself (source: control/design_pid.m:110-112, 128-131).

Structural modes here means the short period and the Dutch roll; the phugoid and the spiral are excluded because they are slower than 1 rad/s and the rate loops are not there to handle them anyway (source: control/design_pid.m:841-848). The rule is to take those eigenvalues of the longitudinal and lateral state matrices whose magnitude is above 1, take the smallest of them, divide by 2, and clamp the result between 10 and 20 rad/s (source: control/design_pid.m:851-861).

Substituting this run's numbers: the lateral-directional eigenvalue magnitudes above 1 are 94.5584 and 14.3792; the longitudinal ones are 53.4905 and 24.3635; the smallest is the Dutch roll at 14.3792, half of it is 7.19, which the lower bound raises to 10.0. In the delivery report the `wc_target` of all three rate loops is 10.00 (source: results/control_pid_report.txt:9-11), consistent with this derivation.

Relation between delivered and nominal gains: in this run `kp_nominal` and `kp_used` are equal loop by loop, p is 0.76075, q is 0.86345 and r is 0.89624, and none of them fell back (source: results/control_pid_report.txt:19-21).

Polarity also needs mention. The polarity is searched for, trying both signs (source: control/design_pid.m:314). In this run the p loop is positive, the q loop is negative and the r loop is negative (source: results/control_pid_report.txt:9-11).

### 5.3 How the attitude loops are fixed by bandwidth separation

The output of an attitude loop is a rate command, so its plant is "the already closed rate loop with one integrator cascaded after it". The source takes the plant exactly that way (source: control/design_pid.m:132-133, 137-138).

The design target is the closed-loop bandwidth, and the four outer loops each take their target from the parameter file: roll attitude 3.0, pitch attitude 2.5, airspeed 0.8, height 0.4 rad/s (source: params/uacs_twin_params.m:473-476).

The bandwidth separation criterion is: the outer loop speed divided by the inner loop speed must be less than 0.5 (source: control/design_pid.m:184-187). The reason is the basic assumption of cascade control, that the outer loop must be slow enough for the inner loop to be treated as instantaneous tracking, otherwise the two loops fight each other.

Measured separation ratios and target values (source: results/control_pid_report.txt:47-53, target values computed by the formulas at control/design_pid.m:179-183):

| Separation ratio | Measured | Target | Verdict |
|---|---|---|---|
| Roll attitude over roll rate | 0.300 | 0.300 | pass |
| Pitch attitude over pitch rate | 1.131 | 0.250 | out of limit |
| Airspeed over pitch rate | 0.438 | 0.080 | pass but close to the cap |
| Height over pitch attitude | 0.213 | 0.160 | pass |
| Height over airspeed | 0.550 | 0.500 | out of limit |

The separation criterion does not pass overall. The report itself states: a ratio greater than or equal to 1 means the outer loop is faster than the inner loop it is supposed to command, this is a deficiency of design margin, not instability, and the eigenvalue criterion still passes (source: results/control_pid_report.txt:54-55). The two out-of-limit entries must be reported as they are, and must not be dropped just because they are not fatal.

### 5.4 Why the rate loops have no integral

The reason is written in the source comment, and it is physics rather than style: pitch rate is the time derivative of pitch angle (source: control/design_pid.m:467-477). A constant nonzero pitch rate means the pitch angle grows without bound, so a stable aircraft can never hold a nonzero steady-state pitch rate, and any correct longitudinal model has a DC gain from elevator to pitch rate equal to zero.

Measurement: `dcgain(Gq)` is exactly 0, and there is a transmission zero at the origin (source: control/design_pid.m:470-471). Adding an integral to such a plant amounts to placing another pole at the origin, which only pushes the loop the wrong way.

The delivered `ki_reason` field records the reason loop by loop; the text for the rate loops is "this channel's plant has a pole at the origin, and adding another origin pole would make the loop unstable, and this loop wants damping, not holding a steady rate" (source: control/design_pid.m:358-361).

The cost is that the pitch rate loop has no steady-state accuracy to speak of. That is not a defect, holding attitude is the job of the attitude loop.

### 5.5 Why the rate-command limiter is mandatory

The limit value is 70% of the full-deflection steady-state rate (source: params/uacs_twin_params.m:517). Two formulas (source: params/uacs_twin_params.m:519-524):

```
Roll limit = rate_cmd_frac * (|Cl_da|*deg2rad(aileron.max_deg)/|Cl_p|) * 2*V/b
Pitch limit = rate_cmd_frac * (|Cm_de|*deg2rad(elevator.max_deg)/|Cm_q|) * 2*V/cbar
```

The physical meaning is: with the aileron or elevator at full deflection, the roll moment and the pitch moment each balance their own damping moment, and the steady-state rate is the term in parentheses. The 70% is taken so that the loop still has margin left to correct (source: params/uacs_twin_params.m:512-516).

The yaw limit does not use either formula above, it takes the smaller of the coordinated turn limit and the yaw rate loop bandwidth target (source: params/uacs_twin_params.m:570-574):

```
r_lim = min(g*tan(bank_max_deg)/V_cruise, bw.rate_r_rad_s)
```

It uses the same formula as the coordinated turn limit the stick itself can command, so the limiter is never tighter than the stick.

The three limit values delivered in this run (source: not listed directly in results/control_pid_report.txt, taken from the parameter file formulas and marked as derived values):

```
p_lim = 0.70 * 0.682235 = 0.477564 rad/s
q_lim = 0.70 * 1.668329 = 1.167831 rad/s
r_lim = min(9.80665*tan(45deg)/25, 5.0) = 0.392266 rad/s
```

The 0.682235 and 1.668329 above are the full-deflection steady-state rates. These two intermediate quantities are taken from docs/report_sources/pid_facts.md:168, and this task could not recompute them digit for digit from the source, so they are marked as unconfirmed.

What happens without this limiter is recorded in the source with a measured cost: with the roll stick thrown fully over once, the roll loop demanded 1.40 rad/s, under sustained error it ran up to 17.73 rad/s, while the airframe can only sustain 1.23 rad/s. The aileron then saturated, sideslip reached 9 degrees, and the aircraft rolled over (source: params/uacs_twin_params.m:506-510).

The linear check records the same thing: previously the limit was ten times the loop bandwidth and never took effect, so a 20 degree roll command would demand 2.07 rad/s while the limit is 0.478 (source: control/pid_cascade_sim.m:56-65). A design check must verify the same set of limits as the model, otherwise it is not verifying the same controller.

The limit lands in the model as three saturation blocks, with the upper and lower variables published from the same field by the design-variable push script (source: sim/sim_build_manual_pid.m:200-205, sim/sim_push_design_vars.m:105-110). The attitude loops must pass through the limiter before they can connect to the rate loops, so that a large attitude error can only demand a rate the airframe can actually deliver, and the reported rate command is also the limited value (source: sim/sim_build_manual_pid.m:257-265).

### 5.6 The yaw rate loop may be delivered open

This item must distinguish "what is in the delivered cache" from "what a rerun now would produce".

In the delivered cache the yaw rate loop is closed, with Kp equal to -0.896236 and a negative sign (source: results/control_pid_report.txt:11). Its margins are a gain margin of 2.507 times, equivalent to 7.98 dB, a phase margin of 95.6 degrees, and a crossover frequency of 0.196 rad/s (source: results/control_pid_report.txt:35). This is a usable loop.

The current `control/design_pid.m` in the repository has evolved. Its comment states that the yaw rate channel will be delivered open, with Kp set to zero and designed set to false (source: control/design_pid.m:223-237). The physical reason given is: the transfer function from rudder to yaw rate has a right half plane zero at +0.819 plus or minus 0.463i, whose magnitude peaks at 2.31 as frequency tends to zero, the loop crosses the unit circle twice, and the subsequent phase crossover makes any gain greater than 0.433 unstable, while below it the loop never crosses 1, so adding it changes nothing (source: control/design_pid.m:226-231).

The relation between the two sentences needs to be clear. The first sentence is the comment's description of the nature of the channel, and the second sentence, "any gain greater than 0.433 is unstable", is a hard criterion. The delivered 0.896236 is greater than 0.433. These two things cannot both hold.

Incidentally, another note paraphrases the same comment as "any gain greater than 0.433 is unstable, and below it the loop never crosses 1" (source: docs/report_sources/pid_facts.md:141). Checking back against the source in this task, the comment text is indeed written that way. So the contradiction is not in the paraphrase, it is that the criterion in the comment conflicts with the delivered gain itself.

The same comment also gives another number for the yaw loop: `|Gr|` peaks at 2.31 as frequency tends to zero, and the loop crosses the unit circle twice.

There is one more related record: the previous generation delivered a pitch rate loop gain of 0.600, and that channel also measured 66% pitch overshoot, which the comment attributes to the gain having crossed the stability boundary (source: control/design_pid.m:301-303). The q loop gain delivered in this run is -0.863454, whose absolute value is likewise greater than 0.433.

There is a counterargument here that can be computed directly, using the delivery report's own numbers: the gain margin of the yaw rate loop is 2.507 times, equivalent to 7.98 dB (source: results/control_pid_report.txt:35). The definition of gain margin is exactly how much more the loop can be amplified before instability, so the real boundary of this delivered version is 0.896236 times 2.507, approximately 2.247, not 0.433. In other words, by the report's own margin reading, the 0.433 boundary does not hold on the delivered aircraft.

The conclusion is as follows, stated plainly:

1. What was flying in this run is seven closed loops, with the yaw rate loop Kp equal to -0.896236; this has two pieces of evidence, the report and the delivered cache, and it is corroborated by a gain margin of 7.98 dB.
2. The current source would take the open branch, with delivered Kp equal to zero.
3. The 0.433 boundary given by the source comment and the 2.507 times gain margin in the delivery report contradict each other. By the margin, the boundary is about 2.247.
4. On what plant and under what conditions the number 0.433 holds cannot be decided in this task, it is marked unconfirmed and needs manual review. The aerodynamic derivative inconsistency listed in section 9.2 may be one clue, but this task cannot prove it.

### 5.7 How the height loop and the pitch clamp are cascaded

The output of the height loop is not the elevator, it is a pitch angle command (source: control/design_pid.m:206). The pitch angle command passes through a saturation first, then enters the pitch attitude loop (source: sim/sim_build_manual_pid.m:268, 174-175).

The cascade order is:

```
height error -> height loop PID -> pitch angle command -> saturation to plus or minus 30 degrees -> pitch attitude loop -> pitch rate command
```

The upper and lower variables of the saturation are `sim_pid_h_lim` and `sim_pid_h_lim_neg`, taken from the pitch envelope of 30 degrees converted to radians (source: sim/sim_push_design_vars.m:102-103).

The plant of the height loop is "the already closed pitch angle loop multiplied by the cruise speed, with one integrator cascaded after it" (source: control/design_pid.m:147-154). The source states explicitly that the angle of attack term in `h_dot = V0*(theta - alpha)` is deliberately omitted and covered by the closed-loop longitudinal check in the design job (source: control/design_pid.m:147-149). This is a known simplification, not an oversight.

The delivered height loop gain is Kp equal to +0.016092 and Ki equal to 0 (source: results/control_pid_report.txt:15). The reason the integral gain is zero differs from the rate loops; the criterion in the source is: no integral gain reduces the weighted cost, so this loop carries no integral (source: control/design_pid.m:451-453). So the height loop degenerates in practice to pure proportional, and the height steady-state error is removed by the integral term of the pitch attitude loop itself.

### 5.8 Why the derivative term is a first-order filtered derivative

Pure differentiation is physically unrealizable, because it has no upper bound on amplifying high-frequency noise. What is used is `Kd*s/(1 + Tf*s)`, one zero in the numerator and one pole in the denominator, and at high frequency the magnitude is capped at `Kd/Tf` (source: control/design_pid.m:207).

The baseline value of the filter time constant is 0.010 s (source: params/uacs_twin_params.m:477). During the outer loop search it is scaled by a multiplier, and the multiplier takes the value 1 or 3, with the delivered field storing the scaled value (source: control/design_pid.m:272, 388). In this run the measured Tf of the roll angle loop is 0.0300 s, that is, a multiplier of 3 was taken; the other six loops are all 0.0100 s (source: results/control_pid_report.txt:9-15).

The source gives one constraint: the product of the multiplier and the baseline value must not be below the outer loop sample time of 0.010 s, otherwise the derivative would be unrealizable on the sampled loop (source: control/design_pid.m:375-377). Note that the source has only this comment, with no corresponding assertion or truncation (source: docs/report_sources/pid_facts.md:177, and the check back against the source in this task agrees).

The discretization method is the Tustin substitution `s = (2/Ts)*(1 - z^-1)/(1 + z^-1)` (source: sim/sim_pid_coeffs.m:11). The discrete coefficients are closed form (source: sim/sim_pid_coeffs.m:12, 31-34):

```
D(z) = [2*Kd/Ts, -2*Kd/Ts] / [1 + 2*Tf/Ts, -1 + 2*Tf/Ts]
both vectors are normalized by the leading denominator term
```

For the rate loops Kd equals zero, and in that case the numerator degenerates to a scalar zero and the denominator to one, so the block is a pure zero rather than two taps cancelling each other (source: sim/sim_pid_coeffs.m:16-17, 26-30).

In the model the derivative term is a discrete transfer function block, whose sample time is the sample time of the loop it sits in (source: sim/sim_add_pid.m:51-52). The sample time assignment rule is 0.0025 s for the three rate loops and 0.010 s for the four outer loops (source: sim/sim_push_design_vars.m:31-33, params/uacs_twin_params.m:458-459).

There is one inconsistency here that must be stated plainly: in the linear check script `pid_cascade_sim.m` the derivative filter uses backward Euler, with the state update written as `xf = xf + (Ts/Tf)*(e - xf)` and the derivative branch contributing `Kd*(e - xf)/Tf` (source: control/pid_cascade_sim.m:184-187, 147, 159, 162). The model uses Tustin (source: sim/sim_pid_coeffs.m:11). The two are not the same discretization, so the linear check and the model differ in the discretization convention on the derivative term.

### 5.9 How anti-windup is actually implemented in the model

The problem first. What the integrator does is accumulate past errors. If the actuator is already at its stop and the error persists, the integrator keeps accumulating, up to a very large value. When the error changes sign, the integrator must first discharge that large lump before the loop moves back, which shows up as a long overshoot or sluggishness. This phenomenon is called integral windup.

The project has two mechanisms, with different roles.

Mechanism one, back-calculation. The linear check script implements this one. The method scales the actuator saturation error back to this loop through the downstream loop gain and adds it to the integrator increment (source: control/pid_cascade_sim.m:190-193, 195). The four scaling coefficients are (source: control/pid_cascade_sim.m:95-98):

```
Kaw_th  = 1/(Kq*Ktheta)
Kaw_h   = 1/(Kq*Kh)
Kaw_phi = 1/(Kp_gain*Kphi)
Kaw_V   = 1/KV
```

The back-calculation gain Kb is taken from the parameter file with a value of 1.0 (source: control/design_pid.m:84, params/uacs_twin_params.m:478). The four integrator update equations (source: control/pid_cascade_sim.m:195, 198, 202, 206):

```
i_th  = i_th  + Ts*(e_th  + Kb*Kaw_th *(de_sat - de_cmd))
i_V   = i_V   + Ts*(e_V   + Kb*Kaw_V  *(T_cmd  - T_cmd_raw))
i_h   = i_h   + Ts*(e_h   + Kb*Kaw_h  *(theta_c - theta_c_raw))
i_phi = i_phi + Ts*(e_phi + Kb*Kaw_phi*(da_sat - da_cmd))
```

The sign of the correction comes from the sign of the downstream loop gain, so the correction is always opposite to the error that caused the saturation (source: control/pid_cascade_sim.m:192-193).

Mechanism two, integrator output limiting. The Simulink model takes this one. Inside the PID subsystem the integral branch is a discrete-time integrator with output limiting enabled, and the upper and lower variables are `sim_pid_<loop>_Isat` and `sim_pid_<loop>_Isat_neg` (source: sim/sim_add_pid.m:46-50).

The difference must be stated plainly: the model does not implement back-calculation. `sim_add_pid.m` says so itself, the `gains.<loop>.Kb` in the design is a back-calculation gain while the model uses output limiting, and the two differ only in how fast the integrator backs off during saturation (source: sim/sim_add_pid.m:29-32). In other words, the delivered Kb field is not used in the model.

The integral clamp values the model uses come from the state-domain fields, and the priority for taking a value is to look up this loop's entry in `P.ctrl.int_state_limit` first, fall back to the rate loop control surface travel if absent, and fall back to the airspeed loop state clamp if still absent (source: sim/sim_push_design_vars.m:91-97). The clamp value also passes through a 1e-12 floor, because the discrete integrator requires a strictly positive gain and a positive upper limit (source: sim/sim_push_design_vars.m:73-79, 98). In this run the four state clamps measured are roll 0.69813, pitch 0.55851, airspeed 50.0 and height 400.0 (source: results/control_pid_report.txt:12-15).

The clamp is positioned as a safety net, while back-calculation is the anti-windup mechanism, and the purpose of `int_state_margin = 8.0` is to keep the clamp out of the way of back-calculation (source: params/uacs_twin_params.m:489-499). The source records the cost of setting margin to 1: under a 30 degree pitch command the achievable pitch drops from 28.47 degrees to 26.79 degrees, the settling time rises from 4.768 s to 5.583 s, and the anti-windup on/off comparison prints identical to the last digit, so the comparison itself becomes meaningless (source: params/uacs_twin_params.m:489-495).

The V6 criteria therefore include one extra check: the fraction of time the clamp is active over the whole run must be below 10%, otherwise the clamp becomes the anti-windup itself (source: control/uacs_job_control_design.m:156, 160). In this run the clamp hit fraction is 0.00% in both the pitch and the airspeed saturation cases (source: results/control_pid_report.txt:68, 71).

### 5.10 How gains fall back when stability is constrained

The fallback is triggered when the stability criterion fails. The criterion itself is that the closed-loop poles lie strictly in the left half plane, `ok = max(real(p)) < -tol_axis`, with a default tolerance of 1e-7 (source: control/design_pid.m:635, 67-68).

The closed-loop poles are computed with `pole(feedback(C*G,1))`, after a reduction with tolerance 1e-9 (source: control/design_pid.m:630-631). Why the tolerance must be 1e-9: 1e-7 would cancel the real closed-loop pole of the pitch rate loop at the origin, so the criterion reports stable, while computing the feedback without reduction reports plus 4.06 (source: control/design_pid.m:620-625).

The fallback form for the rate loops is not a step-by-step reduction, instead it searches for the minimum-cost point on a geometric grid, with a grid spanning one hundredth to one thousand times the nominal gain in 161 points, plus one 11-point refinement (source: control/design_pid.m:310, 332-333). The cost function contains a control effort penalty, whose role is to make the delivered gain land on "the smallest gain that satisfies damping and margins" (source: control/design_pid.m:321-326).

The hard gate for the rate loops is a vector margin of not less than 0.60, and failing it assigns a cost of 1000 directly (source: control/design_pid.m:517-520). The vector margin is defined as `min|1 + C*G(jw)|`, that is, the shortest distance from the Nyquist locus of the loop to the critical point, evaluated on a shared frequency grid (source: control/design_pid.m:909-915). The basis for calibrating this gate is written in the comment: the roll loop measures 0.9139, the pitch loop 0.8237 and the yaw loop 0.5193, and 0.60 falls between 0.53 and 0.82, which separates the channels that pass from those that do not (source: control/design_pid.m:511-516).

When the candidate set is empty it returns a nominal gain of NaN and a cost of infinity, without throwing (source: control/design_pid.m:345-354). In that case the loop is delivered open, with Kp equal to zero, designed false and sign one (source: control/design_pid.m:233-237). So "no stable gain found" has a defined exit and does not drag the other six loops down with it.

The outer loops use a different fallback criterion, an assertion: if no stable solution is found it throws `design_pid:noStableTuning` directly (source: control/design_pid.m:441-442). The `gain_reduced` field of the outer loops is hardcoded to false (source: control/design_pid.m:268), but `kp_scale` and `int_reduced` are computed as they really are. The criterion for the integral fallback is `Ki < Kp*frac_nominal*wc`, with the nominal integral zero ratio fixed at 0.5 (source: control/design_pid.m:270, 449).

The two nominal integral zero ratio constants are: 0.5 when the plant has a pole at the origin, and 0.2 when it does not (source: control/design_pid.m:97).

The table in the report showing "where the search gave ground" has seven columns: kp_nominal, kp_used, scale, zf_nom, zf_used, gain_red, int_red (source: control/uacs_job_control_design.m:490-503). In this run all seven scale values are empty, meaning no loop fell back (source: results/control_pid_report.txt:17-25). The table's note explains what a scale below 1 means: the roll attitude gain is reduced to keep the spiral pole in the left half plane, because this loop cancels the dihedral effect that the spiral mode depends on (source: control/uacs_job_control_design.m:504-507).

### 5.11 Delivered cache gains against the current source

It must be stated clearly: the current `control/design_pid.m` in the repository has evolved, and re-running the design once is not the same thing as the delivered cache. There are at least three differences.

The first difference, the yaw rate loop. The current source would take the open branch with Kp equal to zero on that channel (source: control/design_pid.m:223-237, 345-354). The delivered cache is closed at -0.896236 (source: results/control_pid_report.txt:11).

The second difference, the crossover frequency target of the rate loops. The current source derives it from the airframe structural modes, and the substitution in this run gives 10.0 rad/s (source: control/design_pid.m:851-861). The `wc_target` in the delivery report is also 10.00 (source: results/control_pid_report.txt:9-11), so this item is consistent.

The third difference, the aerodynamic derivatives in the parameter file do not match the delivered linearization. This has little to do with the PID, but it affects any rerun, see the comparison table in sections 3 and 9.4.

Conclusion: to cite the gains that were actually flying at design time, use the delivered cache and the two reports. To predict what a rerun would produce, the three differences above must first be stated, and the two must not be mixed.

## 6. How the LQR is tuned

### 6.1 What problem the LQR solves

LQR stands for linear quadratic regulator. It writes "how small do you want the state deviations to be" and "how much control effort are you willing to spend" as two tradeoff matrices, and then solves once for a globally optimal feedback gain.

The difference from the PID is in the design method. The PID is designed loop by loop, a person fixes the structure first and then tunes the parameters. The LQR designs a whole submodel in one go, the cost function contains states and control inputs at the same time, and the resulting gain automatically carries cross terms.

The cost function is the time integral of the weighted square of the state deviation plus the weighted square of the control input. Q is the weight matrix of the states and R is the weight matrix of the control inputs.

### 6.2 The Bryson choice of Q and R

The Bryson rule in one sentence: each diagonal entry is "the inverse of the square of the maximum acceptable deviation for that quantity" (source: params/uacs_twin_params.m:530-531). The meaning is "how much counts as one unit of deviation". When the deviation reaches that magnitude, the cost is one.

The original definitions of the four weights (source: params/uacs_twin_params.m:532-535):

```
lqr.long.q_diag = [1/2.0^2, 1/deg2rad(5)^2, 1/deg2rad(15)^2, 1/deg2rad(10)^2]
lqr.long.r_diag = [1/deg2rad(20)^2, 1/0.30^2]
lqr.lat.q_diag  = [1/deg2rad(5)^2, 1/deg2rad(60)^2, 1/deg2rad(30)^2, 1/deg2rad(20)^2]
lqr.lat.r_diag  = [1/deg2rad(20)^2, 1/deg2rad(20)^2]
```

Translated item by item into physical meaning, unit and value. The value column is recomputed from the formulas and is digit for digit identical to the delivery report (source: results/control_lqr_report.txt:12-15).

Longitudinal Q:

| Index | State | Physical meaning | Unit | Tolerance meaning | Value |
|---|---|---|---|---|---|
| 1 | dV | Airspeed deviation | m/s | 2.0 m/s counts as one unit of deviation | 0.25 |
| 2 | dalpha | Angle of attack deviation | rad | 5 degrees counts as one unit of deviation | 131.312 |
| 3 | dq | Pitch rate | rad/s | 15 degrees per second counts as one unit of deviation | 14.5903 |
| 4 | dtheta | Pitch angle | rad | 10 degrees counts as one unit of deviation | 32.8281 |

Longitudinal R:

| Index | Input | Physical meaning | Unit | Tolerance meaning | Value |
|---|---|---|---|---|---|
| 1 | dde | Elevator increment | rad | 20 degrees counts as one unit of control | 8.20702 |
| 2 | dT | Total thrust increment | N | 0.30 N counts as one unit of control | 11.1111 |

Lateral-directional Q:

| Index | State | Physical meaning | Unit | Tolerance meaning | Value |
|---|---|---|---|---|---|
| 1 | dbeta | Sideslip angle deviation | rad | 5 degrees counts as one unit of deviation | 131.312 |
| 2 | dp | Roll rate | rad/s | 60 degrees per second counts as one unit of deviation | 0.911891 |
| 3 | dr | Yaw rate | rad/s | 30 degrees per second counts as one unit of deviation | 3.64756 |
| 4 | dphi | Roll angle | rad | 20 degrees counts as one unit of deviation | 8.20702 |

Lateral-directional R:

| Index | Input | Physical meaning | Unit | Tolerance meaning | Value |
|---|---|---|---|---|---|
| 1 | dda | Aileron increment | rad | 20 degrees counts as one unit of control | 8.20702 |
| 2 | ddr | Rudder increment | rad | 20 degrees counts as one unit of control | 8.20702 |

Three details.

First detail, the thrust tolerance is written as an absolute quantity of 0.30 N, not an angle (source: params/uacs_twin_params.m:533). This term has different dimensions from the others and must not be read as an angle.

Second detail, aileron and rudder have equal weight in the lateral-directional part, both 20 degrees (source: params/uacs_twin_params.m:535).

Third detail, the roll rate tolerance is set very loose, 60 degrees per second only counts as one unit of deviation, which amounts to telling the design "rolling a bit faster is fine". This explains why the aileron channel gain is the largest of the four channels.

Any change to the weights must record the value before, the value after and the reason, written in the design description (source: control/design_lqr.m:20-21). The two fields `weights_unchanged` and `weights_source` are returned together with the gains, stating that the weights come from the parameter file (source: control/design_lqr.m:168-169). This design did not change the weights.

### 6.3 How the integral augmentation is done

With state feedback alone, the closed loop leaves a steady-state error for a step command. The reason is that state feedback can only drive the states to zero, not drive the output to the command. Adding one integral state removes the steady-state error.

The method is to treat the integral of the regulated output as a new state and extend the system by one order (source: control/design_lqr.m:57-58, and for the lateral-directional part control/design_lqr.m:119-120):

```
A_aug = [A, zeros(4,1); C_int, 0]     5x5
B_aug = [B; zeros(1,2)]               5x2
```

`C_int` is a selection row with one in the column of the regulated output and zeros elsewhere (source: control/design_lqr.m:54-55, and for the lateral-directional part control/design_lqr.m:116-117).

The regulated output can be switched. The longitudinal default is airspeed, that is index 1, and it can be changed to pitch angle, index 4 (source: control/design_lqr.m:33-35, 9). The lateral-directional default is roll angle, index 4, and it can be changed to sideslip angle, index 1 (source: control/design_lqr.m:36-38, 11). An illegal name fails an assertion (source: control/design_lqr.m:175).

The dimensions change accordingly: the gain goes from 2 by 4 to 2 by 5 (source: control/design_lqr.m:62-63, 124-125).

The weight of the integral state takes the Bryson tolerance of the state it regulates (source: control/design_lqr.m:52-53, 114-115):

```
q_int_long = q_diag(1) = 0.25     (longitudinal regulates airspeed)
q_int_lat  = q_diag(4) = 8.207016 (lateral-directional regulates roll angle)
```

The measured values agree with the expressions above (source: results/control_lqr_report.txt:16-17). The reasoning is: the integral state has the dimension of "regulated quantity times time", and weighting it by the tolerance of the regulated quantity itself amounts to saying "how long a small error is allowed".

The augmented Q is a block diagonal matrix (source: control/design_lqr.m:59, 121):

```
Q_aug_long = diag([0.25 131.312 14.5903 32.8281 0.25])
Q_aug_lat  = diag([131.312 0.911891 3.64756 8.20702 8.20702])
R_long     = diag([8.20702 11.1111])
R_lat      = diag([8.20702 8.20702])
```

The measurement is digit for digit identical to the expressions above (source: results/control_lqr_report.txt:21-22, 42-43).

The limiting of the integral state in the model does not use the state-domain bounds above, it takes the error-domain bounds of the parameter file directly: longitudinally the airspeed value of 5.0 m/s, laterally the roll value of 15 degrees equivalent to 0.261799 rad (source: sim/sim_push_design_vars.m:137-140, with the parameters in params/uacs_twin_params.m:479-480). In the model these two bounds are connected to the upper and lower saturation limits of the integrator, with a gain of one and a sample time of the controller period (source: sim/sim_bench_lqr.m:90-93, 131-134).

There is a difference in convention from the PID side to point out here: the integral clamp used by the PID model is the state-domain field `int_state_limit`, while the LQR integral limit uses the error-domain field `int_limit`. The two sets of bounds are not the same quantity (source: sim/sim_push_design_vars.m:91-97, 137-140).

The limiting of the integral state is not in the design function, it is in the parameter file and the push script (source: docs/report_sources/lqr_facts.md:211; the check back against the source in this task read all 256 lines of `control/design_lqr.m` and confirmed there is no limiting statement).

### 6.4 The complete form of the control law

Four expressions (source: control/design_lqr.m:105-106, 163-164):

```
u_dev = u_ff + K*([x_ref; 0] - [x_dev; z])
x_dev = x_abs - x_op
z     = integral(C_int*x_dev - r)
u_abs(u_idx_full) = u_star_full(u_idx_full) + u_dev
```

Item by item.

`x_abs` is the absolute state and `x_op` is the absolute operating point of the submodel. Longitudinally these are trim speed, trim angle of attack, trim pitch rate and trim pitch angle; laterally they are zero sideslip, trim roll rate, trim yaw rate and trim roll angle (source: control/lin_contract.m:64-65). The measured longitudinal operating point is `[25 -0.00529277 0 -0.00529277]`, and the lateral-directional one is `[0 0 0 0]` (source: results/control_lqr_report.txt:28, 49).

`u_idx_full` refers to the positions of the two control inputs within the six-dimensional plant input. Longitudinally they are 2 and 6, that is the elevator and the total thrust; laterally they are 1 and 3, that is the aileron and the rudder (source: control/design_lqr.m:101, 160).

`u_star_full` is the six-dimensional absolute trim input (source: control/design_lqr.m:102, with the source field at control/lin_contract.m:67). The two measured longitudinal entries are an elevator of -0.0141641 rad and a total thrust of 0 N (source: results/control_lqr_report.txt:29).

`u_ff` and `x_ref` are solved together from the same equation (source: control/lqr_reference.m:11-13):

```
A*x_ref + B*u_ff = 0
```

The meaning of this equation is: the reference point must be an equilibrium point of the design model itself. If the reference point is not an equilibrium point, the closed loop settles somewhere else and the command does not match reality.

Longitudinally the unknowns are the angle of attack deviation, the pitch rate deviation, the elevator increment and the total thrust increment, and the known commands are the airspeed deviation and the pitch angle deviation (source: control/lqr_reference.m:33-38):

```
Ml = [A(:,2), A(:,3), B(:,1), B(:,2)]       4x4
rl = -(A(:,1)*dV + A(:,4)*dth)
yl = Ml \ rl
x_ref_l = [dV; yl(1); yl(2); dth]
u_ff_l  = [yl(3); yl(4)]
```

Laterally the unknowns are the roll rate deviation, the yaw rate deviation, the aileron increment and the rudder increment, and the known commands are the sideslip angle deviation and the roll angle deviation (source: control/lqr_reference.m:46-50).

The reachability criterion is that the norm of the residual is below 1e-9 (source: control/lqr_reference.m:20-22, 84-87). The measured longitudinal residual is 8.882e-16 and the lateral-directional one is 2.778e-17 (source: results/control_lqr_report.txt:85).

Substituting the design point itself gives a reference state of zero and a feedforward of zero, because the Jacobian is taken at the trim point and in deviation coordinates the feedforward is zero by construction. The only nonzero feedforward is the absolute actuator quantity (source: control/lqr_reference.m:15-18, with measured results in results/control_lqr_report.txt:77-78).

### 6.5 What equation the feedforward and reference map solve

The reference map is not another equation. It pre-solves the linear relation of `A*x_ref + B*u_ff = 0` with respect to the command into constant matrices, so that no solver has to be placed in the model (source: sim/sim_lqr_refmap.m:11-16).

Why it can be pre-computed as constant matrices: that equation is linear in the command, so solving it once for each of four unit commands is enough to assemble the whole map (source: sim/sim_lqr_refmap.m:20-33).

```
R.long.Tref  4x2  input [dV_c; dtheta_c], output longitudinal reference state
R.long.Uff   2x2  same input, output longitudinal feedforward [dde; dT]
R.lat.Tref   4x2  input [dbeta_c; dphi_c], output lateral-directional reference state
R.lat.Uff    2x2  same input, output lateral-directional feedforward [dda; ddr]
```

The linearity self-check recomputes on a non-unit command and compares item by item against the matrix map result (source: sim/sim_lqr_refmap.m:35-48). The four matrices are pushed into the base workspace under the names `sim_lqr_Tref_long`, `sim_lqr_Uff_long`, `sim_lqr_Tref_lat`, `sim_lqr_Uff_lat` (source: sim/sim_push_design_vars.m:142-146), and are restored in the model with matrix-multiply gain blocks (source: sim/sim_bench_lqr.m:78-81, 119-122).

When `lqr_reference` is not on the path, the reference map script solves the same equations itself (source: sim/sim_lqr_refmap.m:65-83).

### 6.6 Delivered gains and design results

The longitudinal gain is 2 by 5 (source: results/control_lqr_report.txt:23):

```
K_long = [ 0.34062952   1.7888566   -1.0585723   -4.039148    0.17449358
           0.00553504   0.0066485   -0.000349222 -0.02859383  0.00318499]
```

The lateral-directional gain is 2 by 5 (source: results/control_lqr_report.txt:44):

```
K_lat  = [-0.34598738   0.095423189 -0.049195998  1.5577267    0.9717582
           2.7803249    0.013674918 -0.39039585  -0.21081883  -0.2359788]
```

There are four self-check items, and all match item by item:

1. Against the `care` solution, the maximum difference is 2.487e-14 longitudinally and 2.109e-15 laterally, with a tolerance of 1e-6 (source: results/control_lqr_report.txt:27, 48, 64).
2. Riccati residual norm, 1.362e-12 longitudinally and 1.164e-13 laterally (source: results/control_lqr_report.txt:65).
3. All closed-loop poles are in the left half plane, the rightmost being -0.701065 longitudinally and -1.06523 laterally (source: results/control_lqr_report.txt:25, 46).
4. Without integral there is a steady-state error, with integral it is removed, measured at -0.774432 m/s and -4.441e-16 m/s longitudinally (source: results/control_lqr_report.txt:68-69).

### 6.7 Why longitudinal and lateral are designed separately

Three reasons.

First, the cost functions are written separately. Longitudinally we care about airspeed and pitch, laterally we care about sideslip and roll, the two groups have different tolerances, and merging them into one Q only mixes the two things together.

Second, computation. Solving the 12th-order system in one go needs a 12 by 12 Riccati equation; splitting into two 4th-order problems makes the matrix work an order of magnitude smaller, and it allows the regulated output to be switched separately.

Third, the decoupling premise has already been verified. The measured evidence for longitudinal-lateral decoupling at the trim point is a maximum mismatch of 4.263e-14 between the full-set eigenvalues and the union of the two submodel eigenvalues (source: results/plant_modes_report.txt:41).

How the coupling terms are handled, the answer is: on the LQR path they are not handled. At the source level there is no cross feedforward and no decoupler. The cost of the coupling is marked by measured numbers: a 20 degree roll command produces a peak sideslip of 4.225 degrees and a peak yaw rate of 8.742 degrees per second; a 20 m height command produces an angle of attack excursion of 4.477 degrees and a speed loss of 2.175 m/s; a 3 m/s speed command produces a height excursion of 8.288 m (source: results/control_pid_report.txt:77-79). These are measured couplings of the PID-side linear closed loop, and the LQR side has no corresponding measured entries.

The LQR path also has two known gaps. First, there is no height loop in the LQR channel (source: sim/sim_bench_lqr.m:25-26), so height mode has no effect on it. Second, the yaw channel has no outer loop on the PID side either, the rudder is left on the pilot stick, and coordinated turn is not implemented (source: results/control_pid_report.txt:80-81).

### 6.8 How the two control laws are switched

In the bench the PID and the LQR are two parallel subsystems, each running on its own, with their outputs entering a selection switch (source: sim/sim_bench_select.m:4-8). The two have the same output interface, both a four-dimensional vector in the order aileron increment, elevator increment, rudder increment, total thrust increment (source: sim/sim_bench_lqr.m:6, 160-166).

The criterion of the selection switch is that the second input is not equal to zero, so port 1 takes the LQR and port 3 takes the PID, the selection quantity is a constant, and the controller knob on the canvas is bound to it (source: sim/sim_bench_select.m:10-12, 22-23, 28).

The switch needs no recompile, and the only reason is: both controllers run continuously, and switching is only a choice of signal path, changing no block parameter and no model structure (source: sim/sim_bench_select.m:6-8).

The source also writes the cost of doing it this way: the integral states of both controllers keep running, so the saturation history from before the switch is carried over (source: sim/sim_bench_select.m:8). The benefit is that the integrator of the one switched to has already caught up with the error and will not, at the instant of switching, issue a large command that takes a long time to recover from.

### 6.9 A coincidence that can be cross-checked

The two delivered controllers are numerically close at the rate level, and this is not a design convention, the two were designed independently.

```
Roll rate loop: LQR K_lat(1,1) = -0.34599, PID p loop gain = +0.760746
                different structures, values not equivalent, see the explanation below
Yaw rate loop:  LQR K_lat(2,3) = -0.3904,  PID r loop gain = -0.896236
```

Source: results/control_lqr_report.txt:44, results/control_pid_report.txt:9-11.

These two sets of numbers cannot be compared directly in magnitude, because the LQR feeds back a single state while the PID feeds back an error. The two are equivalent only under the following specific conditions: the plant is genuinely decoupled under the action of the controller, and the time scales of the inner and outer loops are separated widely enough. On this aircraft the measured coupling and separation ratios are both unsatisfactory (see sections 5.3 and 6.7), so this equivalence holds only partly.

The part that holds is worth recording: the LQR yaw rate feedback term and the PID yaw rate loop gain are both negative, and both are on the order of 0.4 to 0.9. Two independent methods agreeing on polarity is a piece of corroboration for the polarity.

## 7. How the control surfaces are computed

### 7.1 The complete control allocation chain

What the control law computes is not control surface angles, it is moment commands for three axes and one total thrust command. There must be a stage in between that translates the moments into five actuator commands. This stage is called control allocation.

The chain has three steps (source: sim/sim_bench_mix.m:11-15):

```
Step 1 Add trim: add the trim deflection to each of the three control surfaces
Step 2 Compute the moment command: control surface deflection times the effectiveness matrix gives the three-axis moment command
Step 3 Allocate to five actuators: solve the moment command and the thrust command to get two throttles and three control surfaces
```

The allocator is code under test, the bench does not rewrite it and calls the same S-function directly (source: sim/sim_bench_mix.m:17).

### 7.2 The effectiveness matrix

The effectiveness matrix is the linear map from control surface deflection to three-axis moment, 3 by 3. The construction formulas (source: sim/sim_push_design_vars.m:114-120):

```
qbar_a = 0.5 * rho_sl * V_a^2
Sb     = qbar_a * S * b
Sc     = qbar_a * S * cbar
G_surf = [ Sb*Cl_da, 0,         Sb*Cl_dr
           0,        Sc*Cm_de,  0
           Sb*Cn_da, 0,         Sb*Cn_dr ]
```

The quantities used, one by one.

The dynamic pressure `qbar_a` is computed with a sea level density of 1.225 and an allocation linearization speed of 25.0 m/s, giving 382.8125 Pa (source: sim/sim_push_design_vars.m:114-115, with the parameters in params/uacs_twin_params.m:29, 102).

Three geometric quantities: span 3.40 m, wing area 2.312 square meters, mean aerodynamic chord 0.68 m (source: params/uacs_twin_params.m:152, 155-156). The wing area is obtained from the span squared divided by the aspect ratio, and the mean aerodynamic chord from the wing area divided by the span; both are derived quantities.

Five aerodynamic derivatives: aileron roll control effectiveness 0.0412, aileron adverse yaw -0.0060, roll due to rudder 0.0100 (source: params/uacs_twin_params.m:306-329). The elevator pitch control effectiveness and the rudder yaw control effectiveness are derived quantities (source: params/uacs_twin_params.m:283, 329):

```
Cm_de = -CL_de*Vh*(lh/cbar)
Cn_dr = -Vv*CY_dr*rudder.chord_frac
```

Substituting the parameters corresponding to the delivered cache of this run, the recomputation in this document gives:

```
qbar_a = 382.8125 Pa
Sb     = 382.8125 * 2.312 * 3.40  = 3009.42
Sc     = 382.8125 * 2.312 * 0.68  = 601.88
Cm_de  = -0.636294
Cn_dr  = -0.038168
G_surf = [123.98   0       30.0921
          0       -382.949  0
          -18.0553 0      -114.854]
```

The matrix is digit for digit identical to the delivery report (source: results/control_allocation_report.txt:5-8, and the user-supplied `sim_G_surf` is the same).

The signs of the three diagonal entries are aileron positive, elevator negative, rudder negative (source: docs/handover.md:203, results/bench_check.txt:41). The meaning of the signs is in section 1.6.

The physical meaning of the diagonal entries can be read off directly: aileron at full deflection of 0.436332 rad produces a roll moment of 123.98 times 0.436332, equal to 54.10 N*m; elevator at full deflection produces a pitch moment of 382.949 times 0.436332, equal to 167.09 N*m; rudder at full deflection produces a yaw moment of 114.854 times 0.436332, equal to 50.11 N*m (source: docs/design_notes_control.md:208 gives the first of these, the other two are computed by the same method). These three numbers are the authority, and any moment command beyond the authority can only be partially achieved.

### 7.3 The four-column matrix including differential thrust

Differential thrust can also produce yaw moment, so the complete effectiveness matrix is 3 by 4 (source: control/allocate.m:50-54):

```
G = [ Sb*Cl_da, 0,          Sb*Cl_dr, 0
      0,        Sc*Cm_de,   0,        0
      Sb*Cn_da, 0,          Sb*Cn_dr, y_arm ]
```

The row order is roll, pitch, yaw (source: control/allocate.m:51). The decision variable of the fourth column is differential thrust, defined as right thrust minus left thrust (source: control/allocate.m:50-51).

Two moment arms (source: control/allocate.m:47-48):

```
y_arm = P.prop.y_motor - P.mass.y_cg   yaw moment arm of differential thrust, m
z_arm = P.prop.z_thrust_offset         thrust line offset, used for pitch, does not enter G
```

The motor station is 0.90 m (source: params/uacs_twin_params.m:380). The measured fourth column in the delivery report is `[0; 0; 0.9]` (source: results/control_allocation_report.txt:8), consistent with the expression above.

The total thrust column is reported separately: `I*B(4:6,6)` measures `[0; -0.119118; 0]`, which differs from the expected thrust line offset `[0; z_arm; 0]` by 5.371e-14 (source: results/control_allocation_report.txt:10, and the formula in the job is at control/uacs_job_control_design.m:301-312). The meaning is: the total thrust increment produces only a pitching moment, with a moment arm equal to the vertical offset of the thrust line relative to the center of gravity.

### 7.4 The three-step flow

Step 1, add trim. The trim deflection is added to each of the three control surfaces, and the trim deflection comes from `sim_u_trim_surf`, which is entries 1 to 3 of the absolute trim input, in radians (source: sim/sim_push_design_vars.m:123, with the bench wiring at sim/sim_bench_mix.m:72-75).

Step 2, compute the moment command. The three control surface deflections form a three-dimensional vector, which is left-multiplied by the effectiveness matrix to give the three-axis moment command (source: sim/sim_bench_mix.m:41-44):

```
M_cmd = G_surf * [da; de; dr]
```

The thrust command is the trim total thrust plus the increment given by the control law (source: sim/sim_bench_mix.m:45-46, 78). The trim total thrust is obtained by adding the thrust of the two motors individually, using the authoritative thrust map rather than a separately written formula (source: sim/sim_push_design_vars.m:121).

Step 3, allocate to five actuators. The allocator solves two commands and outputs five commands: three control surface deflections and two throttles (source: sim/sim_bench_mix.m:47-52, 84). The sample time of the allocator is 0.010 s (source: sim/sim_alloc_sfun.m:37, with the parameter in params/uacs_twin_params.m:460).

### 7.5 Priority

The allocation is executed in a sequence of three stages, and the order is written in the comment (source: control/allocate.m:14-19):

1. The elevator is solved on its own first and then clamped immediately. Pitch has no second actuator, so its residual is not fed back to other actuators (source: control/allocate.m:80-81, 15-16).
2. The aileron and the rudder are the main players for roll and yaw (source: control/allocate.m:84).
3. Differential thrust supplements, weighted by its moment authority relative to the rudder, and is used only at the very end (source: control/allocate.m:17-19).

The priority weights are a three-dimensional vector, the first two are 1 and the third is the penalty coefficient of differential thrust (source: control/allocate.m:87-90):

```
n_rud  = Sb * |Cn_dr| * dr_lim        yaw moment authority of the rudder, N*m
n_dif  = |y_arm * dT_lim|             yaw moment authority of differential thrust, N*m
p_supp = min(max(n_rud / max(n_dif, eps), 1.0), 1e6)
p_pri  = [1.0; 1.0; p_supp]
```

The weight is taken between 1 and 1e6, and the greater the differential thrust authority, the further back it is pushed (source: control/allocate.m:86-89). The measured weights in this run are aileron 1, rudder 1 and differential thrust 2.46198, that is, differential thrust is treated as 2.46 times more expensive (source: results/control_allocation_report.txt:13).

The physical meaning: differential thrust produces yaw moment through left-right throttle asymmetry, at the cost that total thrust is unchanged while one throttle deviates from the operating point, and the throttle rate limit is also slower than the servo, so it is used only when necessary.

### 7.6 Solution method

What is used is the weighted minimum-norm pseudo-inverse, not the ordinary pseudo-inverse (source: control/allocate.m:103):

```
e = Wi * An' * pinv(An * Wi * An') * b
```

The definitions of the four quantities (source: control/allocate.m:100-102):

```
An = G_ry(:, free) * diag(v_lim(free))       effectiveness matrix after normalization
Wi = diag(1 ./ p_pri(free).^2)               inverse square of the priority used as the weight
b  = M_ry - G_ry(:, ~free) * v(~free)        remaining command after subtracting the frozen components
```

After solving, the limits are multiplied back in (source: control/allocate.m:104-105):

```
v_new(free) = v_lim(free) .* e
```

There are two places here that are easy to misread. The first, `An` multiplies each column by that channel's own limit value, which normalizes the solved quantity into "what percentage of the limit it is", so that channels with different units can be placed in the same minimum-norm problem. The second, the weight is the inverse square of the priority, because the weight in the cost function is squared, so dividing once by the square equals multiplying the cost by the square of the priority.

The iteration runs at most three times, channels that hit their limit are removed from the free set with their values pinned at the limit, and the problem is solved again (source: control/allocate.m:96-114). This is active-set handling, and it answers "how to re-solve the residual command after limiting", not "what to do when the command itself exceeds the authority".

### 7.7 How differential thrust is handled

The total thrust command is first split into one half per motor (source: control/allocate.m:69):

```
T_want = 0.5 * T_cmd
```

The common throttle is inverted from the authoritative propulsion model, and the inversion uses 40 bisection steps, with the comment noting that 40 halvings already reach the double precision limit (source: control/allocate.m:220-244, with the comment at control/allocate.m:222-224).

The amount available for differential is the smaller of the structural limit and the throttle headroom (source: control/allocate.m:204-212):

```
d_cap      = P.prop.differential_limit
d_headroom = min(tc - tc_min, tc_max - tc)
d_eff      = min(d_cap, d_headroom)
dT_lim     = T(tc + d_eff) - T(tc - d_eff)
```

The structural limit is 0.60 throttle units (source: params/uacs_twin_params.m:402). The throttle headroom is taken because total thrust must not change and neither motor may leave the 0 to 1 window.

The differential split solves the equation with a safeguarded regula falsi (false position) method, at most 40 iterations, with a convergence criterion of an absolute residual below 1e-11, and returns zero directly when the whole command is zero (source: control/allocate.m:246-282, with the zero short-circuit at control/allocate.m:251-254). The zero short-circuit is a common normal path, and the comment states this explicitly.

Finally the two throttles are combined and each is clamped (source: control/allocate.m:120-124):

```
dth_l = tc - dth_half
dth_r = tc + dth_half
```

In this run the measured common throttle at the cruise point is 0.4234, the amount available for differential is also 0.4234, and the structural limit is 0.6000, so the limiting factor is the throttle headroom and not the structure (source: results/control_allocation_report.txt:26).

The same report also records a difference between two numbers: the differential thrust authority computed by the allocator is 20.3554 N*m, while the whole-aircraft budget computes 40.6002 N*m, a factor of two. The reason is stated: the whole-aircraft budget assumes thrust proportional to throttle and treats the lower motor as zero thrust, whereas the propulsion model the allocator uses has to subtract the ram drag term and truncate at zero, so the higher motor loses that term and the lower one cannot go negative. The allocator uses the conservative one (source: results/control_allocation_report.txt:31-35).

### 7.8 What happens on saturation

The first rule, hard clamping. The four solved quantities, elevator, aileron, rudder and differential thrust, are all hard clamped (source: control/allocate.m:81, 115-117).

The second rule, report truthfully, do not reallocate. The residual moment is not handed to other actuators, it is reported: `info.M_err`, `info.M_err_norm`, `info.M_err_rel`, and pitch additionally reports `info.err_pitch_Nm` on its own (source: control/allocate.m:141-145).

The third rule, saturation flags. Every channel has a flag, with the criterion that the solved value reaches 1 plus 1e-12 times the limit; the throttles add two more, that the solved throttle was clamped, or that the differential split exceeded the available amount (source: control/allocate.m:151-155). The summary flag is `info.sat_any` (source: control/allocate.m:156).

The fourth rule, the moment actually achieved is computed again with the real throttles (source: control/allocate.m:130-134):

```
T_l   = authoritative thrust map(V, dth_l)
T_r   = authoritative thrust map(V, dth_r)
dT_ach = T_r - T_l
M_ach  = G(:,1:3)*[da;de;dr] + [0;0;y_arm*dT_ach]
```

The requested value and the value solved as-is are recorded separately, in `info.v_cmd` and `info.v_ach` (source: control/allocate.m:146-147). Which values the limits actually used is also reported (source: control/allocate.m:148-150).

The measured saturation case: the command is four times the authority of each axis and the output is aileron zero, elevator -0.436332 rad, rudder zero, left throttle zero and right throttle 0.860130, all within their limits, total thrust conserved, and the unmet moment demand is roll -216.385, pitch -501.279 and yaw -180.103 N*m (source: results/control_allocation_report.txt:19-23).

There is one known gap to write out here. The design notes record a measurement: the roll moment command peaked at 1695.53 N*m while the aileron authority is only 54.10 N*m, so the command is 31 times the authority. The allocator's solution under such an unreachable command is aileron zero, rudder 25 degrees, and both throttles jumping between 0 and 1, which is worse than not using the allocator. The conclusion is that the allocator lacks a rule for "how to degrade when the command itself exceeds the authority", which is a different matter from active-set handling (source: docs/design_notes_control.md:208).

### 7.9 Who manages the rate limits

The rate limits are not inside the allocator. The allocator only copies the control surface rates and throttle rate over from the parameters and reports them, and does no rate integration or rate limiting (source: control/allocate.m:171-173).

The rate limit record in the measured report is aileron 120 degrees per second, elevator 100 degrees per second, rudder 100 degrees per second, and throttle 4.0 throttle units per second (source: params/uacs_twin_params.m:129, 134, 139, 404).

The real rate limiting is in the actuators on the model side, not in the control allocation stage. This point matters to a reader of the report: the control surface commands the allocator reports are position commands, and the rates are realized by the downstream actuators.

## 8. One-page quick reference

### 8.1 Operating point and trim

| Item | Value | Unit | Source |
|---|---|---|---|
| Cruise speed | 25.00 | m/s | params/uacs_twin_params.m:102 |
| Cruise altitude | 150 | m | params/uacs_twin_params.m:101 |
| Angle of attack | -0.303254 | deg | results/plant_trim_report.txt:22 |
| Pitch angle | -0.303254 | deg | results/plant_trim_report.txt:23 |
| Elevator deflection | -0.811542 | deg | results/plant_trim_report.txt:24 |
| Throttle per motor | 0.430065 | 0 to 1 | results/plant_trim_report.txt:25 |
| Normalized trim residual | 1.705e-16 | nondimensional | results/control_lqr_report.txt:6 |
| Stall speed | 9.408319 | m/s | results/atmo_regress_report.txt:47 |
| Minimum trimmable speed | 15.466467 | m/s | results/atmo_regress_report.txt:49 |
| Minimum usable speed | 15.466467 | m/s | results/atmo_regress_report.txt:50 |
| Maximum usable speed | 30.0 | m/s | params/uacs_twin_params.m:561 |
| Maximum thrust limited speed | 30.966467 | m/s | results/atmo_regress_report.txt:52 |

### 8.2 States and inputs

| Item | Content | Source |
|---|---|---|
| 12 states | u v w p q r phi theta psi pn pe pd | plant/linearize_at.m:73 |
| 6 inputs | da de dr dth_l dth_r dT | plant/linearize_at.m:74 |
| Longitudinal states | 1 3 5 8, that is u w q theta | plant/linearize_at.m:61 |
| Longitudinal inputs | 2 6, that is de dT | plant/linearize_at.m:62 |
| Lateral-directional states | 2 4 6 7, that is v p r phi | plant/linearize_at.m:63 |
| Lateral-directional inputs | 1 3, that is da dr | plant/linearize_at.m:64 |
| Output matrices | C is the 12th-order identity matrix, D is a 12 by 6 zero matrix | plant/linearize_at.m:69-70 |
| Linearization step | 1e-6, scaled per component by h = step*max(1,abs(x)) | plant/linearize_at.m:29, 41-42 |
| Equation of motion evaluations | 36 | plant/linearize_at.m:95 |

### 8.3 Contract transform

```
dV     =  cos(a0)*du + sin(a0)*dw
dalpha = -sin(a0)*du/V0 + cos(a0)*dw/V0
dbeta  =  dv/V0
```

Source: control/lin_contract.m:14-16. In this run the measurement gives V0 equal to 25.000000 m/s and a0 equal to -0.005293 rad (source: results/control_lqr_report.txt:9).

### 8.4 PID gains

| Loop | Kp | Ki | Kd | Tf | Error bound | State clamp |
|---|---|---|---|---|---|---|
| p | +0.760746 | 0 | 0 | 0.0100 | not applicable | 0.43633 |
| q | -0.863454 | 0 | 0 | 0.0100 | not applicable | 0.43633 |
| r | -0.896236 | 0 | 0 | 0.0100 | not applicable | 0.43633 |
| phi | +5.938286 | 0 | +0.098971 | 0.0300 | 0.26180 | 0.69813 |
| theta | +4.198453 | +0.748481 | 0 | 0.0100 | 0.17453 | 0.55851 |
| V | +16.959350 | +12.093735 | +30.234337 | 0.0100 | 5.0 | 50.0 |
| h | +0.016092 | 0 | 0 | 0.0100 | 20.0 | 400.0 |

Source: results/control_pid_report.txt:9-15. The units of the error bound and state clamp columns vary by loop, see section 5.1.

### 8.5 PID design constants

| Item | Value | Unit | Source |
|---|---|---|---|
| Rate loop crossover frequency target | 10.0 | rad/s | derived from control/design_pid.m:851-861, cross-checked in results/control_pid_report.txt:9-11 |
| Rate loop vector margin gate | 0.60 | nondimensional | control/design_pid.m:517 |
| Rate loop damping deficit baseline | 0.70 | nondimensional | control/design_pid.m:522 |
| Phase margin contractual floor | 45 | degrees | control/design_pid.m:65 |
| Phase margin working value | 60 | degrees | control/design_pid.m:585 |
| Overshoot penalty onset | 8 | percent | control/design_pid.m:566 |
| Overshoot heavy penalty point | 20 | percent | control/design_pid.m:568 |
| Closed-loop pole criterion tolerance | 1e-7 | nondimensional | control/design_pid.m:68 |
| Pole reduction tolerance | 1e-9 | nondimensional | control/design_pid.m:630 |
| Derivative filter baseline time constant | 0.010 | s | params/uacs_twin_params.m:477 |
| Back-calculation gain Kb | 1.0 | nondimensional | params/uacs_twin_params.m:478 |
| Integral state margin factor | 8.0 | nondimensional | params/uacs_twin_params.m:499 |
| Rate command limit fraction | 0.70 | nondimensional | params/uacs_twin_params.m:517 |
| Nominal integral zero ratio | 0.5 when there is an origin pole, 0.2 otherwise | nondimensional | control/design_pid.m:97 |

### 8.6 PID bandwidth targets

| Loop | Target | Unit | Source |
|---|---|---|---|
| Roll rate | 12.0 | rad/s | params/uacs_twin_params.m:470 |
| Pitch rate | 8.0 | rad/s | params/uacs_twin_params.m:471 |
| Yaw rate | 5.0 | rad/s | params/uacs_twin_params.m:472 |
| Roll attitude | 3.0 | rad/s | params/uacs_twin_params.m:473 |
| Pitch attitude | 2.5 | rad/s | params/uacs_twin_params.m:474 |
| Airspeed | 0.8 | rad/s | params/uacs_twin_params.m:475 |
| Height | 0.4 | rad/s | params/uacs_twin_params.m:476 |

The first three numbers serve only as report cross-references in the design, not as design targets (source: control/design_pid.m:110-112, 128-131). The separation ratio criterion is that outer loop over inner loop is less than 0.5 (source: control/design_pid.m:184-187).

### 8.7 LQR weights

| Channel | Index | State or input | Tolerance | Value |
|---|---|---|---|---|
| Longitudinal Q | 1 | dV | 2.0 m/s | 0.25 |
| Longitudinal Q | 2 | dalpha | 5 degrees | 131.312 |
| Longitudinal Q | 3 | dq | 15 degrees per second | 14.5903 |
| Longitudinal Q | 4 | dtheta | 10 degrees | 32.8281 |
| Longitudinal R | 1 | dde | 20 degrees | 8.20702 |
| Longitudinal R | 2 | dT | 0.30 N | 11.1111 |
| Lateral Q | 1 | dbeta | 5 degrees | 131.312 |
| Lateral Q | 2 | dp | 60 degrees per second | 0.911891 |
| Lateral Q | 3 | dr | 30 degrees per second | 3.64756 |
| Lateral Q | 4 | dphi | 20 degrees | 8.20702 |
| Lateral R | 1 | dda | 20 degrees | 8.20702 |
| Lateral R | 2 | ddr | 20 degrees | 8.20702 |
| Longitudinal integral | 1 | Airspeed integral state | Same as airspeed | 0.25 |
| Lateral integral | 1 | Roll integral state | Same as roll | 8.207016 |

Source: params/uacs_twin_params.m:532-535, cross-checked in measurement at results/control_lqr_report.txt:12-17.

### 8.8 LQR gains

```
K_long = [ 0.34062952   1.7888566   -1.0585723   -4.039148    0.17449358
           0.00553504   0.0066485   -0.000349222 -0.02859383  0.00318499]

K_lat  = [-0.34598738   0.095423189 -0.049195998  1.5577267    0.9717582
           2.7803249    0.013674918 -0.39039585  -0.21081883  -0.2359788]
```

Source: results/control_lqr_report.txt:23, 44. The column order is airspeed, angle of attack, pitch rate, pitch angle, integral state longitudinally; and sideslip angle, roll rate, yaw rate, roll angle, integral state laterally.

### 8.9 Key LQR margins and poles

| Item | Longitudinal | Lateral-directional | Source |
|---|---|---|---|
| Rightmost closed-loop pole | -0.701065 | -1.06523 | results/control_lqr_report.txt:25, 46 |
| Phase margin of the elevator or aileron path | 107.90 degrees | 87.27 degrees | results/control_lqr_report.txt:32, 53 |
| Delay margin of the elevator or aileron path | 0.00859 s | 0.66456 s | results/control_lqr_report.txt:32, 53 |
| Maximum difference from the care solution | 2.487e-14 | 2.109e-15 | results/control_lqr_report.txt:64 |
| Riccati residual | 1.362e-12 | 1.164e-13 | results/control_lqr_report.txt:65 |
| Steady-state error without integral | -0.774432 m/s | -0.007716 rad | results/control_lqr_report.txt:68, 72 |
| Steady-state error with integral | -4.441e-16 m/s | +2.776e-17 rad | results/control_lqr_report.txt:69, 73 |
| Integral limit | 5.0 m/s | 0.261799 rad | sim/sim_push_design_vars.m:137-140 |

### 8.10 Control allocation

| Item | Formula or value | Source |
|---|---|---|
| Effectiveness matrix | see the three rows in section 7.2 | results/control_allocation_report.txt:5-8 |
| Dynamic pressure | 382.8125 Pa | recomputed from sim/sim_push_design_vars.m:114-115, agrees with the report |
| Aileron roll authority | 54.10 N*m | 123.98 times 0.436332 |
| Elevator pitch authority | 167.09 N*m | 382.949 times 0.436332 |
| Rudder yaw authority | 50.11 N*m | 114.854 times 0.436332 |
| Differential thrust yaw authority | 20.3554 N*m | results/control_allocation_report.txt:31 |
| Priority weights | aileron 1, rudder 1, differential thrust 2.46198 | results/control_allocation_report.txt:13 |
| Weighted minimum-norm solution | e = Wi*An'*pinv(An*Wi*An')*b | control/allocate.m:103 |
| Active-set iteration cap | 3 | control/allocate.m:96 |
| Control surface limit | 25 degrees, equivalent to 0.436332 rad | params/uacs_twin_params.m:128, 133, 138 |
| Throttle window | 0 to 1 | params/uacs_twin_params.m:405-406 |
| Differential structural limit | 0.60 throttle units | params/uacs_twin_params.m:402 |
| Allocator sample time | 0.010 s | params/uacs_twin_params.m:460 |

### 8.11 Sample times and actuators

Inner loops and sensors sample at 0.0025 s, outer loops and allocation at 0.010 s, the servo time constant is 0.045 s, the ESC time constant is 0.060 s, the solver is fixed-step ode4, and there is additionally an ode45 step-size independence cross-check (source: params/uacs_twin_params.m:458-463, 130, 135, 140, 403).

### 8.12 Closed-loop performance measured in this run

| Case | Rise time | Overshoot | Steady-state error | Source |
|---|---|---|---|---|
| Pitch angle command 5 degrees | 1.0750 s | 0.00 % | 2.369e-03 rad | results/control_pid_report.txt:58 |
| Airspeed command plus 3 m/s | 2.9325 s | 7.09 % | -2.220e-03 m/s | results/control_pid_report.txt:59 |
| Height command plus 20 m | 4.4525 s | 0.32 % | 6.104e-03 m | results/control_pid_report.txt:60 |
| Roll angle command 20 degrees | 1.1200 s | 1.00 % | 2.578e-02 rad | results/control_pid_report.txt:61 |

The target rise time of the four cases is 2.2 divided by the corresponding bandwidth target, that is pitch 0.88 s, airspeed 2.75 s, height 5.5 s and roll 0.7333 s (source: results/control_pid_report.txt:58-61, with the formula at control/design_pid.m:479-481).

## 9. Unconfirmed and contradictory points

This section is this evidence-gathering task accounting for itself, please review it first.

### 9.1 A conflict of definition: the stability boundary of the yaw rate loop

The source comment says any gain greater than 0.433 on that channel is unstable (source: control/design_pid.m:226-231). In the delivered cache that loop's gain is -0.896236, and in the cache it is "designed" rather than open: `designed = 1`, with a gain margin of 2.507 times (source: results/control_pid_report.txt:11, 33).

The definition of gain margin is exactly how much more the loop can be amplified before instability. So by this delivered version's own numbers, the actual boundary of the yaw rate loop is 0.896236 times 2.507, approximately 2.247, not 0.433. The two things cannot both hold, and the 0.433 and 0.819 in the comment very probably correspond to the previous generation of the plant. Section 5 of this document is written on the side of what the delivered cache is flying, and the conflict is listed here for review.

### 9.2 Three sets of values already re-checked against the model

In this task these quantities were taken directly out of the running model and compared item by item; the conclusion is that they agree, and they are listed here so they do not have to be doubted again.

First, the effectiveness matrix agrees with the linearized model. The maximum relative difference between the three control surface columns of `sim_G_surf` and `I*B(4:6,1:3)` is 1.97e-13, the same as written in `results/control_allocation_report.txt:9`. The inertias taken are `Ixx = 0.836125`, `Iyy = 1.771296`, `Izz = 2.082559` and `Ixz = -0.017585` kg*m^2.

Second, the aerodynamic derivatives agree with the delivered linearization. Substituting `Cl_da = 0.0412` into `qbar*S*b*Cl_da` gives 123.98 N*m per radian, which is exactly the roll diagonal entry of the effectiveness matrix. The other current values are `Cl_p = -0.3875`, `Cm_de = -0.636294`, `Cm_q = -12.236419`, `Cn_dr = -0.038168` and `Cn_beta = +0.111630`.

Third, inverting the delivered `B_lat` for the inertias gives `Ixx = 0.8361` and `Ixz = -0.01758`, digit for digit identical to the parameter file. The roll damping cross-check in `plant_modes_report.txt:106` also uses the current value `|Cl_p| = 0.3875`.

One warning here: the inertias in `docs/acceptance_report.md:73-75` and `docs/design_basis_sizing.md:43` are another set of numbers, `Ixx_cg = 1.9859`, `Iyy_cg = 4.5429` and `Izz_cg = 5.8069` kg*m^2. Those two describe the configuration from the sizing stage with an aspect ratio of 5.8 and a wing area of 1.9931 square meters, while frozen rev 11 has an aspect ratio of 5.00 and an area of 2.3120 square meters. When citing inertias, areas and aerodynamic derivatives, do not cross between the two sets.

### 9.3 Items in the two evidence notes that this task could and could not confirm

The items that could be confirmed were checked line by line back against the source in this task, and these citations can be taken at face value: lines 76, 95-97, 104-106, 110-112, 114-155, 184-187, 200-213, 216-256, 285-362, 364-458, 460-596, 616-631, 635, 806-818, 841-866, 909-915, 996-1000 and 1057-1065 of `design_pid.m`; lines 17-21, 33-38, 52-63, 88-93, 99-106, 119-125, 149-154, 163-169, 186-221 and 223-252 of `design_lqr.m`; lines 5-16, 29-30, 36-43 and 50-67 of `lin_contract.m`; lines 14-19, 30-54, 69-124, 130-173, 204-244 and 246-282 of `allocate.m`; lines 28-30, 35-42, 46-89, 95, 132-164, 182-189, 202-214 and 233-251 of `linearize_at.m`; lines 8-13, 30, 39-49, 54-57, 61-65, 72-76, 139-145, 154-158 and 178-200 of `trim_cruise.m`; lines 30-33, 73-79, 91-110, 112-146 and 148-155 of `sim_push_design_vars.m`; lines 29-32 and 44-61 of `sim_add_pid.m`; lines 8-34 of `sim_pid_coeffs.m`; lines 25-26, 90-93, 131-134 and 160-166 of `sim_bench_lqr.m`; lines 4-12 and 22-28 of `sim_bench_select.m`; lines 11-17, 41-52 and 72-84 of `sim_bench_mix.m`; lines 13-17, 101-104, 128-141, 152-156, 187-193, 227-233, 243-246, 282-283, 306-329, 368-371, 380, 402-406, 458-463, 469-480, 499-504, 506-527, 532-535 and 550-574 of `params/uacs_twin_params.m`; lines 127, 169-200 and 239-251 of `params/init_uacs_twin.m`.

The items that could not be confirmed or that must be kept open, four in total.

The first, `docs/report_sources/pid_facts.md` has 182 numbered entries, and this task spot-checked the part related to section 5. The three intermediate numbers given in entry 168, 0.682235, 1.668329 and -0.3875, were checked only as far as the formula in this task, not as far as the value; of these, -0.3875 agrees with the current parameter file. The final three values of the rate command limit are upgraded to measured confirmation: `p = 0.477564`, `q = 1.167831` and `r = 0.392266` rad/s, read directly out of the base workspace of the running model, and digit for digit identical to the derived values given by the formulas in section 5.

The second, the numbers in entries 26, 27, 29, 43, 44, 85, 87, 88 and 140 of that note that are marked "derived value, not measured" were checked only as far as the expression in this task, not as far as the value.

The third, the paraphrase in `docs/report_sources/lqr_facts.md` of the header comment and step size of `linearize_at.m` agrees with the check in this task, including the inconsistency that "the header comment example writes 1e-5 while the default value is 1e-6" (source: plant/linearize_at.m:5, 29).

The fourth, neither note checked the block parameter snapshots saved in the `.slx` file. This document likewise reads only the source and the text reports, it did not open the model file and did not read the binary contents of `gains_baseline.mat`. Every statement about "what is in the delivered cache" rests on the two text reports, which are digit for digit identical to the values the user supplied.

### 9.4 Two more places where the documents and the source differ

The first, the criterion for the augmented closed-loop steady-state error. The job code uses an absolute value below 1e-9 (source: control/uacs_job_control_design.m:64-65), whereas the test description document writes it as "less than one percent of the command" (source: docs/report_sources/lqr_facts.md:313, paraphrasing docs/test_architecture.md:151). The two are not the same quantity.

The second, `step_metrics.m` for the PID has no call site in `design_pid.m` (source: docs/report_sources/pid_facts.md:18), but it is called four times in the design job script to take measurements (source: control/uacs_job_control_design.m:111-114). The statement "no call site" must be scoped, otherwise it will be read as saying this file is dead code.

### 9.5 What this task did not do

It did not run MATLAB, did not run any program that would modify project files, did not open the `.slx`, and did not run the design job.

Therefore this document cannot give a conclusion on the following three things: what a rerun of `design_pid` would produce; what a rerun of `design_lqr` would produce; and which side of the inconsistencies listed in sections 9.1 and 9.2 is actually correct.
