# The bench block by block: `uacs_flight_bench_en.slx`

This document covers one thing only: what every block on the diagram `MATLAB\model\uacs_flight_bench_en.slx` is, what it computes, and where its parameters come from. Block names are the English ones used by that copy of the model. The reader is assumed never to have opened Simulink.

Three words first, they are used throughout.

- Block: one square on the diagram. It is the smallest unit of computation in Simulink, for example a multiplication, a sum, a constant.
- Subsystem: a batch of blocks packed into one large square, from the outside only its input ports and output ports are visible. The bench has nine subsystems, and the main path is those nine large squares chained together.
- Port: the small pointed nub on the edge of a square. One nub is one signal. The connections between blocks are called signal lines in Simulink, drawn as solid lines with arrows, and this document always uses "feeds" to describe the direction.

Source citations come in four forms, used consistently throughout.

- A source citation with line numbers, of the form (source: `sim/sim_bench_mix.m:41`). The line numbers are the ones actually read.
- A reading of the model file, of the form (source: `model/uacs_flight_bench_en.slx`, block `Pilot/k_thr`). This comes from unpacking the `.slx` and reading the XML inside, not from running MATLAB.
- A measured fact supplied by the user, of the form (measured).
- Anything that cannot be determined is written as "unconfirmed" rather than guessed.

On the block count: 70 blocks at the top level, 271 blocks in the whole model, 3 annotations (measured). This document did an independent check as well, unpacking that `.slx` on disk and summing file by file, and got the same 271 blocks and 3 annotations (source: reading of `model/uacs_flight_bench_en.slx`). The canvas used to carry nine annotations, six long explanation boxes plus three section headings. The six explanation boxes have since been deleted, so the three headings are all that is left. Everything below describes the present model. The build script `sim/sim_build_bench.m` checks itself after building, verifying that the nine subsystems, the eight controls, the three state readouts, the five canvas scopes and the one fallback scope are all present (source: `sim/sim_build_bench.m:191-208`).

---

## 1. The whole diagram in one sentence

One sentence: the eight controls on the panel at the far left of the canvas are not wired to anything, they only write numbers into eight constant blocks inside the model; those numbers travel all the way along the main path to the aircraft model, and the states the aircraft computes travel back to become the needles, scopes and four logged channels on the right.

Here is that chain in order, in plain words.

1. On the control panel you drag three knobs and two sliders, and flip three switches.
2. The three knobs and two sliders write a set of "stick positions", ranging from −1 to +1 or 0 to 1. The three switches write three position numbers.
3. These eight numbers land on the literal values of eight constant blocks. A literal value means the block parameter holds a number such as `0.655968`, not a variable name such as `sim_stick_thr`.
4. The Pilot subsystem turns four stick positions into five commands: roll command, pitch command, yaw rate command, airspeed command, altitude command.
5. The Guidance and Mode subsystem uses the "Mode" position to decide which pitch command to use: in attitude mode it uses the pitch angle command computed straight from the stick, in altitude mode it uses the pitch angle computed by the height loop.
6. The five shaped commands go to the PID Controller and to the LQR Controller at the same time. Both controllers compute at every step, neither one stops.
7. The Controller Select subsystem uses the "Controller" position to pick one of the two outputs. What it picks is a four-element "control surface and thrust increment".
8. The Mixing and Allocation subsystem adds the trim amounts to that four-element increment, computes three-axis moment commands, and distributes them to five real actuators: aileron, elevator, rudder, left throttle, right throttle.
9. The 6-DOF Plant is the aircraft itself. It takes in five actuator positions and one wind, and steps forward using the differential equations of twelve states.
10. The Navigation and Measurement subsystem splits the twelve states into three things: nine feedback signals for the controllers, one raw state for the record, and seven readouts with units for the gauges.
11. The nine feedback signals return to step 6, and the loop is closed. The seven readouts go to the twelve round gauges, one stall lamp and five canvas scopes on the right.
12. Every time the simulation stops, the recording hook saves four signals to files and prints one table.

The block diagram in words follows; further indentation means "feeds the next step".

- Eight controls on the panel, not one wire, they only write numbers
  - Pitch Stick, Roll Stick, Rudder Pedal, three knobs writing a stick position from −1 to +1
  - Airspeed Command slider, writing a stick position from 0 to 1, it stands for the airspeed command and not for throttle
  - Crosswind slider, writing an eastward wind speed from −10 to +10
  - Mode, Controller, Stick Source, three switches, each writing a position of 0 or 1
  - Where they land: `k_roll`, `k_pitch`, `k_yaw`, `k_thr`, `js_on` in Pilot, `mode_const` in Guidance and Mode, `k_sel` in Controller Select, `wind_e` in Atmosphere and Wind
- Pilot, four stick positions become five commands `cmd`
  - Guidance and Mode, decides by mode whether the pitch command comes from the stick or from the height loop, outputs five references `ref`
    - PID Controller, seven loops cascaded into two stages, outputs the four-element increment `du`
    - LQR Controller, one longitudinal and one lateral-directional channel, each with integral augmentation, outputs the same four-element increment `du`
      - Controller Select, one switch flipping between the two
        - Mixing and Allocation, adds trim, computes moment commands, distributes to five actuators, outputs the six-element `u6` and the five-element `act5`
          - 6-DOF Plant, twelve-state nonlinear model
            - Navigation and Measurement, splits into nine feedback signals and seven gauge readouts
              - the nine feedback signals return to the first sum after Pilot, and the loop is closed
              - the seven readouts go to the round gauges, the stall lamp, the state readouts and the canvas scopes
              - the four logged channels are written to disk when the simulation stops

---

## 2. Top-level signal flow

### 2.1 One main path

The top level has only one main path, in the order Pilot, Guidance and Mode, the two controllers, Controller Select, Mixing and Allocation, 6-DOF Plant, Navigation and Measurement, and finally back to the downstream of Pilot (source: `sim/sim_build_bench.m:63-74`). Atmosphere and Wind is not on the main path, it separately feeds a second input to the 6-DOF Plant (source: `sim/sim_build_bench.m:73`).

### 2.2 What each wire is

The table below lists every top-level connection in the order of the wiring statements. The dimension is how many numbers that signal carries, the unit is given per component.

| From | To | Signal and dimension | Unit | Source |
|---|---|---|---|---|
| `Pilot/1` | `Guidance and Mode/1` | `cmd` five elements: roll command, pitch command, yaw rate command, airspeed command, altitude command | rad, rad, rad/s, m/s, m in that order | `sim/sim_build_bench.m:66`, `sim/sim_bench_pilot.m:4` |
| `Pilot/1` | `cmd/1` | the same `cmd`, wired to top-level output port 2 as an observation point | same as above | `sim/sim_build_bench.m:65-66` |
| `Pilot/2` | `Source Readout/1` | `state` one element, the stick source position | dimensionless, 0 virtual stick, 1 joystick | `sim/sim_bench_pilot.m:70-73`, `sim/sim_bench_cockpit.m:61` |
| `Navigation and Measurement/1` | `Guidance and Mode/2` | `fb` nine elements: roll, pitch, p, q, r, airspeed, altitude, angle of attack, sideslip angle | rad, rad, rad/s, rad/s, rad/s, m/s, m, rad, rad in that order | `sim/sim_build_bench.m:67`, `sim/sim_bench_nav.m:4-7` |
| `Navigation and Measurement/1` | `PID Controller/2` | the same `fb` | same as above | `sim/sim_build_bench.m:68` |
| `Navigation and Measurement/1` | `LQR Controller/2` | the same `fb` | same as above | `sim/sim_build_bench.m:68` |
| `Guidance and Mode/1` | `PID Controller/1` | `ref` five elements, same component order as `cmd` | same as `cmd` | `sim/sim_build_bench.m:69` |
| `Guidance and Mode/1` | `LQR Controller/1` | the same `ref` | same as above | `sim/sim_build_bench.m:69` |
| `PID Controller/1` | `Controller Select/1` | `du` four elements: aileron increment, elevator increment, rudder increment, thrust increment | the first three rad, the fourth N | `sim/sim_build_bench.m:70`, `sim/sim_bench_pid.m:4-6` |
| `LQR Controller/1` | `Controller Select/2` | a `du` of the same shape, four elements | same as above | `sim/sim_build_bench.m:70`, `sim/sim_bench_lqr.m:4-6` |
| `Controller Select/1` | `Mixing and Allocation/1` | the selected `du`, four elements | same as above | `sim/sim_build_bench.m:71` |
| `Mixing and Allocation/1` | `6-DOF Plant/1` | `u6` six elements: three control surfaces in rad, left and right throttle from 0 to 1, the sixth channel a direct thrust increment held at zero | the first three rad, the fourth and fifth dimensionless, the sixth N | `sim/sim_build_bench.m:72`, `sim/sim_bench_mix.m:4-9` |
| `Atmosphere and Wind/1` | `6-DOF Plant/2` | `wind_ned` three elements: north, east and down wind speeds | m/s | `sim/sim_build_bench.m:73`, `sim/sim_bench_wind.m:4` |
| `6-DOF Plant/1` | `Navigation and Measurement/1` | `x` twelve states | the first three m/s, the fourth to the sixth rad/s, the seventh to the ninth rad, the tenth to the twelfth m | `sim/sim_build_bench.m:74`, `plant/flow_state.m:28-36`, `plant/eom_6dof.m:28-30` |
| `6-DOF Plant/1` | `x/1` | the same twelve states, top-level output port 1 | same as above | `sim/sim_build_bench.m:74` |

Besides the main path there are several taps that only serve the gauges and the record, they do not enter the closed loop.

| From | To | Signal and dimension | Unit | Source |
|---|---|---|---|---|
| `PID Controller/2` | `rc_split` | `rate_cmd` three elements: p command, q command, r command | rad/s | `sim/sim_bench_pid.m:66`, `sim/sim_bench_cockpit.m:72-74` |
| `Navigation and Measurement/3` | `inst_split` | `inst` seven elements: airspeed, altitude, angle of attack, roll, pitch, stall flag, yaw rate | m/s, m, deg, deg, deg, 0 or 1, deg/s | `sim/sim_bench_nav.m:11-15`, `sim/sim_bench_cockpit.m:78-86` |
| `Mixing and Allocation/2` | `act_split2` | `act5` five elements: aileron, elevator, rudder, left throttle, right throttle | the first three rad, the last two 0 to 1 | `sim/sim_bench_mix.m:53`, `sim/sim_bench_cockpit.m:139-141` |
| `Mixing and Allocation/3` | `dr_deg/1` | rudder deflection, top-level output port 13 | deg, right positive | `sim/sim_bench_mix.m:65-67`, `sim/sim_bench_cockpit.m:156-157` |
| `Mixing and Allocation/4` | `da_deg/1` | aileron deflection, top-level output port 14 | deg | `sim/sim_bench_mix.m:59-61`, `sim/sim_bench_cockpit.m:158-159` |
| `Mixing and Allocation/5` | `de_deg/1` | elevator deflection, top-level output port 15 | deg | `sim/sim_bench_mix.m:62-64`, `sim/sim_bench_cockpit.m:160-161` |

The top level has 19 output ports in all, numbered 1 to 19 and matching the block names one to one (source: reading of `model/uacs_flight_bench_en.slx`, top-level `Outport` blocks). Port 1 is the twelve states `x`, port 2 is the five-element command `cmd`, ports 3 to 9 are the seven signals `inst` split apart, ports 10 to 12 are the three tracking vectors, ports 13 to 15 are the three control surface deflections, ports 16 to 19 are four narrow vectors. Why ports 16 to 19 exist is explained in section 5: a signal that is only bound by a gauge and used by no real sink is deleted by the Simulink block reduction together with everything upstream of it, and then the gauge receives no data (source: `sim/sim_bench_cockpit.m:166-167`).

### 2.3 Where the four closed loops sit

All four closed loops are closed inside the boxes of subsystems, no return line is visible on the canvas. Their places on the diagram are as follows.

| Loop | Which subsystem | Which blocks | How the error is computed | Where the output goes |
|---|---|---|---|---|
| Attitude loop | `PID Controller` | `pid_phi` and `pid_theta` | `pid_phi` takes `ref` component 1 minus `fb` component 1, `pid_theta` takes `ref` component 2 minus `fb` component 2 | roll rate command through `p_c_lim`, pitch rate command through `q_c_lim` |
| Rate loop | `PID Controller` | `pid_p`, `pid_q`, `pid_r` | respectively the limited p command minus `fb` component 3, the q command minus component 4, the r command minus component 5 and then limited | respectively the aileron, elevator and rudder increments |
| Airspeed loop | `PID Controller` | `pid_V` | `ref` component 4 minus `fb` component 6 | thrust increment `dT` |
| Height loop | `Guidance and Mode` | `pid_h` | `cmd` component 5 minus `fb` component 7 | pitch angle command, limited by `theta_lim` and fed to `theta_switch` |

Source for the two-stage cascade of attitude loop and rate loop: `sim/sim_bench_pid.m:10-15`, `sim/sim_bench_pid.m:72-85`. Source for the airspeed loop: `sim/sim_bench_pid.m:15`, `sim/sim_bench_pid.m:79`. Source for the height loop: `sim/sim_bench_guidance.m:49-54`, `sim/sim_bench_guidance.m:8-9`.

Why the height loop is not inside PID Controller: both controllers need it. In attitude mode it only passes through, in altitude mode it produces the pitch command; putting it inside either controller would make the other controller depend on that block in turn (source: `sim/sim_bench_guidance.m:16-18`). The LQR channel has no height loop, so altitude mode does nothing for the LQR (source: `sim/sim_bench_lqr.m:25-26`).

There is also a limiter between the two stages. The attitude loop is a pure proportional law, so any attitude error becomes a rate demand, and a persisting error makes that demand grow without bound. The rate-command limiter clamps the output of the attitude loop to the angular rate the airframe can actually deliver. Full roll stick once demanded 17.73 rad/s while the airframe could hold only 1.23 rad/s; the aileron saturated, sideslip reached 9 degrees, and the aircraft rolled over (source: `sim/sim_bench_pid.m:17-19`, `params/uacs_twin_params.m:506-517`).

### 2.4 Why the two controllers are said to run in parallel with one switch selecting the output

Three pieces of evidence.

- The top level wires the same `ref` to `PID Controller/1` and to `LQR Controller/1` at once, and the same `fb` to port 2 of both (source: `sim/sim_build_bench.m:68-69`).
- The two controller outputs each occupy one input port of `Controller Select`, `du_lqr` enters port 1 and `du_pid` enters port 3 (source: `sim/sim_build_bench.m:70`, `sim/sim_bench_select.m:28`).
- Inside `Controller Select` there is one `Switch` block whose criterion reads `u2 ~= 0`, with the selection signal on port 2 (source: `sim/sim_bench_select.m:22-23`). `Switch` is a pure selector, both inputs are computed at every step, it is not an "enabled subsystem" that would stall the other branch.

The two consequences of doing it this way are written in the source comments: switching is instantaneous and triggers no recompile, and the integrator of the controller being switched to has already caught up with the error; the price is that the integral states of both controllers keep running, so saturation history from before the switch is carried across (source: `sim/sim_bench_select.m:6-8`).

One trap when reading the diagram: only the PID has the two command traces inside `rate_log`. The LQR hands out surface increments directly, with no intermediate "rate command", so after switching to the LQR those two command traces no longer mean the active controller (source: `sim/sim_build_bench.m:120-121`, `docs/handover.md:188-191`).

---

## 3. One table per subsystem

The tables for the nine subsystems are all in this section. In the tables, "Block type" uses the original name from the Simulink library, and "Parameter from" says which base workspace variable that parameter reads; an entry of "literal" means the value was replaced by a number at build time.

### 3.1 Pilot

Position `[860 80 1010 220]` (source: `sim/sim_build_bench.m:53` and `sim/sim_bench_pilot.m:32`). 13 blocks inside.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `k_roll` | `Constant` | `Value` | the variable `sim_stick_roll`, turned into a literal both at build time and when the model is opened | roll stick position, −1 to +1 |
| `k_pitch` | `Constant` | `Value` | the variable `sim_stick_pitch`, same as above | pitch stick position, −1 to +1 |
| `k_yaw` | `Constant` | `Value` | the variable `sim_stick_yaw`, same as above | rudder stick position, −1 to +1 |
| `k_thr` | `Constant` | `Value` | the variable `sim_stick_thr`, same as above | airspeed command stick position, 0 to 1 |
| `stick_mux` | `Mux` | `Inputs` is 4 | none | packs the four stick positions into a four-element virtual stick vector, and names that line "0 virtual stick" |
| `js` | `Reference`, library block `aerolibanimutils/Pilot Joystick` | `JoystickID` is `None`, `OutputConfiguration` is `FourAxis`, `SampleTime` is `sim_ts_outer` | the joystick id comes from `uacs_joystick.id` in section 1.11 of `uacs_go.m`, the sample time from `sim_ts_outer` | reads the four outputs of a real gamepad: roll, pitch, yaw, throttle. By default it touches no hardware and outputs a constant zero |
| `js_pitch_sign` | `Gain` | `Gain` | the variable `sim_js_pitch_sign` | multiplies the joystick pitch channel by +1 or −1, to correct the pull direction |
| `js_mux` | `Mux` | `Inputs` is 4 | none | packs the four joystick channels into a four-element vector with the same dimensions as the virtual stick |
| `js_on` | `Constant` | `Value` | the variable `sim_js_on`, turned into a literal | stick source position, 0 virtual stick, 1 joystick |
| `src_sw` | `Switch` | `Criteria` is `u2 ~= 0`, `Threshold` is `sim_c_zero` | the threshold variable `sim_c_zero` is fixed at 0 | port 2 non-zero outputs port 1, the joystick, otherwise port 3, the virtual stick |
| `pilot` | `Level-2 MATLAB S-Function` | `FunctionName` is `sim_pilot_sfun` | the sample time `sim_ts_outer` is read inside the function | shaping from stick positions to commands, including envelope mapping and rate limiting |
| `cmd` | `Out1` | port 1 | none | sends out the five-element command |
| `state` | `Out1` | port 2 | none | sends out the stick source position, for the state readout at the top level |

Duties and contract: the inputs are four stick positions and one position, the outputs are the five-element command `cmd` (roll command, pitch command, yaw rate command, airspeed command, altitude command) and the one-element position `state` (source: `sim/sim_bench_pilot.m:4`). The actual shaping algorithm is not in the model, it is in `control/pilot_to_command.m`: the stick position maps linearly onto the envelope, the command then passes a climb rate limit, and the altitude command is the pitch stick integrated over time (source: `control/pilot_to_command.m:38-54`, `control/pilot_to_command.m:68-70`). The controllers always read these commands, the raw stick position never enters the control law (source: `control/pilot_to_command.m:19-22`).

### 3.2 Guidance and Mode

Position `[1060 60 1220 210]` (source: `sim/sim_build_bench.m:54` and `sim/sim_bench_guidance.m:24`). 12 blocks inside.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `cmd` | `In1` | port 1 | none | receives the five-element command from Pilot |
| `fb` | `In1` | port 2 | none | receives the nine-element feedback from Navigation and Measurement |
| `cmd_split` | `Demux` | `Outputs` is 5 | none | splits the five-element command into five single wires |
| `fb_split` | `Demux` | `Outputs` is 9 | none | splits the nine-element feedback into nine single wires |
| `mode_const` | `Constant` | `Value` | the variable `sim_flight_mode`, turned into a literal | flight mode position, 0 attitude, 1 altitude |
| `h_err` | `Sum` | `Inputs` is `+-` | none | altitude command minus actual altitude, giving the altitude error |
| `pid_h` | `Subsystem` | seven blocks inside, parameter prefix `sim_pid_h_` | design quantities from `params/uacs_twin_params.m`, published by `sim/sim_push_design_vars.m` | the height loop, error in, pitch angle command out |
| `theta_lim` | `Saturation` | `UpperLimit` and `LowerLimit` | the variables `sim_pid_h_lim` and `sim_pid_h_lim_neg`, equal to the pitch envelope ±30 degrees | clamps the output of the height loop inside the pitch envelope |
| `theta_switch` | `Switch` | `Criteria` is `u2 ~= 0`, `Threshold` is `sim_c_zero` | the threshold `sim_c_zero` | mode non-zero outputs the pitch command from the height loop, otherwise the pitch command computed from the stick |
| `ref_mux` | `Mux` | `Inputs` is 5 | none | packs the five signals into the reference vector `ref` |
| `ref` | `Out1` | port 1 | none | sends out the five-element reference |
| `state` | `Out1` | port 2 | none | sends out the mode position, for the state readout at the top level |

Duties and contract: the inputs are the five-element command and the nine-element feedback, the output is the five-element reference `ref` with the same component order as `cmd` (source: `sim/sim_bench_guidance.m:6-7`). This one box does two things: mode selection, and the height loop. In attitude mode `ref` is just `cmd`; in altitude mode the second component is replaced by the output of `pid_h`, that is, the pitch stick is used as a climb rate command and the height loop pulls the altitude slowly to that value (source: `sim/sim_bench_guidance.m:8-9`, `sim/sim_build_bench.m:108-109`).

### 3.3 PID Controller

Position `[1280 40 1450 210]` (source: `sim/sim_build_bench.m:55` and `sim/sim_bench_pid.m:26`). 23 blocks at the top level of this subsystem, each `pid_*` holds another 7 blocks, so this subsystem is 72 blocks in total.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `ref` | `In1` | port 1 | none | receives the five-element reference |
| `fb` | `In1` | port 2 | none | receives the nine-element feedback |
| `ref_split` | `Demux` | `Outputs` is 5 | none | splits the reference |
| `fb_split` | `Demux` | `Outputs` is 9 | none | splits the feedback |
| `phi_err` | `Sum` | `Inputs` is `+-` | none | roll command minus roll |
| `theta_err` | `Sum` | `Inputs` is `+-` | none | pitch command minus pitch |
| `r_err` | `Sum` | `Inputs` is `+-` | none | yaw rate command minus yaw rate. The limiter is cascaded after this stage |
| `V_err` | `Sum` | `Inputs` is `+-` | none | airspeed command minus airspeed |
| `pid_phi` | `Subsystem` | inner parameter prefix `sim_pid_phi_` | sample time taken from `Ts_outer` | roll attitude loop |
| `pid_theta` | `Subsystem` | prefix `sim_pid_theta_` | sample time taken from `Ts_outer` | pitch attitude loop |
| `pid_V` | `Subsystem` | prefix `sim_pid_V_` | sample time taken from `Ts_outer` | airspeed loop |
| `p_c_lim` | `Saturation` | `UpperLimit` is `sim_rate_cmd_lim_p`, `LowerLimit` takes the negative | computed by `params/uacs_twin_params.m` from the steady angular rate at full surface | roll rate command limiter |
| `q_c_lim` | `Saturation` | `UpperLimit` is `sim_rate_cmd_lim_q` | same as above | pitch rate command limiter |
| `r_c_lim` | `Saturation` | `UpperLimit` is `sim_rate_cmd_lim_r` | same as above, taken as the smaller of the coordinated turn angular rate and the yaw bandwidth | yaw rate limiter |
| `p_err` | `Sum` | `Inputs` is `+-` | none | the limited p command minus actual p |
| `q_err` | `Sum` | `Inputs` is `+-` | none | the limited q command minus actual q |
| `pid_p` | `Subsystem` | prefix `sim_pid_p_` | sample time taken from `Ts_ctrl` | roll rate loop, outputs the aileron increment |
| `pid_q` | `Subsystem` | prefix `sim_pid_q_` | sample time taken from `Ts_ctrl` | pitch rate loop, outputs the elevator increment |
| `pid_r` | `Subsystem` | prefix `sim_pid_r_` | sample time taken from `Ts_ctrl` | yaw rate loop, outputs the rudder increment |
| `rate_mux` | `Mux` | `Inputs` is 3 | none | packs the rate command vector `rate_cmd` |
| `rate_cmd` | `Out1` | port 2 | none | sends out the three-element rate command, for the scopes and the record to see |
| `du_mux` | `Mux` | `Inputs` is 4 | none | packs the four-element increment `du` |
| `du` | `Out1` | port 1 | none | sends out the four-element increment |

Inside every `pid_*` the structure is exactly the same, only the parameter prefix differs. The table below takes one loop as the example, the seven loops are copies of it (source: `sim/sim_add_pid.m:34-61`).

| Inner block | Block type | Key parameters | Parameter from | What it computes |
|---|---|---|---|---|
| `e` | `In1` | port 1 | none | receives the error of this loop |
| `Kp` | `Gain` | `Gain` | the variable `sim_pid_<loop>_Kp_abs` | proportional term, taking the absolute value of the gain |
| `Ki` | `Discrete-Time Integrator` | `gainval` takes `sim_pid_<loop>_Ki_abs`, `SampleTime` takes `sim_pid_<loop>_Ts`, `LimitOutput` is `on`, the limits take `sim_pid_<loop>_Isat` and `_Isat_neg` | published by `sim/sim_push_design_vars.m` | integral term, its output is limited, and that is how anti-windup is done |
| `Kd` | `Discrete Transfer Fcn` | `Numerator` takes `sim_pid_<loop>_Dnum_abs`, `Denominator` takes `sim_pid_<loop>_Dden` | same as above, coefficients computed by the bilinear transform | derivative term, with a first-order filter |
| `sum` | `Sum` | `Inputs` is `+++` | none | adds the three terms |
| `pol` | `Gain` | `Gain` takes `sim_pid_<loop>_pol` | same as above, equal to the sign of the proportional gain | the polarity of the whole loop is multiplied in once, here |
| `u` | `Out1` | port 1 | none | sends out the output of this loop |

Duties and contract: the inputs are the five-element reference and the nine-element feedback, the outputs are the four-element increment `du` and the three-element rate command `rate_cmd` (source: `sim/sim_bench_pid.m:4-7`). All seven loops are built by the same generator function, and the gains are read from the base workspace (source: `sim/sim_bench_pid.m:9`). The wiring is identical wire for wire with the model under test `model/uacs_manual_pid.slx`, including the special treatment on the yaw channel: the limiter is cascaded after the r error rather than after the command, so component 3 of `rate_cmd` is the unlimited `r_c` (source: `sim/sim_bench_pid.m:21-23`, `sim/sim_bench_pid.m:86-87`).

Why the polarity is implemented with a single gain stage only: the three branches need different signs, and the discrete integrator accepts only a positive `gainval`, so the three gain blocks carry the magnitude and the polarity is put in one place, after the sum (source: `sim/sim_add_pid.m:18-27`, `sim/sim_push_design_vars.m:60-65`). The pitch attitude loop is exactly the one that needs a negative polarity: its plant has a negative DC gain, positive elevator gives a nose-down moment (source: `sim/sim_add_pid.m:22-24`).

### 3.4 LQR Controller

Position `[1280 250 1450 450]` (source: `sim/sim_build_bench.m:56` and `sim/sim_bench_lqr.m:29`). 45 blocks inside. LQR is a linear quadratic regulator, in one sentence: it takes a gain table computed in advance and multiplies the state deviation by it to get the control input directly, with no per-loop tuning of proportional, integral and derivative terms.

The longitudinal channel has 19 blocks.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `long_abs` | `Mux` | four elements | none | packs the actual state in the order `[airspeed angle of attack q pitch]` |
| `long_dev` | `Sum` | `Inputs` is `+-` | none | actual state minus the trim operating point, giving the deviation |
| `xop_long` | `Constant` | `Value` | the variable `sim_lqr_xop_long` | the four-element longitudinal trim operating point |
| `xop_long_split` | `Demux` | four ways | none | splits the operating point, component 1 is the trim airspeed, component 4 is the trim pitch angle |
| `long_cmd` | `Mux` | two elements | none | packs the two command-side elements: airspeed command and pitch command |
| `long_cmd_op` | `Mux` | two elements | none | packs the two elements of the same operating point |
| `long_ref` | `Sum` | `Inputs` is `+-` | none | command minus operating point, turning an absolute command into a deviation command |
| `long_ref_split` | `Demux` | two ways | none | splits out the deviation airspeed command and the deviation pitch command |
| `C_long` | `Gain` | `Gain` is `sim_lqr_C_long`, multiplication mode `Matrix(K*u)` | `sim/sim_push_design_vars.m:132` | picks the regulated output out of the deviation states, the longitudinal one being airspeed |
| `long_int_err` | `Sum` | `Inputs` is `+-` | none | regulated output minus its command, giving the integrator input |
| `long_int` | `Discrete-Time Integrator` | `gainval` is `sim_c_one`, `SampleTime` is `sim_ts_ctrl`, `LimitOutput` is `on`, the limits are `sim_lqr_zlim_long` and `_neg` | sample time and limits both published by `sim/sim_push_design_vars.m` | integral state, used to remove the steady-state error |
| `long_aug_dev` | `Mux` | two elements | none | packs `[deviation states; integral state]` |
| `long_aug_ref` | `Mux` | two elements | none | packs `[reference deviation; 0]` |
| `long_err` | `Sum` | `Inputs` is `+-` | none | reference minus actual, giving the augmented error |
| `K_long` | `Gain` | `Gain` is `sim_lqr_K_long`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:130` | gain matrix times the augmented error |
| `Tref_long` | `Gain` | `Gain` is `sim_lqr_Tref_long`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:143` | reference feedforward, computing the control input from the deviation command |
| `Uff_long` | `Gain` | `Gain` is `sim_lqr_Uff_long`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:144` | trim feedforward |
| `long_uff_sum` | `Sum` | `Inputs` is `++` | none | feedforward plus feedback |
| `u_long_split` | `Demux` | two ways | none | splits into elevator and thrust |

`C_long` picks airspeed and `C_lat` picks roll, both from the default options of the design function (source: `control/design_lqr.m:33-37`, `control/design_lqr.m:52`, `control/design_lqr.m:114`).

The lateral-directional channel has 19 blocks and is exactly symmetric in structure to the longitudinal channel, only the state order becomes `[sideslip angle p r roll]` and the regulated output becomes roll (source: `sim/sim_bench_lqr.m:118-157`).

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `lat_abs` | `Mux` | four elements | none | packs the actual state in the order `[sideslip angle p r roll]` |
| `lat_dev` | `Sum` | `Inputs` is `+-` | none | minus the trim operating point |
| `xop_lat` | `Constant` | `Value` | the variable `sim_lqr_xop_lat` | the four-element lateral-directional trim operating point |
| `xop_lat_split` | `Demux` | four ways | none | splits the operating point |
| `lat_cmd` | `Mux` | two elements | none | packs the two command-side elements: the sideslip angle command fixed at zero, the roll command from the reference |
| `lat_cmd_op` | `Mux` | two elements | none | packs the two operating-point-side elements |
| `lat_ref` | `Sum` | `Inputs` is `+-` | none | turns them into deviation commands |
| `lat_ref_split` | `Demux` | two ways | none | splits out the deviation sideslip angle command and the deviation roll command |
| `C_lat` | `Gain` | `Gain` is `sim_lqr_C_lat`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:133` | picks out the regulated output, the lateral-directional one being roll |
| `lat_int_err` | `Sum` | `Inputs` is `+-` | none | regulated output minus command |
| `lat_int` | `Discrete-Time Integrator` | `gainval` is `sim_c_one`, `SampleTime` is `sim_ts_ctrl`, the limits are `sim_lqr_zlim_lat` and `_neg` | `sim/sim_push_design_vars.m:139-140` | lateral-directional integral state |
| `lat_aug_dev` | `Mux` | two elements | none | packs the augmented actual quantity |
| `lat_aug_ref` | `Mux` | two elements | none | packs the augmented reference quantity |
| `lat_err` | `Sum` | `Inputs` is `+-` | none | augmented error |
| `K_lat` | `Gain` | `Gain` is `sim_lqr_K_lat`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:131` | gain matrix times the augmented error |
| `Tref_lat` | `Gain` | `Gain` is `sim_lqr_Tref_lat`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:145` | reference feedforward |
| `Uff_lat` | `Gain` | `Gain` is `sim_lqr_Uff_lat`, `Matrix(K*u)` | `sim/sim_push_design_vars.m:146` | trim feedforward |
| `lat_uff_sum` | `Sum` | `Inputs` is `++` | none | feedforward plus feedback |
| `u_lat_split` | `Demux` | two ways | none | splits into aileron and rudder |

7 blocks are common to both channels.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `ref` | `In1` | port 1 | none | receives the five-element reference |
| `fb` | `In1` | port 2 | none | receives the nine-element feedback |
| `ref_split` | `Demux` | five ways | none | splits the reference |
| `fb_split` | `Demux` | nine ways | none | splits the feedback |
| `const_zero` | `Constant` | `Value` | the variable `sim_c_zero`, fixed at 0 | used by the sideslip angle command and by the second component of the augmented reference |
| `du_mux` | `Mux` | four elements | none | packs `[aileron elevator rudder thrust]` in the same order as the PID |
| `du` | `Out1` | port 1 | none | sends out the four-element increment |

Duties and contract: the inputs are exactly the same as the PID, and the output is also the four-element increment, so Controller Select needs only one `Switch` (source: `sim/sim_bench_lqr.m:4-6`). The reference quantities are computed from `ref` on the spot: `ref` is an absolute command and the LQR wants deviations, so each quantity has the trim operating point subtracted from it (source: `sim/sim_bench_lqr.m:13-20`). The operating point and the gains come from the same design result, the quantity subtracted and the gains come from the same operating point (source: `sim/sim_bench_lqr.m:19-20`, `sim/sim_push_design_vars.m:134-135`).

One asymmetry that needs review: the PID integral limits take the state-domain parameter `P.ctrl.int_state_limit`, while the limits of the two LQR integrators take the error-domain parameter `P.ctrl.int_limit` (source: `sim/sim_push_design_vars.m:91-99` and `sim/sim_push_design_vars.m:137-140`). The definitions of the two domains and the conversion between them are in `params/uacs_twin_params.m:499-504`. Whether this asymmetry is deliberate cannot be determined by this document, it is entered on the list of items to review.

### 3.5 Controller Select

Position `[1520 60 1670 180]` (source: `sim/sim_build_bench.m:57` and `sim/sim_bench_select.m:15`). 6 blocks inside.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `du_pid` | `In1` | port 1, the line is named "0 PID" | none | receives the output of the PID |
| `du_lqr` | `In1` | port 2, the line is named "1 LQR" | none | receives the output of the LQR |
| `k_sel` | `Constant` | `Value` | the variable `sim_ctrl_sel`, turned into a literal | controller position, 0 PID, 1 LQR |
| `sw` | `Switch` | `Criteria` is `u2 ~= 0`, `Threshold` is `sim_c_zero` | the threshold `sim_c_zero` | position non-zero takes port 1, the LQR, otherwise port 3, the PID |
| `du` | `Out1` | port 1 | none | sends out the selected four-element increment |
| `state` | `Out1` | port 2 | none | sends out the controller position, for the state readout at the top level |

Duties and contract: two four-element increments in, one four-element increment out, plus one position. This box is the thinnest subsystem in the whole bench, and it serves one purpose only: to let the PID against LQR comparison be switched back and forth within one simulation without stopping to change the model (source: `sim/sim_bench_select.m:6-12`).

### 3.6 Mixing and Allocation

Position `[1720 40 1890 220]` (source: `sim/sim_build_bench.m:58` and `sim/sim_bench_mix.m:20`). 23 blocks inside.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `du` | `In1` | port 1 | none | receives the four-element increment |
| `du_split` | `Demux` | four ways | none | splits into three surface increments and the thrust increment |
| `const_utrim` | `Constant` | `Value` | the variable `sim_u_trim_surf`, equal to the first three actuator quantities of the trim state | trim surface deflections, unit rad |
| `utrim_split` | `Demux` | three ways | none | splits the three trim deflections |
| `const_Ttrim` | `Constant` | `Value` | the variable `sim_T_trim` | trim total thrust, unit N |
| `const_zero` | `Constant` | `Value` | the variable `sim_c_zero`, fixed at 0 | fills channel 6 of `u6` with zero |
| `da_sum` | `Sum` | `Inputs` is `++` | none | aileron increment plus trim deflection |
| `de_sum` | `Sum` | `Inputs` is `++` | none | the same for the elevator |
| `dr_sum` | `Sum` | `Inputs` is `++` | none | the same for the rudder |
| `surf_mux` | `Mux` | three elements | none | packs the three-element surface vector |
| `G_gain` | `Gain` | `Gain` is `sim_G_surf`, multiplication mode `Matrix(K*u)` | a 3×3 matrix computed from dynamic pressure and control effectiveness derivatives, see `sim/sim_push_design_vars.m:113-120` | converts surface deflections into three-axis moment commands |
| `T_sum` | `Sum` | `Inputs` is `++` | none | thrust increment plus trim thrust |
| `alloc` | `Level-2 MATLAB S-Function` | `FunctionName` is `sim_alloc_sfun` | the sample time `sim_ts_alloc` is read inside the function | the allocator, solves the moment and thrust commands and returns five actuator quantities |
| `act_split` | `Demux` | five ways | none | splits the five actuator quantities |
| `u6_mux` | `Mux` | six elements | none | packs the six-element actuator vector `u6` |
| `act5` | `Out1` | port 2 | none | sends out the five-element actuator vector, for the gauges and the record to see |
| `u6` | `Out1` | port 1 | none | sends out the six-element actuator vector, feeding the aircraft model directly |
| `da_deg` | `Gain` | `Gain` is `180/pi` | none | converts the aileron deflection into degrees |
| `de_deg` | `Gain` | `Gain` is `180/pi` | none | converts the elevator deflection into degrees |
| `dr_deg` | `Gain` | `Gain` is `-180/pi` | none | converts the rudder deflection into degrees and flips the sign, so that right rudder is positive |
| `da_deg_out` | `Out1` | port 4 | none | sends out the aileron angle to top-level output port 14 |
| `de_deg_out` | `Out1` | port 5 | none | sends out the elevator angle to top-level output port 15 |
| `dr_deg_out` | `Out1` | port 3 | none | sends out the rudder angle to top-level output port 13 |

Duties and contract: the input is the four-element increment, the outputs are the six-element actuator vector, the five-element actuator vector and three surface angles (source: `sim/sim_bench_mix.m:4-5`). Three steps: each of the three surfaces has its trim deflection added, the control effectiveness matrix converts the deflections into moment commands, and then the allocator solves for the five actuator quantities that correspond to the moment commands and the thrust command (source: `sim/sim_bench_mix.m:11-15`). The allocator itself is code under test, it is not rewritten here, the same S-function is called directly (source: `sim/sim_bench_mix.m:17`). Channel 6 of `u6` is the direct thrust increment input of the aircraft model, the bench does not use it and holds it at zero, thrust is carried by the differential of the two throttles (source: `sim/sim_bench_mix.m:8-9`, `sim/sim_alloc_sfun.m:11-12`).

Why the rudder sign is flipped: in the aerodynamic convention positive rudder is trailing edge left and nose left, while turning that rudder knob on the control panel to the right means nose right. Without the flip, the stick goes right and the needle moves left (source: `sim/sim_bench_mix.m:56-58`, `sim/sim_bench_mix.m:65-67`). Why the three surface angles get three extra output ports: the output of the allocator is a five-element vector and binding it element by element is not possible, so the conversion into degrees is done here first and then exported (source: `sim/sim_bench_mix.m:55-56`).

### 3.7 6-DOF Plant

Position `[1960 40 2130 240]` (source: `sim/sim_build_bench.m:59`). 4 blocks inside.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `u_act` | `In1` | port 1 | none | receives the six-element actuator vector |
| `wind_ned` | `In1` | port 2 | none | receives the three-element wind |
| `plant_eom` | `Level-2 MATLAB S-Function` | `FunctionName` is `sim_plant_sfun`, 12 continuous states, sample time set to continuous | all numbers are read from the base workspace struct `P`, the initial value is read from `sim_x0` | twelve-state rigid-body equations of motion |
| `x_state` | `Out1` | port 1 | none | sends out the twelve states |

Duties and contract: the inputs are the six-element actuator vector and the three-element wind, the output is the twelve states. The twelve states are, in order, the three body-axis velocities, the three angular rates, the three attitude angles and the three positions, where the third position component is altitude with down positive (source: `plant/flow_state.m:28-36`, `plant/eom_6dof.m:28-30`, `plant/flow_state.m:47-49`). It is continuous, so the same block can run the fixed-step ode4 main path and also the variable-step ode45 comparison (source: `sim/sim_plant_sfun.m:9-14`). The initial value is taken from the base workspace variable `sim_x0` if present, otherwise from the trim state `trim.x_star`, and if neither is present it raises an error rather than silently starting from zero (source: `sim/sim_plant_x0.m:9-16`). In the bench `sim_x0` is published as the trim state by the parameter chain (source: `params/uacs_apply_config.m:47-49`, `sim/sim_push_scenario.m:38`).

### 3.8 Navigation and Measurement

Position `[1960 300 2120 500]` (source: `sim/sim_build_bench.m:60` and `sim/sim_bench_nav.m:22`). 18 blocks inside.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `x` | `In1` | port 1 | none | receives the twelve states |
| `x_split` | `Demux` | twelve ways | none | splits the states |
| `meas` | `Level-2 MATLAB S-Function` | `FunctionName` is `sim_meas_sfun`, continuous direct feedthrough | none | computes the air data from the states: airspeed, angle of attack, sideslip angle, dynamic pressure |
| `meas_split` | `Demux` | four ways | none | splits the air data, component 1 is airspeed |
| `h_neg` | `Gain` | `Gain` is `sim_c_neg1`, that is −1 | none | flips position with down positive into altitude with up positive |
| `fb_mux` | `Mux` | nine elements | none | packs the feedback vector |
| `fb` | `Out1` | port 1 | none | sends out the nine-element feedback |
| `x_out` | `Out1` | port 2 | none | sends out the twelve raw states |
| `alpha_deg` | `Gain` | `Gain` is `180/pi` | none | converts the angle of attack into degrees |
| `phi_deg` | `Gain` | `Gain` is `180/pi` | none | converts roll into degrees |
| `theta_deg` | `Gain` | `Gain` is `180/pi` | none | converts pitch into degrees |
| `r_deg` | `Gain` | `Gain` is `180/pi` | none | converts the yaw rate into degrees per second |
| `alpha_abs` | `Abs` | none | none | takes the absolute value of the angle of attack, the stall criterion does not care about the sign |
| `const_stall` | `Constant` | `Value` | the variable `sim_alpha_stall_abs`, equal to the absolute stall angle of attack in radians | stall threshold |
| `stall_rel` | `Relational Operator` | `Operator` is `>`, boolean output | none | true when the absolute angle of attack passes the threshold |
| `stall_dbl` | `DataTypeConversion` | `OutDataTypeStr` is `double` | none | converts the boolean into double, otherwise the mux block reports a data type mismatch |
| `inst_mux` | `Mux` | seven elements | none | packs the gauge tap vector |
| `inst` | `Out1` | port 3 | none | sends out the seven gauge readouts |

Duties and contract: the input is the twelve states, the outputs are the nine-element feedback, the twelve raw states and the seven gauge readouts (source: `sim/sim_bench_nav.m:4-5`). The order of the nine feedback signals is fixed here, and all three downstream subsystems split it in this order: roll, pitch, p, q, r, airspeed, altitude, angle of attack, sideslip angle (source: `sim/sim_bench_nav.m:6-7`). The order of the seven gauge readouts is airspeed, altitude, angle of attack, roll, pitch, stall flag, yaw rate, and the angles are converted into degrees here because the gauge scales are given in degrees (source: `sim/sim_bench_nav.m:11-16`). Why altitude takes a minus sign: the body z axis points down, component 12 of the state vector is the downward position, so up positive needs −pd (source: `sim/sim_bench_nav.m:9`). The stall comparison uses the two-input form rather than the block's built-in constant threshold, because the parameter name of that threshold differs between releases (source: `sim/sim_bench_nav.m:53-54`).

### 3.9 Atmosphere and Wind

Position `[1520 300 1680 460]` (source: `sim/sim_build_bench.m:61` and `sim/sim_bench_wind.m:17`). 8 blocks inside. This subsystem has no inputs.

| Block name | Block type | Key parameters | Parameter from | What this block computes |
|---|---|---|---|---|
| `wind_base` | `Constant` | `Value` | the variable `sim_dist_wind_ned`, three elements | constant wind, three earth-fixed components |
| `wind_e` | `Constant` | `Value` | the variable `sim_dist_wind_e`, turned into a literal | the eastward wind given by the crosswind slider on the canvas |
| `e_zero` | `Constant` | `Value` | the variable `sim_c_zero` | fills the other two directions with zero |
| `e_mux` | `Mux` | three elements | none | packs a three-element vector with only the east component non-zero |
| `gust` | `Sine Wave` | `Amplitude` takes `sim_dist_gust_amp`, `Frequency` takes `sim_dist_gust_freq`, `SampleTime` is 0, that is continuous | amplitude and frequency are published by section 1.10 of `uacs_go.m`, an amplitude of zero means off | gust scalar |
| `gust_gain` | `Gain` | `Gain` takes `sim_dist_gust_dir`, multiplication mode `Matrix(K*u)` | same as above, the direction is a NED unit vector | scalar sine times direction vector, giving the three-element gust |
| `wind_sum` | `Sum` | `Inputs` is `+++` | none | adds the constant wind, the crosswind trim and the gust |
| `wind_ned` | `Out1` | port 1 | none | sends out the three-element wind |

Duties and contract: no inputs, output is the three-element wind in m/s (source: `sim/sim_bench_wind.m:4`). All three terms come from `uacs_dist` in section 1.10 of `uacs_go.m`, plus the crosswind slider on the canvas (source: `sim/sim_bench_wind.m:6-11`). The gust uses a vector gain rather than being packed component by component, so that the direction is set by a parameter and the model does not have to be changed (source: `sim/sim_bench_wind.m:13-14`). The crosswind slider gives a constant wind in the earth-fixed east direction, it does not rotate with the aircraft attitude, which means that once the aircraft heading changes, the direction this wind arrives from in body axes changes with it (source: `sim/sim_build_bench.m:155-157`).

---

## 4. The eight interactive controls

None of the eight controls has a port, not one wire is connected to them. They are Dashboard controls, and they work by binding: a control is bound to one parameter of one block, and dragging it is the same as changing that parameter (source: `sim/sim_bench_cockpit.m:6-7`). All eight are bound to the `Value` parameter of their own constant block (source: `sim/sim_hmi.m:131-134`).

| Control name | Control type | Bound to which parameter of which block | Base workspace variable name | Range | Physical meaning |
|---|---|---|---|---|---|
| `Pitch Stick` | knob `KnobBlock` | `Value` of `Pilot/k_pitch` | `sim_stick_pitch` | −1 to +1 | in attitude mode it is the pitch angle command, full stick ±30 degrees; in altitude mode it is the climb rate command, full stick ±6 m/s |
| `Roll Stick` | knob | `Value` of `Pilot/k_roll` | `sim_stick_roll` | −1 to +1 | roll angle command, full stick ±45 degrees |
| `Rudder Pedal` | knob | `Value` of `Pilot/k_yaw` | `sim_stick_yaw` | −1 to +1 | yaw rate command, the full-stick command value is clamped to the coordinated turn angular rate, about 22.5 degrees per second with the present parameters |
| `Airspeed Command` | slider `SliderBlock` | `Value` of `Pilot/k_thr` | `sim_stick_thr` | 0 to 1 | airspeed command, not a throttle opening. The commanded airspeed equals `V_min` plus stick position times `V_max` minus `V_min` |
| `Crosswind East mps` | slider | `Value` of `Atmosphere and Wind/wind_e` | `sim_dist_wind_e` | −10 to +10, unit m/s | adds a constant wind to the earth-fixed east direction, it does not rotate with attitude |
| `Mode` | toggle switch `ToggleSwitchBlock` | `Value` of `Guidance and Mode/mode_const` | `sim_flight_mode` | attitude 0, altitude 1 | decides whether the pitch stick is a pitch angle command or a climb rate command |
| `Controller` | rotary switch `RotarySwitchBlock` | `Value` of `Controller Select/k_sel` | `sim_ctrl_sel` | PID 0, LQR 1 | decides which controller set the output uses |
| `Stick Source` | toggle switch | `Value` of `Pilot/js_on` | `sim_js_on` | virtual stick 0, joystick 1 | decides whether the stick positions come from the canvas or from the gamepad |

Source for the positions, labels and ranges of the controls: `sim/sim_bench_cockpit.m:44-55`. Source for the switch position texts: `sim/sim_hmi.m:71-79`; the three sets of position data read locally from the `.slx` are "attitude altitude", "PID LQR", "virtual stick joystick", consistent with the source. The ranges are written into block parameters (source: `sim/sim_hmi.m:65-68`), and reading the `.slx` locally gives −1 to 1 for all three knobs, −10 to 10 for the crosswind slider, and 0 to 1 for the airspeed command slider.

Where a few numbers in the table come from: the envelope angle at full pitch stick comes from `uacs_limits.pitch_max_deg` in section 1.8 of `uacs_go.m`, the envelope angle at full roll stick from `uacs_limits.bank_max_deg` in the same section, and the climb rate limit from `uacs_limits.climb_rate_max_mps` in the same section (source: `uacs_go.m:127-130`). The commanded angular rate at full rudder stick is the smaller of the coordinated turn angular rate and the yaw bandwidth, about 22.5 degrees per second with the present parameters (source: `control/pilot_to_command.m:44-45`, `params/uacs_twin_params.m:570-574`).

The variable correspondence of the eight controls is written in a single table, and both the build and the model open use this one table (source: `sim/uacs_bench_seed_list.m:13-22`).

### 4.1 The Airspeed Command slider gives an airspeed command

This point is explained separately because it is the easiest one to misread. The slider reading 0 to 1 is not a throttle opening, it is the stick position of the airspeed command. The mapping is hard-coded in `control/pilot_to_command.m`: the commanded airspeed equals `V_min` plus stick position times `V_max` minus `V_min` (source: `control/pilot_to_command.m:53`). With the present parameters `V_min` is 15.4665 m/s and `V_max` is 30 m/s (source: `docs/handover.md:93-96`).

- Stick position 0: commanded airspeed 15.4665 m/s. This is not throttling back, it asks the aircraft to hold this speed, and the speed loop keeps feeding thrust to hold it there.
- Stick position 0.655968: commanded airspeed about 25.0 m/s, that is the design trim speed. The default stick position is this number (source: `docs/handover.md:87`).
- Stick position 1: commanded airspeed 30 m/s.

The algorithm for the default stick position is in the parameter chain: cruise speed minus `V_min`, divided by `V_max` minus `V_min` (source: `params/uacs_apply_config.m:162-164`). A source comment writes the result as 0.6560, the exact value is 0.655968 (source: `params/uacs_apply_config.m:160-161`, `docs/handover.md:87`). One point to stress here: filling in the trim throttle 0.4301 is wrong, that requests 21.7 m/s and the aircraft will decelerate all the way away from trim (source: `params/uacs_apply_config.m:158-161`).

### 4.2 Why controls must be bound to literals

Conclusion: a control can only drive a block parameter whose value is a literal. A parameter written as a variable expression cannot be bound.

- Binding to a base workspace variable is a fake path. Binding that way can be set up, `isBound` returns true, and the `setValue` of the control itself really does change that variable, but changing that variable while the simulation is running is completely ignored by the simulation: the measured final pitch angle is 0.013 degrees, the same as not touching it. Binding to a literal value parameter takes effect immediately: 9.272 degrees, and the intervention itself takes only 0.04 seconds (source: `sim/sim_hmi.m:24-27`).
- The code section that creates the controls carries an assertion, it errors on the spot when the parameter is an expression, and what it reports is the sentence "the stick input must be turned into a literal before binding" (source: `sim/sim_hmi.m:128-130`).
- Turning values into literals is done in the assembly script and binding is done in the cockpit script, the order cannot be reversed (source: `sim/sim_build_bench.m:76-79`, `sim/sim_build_bench.m:95`).
- The initial values are still decided by `uacs_go.m`, pushed once when the model is opened by `uacs_bench_seed`, and both places use the same correspondence table (source: `uacs_bench_seed.m:6-9`, `uacs_bench_seed.m:22-34`).
- One piece of corroborating evidence from reading the `.slx` locally: when the value of a constant block equals the built-in default of that block type, Simulink does not write that parameter into the file. The copy of the model on disk has no `Value` parameter for `Pilot/k_thr`, and the default of `Value` happens to be 1. This is not a lost binding, the binding data of the airspeed command slider in the same file ends with `/k_thr` explicitly.

---

## 5. Gauges and scopes

### 5.1 Twelve round gauges and one stall lamp

Round gauges bind to signals, all in engineering units, the ranges are written into block parameters (source: `sim/sim_bench_cockpit.m:88-108`).

| Gauge name | Which signal it binds to | Range | Unit | Source |
|---|---|---|---|---|
| `Airspeed` | component 1 of `Navigation and Measurement/meas_split` | 0 to 40 | m/s | `sim/sim_bench_cockpit.m:91` |
| `Airspeed Command Gauge` | component 4 of `ref_split` | 0 to 40 | m/s | `sim/sim_bench_cockpit.m:92` |
| `Altitude` | component 1 of `Navigation and Measurement/h_neg` | 0 to 400 | m | `sim/sim_bench_cockpit.m:93` |
| `Angle of Attack` | component 1 of `Navigation and Measurement/alpha_deg` | −15 to 20 | degrees | `sim/sim_bench_cockpit.m:94` |
| `Roll` | component 1 of `Navigation and Measurement/phi_deg` | −60 to 60 | degrees | `sim/sim_bench_cockpit.m:95` |
| `Roll Command` | component 1 of `ref_split` | −60 to 60 | degrees | `sim/sim_bench_cockpit.m:96` |
| `Pitch` | component 1 of `Navigation and Measurement/theta_deg` | −30 to 30 | degrees | `sim/sim_bench_cockpit.m:97` |
| `Pitch Command` | component 2 of `ref_split` | −30 to 30 | degrees | `sim/sim_bench_cockpit.m:98` |
| `Yaw Rate` | component 1 of `Navigation and Measurement/r_deg` | −60 to 60 | degrees per second | `sim/sim_bench_cockpit.m:99` |
| `Rudder Gauge Right Positive` | component 1 of `Mixing and Allocation/dr_deg` | −30 to 30 | degrees, right positive | `sim/sim_bench_cockpit.m:100` |
| `Aileron Gauge Raw` | component 1 of `Mixing and Allocation/da_deg` | −30 to 30 | degrees | `sim/sim_bench_cockpit.m:101` |
| `Elevator Gauge Raw` | component 1 of `Mixing and Allocation/de_deg` | −30 to 30 | degrees | `sim/sim_bench_cockpit.m:102` |
| `Stall Lamp` | component 1 of `Navigation and Measurement/stall_dbl` | none | a lit lamp means the angle of attack has passed the stall angle of attack | `sim/sim_bench_cockpit.m:108` |

The three command gauges sit immediately to the right of the corresponding live gauge, and that is deliberate: tracking quality is visible without opening a window (source: `sim/sim_bench_cockpit.m:28-30`). The command gauges bind components of `ref`, so in altitude mode the Pitch Command gauge shows the effective pitch command computed by the height loop, not the stick position (source: `sim/sim_bench_cockpit.m:30`).

Note two places that are easy to misread. The Aileron Gauge and the Elevator Gauge give the raw aerodynamic deflection, whose sign does not necessarily match the sign of the stick position (source: `sim/sim_build_bench.m:126-127`). The altitude command `h_c` has no corresponding gauge, it is not among the twelve gauges.

### 5.2 Three state readouts

A toggle switch and a rotary switch do not display their current position, so each of the three switches has one state readout. All three are classic `Display` blocks, each connected by a solid line (source: `sim/sim_bench_cockpit.m:57-61`, `sim/sim_bench_cockpit.m:224-229`).

| Readout block | Which signal it takes | What it shows | Source |
|---|---|---|---|
| `Mode Readout` | `Guidance and Mode/2` | 0 attitude, 1 altitude | `sim/sim_bench_cockpit.m:59` |
| `Controller Readout` | `Controller Select/2` | 0 PID, 1 LQR | `sim/sim_bench_cockpit.m:60` |
| `Source Readout` | `Pilot/2` | 0 virtual stick, 1 joystick | `sim/sim_bench_cockpit.m:61` |

The previous version used three Dashboard lamps, and it was measured that after binding those three lamps to signals they all stayed at the default grey during a run and received no data, so they were replaced by readout blocks that go through ports and are certain to have numbers (source: `sim/sim_bench_cockpit.m:15-17`).

### 5.3 Five canvas scopes and one fallback scope

The five tracking plots use `DashboardScope`, plotting inside the canvas, with no pop-up window (source: `sim/sim_bench_cockpit.m:198-202`).

| Plot name | Which signal it binds to | Traces | What that signal is | Source |
|---|---|---|---|---|
| `Attitude Tracking` | component 1 of `att_mux` | 4 | roll command, actual roll, pitch command, actual pitch | `sim/sim_bench_cockpit.m:112-121`, `sim/sim_bench_cockpit.m:198` |
| `Rate Tracking p q` | component 1 of `rate_mux_pq` | 4 | p command, actual p, q command, actual q | `sim/sim_bench_cockpit.m:168-174`, `sim/sim_bench_cockpit.m:199` |
| `Rate Tracking r Rudder` | component 1 of `rate_mux_r` | 2 | r command, actual r | `sim/sim_bench_cockpit.m:176-181`, `sim/sim_bench_cockpit.m:200` |
| `Three Surfaces` | component 1 of `surf_mux` | 3 | aileron, elevator, rudder | `sim/sim_bench_cockpit.m:183-189`, `sim/sim_bench_cockpit.m:201` |
| `Left and Right Throttle` | component 1 of `thr_mux` | 2 | left throttle, right throttle | `sim/sim_bench_cockpit.m:191-196`, `sim/sim_bench_cockpit.m:202` |

One `DashboardScope` was measured to plot at most four traces: of three blocks with byte-identical parameters, the one bound to four traces plots, the ones bound to five and six are entirely empty. So they were split one block per four traces (source: `sim/sim_bench_cockpit.m:164-165`). After being split narrower, the wide Mux blocks are still kept, they continue to feed the fallback scope and the signal logging (source: `sim/sim_bench_cockpit.m:165-167`).

The fallback one is a classic `Scope`, block name `Fallback Scope Three Signals`, its three inputs are the attitude, rate, and surface and throttle sections in order, arranged in three rows one above the other, configured not to open automatically, it opens on a double click (source: `sim/sim_bench_cockpit.m:205-209`). Why it is kept: `DashboardScope` has no ports, so whether data arrives depends on the binding having been written correctly, while the classic `Scope` has input ports and is certain to have data (source: `docs/handover.md:1482`).

Why the scopes are inside the canvas rather than in pop-up windows: `DashboardScope` is a canvas control, plotting takes no window and there is no need to switch back and forth between several windows (source: `sim/sim_bench_cockpit.m:32-35`). By the official contract its binding must be a cell array whose elements are `Simulink.HMI.SignalSpecification`, a single object is rejected (source: `sim/sim_bench_cockpit.m:32-33`, `sim/sim_hmi.m:28-29`).

---

## 6. The four logged signals

The four signals have logging enabled in the model and have names, the names carry no spaces or brackets, which makes them easy to fetch by name (source: `sim/sim_bench_cockpit.m:211-217`, `sim/sim_bench_cockpit.m:231-239`). After a run the base workspace holds `logsout` (source: `docs/handover.md:207-208`).

| Log name | Who sends it out | Channels | Channel definition | Unit | Which gauge on the canvas reads it |
|---|---|---|---|---|---|
| `inst_log` | output 3 of `Navigation and Measurement` | 7 | airspeed, altitude, angle of attack, roll, pitch, stall flag, yaw rate | m/s, m, degrees, degrees, degrees, 0 or 1, degrees per second | `Airspeed`, `Altitude`, `Angle of Attack`, `Roll`, `Pitch`, `Stall Lamp`, `Yaw Rate` |
| `att_log` | output 1 of `att_mux` | 4 | roll command, actual roll, pitch command, actual pitch | rad | the `Attitude Tracking` plot |
| `rate_log` | output 1 of `rate_mux` | 6 | p command, actual p, q command, actual q, r command, actual r | rad/s | the `Rate Tracking p q` and `Rate Tracking r Rudder` plots |
| `act_log` | output 1 of `act_mux` | 5 | aileron, elevator, rudder, left throttle, right throttle | the first three rad, the last two 0 to 1 | the `Three Surfaces` and `Left and Right Throttle` plots |

The channel order is fixed in the source, changing it in one place will not change it in the other (source: `sim/sim_build_bench.m:136-142`). The report fetches data by these four names and prints the table (source: `verify/bench_report.m:39-42`).

Two things not to confuse: the raw rudder deflection is channel 3 of `act_log`, that is, how far the surface moved; rudder tracking is the r command against r in `rate_log`, that is, whether the angular rate followed (source: `sim/sim_build_bench.m:141-142`).

### 6.1 Difference in sample rate

Measured: the sample interval of `act_log` is 0.01 s, the other three are 0.0025 s (measured). The mechanism is that the sample rate of a logging point is decided by the fastest source on that signal.

- The only source of `act_log` is `act_mux`, and all five inputs of `act_mux` come from output 2 of `Mixing and Allocation`, which is the output of the allocator S-function, with sample time `Ts_alloc`, that is 0.01 s (source: `sim/sim_bench_cockpit.m:139-146`, `sim/sim_alloc_sfun.m:37`).
- The other three all mix in a continuous or fast-ticking component: `inst_log` comes from the state vector, and `att_log` and `rate_log` each pack the command and the actual value into the same mux block, while the actual values come from continuous states (source: `sim/sim_bench_cockpit.m:112-135`, `sim/sim_bench_nav.m:39-41`).

The split shows up in how the analysis scripts in this project read the data: the reporting job resamples the fast signals at 0.005 s and the actuator signals at 0.01 s, exactly along these two families (source: `run/uacs_job_bench_stick.m:33-34`, `run/uacs_job_bench_stick.m:157-173`). The time base of the report takes the time vector of `inst_log` (source: `verify/bench_report.m:51`). The time vectors of the four channels are stored separately in the file, do not assume they are the same length.

Whether the display side of the gauges and scopes applies any smoothing such as a moving average is not written in the source, this document leaves it unconfirmed. What is certain is that the logging side does no smoothing: `bench_report.m` reads the raw data directly.

---

## 7. Sample times and solver

### 7.1 The four sample times

| Name | Base workspace variable | Value | Meaning | Source |
|---|---|---|---|---|
| `Ts_ctrl` | `sim_ts_ctrl` | 0.0025 s | inner loop sampling, and the step of the fixed-step solver | `params/uacs_twin_params.m:458`, `uacs_go.m:145` |
| `Ts_outer` | `sim_ts_outer` | 0.010 s | sampling of the attitude loop, the airspeed loop and the height loop | `params/uacs_twin_params.m:459`, `uacs_go.m:146` |
| `Ts_alloc` | `sim_ts_alloc` | 0.010 s | sampling of the allocator and the mixing | `params/uacs_twin_params.m:460`, `uacs_go.m:147` |
| `Ts_sensor` | `sim_ts_sensor` | 0.0025 s | sensor sampling | `params/uacs_twin_params.m:461`, `uacs_go.m:148` |

All four values are declared in section 1.9 of `uacs_go.m`, mapped through the parameter chain into the parameter struct, and then written into the base workspace by the design publisher (source: `params/uacs_apply_config.m:118-122`, `sim/sim_push_design_vars.m:39-42`).

### 7.2 Which blocks use which sample time

| Sample time | Which blocks | Source |
|---|---|---|
| `sim_ts_ctrl` | solver step; the two discrete integrators of the LQR | `sim/sim_new_model.m:31`, reading of `model/uacs_flight_bench_en.slx`, blocks `LQR Controller/long_int` and `LQR Controller/lat_int` |
| `sim_ts_outer` | the joystick block in Pilot; the pilot S-function; the integral and derivative terms of the three PIDs of the attitude loop, the airspeed loop and the height loop | reading of `model/uacs_flight_bench_en.slx`, block `Pilot/js`; `sim/sim_pilot_sfun.m:33`; `sim/sim_push_design_vars.m:31-33`; the parameter names of the seven loops are in the reading of `model/uacs_flight_bench_en.slx` |
| `sim_ts_alloc` | the allocator S-function | `sim/sim_alloc_sfun.m:37` |
| nominally `Ts_ctrl` but subdivided per loop | the PID of the three rate loops take `Ts_ctrl`, the attitude loop and the airspeed loop take `Ts_outer`, the height loop takes `Ts_outer` | `sim/sim_push_design_vars.m:31-33` |
| continuous | the twelve states of the aircraft model; the air data S-function; the gust sine | `sim/sim_plant_sfun.m:44`, `sim/sim_meas_sfun.m:30`, `sim/sim_bench_wind.m:30` |
| `sim_ts_sensor` | appears in the source only in the publisher, it does not appear once in the whole XML read locally from the `.slx` | `sim/sim_push_design_vars.m:42` |

`sim_ts_sensor` and `sim_ts_ctrl` have the same value, both 0.0025 s, so even if some place were switched to the sensor sample time, with the present parameters no difference would show up numerically. No block in the bench references it.

One reason a sample time has to be set explicitly: the default sample time of the joystick block is 1/30 s, which is not an integer multiple of the fixed step 0.0025 s, and without setting it, updating the model reports that the sample time is not an integer multiple of the fixed step (source: `sim/sim_bench_pilot.m:17-19`).

### 7.3 Why fixed-step ode4 and not variable step

The solver settings of the model: fixed step, `ode4`, step taken from `sim_ts_ctrl`, stop time taken from `sim_bench_t_end`, start time 0, algebraic loop reports an error, solver parameter checking off (source: reading of `model/uacs_flight_bench_en.slx`, configuration set; the corresponding generating code is `sim/sim_new_model.m:28-39`).

Three reasons.

- The architectural convention is fixed-step `ode4`, variable step is kept for the step-independence cross-check (source: `sim/sim_new_model.m:9-11`).
- The model under test has to run under both fixed step and variable step, and a model reference requires the referenced model and the parent to use the same solver, so the aircraft model is an embedded copy rather than a model reference (source: `sim/sim_add_plant_subsystem.m:5-10`).
- The bench is meant to be dragged: fixed step keeps the computation per step constant, so a control dragged takes effect at the next sample point. The measured intervention takes effect immediately during a run (source: `sim/sim_hmi.m:24-27`).

The speed cost has to be stated plainly: measured locally, the model runs 13 to 18 times slower than real time. A 12 second section costs about 130 to 190 seconds of wall clock (source: `run/uacs_job_bench_stick.m:8-9`). One old statement in the source does not match this: the comment at `uacs_go.m:144` says "about 7 seconds of real time per second at a 0.0025 s step", that is 7 times, which is optimistic. The measurement governs.

---

## 8. How to operate this bench

From opening it to seeing a curve, do it in this order.

1. Double-click `MATLAB\model\uacs_flight_bench_en.slx` in the file explorer. No `addpath` first, and no initialization first (source: `docs/handover.md:51-54`).
2. When it opens the first callback runs: `PreLoadFcn` runs `uacs_go.m`, pushing the parameters, the trim, the gains and the default scenario into the base workspace (source: reading of `model/uacs_flight_bench_en.slx`, callback `PreLoadFcn`; `run/build_bench_models.m:39`). This chain includes the knob-to-parameter mapping, the trim, the linearization and the gain computation (source: `params/uacs_apply_config.m:38-43`).
3. Immediately after that the second callback: `PostLoadFcn` calls `uacs_bench_seed`, which writes the initial values of the eight stick inputs into the literal parameters of the model according to that single correspondence table (source: reading of `model/uacs_flight_bench_en.slx`, callback `PostLoadFcn`; `uacs_bench_seed.m:22-34`).
4. Press the green Run. Before every press of Run, `InitFcn` runs `uacs_ensure`, which only fills in missing variables and does nothing when not one is missing, so pressing Run repeatedly does not rerun the whole parameter chain and its several minutes of printing (source: reading of `model/uacs_flight_bench_en.slx`, callback `InitFcn`; `uacs_ensure.m:30-32`, `uacs_ensure.m:4-9`).
5. The simulation length is in the StopTime box in the toolbar, it holds the variable `sim_bench_t_end`, default 20 seconds (source: `uacs_go.m:144`, `params/uacs_apply_config.m:181`). The first Run includes compilation and is slow, later Runs in the same session are much faster (source: `docs/handover.md:285`).
6. During a run drag the controls on the panel at the left directly, there is no need to stop (source: `sim/sim_build_bench.m:97`).
7. When the simulation stops, `StopFcn` calls `uacs_bench_record`, which archives this run (source: reading of `model/uacs_flight_bench_en.slx`, callback `StopFcn`).

### 8.1 Which files the recording hook produces

| File | Content | Source |
|---|---|---|
| `results/bench_last_run.mat` | the complete signal record of the most recent run, a fixed name, overwritten on every run | `uacs_bench_record.m:8-10`, `uacs_bench_record.m:53-57` |
| `results/bench_report.txt` | the table of the most recent run, printed by the report function | `uacs_bench_record.m:60-65`, `verify/bench_report.m:47` |
| `results/runs/bench_<timestamp>.mat` | the archive of the same run, the timestamp is of the form `yyyymmdd_HHMMSS` | `uacs_bench_record.m:28`, `uacs_bench_record.m:54` |
| `results/runs/bench_<timestamp>.txt` | the table of the same run, with the same name as the archived `.mat` | `uacs_bench_record.m:62` |
| `results/bench_run_trace.txt` | an appended log, one line per run, recording the timestamp, the model name, the two output paths and the data source; a failure at any step is written here as well | `uacs_bench_record.m:26`, `uacs_bench_record.m:48`, `uacs_bench_record.m:58` |

The archive directory keeps only the most recent 5 runs, older ones are deleted by timestamp name, the `.mat` and the same-named `.txt` are deleted together (source: `uacs_bench_record.m:22`, `uacs_bench_record.m:66-80`). The two ways the data can arrive are both accepted, it first takes `out.logsout` and falls back to `logsout`; when neither is present it writes a line saying so, it does not leave a blank (source: `uacs_bench_record.m:33-48`). The whole thing is wrapped in error handling, a failing hook only leaves a trace and never interrupts the flight (source: `uacs_bench_record.m:17-18`, `uacs_bench_record.m:82-87`).

### 8.2 How the report is generated

`verify/bench_report.m` reads the signal record in the base workspace, prints a table with channel names for the four signals, and writes `results/bench_report.txt` (source: `verify/bench_report.m:8-12`, `verify/bench_report.m:39-42`). The table has five sections: airspeed and altitude, angle of attack and stall, attitude tracking, rate tracking, surfaces and throttle, and a few concluding sentences at the end (source: `verify/bench_report.m:50-127`). The concluding paragraph is for eyeballing, it splits a phenomenon such as "airspeed stuck" into two kinds: one is the speed loop holding the speed at the command, the other is a real problem (source: `verify/bench_report.m:112-120`). The report only reads the workspace and the model parameters, it does not change the model (source: `verify/bench_report.m:10-11`). It can also be launched on its own, the job entry is `run_job('bench_report')` (source: `docs/handover.md:543`).

### 8.3 How layout changes are preserved

The bench model is rebuilt in full by the build script, and a rebuild deletes that file on disk first (source: `sim/sim_new_model.m:13-18`, `sim/sim_build_bench.m:49`). So the positions you arranged on the canvas exist only in the `.slx`, and one rebuild loses them. Three things exist for this.

| File | Purpose | Source |
|---|---|---|
| `run/uacs_job_bench_layout.m` | the job `run_job('bench_layout')`, reads the model on disk, exports the position of every block and every annotation, and writes a diagnostic file `results/bench_layout.txt` | `docs/handover.md:294` |
| `sim/uacs_bench_layout_list.m` | an automatically generated data file, holding two tables of block positions and annotation positions, do not edit it by hand, edits are overwritten by the next export | `sim/uacs_bench_layout_list.m:4-11` |
| `sim/uacs_bench_layout_apply.m` | applied at the end of a rebuild. It only applies blocks that still exist; a newly created block has no record in the data and stays at the position the build script gave it | `sim/uacs_bench_layout_apply.m:10-11`, `sim/sim_build_bench.m:218` |

The way to keep the layout is fixed at two steps: arrange it in the interface and save the `.slx`, then rerun `run_job('bench_layout')` (source: `docs/handover.md:333`). After that any rebuild applies this data back, and the build log prints a line "canvas layout: applied N blocks, M present in the data but not in the model; applied K annotations, L missing" (source: `sim/sim_build_bench.m:219-220`).

An annotation can only be keyed by its body text or its first-line title, because an annotation has no identity independent of its text (source: `sim/uacs_bench_layout_apply.m:12-15`). Reading locally gives two numbers: the layout data file records 271 blocks and 9 annotations, while the model on disk now has 271 blocks and 3 annotations, so on the annotation count the two no longer agree. The six explanation boxes were deleted from the canvas after that layout file was written, and the layout file has not been re-exported since. Rerun `run_job('bench_layout')` after any arrangement change to bring the two back into step. That "N blocks" in the apply result is the output of one particular build at that moment and is not written to disk, the two must not be confused (source: `docs/handover.md:301-308`).

### 8.4 Rebuilding and changing parameters

- Rebuild the bench: `run_job('bench_build')`, the script body is `run/build_bench_models.m`, it rebuilds the bench and the aerodynamic test page, two interactive models, in one go (source: `docs/handover.md:541`, `run/build_bench_models.m:42-45`).
- Change the simulation length: change `uacs_sim.bench_run_s` in section 1.9 of `uacs_go.m` and press F5, or type a number straight into that toolbar box (source: `uacs_go.m:139-144`).
- Change the envelope: change the two envelope angles in section 1.8 (source: `uacs_go.m:127-128`). After widening them the V1 to V13 acceptance conclusions no longer apply (source: `sim/sim_build_bench.m:152-153`).
- Change the crosswind direction or add a gust: change section 1.10, that is where the three NED components and the gust switch are (source: `sim/sim_build_bench.m:158-160`).
- Use a gamepad: change `uacs_joystick.id` in section 1.11 to `Joystick1`, rerun a rebuild, then flip the Stick Source on the panel to "joystick" (source: `sim/sim_build_bench.m:163-167`).

### 8.5 Two operational traps

- The initial value push hangs on opening the model, not on every Run. Hanging it on every Run would reset the stick positions you dragged during a session back to the values in `uacs_go.m` the moment you press Run (source: `uacs_bench_seed.m:11-12`).
- A scripted run that ends by restoring the `Value` of those five constants to variable expressions leaves the panel undraggable for the rest of that model session, an example being `run/uacs_job_bench_stick.m:223-227`. A control cannot drive a parameter whose value is an expression, so only reopening the model or a rebuild restores it (source: `sim/sim_hmi.m:33-34`).

---

## 9. One page quick reference

### 9.1 The eight controls

| Control | Block it binds to | Variable | Range |
|---|---|---|---|
| `Pitch Stick` | `Pilot/k_pitch` | `sim_stick_pitch` | −1 to 1 |
| `Roll Stick` | `Pilot/k_roll` | `sim_stick_roll` | −1 to 1 |
| `Rudder Pedal` | `Pilot/k_yaw` | `sim_stick_yaw` | −1 to 1 |
| `Airspeed Command` | `Pilot/k_thr` | `sim_stick_thr` | 0 to 1, airspeed command 15.4665 to 30 m/s |
| `Crosswind East mps` | `Atmosphere and Wind/wind_e` | `sim_dist_wind_e` | −10 to 10 m/s |
| `Mode` | `Guidance and Mode/mode_const` | `sim_flight_mode` | 0 attitude, 1 altitude |
| `Controller` | `Controller Select/k_sel` | `sim_ctrl_sel` | 0 PID, 1 LQR |
| `Stick Source` | `Pilot/js_on` | `sim_js_on` | 0 virtual stick, 1 joystick |

### 9.2 Gauges and plots

| Category | Count | Bound to what |
|---|---|---|
| Round gauges | 12 | 6 bound to the taps of Navigation and Measurement, 3 bound to components of the reference vector, 3 bound to surface angles converted by Mixing and Allocation |
| Stall lamp | 1 | stall flag |
| State readouts | 3 | the position output of each of the three subsystems |
| Canvas scopes | 5 | attitude 4 traces, rate pq 4 traces, rate r 2 traces, surfaces 3 traces, throttle 2 traces |
| Fallback scope | 1 | attitude, rate, and surfaces and throttle, three sections in three rows one above the other, it does not open automatically |

### 9.3 The four logged signals

| Log | Source | Channels | Sample interval |
|---|---|---|---|
| `inst_log` | output 3 of `Navigation and Measurement` | 7 | 0.0025 s |
| `att_log` | `att_mux` | 4 | 0.0025 s |
| `rate_log` | `rate_mux` | 6 | 0.0025 s |
| `act_log` | `act_mux` | 5 | 0.01 s |

### 9.4 Sample times and solver

| Item | Value | Variable |
|---|---|---|
| Solver | fixed step `ode4` | none |
| Step | 0.0025 s | `sim_ts_ctrl` |
| Stop time | default 20 s | `sim_bench_t_end` |
| Inner loop | 0.0025 s | `sim_ts_ctrl` |
| Attitude and energy loops | 0.010 s | `sim_ts_outer` |
| Allocator | 0.010 s | `sim_ts_alloc` |
| Sensor | 0.0025 s | `sim_ts_sensor`, no block in the bench references it |

---

## 10. Places that disagree with `docs/handover.md`

The source governs; the items are listed one by one.

1. The number of canvas scopes. The comment at `sim/sim_build_bench.m:10` says "three canvas scopes", the actual number is five, and the self-check assertion in the same file already says 5 (source: `sim/sim_build_bench.m:10` and `sim/sim_build_bench.m:205-206`). `run/build_bench_models.m:8` carries the old wording "three" as well. This is a comment that did not keep up with a change, not a problem with the model.
2. The number of top-level bands. `sim/sim_build_bench.m:6-7` says "the top level holds only four kinds of things, split into four bands from left to right", and then immediately lists only three kinds: the control panel, the control law and plant, the instruments (source: `sim/sim_build_bench.m:6-10`).
3. The number of blocks of Chinese explanation. `docs/handover.md:199` says "there are five blocks of Chinese explanation on the canvas" and lists five, whereas at the time the canvas carried six body annotations plus three band titles, 9 annotations in all. Six of those have since been deleted, leaving the three band titles. `docs/handover.md:373` itself also describes `note_log` as a newly added annotation (source: `sim/sim_build_bench.m:97-167`, `docs/handover.md:199`, `docs/handover.md:373`).
4. Run speed. The comment at `uacs_go.m:144` says about 7 seconds of real time per second, while scripted runs and interactive runs alike measure 13 to 18 times (source: `uacs_go.m:144`, `run/uacs_job_bench_stick.m:8`). The measurement governs, that comment is optimistic.
5. Completeness of the output port numbering. `docs/handover.md:136-137` mentions only that the top level gained two sink output ports, saying that `da_deg` is port 14 and `de_deg` is port 15, and does not mention port 13. In the model port 13 is `dr_deg`, all three are there (source: `sim/sim_bench_cockpit.m:156-161`, reading of `model/uacs_flight_bench_en.slx`).

One more observation of the snapshot on disk, worth noting during review. Reading the `.slx` locally gives: the literal value of `Pilot/k_yaw` is 0.9778859456380209, `k_roll` and `k_pitch` are small numbers close to zero, and `k_thr` has no `Value` parameter written. These numbers are the trace left by someone who dragged the controls in the interface and saved, they are not the defaults of `uacs_go.m`; when the model is opened `PostLoadFcn` overwrites them from `uacs_go.m` (source: `uacs_bench_seed.m:22-34`). The reason `k_thr` has no such parameter is that its value equals the built-in default 1 of the constant block, and Simulink does not write a parameter into the file when it equals the default.
