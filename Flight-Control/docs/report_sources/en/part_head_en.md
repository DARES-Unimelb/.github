# UAV Flight Control Test Bench: how it is built and why

Date 2026-10-12. Frozen parameter set rev 11 / 0.5.0.

Two models sit in `MATLAB/model/` and they are the same model:

| File | What it is |
|---|---|
| `uacs_flight_bench.slx` | The maintained bench. Block names, line names and annotations are in Chinese. |
| `uacs_flight_bench_en.slx` | A copy in which every block, signal line and annotation carries an English name. Block positions, sizes, parameters and wiring are untouched; only names and text differ. |

Generate the second one from the first with `sim/bench_localize_en.m`. It copies the file, renames, and saves. It never rebuilds, so whatever arrangement you have on the canvas survives.

The names inside the signal lines and in the block dialogs are already English in both copies, because they are MATLAB variable names. Only the labels you read on the canvas differ.

## What this document is for

This is written for club members who have never opened this project. It answers four questions: what the bench actually computes, how the control law inside it was designed, what every block on the canvas is for, and which parts are still not right.

It contains no test results. Flight test data, the stick-sweep campaign and the performance numbers live in the Chinese report `docs/design_tutorial_report.md` and in `results/bench_stick_*`.

The document has three parts.

- Part 1, how the control law was designed. Coordinates and sign conventions, how the trim point is solved, how the state-space model is obtained, how to get a transfer function out of it, the seven PID loops, the LQR channels, and how the five actuators are driven from three moment commands. This part answers "how was it designed".
- Part 2, the Simulink bench block by block. Every subsystem, every block, every signal, every control on the panel, and where each parameter comes from. This part answers "what is that block on the canvas".
- Part 3, known issues and open items. What is wrong, what is merely undocumented, and what has not been checked yet.

Each part numbers its own sections, so "section 5" means section 5 of the part you are in.

## The aircraft

| Quantity | Value |
|---|---|
| Configuration | Twin-engine fixed wing, one propeller on each wing |
| Control effectors | Ailerons, elevator, rudder, left throttle, right throttle. Five in total |
| All-up mass | 17.000 kg |
| Payload | 12 kg |
| Wing span | 3.400 m, hard limit 3.50 m |
| Aspect ratio | 5.00 |
| Wing area | 2.3120 m2, derived from span and aspect ratio |
| Mean aerodynamic chord | 0.6799 m |
| Inertia | Ixx 0.836125, Iyy 1.771296, Izz 2.082559, Ixz -0.017585 kg m2 |
| Airspeed envelope | 15.4665 to 30.00 m/s |
| Bank authority | plus or minus 45 deg |
| Pitch authority | plus or minus 30 deg |
| Surface travel | aileron, elevator and rudder each plus or minus 25 deg |
| Stall angle of attack | 14.0 deg |
| Trim point | 25.00 m/s level flight, angle of attack -0.3033 deg, elevator -0.8115 deg, throttle 0.4301 |

The lower end of the airspeed envelope, 15.4665 m/s, is not the stall speed. It is the slowest speed at which the trim equations still have a solution with the elevator authority available. The reason is worked through in section 3.1 of `docs/design_report.md`. That distinction matters: the lower limit moves if you change the elevator authority or the centre of gravity, and it does not move if you change the stall characteristics.

## What the bench computes, in one paragraph

A pilot stick block turns four stick positions into five commands: a bank angle, a pitch angle, a yaw rate, an airspeed and an altitude. A guidance block decides whether the pitch command is taken from the stick directly or from a height loop. Two controllers run side by side, a cascade of PID loops and an LQR pair, and a switch picks which one drives the aircraft. The chosen controller does not output surface angles; it outputs three moment commands and one thrust command. A control allocator turns those four numbers into five actuator commands, because five actuators for four demands is an over-determined problem and the extra freedom has to be resolved by a priority rule. The plant is a twelve-state rigid-body model with a simple aerodynamic coefficient build-up. A navigation block splits the state into the nine signals the controllers want and taps seven more for the instruments on the canvas.

Both controllers run at all times, not one or the other. Switching is therefore instantaneous and does not require recompiling the model. The price is that both controllers keep integrating while the other one flies, so a controller can carry saturation history across a switch.

## Sample times and solver

| Quantity | Value | Used by |
|---|---|---|
| `Ts_ctrl` | 0.0025 s, 400 Hz | rate loops, attitude loops, LQR integrators, plant integration step |
| `Ts_outer` | 0.010 s, 100 Hz | joystick sampling, outer-loop pacing |
| `Ts_alloc` | 0.010 s, 100 Hz | control allocator, so surface commands only change every 10 ms |
| `Ts_sensor` | 0.0025 s | sensor model |

The solver is fixed-step `ode4` with the step taken from `sim_ts_ctrl`, and the run length is `sim_bench_t_end`. Fixed step rather than variable step because the control law is discrete and the simulation step has to line up with the controller sample time. The plant is continuous, so `ode4` integrates it. See section 7 of Part 2 for why `Ts_alloc` being ten times slower than `Ts_ctrl` matters.
