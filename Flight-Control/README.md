# Flight Control: interactive Simulink test bench

A twin-engine fixed-wing UAV flight-control test bench, built in MATLAB R2024a and Simulink.
This folder is self-contained: clone or copy it, open one model, press Run. Nothing needs to
be added to the MATLAB path by hand.

## What is in here

| Path | What it is |
|---|---|
| `model/uacs_flight_bench_en.slx` | The interactive bench, all labels in English. Start here if you read English. |
| `model/uacs_flight_bench.slx` | The same bench, maintained copy, block names in Chinese. |
| `model/uacs_aero_test.slx` | Aerodynamic test page: drag angle of attack, sideslip, airspeed and surface deflections and watch the coefficients, forces and moments. |
| `model/uacs_lqr.slx`, `model/uacs_manual_pid.slx` | The two controllers as standalone models, used for the recorded acceptance runs. |
| `model/uacs_plant.slx`, `model/uacs_plant_ref.slx` | The plant alone, and the plant against a reference implementation. |
| `docs/design_tutorial_report_en.md` | Full English write-up: how the control law is designed, what every block does, and what is still wrong. |
| `uacs_go.m` | The one file you edit to change parameters. Everything else follows from it. |
| `params/`, `plant/`, `control/`, `sim/`, `verify/`, `sizing/` | The code the models call. |

## Requirements

- MATLAB R2024a with Simulink.
- Aerospace Blockset (the joystick input block).
- Control System Toolbox. Optimization Toolbox is used when the design has to be recomputed.

No other toolboxes are needed. The models do not use Simulink 3D Animation.

## How to run it

1. Copy this folder anywhere on your machine. Keep the folder structure as it is, the models
   find the code relative to their own location.
2. Open `model/uacs_flight_bench_en.slx`. The first open runs the parameter setup and prints a
   long banner; that takes about half a minute.
3. Press Run. The run length comes from the StopTime box in the toolbar, default 20 s.

Then drag the controls on the left of the canvas. The knobs and sliders take effect while the
simulation is running, there is no need to stop and restart.

If MATLAB cannot find something when you open the model, the usual cause is that the folder
structure was changed. The models resolve the code from their own file location, so
`<folder>/model/<name>.slx` must stay two levels below the root of this folder.

## The controls

| Control | What it does |
|---|---|
| Pitch Stick | Pitch attitude command, or climb-rate command in altitude mode |
| Roll Stick | Bank angle command |
| Rudder Pedal | Yaw rate command |
| Airspeed Command | Commanded airspeed, 15.47 to 30 m/s. This is not a throttle |
| Crosswind East mps | Constant wind from the east, in m/s |
| Mode | Attitude mode, or altitude mode where the pitch stick drives the height loop |
| Controller | Selects PID or LQR. Both run at all times, switching is instant |
| Stick Source | Virtual stick, or a real gamepad |

Twelve gauges, a stall lamp, three state readouts and five scopes show what the aircraft is
doing. See section 5 of Part 2 of the English report for which gauge shows what.

## Before you change anything

`uacs_go.m` is the only file that should need editing for normal use. It holds the geometry,
the masses, the aerodynamic derivatives, the control bandwidth targets, the envelope limits and
the disturbance settings. Change a number there, run it, reopen the model, and everything
downstream follows: trim, linearisation, gains and the block parameters.

Changing a parameter may trigger a recompute of the trim point, the linearisation and the gain
design. That takes one to four minutes and prints a lot; it is cached afterwards.

## Known issues

`docs/design_tutorial_report_en.md` Part 3 lists them in order of severity. The two that matter
most for anyone using the bench:

- The LQR path has no rate-command limiter, and the control allocator then starves the roll
  channel. A full roll command on the LQR does not produce a sustained bank.
- The LQR path ignores the rudder stick entirely. The rudder knob does nothing when LQR is
  selected.

## A note on speed

The bench runs about 13 to 18 times slower than real time: one second of flight costs 13 to 18
seconds of wall clock, so a 20 second run takes about five minutes. This is inherent to the
interpreted plant model stepping at 400 Hz, not to the interface. Lower `bench_run_s` in
section 1.9 of `uacs_go.m` if you want a quicker look at a trend.
