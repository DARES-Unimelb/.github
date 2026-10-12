# Part 3. Known issues and open items

Everything here is either wrong, undocumented, or unchecked. The order is roughly by how much it affects anyone using the bench.

## 1. The LQR path has no rate-command limiter, and the allocator then starves the roll channel

The PID attitude loops pass their output through a rate-command limiter: `sim_rate_cmd_lim_p` is 0.477564 rad/s, with matching limits for pitch and yaw. That limiter is not decoration. Before it existed, full roll stick demanded 17.73 rad/s while the airframe can hold 1.23; the aileron saturated, sideslip built to 9 degrees and the aircraft rolled inverted. The design conclusion is recorded in section 3.4 of `docs/design_report.md`.

The LQR path has no such limiter anywhere. Its lateral gain `K_lat(1,4) = 1.5577` acts directly on the bank error, so a 45 degree command asks for roughly 70 degrees of aileron, which is 2.8 times the 25 degree authority.

The control allocator then runs in its saturated branch. The delivered allocation report records what that branch returns when the demand is four times authority: aileron and rudder driven to zero, differential thrust used instead.

The consequence you can see on the bench is that a full roll command on the LQR produces a brief aileron pulse, after which the aileron returns to zero and the aircraft does not bank. This is also why the LQR never exceeds the 45 degree bank limit: the roll channel is not delivering, so the limit is never tested.

Possible fixes: add a rate limiter on the LQR lateral reference, or make the allocator preserve the roll demand in its saturated branch, or both.

## 2. The LQR path ignores the yaw stick

Inside the LQR subsystem the lateral reference vector is `[dbeta_c; dphi_c]`, wired as a constant zero and the roll command. The yaw rate command `ref(3)` never appears in that subsystem.

This is not a wiring mistake in the ordinary sense. The LQR lateral state feedback already treats yaw rate as one of the states it regulates, so the design did not need an external yaw rate command. But nothing on the canvas says so, and the consequence is that a user who switches to LQR and then turns the rudder knob sees nothing happen at all.

Possible fixes: wire `ref(3)` into the lateral reference and adjust `Tref_lat` accordingly, or label the rudder control on the canvas as inactive while LQR is selected.

## 3. The LQR integrator clamps use error-domain bounds

`sim/sim_push_design_vars.m` gives the two PID integrators the state-domain bound `P.ctrl.int_state_limit`, but gives the two LQR integrators `P.ctrl.int_limit`. Those two fields are not the same kind of number.

`params/uacs_twin_params.m:479-504` defines `int_limit` as the persistent error the loop is meant to absorb, in units of the error itself, and derives `int_state_limit` from it as `8 * int_limit / bw`, in units of error times seconds. An integrator state is an error held over time, so it belongs to the second kind.

The numbers: the LQR airspeed integrator is clamped at 5.0 where the derived bound is 50.0, and the bank integrator at 0.2618 rad where the derived bound is 0.698 rad. Ten times and 2.7 times too tight.

This is the same mistake the PID side once had. `sim/sim_push_design_vars.m:86-90` records that before rev 11 the PID was pushed `gains.<loop>.int_limit`, which is an error-domain bound and not a state bound, and the model flew with a clamp that was 2.5 times the design report's on pitch and 0.8 times on airspeed. Rev 11 fixed the PID side. The four LQR lines were not changed.

Suggested fix: change those four lines to `P.ctrl.int_state_limit.V` and `P.ctrl.int_state_limit.phi`.

## 4. The PID airspeed loop makes the throttle chatter

The delivered speed loop gains are `Kp = 16.959350`, `Ki = 12.093735` and `Kd = 30.234337`. Its bandwidth target is 0.8 rad/s, the slowest of the seven loops.

In steady cruise the airspeed error is of the order of 0.01 m/s. A derivative gain of 30 turns any high-frequency content in that error into a full-scale thrust command. On the bench the two throttles swing together between 0 and 1 in level cruise. The average sits close to the trim value, which is why the airspeed trace looks perfectly steady and the problem is easy to miss.

Suggested fix: drop the derivative term on the airspeed loop, or filter the airspeed feedback before it reaches the derivative. The check is the number of throttle reversals in a centred-stick cruise.

## 5. The bench runs 13 to 18 times slower than real time

One second of simulated time costs 13 to 18 seconds of wall clock. A 12 second run takes about two minutes. Patience is required, and the lag is not the user interface.

Where the time goes: the plant right-hand side `sim_plant_rhs` costs 3.37 ms per call, `ode4` calls it four times per 0.0025 s step, so the plant alone is about 5.4 seconds per simulated second. The rest of the diagram costs another 9 to 12 seconds per simulated second. The instruments are not the problem: commenting out all twenty-two gauge, scope and readout blocks saves only 13 percent.

The cost is inherent to a plant written as an interpreted MATLAB S-function and stepped at 400 Hz. Lower `bench_run_s` in section 1.9 of `uacs_go.m` if you want a quicker look at a trend.

## 6. Documentation and delivered numbers disagree in a few places

None of these change how the model behaves, but each of them will mislead a reader who trusts the text.

- The yaw rate loop stability boundary. A comment in `control/design_pid.m:226-231` says any gain above 0.433 is unstable for that channel. The delivered cache has that loop closed at -0.896236 with a gain margin of 2.507, which puts the real boundary at about 2.247. The two cannot both be right, and the comment most likely refers to an earlier airframe.
- Two sets of inertias and areas are in circulation. `docs/acceptance_report.md:73-75` and `docs/design_basis_sizing.md:43` quote the sizing-stage configuration: aspect ratio 5.8, area 1.9931 m2, `Ixx = 1.9859`. The frozen rev 11 configuration is aspect ratio 5.00, area 2.3120 m2, `Ixx = 0.836125`. Quoting across the two will produce wrong numbers.
- The canvas is described as it was a few revisions ago. `docs/handover.md:199` mentions five note boxes where there are now three section headings, and `sim/sim_build_bench.m:10` still says three canvas scopes where there are five.
- `uacs_go.m:144` says a run costs about 7 seconds of wall clock per simulated second. The measured figure is 13 to 18.

## 7. Things nobody has checked yet

- Whether the standalone model `model/uacs_lqr.slx` shows the same 50 Hz elevator limit cycle that the bench LQR shows.
- What a fresh run of `control/design_pid.m` would produce. The source in the tree now delivers the yaw rate loop open, with `Kp = 0`, when the search finds no stable gain; the delivered cache has it closed at -0.896236.
- What a fresh run of `control/design_lqr.m` would produce.

## 8. The English copy drifts from the maintained model

`model/uacs_flight_bench_en.slx` is a snapshot. If the Chinese model is edited, the English copy does not follow. Regenerate it with:

```matlab
   addpath('C:\Ash\Projects\DARES_UACS\MATLAB\sim');
   bench_localize_en();
```

That deletes the English copy and rebuilds it from the current Chinese file. Any arrangement you made in the English copy is lost. That is intentional: the maintained model is the Chinese one.

Two operational traps worth knowing, both of which have cost time on this project already.

First, `exist(path, 'file')` returns 4 for a `.slx` file, not 2. A check written as `== 2` silently reports that the model is missing.

Second, a MATLAB session that has loaded a model keeps that copy in memory even after the file changes on disk. Simulink warns about this the next time you try to save, but a script that reads block properties without reloading reads the stale copy. Always close and load again before reading a model you did not just save yourself.
