# Decision notes: the open decisions (2026-10-11)

Background for the decisions that are still open after Marcell's answers of
2026-10-11: **D5, D6, D10, D11**, plus two small points (D15 later, D8 colour).
D5, D6, D10 and D11 were each researched and then re-checked by a second,
independent pass (sources opened, most calculations redone; exceptions are
marked). Numbers marked [EST] are estimates; [SRC] source
code; [DS] datasheet; [SIM] a small test simulation run for these notes (not the
thesis code). Recommendations only: **Marcell decides**. Symbols are in
`glossary.md`.

Answer format that is enough: "D5 b", "D6 b", "D11 A, nominal = tuned", plus
answers to the numbered questions if you have them. D11's options are capital A,
B, C here; they replace the old a/b in the master plan (old a = C).

---

## D5. Motion model (state and prediction)

**Decided 2026-10-14: C** (pose + gyro bias, GNSS velocity and gyro as inputs),
because the buoys are always in view and the course is under open sky. The
rest of this section is the background.

**What it decides.** What the filter predicts with between measurements, and what
is in the state (eq. 3.6). Details: `design.md` §6.1.

| | a) IMU-driven | b) Constant body velocity and turn rate (recommended) | c) Pose only (eq. 3.6 as drafted) |
|---|---|---|---|
| State besides buoys | x, y, ψ, u, v, b_g, b_ax, b_ay (8 + 2N) | x, y, ψ, u, v, ω, b_g (7 + 2N) | x, y, ψ (3 + 2N) |
| Gyro bias estimated? | yes | yes | no |
| Uses the accelerometer? | yes (needs roll/pitch to remove gravity) | no | no |
| Work | most | medium | least |

Why the choice matters less than it looks: **buoys that were mapped before a GNSS
outage pin the position during it**, whatever the motion model (one scan of a
mapped gate pair: about 4-15 cm [SIM]). The motion model decides the error only
where no mapped buoy is in view (e.g. the Task 5 marina; about half of the
course's bounding box if the fake detector's range equals the lidar's 2-beam
range, more with a shorter range) or when the buoys are first seen during the outage.

Drift in a sky-view outage (no GNSS position, no GNSS heading), 1.5 m/s, no buoy
in view [SIM]:

| Outage | a, accel bias 0.02 m/s² not estimated | a, biases estimated before (filter's own 1σ) | b, start speed error 0.05 m/s | b, boat slows by 1 m/s | c, gyro bias not estimated |
|---|---|---|---|---|---|
| 10 s | 1.0 m | 0.07 m | 0.5 m | about 8 m | 1.0° heading, 0.13 m sideways |
| 60 s | 36 m | 2.8 m | 3.0 m | about 58 m (8 m + 1 m/s × 50 s) | 5.8° heading, 4.6 m sideways |

- **a** is clearly better than b in short buoy-free outages on a steady run, and it
  follows speed changes. Its catch: every roll/pitch error leaks in as a false
  acceleration (g·Δθ: 0.1° gives 0.017 m/s², about the size of the turn-on
  accelerometer bias). There is no published attitude-accuracy figure for
  ArduPilot to base that error on (D11), so a's outage results would rest on an
  [EST]. It also breaks the draft's "attitude errors only enter through the
  projection".
- **b** cannot see speed changes during a buoy-free outage. Waves (D9 a) also
  make the speed oscillate, which b treats as process noise. It does not depend
  on the roll/pitch model at all. The IMU study then covers the gyro only.
- **c** has no gyro-bias state, so it drifts in heading in every outage. Now that
  outages are core (D9 a), it is the weakest.
- A middle option exists: b plus the accelerometer as a *measurement* of the
  forward acceleration. It lets b notice slowdowns, at the cost of one more
  update and the roll/pitch dependence for that axis.

**Recommendation: b as the main model** (a BSc-sized job, independent of the
unsourced roll/pitch error), named "constant turn rate and velocity
(CTRV-type) with sway and gyro bias". a later as an optional variant on the same
bags (cheap with record-once-replay-many), which would itself be a result.
Choose a now if an accelerometer result, or outages with speed changes away from
buoys, must be in the thesis.

Questions:
1. In the outage runs, does the boat keep a steady speed, or slow down and stop
   while GNSS is out? With b that decides the result.
2. Where do the outages go: where mapped buoys are in view (tests SLAM), in
   buoy-free stretches (tests the motion model), or both (recommended)?

### D5 comparison: the three candidates (2026-10-14)

Asked by Marcell after "is velocity really necessary?". Numbers are those above
(1.5 m/s, sky-view outage, no buoy in view; [SIM] from the D5 research).

| | **A. IMU-driven** (velocity from the IMU, in the state) | **B. Constant speed and turn rate** (velocity in the state, no accelerometer) | **C. Pose + gyro bias** (velocity from GNSS Doppler as an input) |
|---|---|---|---|
| State | x, y, θ, u, v, b_g, b_ax, b_ay + buoys (8 + 2N) | x, y, θ, u, v, ω, b_g + buoys (7 + 2N) | x, y, θ, b_g + buoys (4 + 2N) |
| Prediction driven by | measured acceleration and turn rate | the estimated velocity itself | GNSS velocity and gyro turn rate |

**A. IMU-driven**

- Strengths: follows speed changes even without GNSS; best in short buoy-free
  outages if the biases were estimated before (own 1σ 0.07 m after 10 s, 2.8 m
  after 60 s); uses every sensor the boat has, so the IMU study covers the
  accelerometer too; closest to what real navigation systems do (ArduPilot's
  EKF3, GNSS/INS units).
- Weaknesses: needs roll and pitch to remove gravity (0.1° error ≈ 0.017 m/s²
  false acceleration); most work (gravity, IMU lever arm, 8-state Jacobians,
  prediction at 50-200 Hz); most tuning (accelerometer noise and two biases).
- Opportunities: a strong thesis result ("a consumer IMU is enough") covering
  both gyro and accelerometer; outages with speed changes become testable.
- Threats: results rest on a roll/pitch error model that has no published
  source ([EST]); the draft's "attitude errors only enter through the projection"
  stops being true; time risk for a BSc schedule.

**B. Constant speed and turn rate**

- Strengths: no dependence on roll/pitch or the accelerometer; medium work
  (7 states, simple Jacobians); keeps the planar argument intact; gyro bias
  estimated (heading drift 0.3-0.7° in 60 s); velocity and turn rate are
  estimated, so the follower could later steer on the filter (M7.3).
- Weaknesses: blind to speed changes when GNSS and buoys are missing (a 1 m/s
  slowdown ≈ 8 m error after 10 s); waves make the speed oscillate, which only
  the process noise covers; the accelerometer is unused.
- Opportunities: a can be added later as a variant on the same recordings and
  compared; "b plus the accelerometer as a measurement of forward acceleration"
  is a small upgrade.
- Threats: process-noise tuning decides the outage results; if outages include
  stops, NEES fails in those runs.

**C. Pose + gyro bias, GNSS velocity as input**

- Strengths: smallest change to the thesis state (x, y, θ plus one bias); least
  work; GNSS velocity is measured in the world frame, so it already includes
  sideways drift and water current, and position prediction doesn't depend on
  heading; gyro bias estimated, so no heading drift.
- Weaknesses: in a sky-view outage the GNSS velocity disappears too, so the
  filter must hold the last value with an invented uncertainty (in practice like
  b, but without an estimated velocity to hold); no velocity estimate to publish,
  so the follower can't steer on the filter (M7.3 not possible); the IMU study
  covers the gyro only; the corruptor must also produce a noisy GNSS velocity.
- Opportunities: closest to Thrun's textbook EKF-SLAM (velocity and turn rate as
  the control input u_t), which matches the notation tables.
- Threats: GNSS velocity errors become process noise, and they are not white
  (multipath, same satellites as the position), so A3 is broken a second way;
  the UM982 velocity accuracy is not checked yet ([DS, to check]); outage
  results depend entirely on the "hold last speed" rule.

**In one line each:** A = most capable, most work, rests on an unsourced
roll/pitch error. B = balanced, independent of roll/pitch, weak only when the
speed changes during a blind outage. C = simplest and closest to the textbook,
but outages are its weak spot, and outages are a core experiment (D9 a).

## D6. The slowly varying GNSS error

**What it decides.** GNSS errors change slowly (seconds to minutes), so
consecutive fixes carry almost the same error. Treated as white noise, the filter
"averages away" an error that does not average away, and becomes overconfident.

Test [SIM]: 1D filter, 10 Hz fixes, 200 runs × 900 s per setting.

| | a) White, R inflated | b) Error in the state (2 states, reset at a mode switch) |
|---|---|---|
| Standalone, honest variance (factor 1) | ANEES 9-29 instead of 1 | ANEES 0.95-1.07 |
| Inflation factor needed | 25-127 for standalone, 1.2-2.1 for RTK (1D toy filter, illustrative only); depends on the filter's tuning, not on the receiver | none |
| With buoys in the state | map NEES grows the longer a buoy is seen; no constant factor works | consistent |
| After RTK → standalone | position follows the new offset at once (0.85 m) | keeps RTK quality for a while (0.20 m) |
| Extra work | none | about half a day to a day in M3.3b; state 9 + 2N with D5 b |

The catch with b: it is only consistent if its slow-error size σ_ss is about
right (half the true value gives ANEES about 4; a 3× error in T_c barely
matters). σ_ss per mode becomes the key filter setting; it must be set
deliberately different from the corruptor's value (no inverse crime) and stated
in the thesis. Recommended: a bit larger than the corruptor's (e.g. 1.5×), which
makes the filter cautious rather than overconfident. The RTK numbers above depend on the
assumed RTK slow error (6 mm [EST]); with none, the RTK problem disappears.
A third textbook option, measurement differencing (Bryson-Henrikson), was tested
and fails with our white + slow error mix, so it is not offered.

**Recommendation: b in the main filter** (moved from optional M7.2 into M3.3b),
with a kept as a switch. Reason: D9 a made GNSS modes and outages core
experiments; with a, their NEES results would mostly show the chosen inflation
factor, not the filter. The planned A3 test becomes a clean "white vs. modelled"
comparison on the same bags. Fallback if time is short: a with one factor per
mode, NEES claims limited to RTK-fixed runs.

Questions:
3. When GNSS returns after an outage in the same mode, does the slow error get a
   new draw (as at a mode switch)? Recommended: yes after a gap longer than a set
   time, and the filter resets on the same gap rule.
4. OK to set the filter's σ_ss about 1.5× the corruptor's (and T_c 2×)?
5. Should the covariance the corruptor reports always be the **total** error
   (white + slow), as a receiver's RMS spec means? `design.md` §4.2 mixes the two
   now (RTK reports only the white part, standalone the total). Recommended: total.

## D10. Number of seeds (runs per setting)

You wanted to decide once the run speed is known. With D4 a (record once, replay
many), **the speed that matters is the replay speed**, not Gazebo's:

- Gazebo's real-time factor only sets how long the 3-5 recordings take (5 min of
  boat time each).
- Every seed is an offline slam_replay run. Its speed depends on how the filter
  update is written: about 1-3 s per 5-minute bag if it uses the sparse
  structure (only the parts of the matrices a measurement touches, `design.md`
  §6 Numerics), 43-374 s if it multiplies full matrices [EST: one timing program
  in the first pass, not rerun].
- The sweep in `design.md` §11 has about 26-31 settings on one trajectory, 74-89
  on three (plus 2-3 per trajectory if the D11 processing levels are added). At M = 50 that is 1,300-4,450 runs.

What M buys: the 95 % ANEES band (n = 3) is 2.02-4.16 at M = 20 and 2.36-3.72 at
M = 50 (both checked). The matched check (M3.6) then catches covariance errors
above about 11-13 % (M = 20) or 7-8 % (M = 50).

**Decision rule** (apply after M3.5):
1. M0.2 reports the normal headless real-time factor (only for recording time).
2. M3.5 reports the wall time t_run of one replay plus evaluation of a 5-min bag
   (added to its "done when").
3. Count the settings S in the final sweep file; compute S × 50 × t_run / cores.
   If that fits in one night (8 h), use M = 50 everywhere. If not, M = 50 for the
   matched check and the nominal setting, M = 20 for the rest of the sweep
   (recommended default). Don't go below 20; if even that doesn't fit, the filter
   update is probably dense and should be fixed first.

One caveat: all seeds replay the same recording, so errors that don't depend on
the noise (the motion model's approximations, the planar lever arm) are shared by
every run and don't average out. A larger M narrows the band around a possibly
biased ANEES, so the matched check can get harder to pass as M grows.

Question:
6. Does the sweep run on one trajectory (the course run) or on several (also the
   M1.7 manoeuvres)? It changes the run count about 3×.

## D11. What the corrupted IMU message carries

You decided: "Corrupted IMU messages should follow real world scenarios". So the
question became: what does the **real** IMU message on the boat look like? Read in
ArduPilot's source code (AP_DDS, Rover 4.6.3, 4.7.1 and master, 2026-10-11) [SRC]:

- Topic `/ap/imu/experimental/data` (sensor_msgs/Imu). AP_DDS is not in the
  standard Pixhawk 6X firmware: the team needs a build with DDS enabled.
- **Gyro and accelerometer**: calibrated, but with their bias still in. Low-pass
  filtered with Rover's defaults at 4 Hz (gyro, about 56 ms delay) and 10 Hz
  (accel, about 22 ms). New values only at the 50 Hz main loop; the topic
  repeats each value about 3 times (published at most every 6 ms). The time stamp
  is when it was published, not when it was measured. Together the gyro is about
  66 ms older than its stamp. In a small filter test [SIM] at 0.15 rad/s this
  gives a heading error peak of about 0.45° when a turn starts or ends (NIS up to
  about 50 instead of 1), but only about 0.03° in a steady turn; with the tuned
  setting (about 14 ms) 0.10°. It mainly hurts consistency (NEES/NIS) and the
  gate.
- **Orientation**: ArduPilot's own estimate (EKF3), **including yaw**. With the
  UM982 set up as ArduPilot's yaw source, that yaw comes from the same
  dual-antenna heading our filter uses, so using both counts it twice. (Also: the
  quaternion's components are written in the wrong order, and the frame is
  NED/FRD, not ENU/FLU, so the real boat will need a small converter node, not
  just a topic rename.)
- No published accuracy for ArduPilot's roll/pitch was found.

| | A) Real signals, no orientation (recommended) | B) Same fields as the real message | C) Ideal raw IMU (old D11 a) |
|---|---|---|---|
| Gyro, accel | datasheet noise and bias, then ArduPilot's processing as switches: low-pass, 50 Hz hold, stamp at publish | as A | datasheet noise and bias only, 200 Hz |
| Orientation | not published (marked "not available") | roll, pitch and yaw, each with an EKF3-like error [EST] | not published |
| Roll/pitch | a separate message only when something needs it (D5 a, D15 b, M5) | inside the message | separate message |
| Yaw leak risk | none | yaw is published; every estimator must be shown not to use it | none |
| Real-world? | as real as the source code allows | closest in form, but the yaw and roll/pitch errors would be guesses | no: neither real link delivers this |

**Recommendation: A.** It copies what the boat really delivers to a filter, keeps
the true heading out, and needs no unsourced attitude model as long as nothing
uses roll/pitch (with D5 b, D15 a and M5 optional, nothing in the core does). The
"white" profile (all switches off) stays for the matched check. Real-world
effects become noise-profile switches, so they are also a sweep axis.

Which setting is "nominal": three levels, **ideal** (no processing), **tuned**
(20 Hz filter, 200 Hz loop: about 14 ms age) and **Rover default** (about 66 ms
age). The filter cut-offs and loop rate are normal ArduPilot parameters
(`INS_GYRO_FILTER`, `INS_ACCEL_FILTER`, `SCHED_LOOP_RATE`); raising them also
changes Rover's own controllers. So "tuned" as nominal assumes the team raises
them, "Rover default" assumes they don't (the second check was split on this).
Pick the one the boat will actually run; the thesis states the assumption, and
the other levels are a sensitivity study. The filter is run without
delay compensation (or with a compensation that differs from the simulated
delay), so it does not "know" the answer.

Other real effects, ranked: a per-run scale-factor and axis-misalignment error
(datasheet ±0.2 % [DS]; tilt [EST]) is worth one switch. Vibration, clipping,
quantisation, dropped messages and switching between the 3 IMUs are better stated
as limitations.

Two findings that change numbers (each needs your OK, because they are values in
`asv.yaml` and the thesis):
- The accelerometer noise density at the ±32 g range ArduPilot uses is
  110 µg/√Hz, not the 70 µg/√Hz (±8 g row) now in `asv.yaml` [DS DS-000577,
  Table 2]. Only matters if the accelerometer is used (D5 a, or the middle
  option).
- How a datasheet noise density turns into noise per sample (σ = η/√dt in
  `design.md` §4.1, or √2 smaller by TDK's own definition) should be stated
  once; the M2.5 brief settles it.

Questions:
7. D11: A, B or C? And the nominal level: ideal, tuned or Rover default?
8. Should the per-run scale/misalignment error be in the nominal profile, or only
   in its own study (so the white-noise study stays clean)?
9. OK to change the accelerometer density to 110 µg/√Hz in `asv.yaml`?

## Smaller points

- **D15** (GNSS lever arm): decided a for now; the M3.3b brief raises it again,
  as you asked. Nothing to do before then.
- **D8, colour**: "true colour" is implemented as: the fake detector gives the
  buoy's true colour whenever a camera would see it (inside a camera's field of
  view and range), and UNKNOWN elsewhere; colour mix-ups are a setting, default
  0.
10. D8 colour: true colour only in camera view (recommended; keeps the
    UNKNOWN-colour path of the data association tested) or everywhere?
