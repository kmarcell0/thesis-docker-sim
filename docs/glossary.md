# Glossary

Plain-language explanations of the symbols and terms used in `master_plan.md`,
`design.md` and the code. These are study notes for understanding, not text for
the thesis.

Where one of Marcell's literature notes explains a term in more depth, its name is
given in brackets, e.g. [noise-models-notes]. Those notes live in Marcell's
Claude project folder; they are **not** in this repository.

---

## Symbols

One meaning per symbol. "Draft" means the symbol is already used in the thesis
draft (Chapter 3; equation numbers as in the draft PDF of 2026-10-08, they may
move). "Proposed" means Marcell has not chosen it yet: Claude Code
proposes, Marcell decides, and then this table is updated.

| Symbol | Meaning | Unit | Status |
|---|---|---|---|
| x, y, ψ | boat position (east, north) and yaw, counter-clockwise from east | m, rad | draft, eq. 3.1 |
| φ, θ | roll and pitch (Z-Y-X Euler angles) | rad | draft, Planar Motion |
| α | GNSS azimuth of the bow, clockwise from north; ψ = π/2 − α | rad | draft, Planar Motion |
| 𝒲, ℬ, 𝒮, ℋ | frames: world (ENU), boat body, sensor, level frame | | draft |
| R_y, R_x, R_ℬ𝒮 | rotation matrices in eq. 3.3 (not noise covariances) | | draft, eq. 3.3 |
| m_j | position of buoy j | m | draft, eq. 3.4 |
| N | number of landmarks (buoys) in the map | | draft, eq. 3.4-3.6 |
| ξ | full filter state: pose, gyro bias b_g and buoys (D5 C) | | decided notation (2026-10-14 tables) |
| r | range to a buoy | m | draft (Planar Motion) |
| r_b | buoy radius | m | draft (Static Landmarks) |
| β | bearing to a buoy, counter-clockwise from the bow | rad | proposed |
| u, v | surge and sway speed (forward and sideways, boat frame) | m/s | proposed |
| ω | yaw rate (turn rate) | rad/s | proposed (r is already the range) |
| b_g | gyro bias | rad/s | proposed |
| b_ax, b_ay | accelerometer biases (forward, sideways; only with D5 a) | m/s² | proposed |
| l | lever arm of the main GNSS antenna (its position in base_link) | m | proposed |
| P | state covariance | | proposed |
| R | process noise covariance (Thrun; D14 a). Also the letter of the rotation matrices in eq. 3.3, so one of them may get another letter in the thesis | | decided (D14 a) |
| Q | measurement noise covariance (Thrun; D14 a) | | decided (D14 a) |
| η | white-noise density | e.g. rad/s/√Hz | proposed |
| n_conf | sightings needed before a tentative landmark is confirmed | | proposed |
| σ_ss, T_c | size (steady-state standard deviation) and correlation time of a Gauss-Markov error | unit of the signal, s | proposed |
| τ | Allan averaging time (only this) | s | proposed |
| M | number of repeated runs (seeds) per setting | | proposed |
| n | number of values checked in NEES or NIS (degrees of freedom) | | proposed |
| ℓ_i | length of route segment i | m | proposed (L is the queue delay) |
| W_i | waypoint i of the route (W0 = start, as in `milestone1_plan.md`) | m | proposed (replaces P_k: P is the covariance) |
| i | route segment index; segment i runs from W_i to W_{i+1} | | proposed (k stays the time step) |
| n_w | number of waypoints after the start (W1 … W_{n_w}), also the number of route segments; 9 for Task 1 | | proposed (N is the landmark count) |
| γ_i | direction of segment i, counter-clockwise from east | rad | proposed (replaces α_k: α is the GNSS azimuth) |
| s, e | along-track progress and cross-track error (e > 0: boat left of the path) | m | proposed (the NEES error vector in `design.md` §10 is a different e) |
| Δ | lookahead distance of the line-of-sight guidance | m | proposed |
| R_switch | switch distance: go to the next segment when s ≥ ℓ_i − R_switch | m | proposed (subscripted, so not the process noise R) |
| p_LOS | line-of-sight target point: Δ ahead of the boat's projection, measured along the route | m | proposed |
| d_end | distance left to the last waypoint, along the route: (ℓ_i − s) + Σ_{j>i} ℓ_j | m | proposed |
| ψ_d, ψ̃ | desired heading and heading error ψ̃ = wrap(ψ_d − χ); with course feedback (10.13 a) the error is taken against the course χ, not ψ | rad | proposed |
| χ, u_min | course: direction of the boat's velocity, χ = wrap(ψ + atan2(v, max(u, u_min))); u_min is the low-speed guard | rad; m/s | proposed (the sideslip atan2(v, u) has no symbol: β is the buoy bearing) |
| K_ψ, K_ω | heading-control gains (on heading error and on yaw rate); steer is in rad | none, s | proposed (replaces K_r) |
| ψ_spin, K_spin, K_s | spin-assist threshold, ramp gain and proportional gain: spin = clamp(K_s ψ̃ + sign(ψ̃)·clamp(K_spin(\|ψ̃\| − ψ_spin), 0, spin_max), ±spin_max) (10.3 b, 10.16 b) | rad; 1/rad; 1/rad | proposed |
| ψ_turn | turn threshold: while \|ψ̃\| > ψ_turn the throttle may not be negative (decision 10.2 c) | rad | proposed |
| u_cmd, u_ref | commanded forward speed and the segment's speed: u_cmd = min(u_ref·max(0, cos ψ̃), K_stop d_end) | m/s | proposed |
| K_stop | gain that slows the boat near the end, u_cmd ≤ K_stop d_end | 1/s | proposed |
| T_ff | feedforward thrust that holds a speed against the drag model, X_u u + X_uu u\|u\| | N | proposed |
| K_P, K_I, I | speed PI gains and the integral of the speed error | 1/(m/s), 1/m; m | proposed |
| δ | pod (steering) angle; positive turns the boat right | rad | proposed (the buoy offset in `design.md` §6 is d) |
| T_L, T_R | left and right thrust | N | proposed |
| w_g | gate width: distance between the centres of a gate's red and green buoys | m | proposed |
| o | gate offset: where the boat's centre crosses the gate line, from the gate centre, > 0 towards the red buoy | m | proposed |
| c_j | clearance: shortest distance from the boat's rectangle to the surface of buoy j (< 0: overlap) | m | proposed |
| L, b_w | boat length (hull length) and half width (hull spacing/2 + hull radius) | m | proposed |
| d_m | gate margin: required gap between the boat's side and a buoy's surface | m | proposed |
| X_u, X_uu; n_R, n_RR | linear and quadratic drag in surge and in yaw (`xU`, `xUU`, `nR`, `nRR` in `asv.yaml`) | | proposed |

---

## Terms

**Accuracy, precision.** *Accuracy*: how close the estimate is to the truth
(error against ground truth). *Precision*: how much results scatter over repeated
runs. The evaluation reports both: the mean error (bias) and the spread.

**Allan deviation (ADEV).** A way to see what kind of noise a sensor has. Average
the signal over windows of length τ and look at how much neighbouring averages
differ. White noise falls as 1/√τ. A Gauss-Markov bias (our model) adds a hump:
the curve rises to a peak of about 0.62·σ_ss at τ ≈ 1.9·T_c and then falls again.
Real IMUs also show a flat bottom, called *bias instability*, caused by *flicker
noise*; the textbook rule "flat bottom ÷ 0.664 = bias instability" is for that flat
bottom, not for our hump. [noise-models-notes]

**ANEES.** NEES (see there) averaged over many repeated runs at the same time
step. Averaging makes the chi-square band narrow enough to be a real test.

**Anti-windup.** A guard in a PI controller: while the output is at its limit,
the integral stops growing, so it doesn't overshoot when the limit is left.

**Azimuth, heading, yaw.** *Azimuth* α (what a GNSS receiver reports) is measured
clockwise from north. *Yaw* ψ (what the filter uses) is measured counter-clockwise
from east. ψ = π/2 − α. *Heading* is the everyday word for either; in these docs
"GNSS heading" means α and "heading error" means the error in ψ. (An *azimuth
thruster* is something else: a propeller pod that can turn.)

**Bag (rosbag2).** A recording of ROS topics to a file, which can be played back
or read by a program. Jazzy stores it in the MCAP format by default.

**Baseline, base station.** *Baseline*: the distance between the two GNSS
antennas (1.657 m on our boat). *Base station*: the receiver on shore that sends
RTK corrections. Losing the base's corrections is not the same as losing the sky.

**Bias (turn-on and wandering).** A sensor error that does not average out. A
*turn-on bias* is a fixed offset drawn once at start-up. A *wandering bias*
changes slowly over time (modelled as a Gauss-Markov process).

**Bridge (ros_gz_bridge).** The program that copies messages between Gazebo's
own message system and ROS 2 topics. Its topic list is generated into
`config/asv_bridge.yaml`.

**Callback, executor (ROS 2).** A callback is a function the node hands to
ROS, which calls it when something happens: a message arrives (subscription)
or a timer fires. The executor is the loop (`rclcpp::spin`) that calls them,
by default one at a time in one thread, so two callbacks never run at once
and need no locking. The waypoint follower (M1.3) stores the latest odometry
in one callback and computes the commands in a timer callback.

**Centroid bias.** The lidar only sees the near side of a buoy, so the centre of
its points lies closer to the boat than the buoy's real centre, by about π·r_b/4
for a round buoy of radius r_b.

**cgroup, container limits.** Linux groups the processes of a container in a
*cgroup* and counts their CPU time and memory together. Docker limits use it:
`--cpuset-cpus` lets the container use only the listed cores, `--memory` caps
its RAM (above it the kernel kills a process, see OOM kill).

**Chi-square (χ²) band.** If the filter's uncertainty is honest, certain error
measures (NEES, NIS) follow a chi-square distribution. The band is the range
where 95 % of values should fall. Above the band: the filter is over-confident
(P too small). Below: under-confident (P too large).

**Clean / realistic data.** *Clean*: perfect sensor data from Gazebo, without
noise. *Realistic*: the same data after our corruptor has added noise, biases and
outages.

**Clearance.** The gap between the boat and a buoy: the shortest distance from
the boat's outline (a rectangle, turned with the heading) to the buoy's surface.
Zero or negative means they touch, i.e. a collision.

**Coloured noise.** Noise that is correlated over time (not white), e.g. the
slowly varying GNSS error.

**Common random numbers.** Using the same random noise draws when comparing two
settings, so differences in the results come from the settings and not from luck.
Achieved with one random stream per sensor.

**Consistency (of a filter).** A filter is consistent if the uncertainty it
reports (P) matches its actual errors. Checked with NEES and NIS.

**Constant-velocity model, CTRV.** A motion model in which the prediction assumes
speed and turn rate stay the same; changes are absorbed by the process noise. D5 option b (not chosen)
holds forward speed, sideways speed and turn rate constant in the boat frame,
which is a CTRV-type model (constant turn rate and velocity), not the
straight-line constant-velocity model.

**AP_DDS, EKF3.** AP_DDS is ArduPilot's built-in ROS 2 link (topics `/ap/...`).
EKF3 is ArduPilot's own state estimator; the orientation in its IMU topic comes
from it.

**NED/FRD vs. ENU/FLU.** Two axis conventions. ENU (east, north, up) for the world
and FLU (forward, left, up) for the boat are the ROS standard (REP-103) used in
the simulation. ArduPilot uses NED (north, east, down) and FRD (forward, right,
down).

**Low-pass filter.** Removes fast changes from a signal; the price is a delay.

**Noise profile.** A file `asv_sim/config/noise_profiles/<name>.yaml` that sets
the corruptor's multipliers, switches and seed. The "white" profile turns every
extra effect off (white noise only), for the matched check.

**Real-time factor.** Simulated seconds per wall-clock second (1.0 = real time).

**Core-seconds per second.** CPU load written as how many seconds of CPU time
the code needs for one second of data: 0.25 means a quarter of one core. Unlike
"CPU %" it does not depend on how many cores the machine has. Linux reports a
thread's CPU time in `/proc/<pid>/task/<tid>/schedstat` (with the time it
waited for a free core).

**Corruptor.** Our node that turns clean data into realistic data.

**Covariance.** A matrix describing how uncertain an estimate is (and how
uncertainties are related). Its diagonal holds the variances (σ²).

**Covariance inflation.** Deliberately making a noise covariance larger than the
true noise, to cover effects the model leaves out (e.g. correlated errors).

**Data association.** Deciding which known buoy a new measurement belongs to, or
whether it is a new buoy. [data-association-map-management-notes]

**Degrees of freedom (NEES, NIS).** The number of values being checked: 3 for
(x, y, ψ), 2 for a buoy or a GNSS position, 1 for the heading.

**EKF (extended Kalman filter).** A Kalman filter for non-linear models: it
linearises the models around the current estimate using Jacobians.
[probabilistic-state-estimation-notes]

**EKF-SLAM.** An EKF whose state contains both the boat's pose and the positions
of all buoys, so it estimates the map and the boat together.
[ekf-slam-formulation-notes]

**ENU, ECEF, WGS84.** *ENU*: a local flat frame with x = east, y = north, z = up,
around a chosen reference point. *ECEF*: Earth-centred Cartesian coordinates.
*WGS84*: the standard model of the Earth's shape (an ellipsoid) used by GPS.
Converting latitude/longitude to ENU goes through ECEF. [coordinate-frames-notes]

**Extrinsics, lever arm.** *Extrinsics*: where a sensor is mounted on the boat and
how it is rotated. *Lever arm*: the offset from the boat's reference point to the
sensor. Our main GNSS antenna is 0.83 m from base_link, so its position is not the
boat's position.

**Fake detector.** A program that makes buoy measurements directly from ground
truth plus controlled noise, instead of from lidar and camera data.

**Feedforward.** The part of a control command computed from a model instead of
from the error: here the thrust the drag model says is needed for the commanded
speed. The feedback (PI) part only corrects what the model gets wrong.

**Frame, TF, TF tree.** A *frame* is a coordinate system (world, map, base_link,
lidar_link, ...). *TF* is the ROS system that knows how frames are placed
relative to each other. The *TF tree* is the set of all these links; each frame
can have only one parent.

**Gate (course).** Two buoys, red and green, that the boat must pass between
(Task 1 has two). Not the same as the Mahalanobis gate below. The gate monitor
reports each crossing as PASS, FAIL_CLOSE (too close to a buoy) or
FAIL_OUTSIDE (went round a buoy).

**Gate (Mahalanobis gate).** A test that rejects a measurement if it is too far
from where a buoy is expected, measured in units of the expected uncertainty
(Mahalanobis distance) rather than in metres.

**Gauss-Markov process (first order).** A random signal that changes slowly:
each step it keeps most of its old value and adds a little fresh noise. Two
numbers describe it: its typical size σ_ss and its correlation time T_c (how long
it "remembers"). Used for wandering biases and slowly varying GNSS error.

**GNSS Doppler velocity.** The boat's velocity measured by the GNSS receiver from
the Doppler shift of the satellite signals. Lost together with the position in a
sky-view outage.

**Gravity removal.** An accelerometer also measures gravity. To use it for
motion, the gravity part must be subtracted, which needs roll and pitch; an error
of 1° in them leaves about 0.17 m/s² of false acceleration (9.81 · sin 1°).

**Ground truth.** The exact, true value (pose of the boat, positions of buoys),
known only in simulation. Used to measure the filter's error.

**Headless.** Running Gazebo without its window, e.g. for automated runs.

**Hungarian assignment.** An algorithm that pairs two sets of points one to one
so that the total distance is smallest. Used to match estimated buoys to true
buoys.

**Innovation.** The difference between what a sensor measured and what the
filter predicted it would measure. *S* is its expected covariance.

**Inverse crime.** Testing a method on data made with exactly the model the
method assumes. The results then look better than reality. Example: using
Gazebo's own drag numbers inside the filter. [noise-models-notes, kaipio2007statistical]

**Jacobian.** The matrix of partial derivatives of a model. The EKF uses it to
linearise. We check each one against a numerical derivative.

**Joseph form.** A way of writing the covariance update that keeps P symmetric
and positive despite rounding errors.

**Launch file, OpaqueFunction.** A *launch file* (Python) starts several ROS
nodes and Gazebo with one command and gives them their parameters. An
*OpaqueFunction* runs plain Python while the launch starts, e.g. to read
`asv.yaml` and compute values before a node starts.

**Latched topic.** A topic whose last message is kept and delivered to
subscribers that join later, but only if the subscriber also asks for it (QoS
durability "transient local"; in RViz, set the display's Durability to Transient
Local) and only while the publishing node is still running. A default subscriber
gets nothing. Used for things published once, like the true buoy map.

**Level frame (ℋ).** A frame that turns with the boat's heading but stays
horizontal (no roll or pitch). Lidar and camera points are rotated into it before
use (thesis eq. 3.3).

**Line-of-sight (LOS) guidance.** Steering towards a point a fixed distance
(the lookahead Δ) ahead on the path, instead of towards the next waypoint. Far
from the path the boat heads back to it; on the path it follows it. Here
the lookahead is measured along the route, so near a corner the target point
already lies on the next segment (decision 10.6 a in `milestone1_plan.md`).
*Course-based LOS* (Fossen): the LOS direction ψ_d is compared with the
course χ (where the boat moves), not the heading ψ (where the bow points), so
sideways drift is corrected too (decision 10.13 a).

**Matched / mismatched.** *Matched*: the filter's noise settings equal the noise
the corruptor actually adds (and the corruptor adds nothing the filter does not
model). *Mismatched*: they differ, as they always do on a real boat.

**Monte Carlo runs.** Repeating the same experiment many times with different
random noise (different seeds) to see the spread of results.

**NEES (normalised estimation error squared).** The actual error, weighed by the
filter's own uncertainty: eᵀP⁻¹e. If the filter is honest, its average equals the
number of values checked (3 for x, y, ψ).

**NIS (normalised innovation squared).** The same idea as NEES, but for
innovations. Needs no ground truth, so it also works on a real boat.

**Noise density.** How strong white noise is, independent of the sample rate (e.g.
rad/s/√Hz). The noise per sample is the density divided by √dt (one convention;
TDK's definition gives a value √2 smaller: see `design.md` §4.1).

**OOM kill.** When a cgroup (or the whole computer) runs out of memory, the
kernel's out-of-memory killer ends a process. In the benchmark it marks the RAM
size that is too small.

**OSPA.** A single number that measures map error including missing and extra
buoys. Optional.

**Percentiles (p50, p95, p99).** The value below which 50, 95 or 99 % of the
measurements lie. p50 is the median. For timing, p99 shows the slow cases that
the mean hides, and is steadier than the single maximum.

**Physics step.** Gazebo's fixed time step (4 ms in our world). Sensors can only
produce a sample on a physics step.

**Process noise, measurement noise.** *Process noise*: how much the motion model
can be wrong per step. *Measurement noise*: how wrong a sensor reading can be.
Decided (D14 a, 2026-10-11): R is the process noise covariance and Q the
measurement noise covariance, as in Thrun and in the Section 2.1 notation table.
Careful when reading sources: Kalman-filter texts such as Bar-Shalom use the
opposite, and so do the literature notes files (noise-models,
ekf-slam-formulation, vehicle-motion-model). Code uses names without letters
(`process_noise_cov`, `meas_noise_cov`).

**QoS (quality of service).** ROS 2 settings for how messages are delivered
(reliable or best effort, kept for late joiners or not). Publisher and subscriber
must be compatible or they don't connect.

**Release build.** Compiling with optimisation (`-DCMAKE_BUILD_TYPE=Release`).
Without it, Eigen code can be many times slower, so timings would be
meaningless.

**RMSE.** Root mean square error: the square root of the average squared error.
For the cross-track error e (M1.5): e_rms = sqrt(mean(e_k²)) over the RUNNING
phase; unlike max |e| it says how close the boat stays on average.

**rosbag2_py.** ROS 2's Python library for reading bags (part of the ROS
install). The time stored with each message is when the recorder received it;
with `--use-sim-time` that is simulation time. `tools/analyze_run.py` uses it.

**RSS, PSS (memory).** *RSS* (resident set size): the RAM a process uses now,
including shared libraries; its peak is `VmHWM`. *PSS*: the same, but each
shared page is split between the processes that share it, so PSS values can be
added up.

**RTK fixed / float / standalone.** GNSS accuracy levels. *Standalone*: no
corrections, metre level. *RTK float*: corrections used but not fully solved,
decimetre level. *RTK fixed*: fully solved, centimetre level.
[gnss-aided-navigation-notes]

**rosidl, interface package.** rosidl turns `.msg` files into C++ and Python
types. Custom messages usually go in their own package (here `asv_msgs`).

**Seed.** The starting number of a random generator. The same seed gives the same
"random" noise, which makes runs repeatable.

**Simulation time (`/clock`, `use_sim_time`).** Gazebo has its own clock, which
can run slower or faster than real time and is published on `/clock`. A node with
`use_sim_time:=true` uses that clock instead of the computer's (but a "wall
timer" still runs on the computer's clock).

**Saturation.** A command larger than the actuator can give (e.g. more than
50 N forward thrust). Something has to give way; `mix()` keeps the left/right
thrust difference (the turn) and lowers both sides (M1.2a).

**Sky-view outage.** The antennas can't see the satellites (e.g. under a bridge):
position, Doppler velocity and dual-antenna heading are all lost. Compare with
losing only the base station's corrections, where the receiver falls back to
standalone position but keeps its heading.

**SLAM.** Simultaneous localisation and mapping: estimating where you are and
building the map at the same time. [slam-problem-notes]

**Spin assist.** Turning the boat with opposite thrust on the two thrusters
(left back, right forward turns left), used when the pods alone turn too slowly.
From M1.2f it is also used for small errors (K_s): differential thrust turns
the boat without the sideways push of the aft pods (10.16 b).

**Static / floating model.** In Gazebo a *static* model never moves. Our buoys
are currently *floating*: they bob on the waves and can be pushed. Neither is a
moored buoy (one that moves in a small circle around its anchor).

**State machine.** A fixed set of states (e.g. WAITING, RUNNING, FINISHED,
STOPPED) and the rules for moving between them. At any time the system is in
exactly one state, which decides what it does. A *latched* state is one it
never leaves (the follower's FINISHED and STOPPED, M1.3 S3 a).

**Surge, sway, yaw rate (u, v, ω).** Forward speed, sideways speed and turn rate,
all in the boat frame. [vehicle-motion-model-notes]

**3-DOF manoeuvring model.** A boat model with three degrees of freedom: surge,
sway and yaw (no heave, roll, pitch). Newton's law in each with the thrust and
the drag; used in the closed-loop test (M1.2e) in place of Gazebo. No added
mass, because `model.sdf` has none.

**Course, crabbing.** *Course* χ: the direction the boat actually moves.
It differs from the heading ψ when the boat also moves sideways (v ≠ 0).
*Crabbing*: the bow points one way while the boat drifts another.

**Non-minimum-phase response.** The first reaction goes the wrong way. Our
pods sit behind the centre of gravity: to turn right they push the stern left,
so the whole boat first slides left before it turns (found in M1.2f).

**Zero-order hold.** A command is held constant until the next one arrives
(here for the 50 ms between control steps).

**Explicit Euler.** The simplest way to step a differential equation:
x(t + h) = x(t) + h·ẋ(t). Accurate when the step h is small (here 2.5 ms).

**Sweep.** Running the same experiment over a list of settings (e.g. noise levels)
and collecting the results in one table.

**Thrust allocation (mixing).** Turning what the boat should do (go forward,
turn) into a command for each actuator (two thrusts, two pod angles). Our
allocator is `asv_sim::mix()`.

**Trajectory alignment.** Shifting and rotating an estimated path so that it best
fits the true path before measuring the error. Common in visual SLAM; we don't do
it, because GNSS already ties the estimate to the world frame.

**Unit test.** A small automatic test of one function, run without the simulator
(here with gtest for C++ and unittest for Python).

**Van Loan method.** A standard way to turn a continuous-time noise model into
the process noise for one discrete time step. [noise-models-notes]

**White noise.** Random noise that is independent from one sample to the next.

**Wrapping (angles).** Bringing an angle back into a fixed range, e.g. (−π, π],
so that 359° and −1° count as the same direction. Every angle difference must be
wrapped.
