# Master plan: GNSS-aided EKF-SLAM in the RoboBoat simulation

Version 1.4, 2026-10-14. Replaces draft v0 (`master-code-plan.md` in the Claude
project folder). The change log at the end lists what changed.

**How to use this plan**

- This file says **what** gets built and **in which order**. The details of
  **how** each part works are in [`design.md`](design.md). Symbols and unfamiliar
  words are explained in [`glossary.md`](glossary.md).
- Work happens one **step** at a time (M2.3, M3.1, ...). Before a milestone
  starts, Claude Code writes a short milestone plan (`docs/plans/mN_plan.md`, from
  `docs/plans/TEMPLATE.md`) that expands these steps. Marcell approves it before
  any code is written. The rules for this are in `CLAUDE.md`.
- Each step names the **thesis section** it feeds (by title, because the
  section numbers in the draft are still moving). Equation numbers (eq. 3.3 etc.)
  are those of the draft PDF of 2026-10-08.
- Status marks: `[ ]` not started, `[~]` in progress, `[x]` done, `[?]` waits for
  a decision (the D-numbers are in the step's text; see section 3), `[-]` skipped
  (Marcell chose not to do it).
- With Marcell's OK, milestones may be reordered and a big step may be split into
  sub-steps (M3.3a, M3.3b, ...).

---

## 1. What we are building, in one paragraph

Gazebo simulates the boat, the water and the course, and produces **perfect**
sensor data plus the **true** pose of the boat (ground truth). Our own node, the
**corruptor**, turns the perfect data into **realistic** data by adding noise,
biases and outages that we control exactly. A **detector** produces buoy
measurements (range, bearing, colour). The **EKF-SLAM filter** uses the realistic
data to estimate where the boat is and where the buoys are. The **evaluation**
compares the estimate with ground truth. Because every noise setting is a
parameter, we can **sweep** through noise levels and measure how the accuracy and
consistency of the estimate change, which is the thesis question.

The live system (for driving the boat and watching in RViz):

```
 Gazebo + VRX  (perfect sensors, true pose)
   │ ros_gz_bridge
   ├──► /asv/ground_truth/odom ─────┬──────────────────────────────┐
   │                                │                              │
   ▼ /asv/{imu,gnss,lidar}/...      ▼                              │
 corruptor                   detector, one of:                     │
 (noise, biases, GNSS        • fake detector (M3): ground truth    │
  modes and outages,           + buoys_gt.yaml, controlled noise   │
  heading sensor)            • real perception (M5, if time):      │
   │                           corrupted lidar + camera images     │
   │ /asv/sensors/...               │ /asv/perception/buoys        │
   │                                ▼                              ▼
   └─────────────────────────► EKF-SLAM ──► /asv/slam/... ──► evaluation
                                    │                       (errors, NEES)
                                    ▼
                              RViz (live view)

 Thruster commands come from the waypoint follower (steers on ground truth,
 idea 3), the maneuver player (open loop) or keyboard teleop.
```

How the numbers in the thesis are made (offline, idea 2):

```
 Gazebo run ──► bag (clean sensors + ground truth)
                  │
                  ▼
 slam_replay: corruptor → fake detector → EKF-SLAM, in timestamp order
                  │
                  ▼
 run folder (trajectory, map, innovations) ──► asv_eval ──► results table, figures
```

The corruptor also reads ground truth, to emulate the dual-antenna heading.
The live system is for driving, watching and checking; the thesis results come
from the replay path. The topic names follow decision D3 (section 3). The full
topic table is in `design.md` §1.

Four ideas carry the design. Each one is a decision in section 3.

1. **Perfect data from Gazebo, noise from our own node (D1).** Gazebo's built-in
   noise can't do what the thesis needs. GNSS noise is applied in degrees
   instead of metres, there is no time-correlated GNSS error, no RTK
   fixed/float/standalone behaviour, no dual-antenna heading sensor, and the IMU
   orientation is always perfect. In our own node every noise parameter sits in
   one file, has a random seed, and can be changed without restarting Gazebo.
2. **Record once, replay many times (D4).** Each trajectory is driven once in
   Gazebo and recorded as a rosbag of perfect data. Every noise level then reruns
   corruptor, detector and filter over the same recording, so all settings see
   exactly the same motion, the results repeat exactly for the same seed, and the
   runs go faster than real time. A Gazebo run itself is not exactly repeatable,
   because the commands from ROS reach Gazebo asynchronously and land on slightly
   different physics steps each time.
3. **Drive on ground truth, estimate on the side.** In experiments the boat is
   steered with the true pose, so the path does not depend on how good the
   estimate is. Steering with the estimate ("closed loop") is an optional extra
   (M7).
4. **Fake detector first, real perception only if time allows (D8).** A fake
   detector makes buoy measurements from ground truth plus controlled noise, with
   the buoy's true colour. That lets the filter be built and tested before the
   real lidar and camera pipeline exists, and it separates filter error from
   perception error in the results. Real perception (M5) comes only if time is
   left.

---

## 2. Fix first

These are known problems, found while checking the repository (GitHub commit
6b8be2b of 2026-10-09, the latest push). They come before new features.

1. **The simulated GNSS noise is about 890 m, not 8 mm.** The generator writes
   `horizontal_noise: 0.008` (metres) into Gazebo's NavSat noise, which Gazebo
   applies in **degrees**. 0.008° is about 887 m north and 791 m east at the
   course's latitude (27.37°). See `models/asv/model.sdf:514` and `:537`. Any
   data recorded so far with this model has useless GNSS. Fixed by M2.2.
2. **The C++ code that the old docs describe was never written.**
   `milestone1_plan.md` and `CONTEXT.md` say that `asv_sim::mix()` "already
   exists, unit tested" and that 8 C++ tests pass. (The old `CLAUDE.md` correctly
   says there is no C++ code yet.) No commit has ever contained a C++
   source file of ours, and `CMakeLists.txt` says "No C++ targets yet". `mix()`
   and the C++ test setup are written in M1.2; M0.1 corrects the stale
   statements.
3. **Nothing uses simulation time.** Gazebo stamps every message with simulation
   time (starting at 0), but no node sets `use_sim_time`, so timers and `now()`
   run on the wall clock. That breaks TF lookups, the 20 Hz control loop and any
   timed manoeuvre once Gazebo runs slower than real time. Fixed by M0.2.

---

## 3. Decisions

Each decision has a recommendation. **Marcell decides.** His answer goes into the
Status column as `decided: <option>, <date>: <his reason, in his words>`, and the
reason goes into the thesis section named in the last column. A step that
depends on an open decision is marked `[?]`.

Marcell decided most of them on 2026-10-11. Still open: D6, D10 and the
exact content of the IMU message (D11). Background and options for these are in
[`decision_notes.md`](decision_notes.md). D5 was decided C on 2026-10-14. D11 must be settled before M2.4
and M2.6; D6 before M3.3b; D10 before M3.6.

| ID | Question | Options and consequence | Recommended | Decide before | Status | Justified in thesis |
|---|---|---|---|---|---|---|
| D1 | Where is sensor noise added? | **a)** Our corruptor node: full control, correct units, seeds, no Gazebo restart. **b)** Gazebo's built-in noise: less code, but wrong GNSS units, no GNSS modes, outages or heading sensor, perfect IMU attitude. | a | M2 | decided: a, 2026-10-11 | Simulation Environment → Vehicle and Sensor Models |
| D2 | How is Gazebo's own noise switched off? | **a)** A switch in `asv.yaml` (`gazebo_sensor_noise: false`) that the generator obeys: the datasheet values stay in `asv.yaml` and also feed the corruptor. This is a generator change. **b)** `make_asv.py --set ...=0`: no code change, but the next plain `make_asv.py` run silently brings the noise back. Either way only the IMU, NavSat and lidar noise is switched off; camera pixel noise stays in Gazebo, because the corruptor does not touch images. | a | M2.2 | decided: a, 2026-10-11 | Vehicle and Sensor Models |
| D3 | Which topic names do clean and noisy data use? | **a)** Bridge topics are remapped in the launch file to `/asv/clean/...`; the corruptor publishes on the normal names (`/asv/imu/data`, ...). A filter left on the default names then can't get perfect data by accident, and on the real boat those default names are simply remapped to the real sensor topics (`/ap/...`). **b)** Bridge keeps the normal names; noisy data goes to `/asv/sensors/...`. Simpler, but a node left on a default name silently gets perfect data. | a | M2.3 | decided: **b**, 2026-10-11 | Software Architecture |
| D4 | How do sweeps get their data? | **a)** Record once, replay many (idea 2). **b)** A fresh Gazebo run per setting: slower, and every setting sees a slightly different trajectory. | a | M2.6 | decided: a, 2026-10-11 | Experimental Scenarios |
| D5 | What is the filter's state and motion model? | Three options, compared side by side in `design.md` §6.1: **a)** IMU-driven, **b)** constant body velocity and turn rate (CTRV-type) with sway and gyro bias; the gyro is a measurement, **c)** pose only (eq. 3.6 as drafted). The table shows what each puts in the state, how it uses the gyro and the accelerometer, what happens in a GNSS outage, and what it changes in eq. 3.6. Note: with b the accelerometer is not used, so the IMU study covers only the gyro. | b, but this is your Ch. 3 choice | M2.6 (the corruptor must produce what the model needs) | decided: **C** (pose + gyro bias, GNSS velocity and gyro as inputs; option c plus a gyro-bias state), 2026-10-14: "The asv will allways be in view of the landmarks, the course will be with view of the open sky without any building over it, so GNSS outages are few if any." Comparison in `decision_notes.md`. | Vehicle Motion Model, EKF-SLAM Formulation |
| D6 | How does the filter treat the slowly varying (coloured) GNSS error? | **a)** As white noise with an enlarged covariance (simple; assumption A3 stays as it is and is tested). **b)** Add the slow GNSS error to the state (two extra states, reset at a mode switch; about half a day to a day more in M3.3b). | b in M3.3b, a kept as a switch for the A3 test (because D9 a makes GNSS modes core; `decision_notes.md`). Fallback if time is short: a with one factor per GNSS mode. (Changed 2026-10-11 after the research; was "a".) | M3.3b | open; background in `decision_notes.md` | Noise Models, Further Assumptions (A3) |
| D7 | Where does the true buoy map come from? | **a)** Buoys made static for the experiments (a change to the course generator); a tool reads their exact positions from the world and model files. The thesis states "buoys do not move" as a limitation. **b)** Buoys float; their true poses are logged during each run. More work. Neither option simulates a moored buoy (one that moves in a small circle around its anchor), which is what the draft's Static Landmarks section describes. Either way, one floating run measures how far free buoys drift. | a | M2.7 | decided: a, 2026-10-11: "get buoy position from the course file" | Static Landmarks; World and Course Elements; Ground Truth and Data Recording |
| D8 | Fake detector first, real perception later? | **a)** Yes: the filter can be finished without perception, and results separate filter from perception error. **b)** Build perception first. | a | M3 | decided: a, 2026-10-11: "Get true color of buoys then if we have time add some detection" | Experimental Scenarios |
| D9 | Which experiments are core? | The draft's Problem Statement promises behaviour under "sensor noise, wave conditions and GNSS outages". **a)** GNSS modes and outages become core (cheap with replay), plus a small wave study (2-3 wave heights, one recording each). **b)** Keep waves optional and change the Problem Statement. | a | M6 | decided: a, 2026-10-11 | Problem Statement, Environmental Conditions, Sensitivity Analysis |
| D10 | How many repeated runs (seeds) per setting? | More runs give tighter confidence bounds. For the average NEES (3 pose states) the 95% band is 2.02-4.16 with 20 runs and 2.36-3.72 with 50 runs. | decision rule in `decision_notes.md` (after M3.5): 50 everywhere if S × 50 × t_run / cores fits in one night; else 50 for the matched check and nominal, 20 for the rest; never below 20 | M3.6 | open: Marcell decides "when I have knowledge of the simulations run speed" (2026-10-11); M0.2 and M3.5 measure the speeds | Filter Consistency |
| D11 | What does the corrupted IMU message carry? | Direction decided 2026-10-11: imitate the real ArduPilot IMU topic. **A)** Gyro and accel with datasheet noise, plus ArduPilot's processing as switches (low-pass, 50 Hz hold, publish-time stamp); no orientation; a roll/pitch message only if something needs it. **B)** Like A, plus a full orientation (incl. yaw) with EKF3-like errors [EST]. **C)** Ideal raw IMU: datasheet noise only, separate roll/pitch (the old option a). Also: which processing level is nominal (ideal / tuned / Rover default). Details in `decision_notes.md`. | A; nominal level per what the team will fly (`decision_notes.md`) | M2.4 | direction decided 2026-10-11: "Corrupted IMU messages should follow real world scenarios"; the exact message content is open (`decision_notes.md`) | Vehicle and Sensor Models, Planar Motion (eq. 3.3) |
| D12 | Which heading accuracy is the nominal value? | **a)** Unicore datasheet 0.1° RMS at 1 m baseline (already in `asv.yaml`), giving 0.060° at our 1.657 m baseline. **b)** ArduSimple page 0.14° at 1 m, giving 0.084°. Cite whichever you choose. | a | M2.6 | decided: a, 2026-10-11 | Sensor Configuration; Noise Models |
| D13 | Where does the ROS-free maths live? | **a)** A new package `asv_core` (no ROS, Eigen only) that every other package uses. **b)** Headers exported from `asv_sim`: then the filter package depends on the simulator package. | a | M2.1 | decided: a, 2026-10-11 | Software Architecture, App. C Software Structure |
| D14 | Which letters name the noise covariances? | **a)** Thrun: R = process noise, Q = measurement noise. **b)** Kalman / Bar-Shalom: Q = process, R = measurement; your literature notes files use this. Either way R also names the rotation matrices in eq. 3.3, so one of them may need another letter. The code uses names without letters, so only symbols in docs and comments depend on this. | the same letters as your Section 2.1 notation table (the table from the writing-guide thread uses a) | M3.3 | decided: a, 2026-10-11: "use a) but keep names in code" | Probabilistic State Estimation (2.1); all Ch. 3 equations |
| D15 | Does the GNSS lever arm include roll and pitch? | **a)** Planar: h = [x, y] + C(ψ)·l_xy, as the Planar Motion section says ("attitude errors only enter through the projection"). The tilt error (1.3-1.9 cm at 5° roll or pitch alone, up to 2.8 cm when both are 5°) is covered by the GNSS covariance and stated as a limitation. **b)** 3D lever arm with the measured roll and pitch: more accurate, but attitude errors then also enter the GNSS update, and Planar Motion must say so. | a; its effect shows up in the wave study | M3.3 | decided: a, 2026-10-11: "discuss further when in that part of the coding" (the M3.3b brief raises it again) | Planar Motion; Observation Models |
| D16 | Is the onboard benchmark part of the thesis? (`design.md` §14) | **a)** A small part: the EKF's cost against map size (it backs assumption A6: the update cost "grows quadratically with the state dimension") and one resource table from the replay. **b)** Team engineering only: the steps run and help choose the boat computer, but the numbers stay out of the thesis. | a, but it is your call | M3.7 | decided 2026-10-11, neither option for now: "Leave out benchmarking for now, first lets get a working sim environment" | Software Architecture; Further Assumptions (A6); Discussion and Limitations |
| D17 | Which measurement tools? (`design.md` §14) | **a)** Only what is already there: `std::chrono`, `getrusage`, `/proc`, the container's cgroup files, `docker stats`. No new dependency. **b)** Also add profilers and benchmark libraries (Google Benchmark, GNU `time`, `sysstat`, `heaptrack`; `perf` also needs `cap_add: [PERFMON]` in `compose.yaml`). They show *where* the time goes, but change `docker/` and `compose.yaml`. | a; b only if a shows a problem that needs finding | M3.7 | decided: a, 2026-10-11 ("look D16": only matters if benchmarking comes back) | Toolchain (only if b) |
| D18 | Is a Foxglove dashboard or a shore link part of the work? (The old `milestone1_plan.md` planned both for Milestones 2-3.) | **a)** Optional M7.6: a team tool, not needed for the thesis; RViz covers the live view (M4.1). **b)** A core step after M4.1: more setup (a new dependency, possibly a `compose.yaml` change). | a, unless the team needs it at competition | M4.1 | decided: a, 2026-10-11: "Use RViz for realtime data visualization" | none (Software Architecture if b) |

---

## 4. Milestones and steps

Each step is one commit. "Done when" must be checkable by running something; if
only Marcell can check it (GUI, keyboard), the step says so.

### M0. Housekeeping

M0 needs no approved milestone plan: its briefs and reports are simply logged
in `docs/plans/m0_plan.md`. M0.1, M0.3 and M0.4 use the short brief; M0.2 brings
a new idea (simulation time) and gets the full brief (`CLAUDE.md`).

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[x]` M0.1 | Add the docs and correct the stale ones. Marcell confirms that nothing is left unpushed or untracked on his machine (e.g. no local `include/`, `src/`, `test/` in `asv_sim`). The new `CLAUDE.md`, `docs/` and the corrected `milestone1_plan.md` come in (by the docs pull request or by copying). Add `ws/data/` to `.gitignore` (ask first). Decide what happens to `CONTEXT.md`: remove it (ask first; the new docs replace it) or keep it marked "chat summary, not authoritative". | `git fetch` and `git status` say the branch is up to date with GitHub; `git check-ignore -v ws/data/test.bag` names the new `.gitignore` line; `grep -rn "already exists" ws/src/asv_sim/docs/milestone1_plan.md` finds nothing about `mix()`. | none |
| `[x]` M0.2 | Simulation time everywhere (`design.md` §3): every launch file sets `use_sim_time` for all its nodes; timers use the ROS clock; a small test script in `tools/`. `tools/actuator_check.py` (Step 1) times its phases with the wall clock: switch it to the ROS clock and rerun it once to confirm the Step 1 table, or record it in `design.md` §13 as a known exception. | `ros2 param get <node> use_sim_time` is true for each node; a 20 Hz test timer fires 20 times (±1) per simulated second while Gazebo runs at a real-time factor clearly different from 1 (the report states the measured factor). The report also gives the real-time factor of a normal headless run (for D10: how long each recording takes). | Integration and Time Synchronization |
| `[x]` M0.3 | Confirm installed packages in the container: `dpkg -s ros-jazzy-cv-bridge libopencv-dev ros-jazzy-rosbag2-cpp libpcl-dev ros-jazzy-yaml-cpp-vendor python3-numpy python3-scipy python3-matplotlib` and `python3 -c "import numpy, scipy, matplotlib"`. Write the result into `design.md` §12. (The benchmark tool checks are dropped while benchmarking is left out, D16.) | Result recorded. | Toolchain |
| `[ ]` M0.4 | Stop tracking build output committed by accident: `ws/build`, `ws/install`, `ws/log` (1,263 files) and 14 other `__pycache__` files, with `git rm -r --cached` (the files stay on disk). Ask first. | `git ls-files ws/build ws/install ws/log` prints nothing; the workspace still builds. | none |
| `[ ]` M0.5 | Optional: add the missing `.gitmodules` for `ws/src/vrx` (git already records it as a submodule link to commit 7609d1b, but without a URL) and remove `ws/src/vrx` from `.gitignore`; write a short `README.md` (VRX source and commit, `docker compose up -d`, build, test and run commands). Ask first. | A fresh clone builds after `git submodule update --init`. | Toolchain |

### M1. Drive the course on ground truth

M1 keeps its own plan file: `ws/src/asv_sim/docs/milestone1_plan.md`. Step N
there is M1.N here; it already has Steps 7 and 8 (= M1.7, M1.8). The briefs and
reports of all M1 steps are saved there. Its section 10 lists open points in the
Step 2 design (found 2026-10-10) that the M1.2 brief must settle with Marcell.

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[x]` M1.0 | Package skeleton and sim launch file (Step 0, commit 9cc9fe7). | done: build clean, 28 Python tests, ground truth at about 46 Hz | none |
| `[x]` M1.1 | Actuator check (Step 1, commit f830b90). | done: drag model and steering sign confirmed | Vehicle and Sensor Models |
| `[ ]` M1.2 | Guidance and control logic, pure C++ (Step 2), **including `mix()`** (throttle, steer, spin → the four thruster commands, clamped to the limits in `asv.yaml`) and the first C++ and gtest setup in `asv_sim` (⚠ new `test_depend` `ament_cmake_gtest`). The brief settles the open points in `milestone1_plan.md` section 10. | as in `milestone1_plan.md` | Experimental Scenarios (how the test runs are driven); App. C |
| `[ ]` M1.3 | Waypoint follower node (Step 3). Uses simulation time and a ROS-clock timer (M0.2). ⚠ New dependencies `rclcpp`, `std_msgs`, `nav_msgs`, `geometry_msgs`. | as in `milestone1_plan.md` | Experimental Scenarios; App. C |
| `[ ]` M1.4 | One-command launch `task1.launch.py` (Step 4); sets `use_sim_time` for all nodes. | as in `milestone1_plan.md` | none |
| `[ ]` M1.5 | Tuning and evaluation with `tools/analyze_run.py` (Step 5); bags go to `ws/data/bags/`. | as in `milestone1_plan.md` | Experimental Scenarios |
| `[ ]` M1.6 | Documentation `AUTONOMY.md` (Step 6). | as in `milestone1_plan.md` | App. C |
| `[ ]` M1.7 | **Maneuver player**: plays a timed list of (t, throttle, steer, spin) from YAML through `mix()`, open loop, timed in simulation time; waits for the first `/clock` message before starting. For simple, repeatable test runs (straight line, circle, zig-zag). | Same maneuver run twice; the largest difference between the two true trajectories is measured and reported. | Experimental Scenarios |
| `[ ]` M1.8 | **Keyboard teleop (WASD)** in C++. Calls `mix()`. W/S change the throttle level, A/D the steer level and Q/E the spin level, in steps; Space sets everything to zero. Levels stay set after the key is released, because a terminal sends no key-release events. Publishes zero on exit. The key-to-command logic is a ROS-free function with unit tests; the node only reads keys and publishes. Started only by Marcell in his own terminal (`ros2 run asv_sim teleop --ros-args -p use_sim_time:=true`), never from a launch file (launched nodes get no keyboard). Never runs together with the follower or player. | Unit tests pass. Marcell drives the boat with it (Claude Code can't press keys, so it can't test this part). | none (tool) |

### M2. From perfect to realistic sensor data

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[ ]` M2.1 | **Geodesy helper** in the new ROS-free package `asv_core` (D13 a; ⚠ first new package, `<depend>` on Eigen): latitude/longitude ↔ local east/north/up, using the world's reference point (read from the world SDF) and the WGS84 ellipsoid, the same as Gazebo. Same maths in Python for checks and evaluation. | Unit tests: the reference point maps to (0, 0, 0); a point 100 m east and 100 m north converts there and back within 1 mm; C++ and Python give the same numbers. | Coordinate Frames and Transformations |
| `[ ]` M2.2 | **Switch Gazebo's IMU, NavSat and lidar noise off** (D1 a, D2 a: ⚠ a generator change and a new `asv.yaml` key `gazebo_sensor_noise`) and regenerate; camera noise stays. Update `HOW_IT_WORKS.md` for the switch. | The regenerated `model.sdf` has no `<noise>` in the IMU, NavSat and lidar sensors and still has it in the cameras. On a short recording, each clean GNSS fix, converted with the geodesy helper, equals the true antenna position (ground-truth pose interpolated to the fix stamp, plus the lever arm rotated by the true roll, pitch and yaw). The report gives the largest difference (expected: a few mm; one physics step off at 2 m/s would show as 8 mm). | Vehicle and Sensor Models |
| `[-]` M2.3 | **Topic names**: not needed as a step after D3 b (2026-10-11). The bridge keeps its names (`/asv/imu/data`, `/asv/gnss/fix`, `/asv/lidar/points`: clean data); the corruptor publishes realistic data under `/asv/sensors/...` (M2.6). The topic table in `design.md` §1 is already updated. | skipped | Software Architecture |
| `[?]` M2.4 | `asv_msgs` package (⚠ new package and dependencies, rosidl): BuoyDetection(Array) with an UNKNOWN colour, Landmark(Array), GnssHeading, SensorStatus, and a separate roll/pitch message only if the D11 choice needs one. (The GNSS velocity for D5 C needs no new type: geometry_msgs/TwistWithCovarianceStamped.) | Builds; every field documented with its unit and frame. | Software Architecture |
| `[ ]` M2.5 | **Noise library** in `asv_core` (D13 a): white noise, turn-on bias, slowly wandering (Gauss-Markov) bias, one random stream per sensor, time steps taken from timestamps. | The three noise tests in `design.md` §4.5 pass (white noise alone; Gauss-Markov alone with a short test correlation time; both together at τ = 1, 10 and 30 s against the analytic total). | Noise Models; Vehicle and Sensor Models |
| `[?]` M2.6 | **Corruptor node** (D1 a, D4 a, D5 C, D12 a; open: D11): publishes realistic data under `/asv/sensors/...` (D3 b). IMU (message content per D11); GNSS position with fix modes and outages; GNSS (Doppler) velocity (D5 C); dual-antenna heading, lidar range noise (only needed for real perception, M5); logs every true bias. Nominal values read from `asv.yaml`; noise profiles hold only multipliers and overrides. Update `HOW_IT_WORKS.md` §5.4 and §14: the heading is now emulated by the corruptor (`design.md` §4.3). | Unit tests per sensor; on a recording, the error between corrupted and clean data has the configured mean and spread. A test proves no output contains the true yaw. | Vehicle and Sensor Models; Sensor Configuration; App. B Parameter Tables |
| `[ ]` M2.7 | **True buoy map** (D7 a): a Python tool reads the buoy positions from the course files (task pose in `roboboat_course.sdf` × buoy pose in the generated `rb26_task*` model) and writes `buoys_gt.yaml` (name, colour, class, radius, position); a latched topic publishes it. Set `static=True` for the buoy includes in `make_roboboat2026_course.py` (course generator: ask first) and regenerate; Marcell checks in the Gazebo window that the static buoys sit at the waterline. One run with the original floating buoys measures their drift. | Unit test: Task 1 gate 1 red buoy is at (5.0, 1.524) m. Every buoy include in the regenerated `rb26_task*` models has `<static>true</static>`. Drift result (largest movement per buoy in 5 minutes) written down. | Static Landmarks; World and Course Elements; Ground Truth and Data Recording |
| `[ ]` M2.8 | **Frames and TF**: static world → map identity; a small node publishes world → base_link from ground truth when the filter is not running. RViz config with the lidar cloud in the world frame. | `ros2 run tf2_tools view_frames` shows one connected tree; Marcell checks RViz visually. | Coordinate Frames and Transformations; Ground Truth and Data Recording |
| `[ ]` M2.9 | **Recording profiles** `nav`, `nav_lidar` and `perception` (`design.md` §8). | For each profile, `ros2 bag info` counts equal rate × simulated duration within 1 % for the topics in that profile (IMU 200, GNSS 10, ground truth 50 Hz; lidar 10 Hz in `nav_lidar` and `perception`). | Ground Truth and Data Recording |

### M3. The filter, offline

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[ ]` M3.1 | **Fake detector** (D8 a): from ground truth and the true buoy map. Lidar gives range and bearing (360°, per-class range limits from `asv_report.json`), cameras give the buoy's **true colour** from the map (inside their field of view only). Misses, false alarms and colour mix-ups are settings (mix-ups default 0); optional centroid bias (`design.md` §5). | Unit tests: only buoys in range and view are detected; measured noise statistics match the settings. | Experimental Scenarios |
| `[ ]` M3.2 | **Measurement queue** (D13 a): puts measurements in timestamp order with a short fixed delay, so the filter never applies an old measurement to a newer pose. Shared by replay and live node. | Unit test: shuffled input (within the delay) gives identical output. | Integration and Time Synchronization |
| `[ ]` M3.3a | **EKF core, part 1** (D5 C: state x, y, θ, b_g + buoys; prediction from GNSS velocity and gyro rate minus b_g; D14 a: R = process, Q = measurement noise in docs and comments): state, prediction and process noise (`design.md` §6, §6.1). | Jacobians match a numerical derivative; a hand-made straight run predicts the expected pose and covariance growth. | Vehicle Motion Model; EKF-SLAM Formulation; App. A |
| `[?]` M3.3b | **EKF core, part 2** (D6; D15 a, which this brief raises again as Marcell asked): GNSS position update with the antenna lever arm. | Jacobian test; a hand-made scenario converges to the true position. | Observation Models; App. A |
| `[ ]` M3.3c | **EKF core, part 3**: heading update (with D5 C the gyro is an input, so there is no gyro update; the heading update is what makes b_g observable). | Jacobian tests; wrapped innovations near ±π give the right sign. | Observation Models; App. A |
| `[ ]` M3.3d | **EKF core, part 4**: buoy range-bearing update and new-landmark initialisation (with cross-covariances). | Jacobian tests; a hand-made scenario with two buoys gives the expected map. | Observation Models; EKF-SLAM Filter; App. A |
| `[ ]` M3.4 | **Data association and map management** (D13 a): Mahalanobis gate, colour rule (known colour must match; unknown colour uses the gate only), new buoys start as tentative and are promoted after n_conf sightings. | Unit tests with known correspondences. | Data Association and Map Management (design and implementation sections) |
| `[ ]` M3.5 | **slam_replay** (D4 a; ⚠ `<depend>` on rosbag2_cpp): reads a bag, runs corruptor → detector → filter in timestamp order, writes the run folder (`design.md` §9). | Same config and seed twice gives identical results. The report gives the wall time of one replay and of one evaluation of a real bag (bash `time`, median of 3, Release build: `--cmake-args -DCMAKE_BUILD_TYPE=Release`; also the number of CPU cores, `nproc`) and the bag's duration: the number D10 needs. | Software Architecture; Ground Truth and Data Recording |
| `[?]` M3.6 | **Evaluation** (`asv_eval`, grown from M1's `analyze_run.py`; ⚠ new package, Python libraries per M0.3): position and heading error, map error, NEES and NIS (`design.md` §10). | The matched check in `design.md` §10 passes over M runs (D10): the ANEES is inside its 95 % band at about 95 % of the time steps (at least 90 %). | Metrics; Localization and Mapping Accuracy; Filter Consistency |
| `[-]` M3.7 | Left out for now (D16, 2026-10-11). **Resource numbers from slam_replay** (D16, D17; `design.md` §14): slam_replay also writes `resources.json` to the run folder: wall time and CPU time of each stage (corruptor and detector marked sim-only; queue and filter onboard), message counts, peak memory of the replay process (it includes the bag reader and the sim-only stages, and is labelled so), replay speed (bag duration ÷ filter wall time), and the run settings listed in `design.md` §14.4. Not in `metrics.json`, which must stay identical between runs. | Two runs with the same seed still give identical `metrics.json`; `resources.json` exists and its stage times add up to the total within 5 %. | Software Architecture (if D16 a) |
| `[-]` M3.8 | Left out for now (D16). **EKF cost against map size** (D16, D17): a timing program in the ROS-free package (not a `colcon test`: timing tests are flaky). Seeded synthetic maps with 10 to 200 buoys; times prediction, GNSS update, buoy update and new-landmark initialisation; Release build, warm-up, many repetitions, pinned to one core. CSV to `ws/data/bench/`. | CSV and a log-log plot of time against state size, with the slope against the state size (about 2 for the quadratic update the design expects; about 3 means a dense, cubic implementation that should be fixed). | Further Assumptions (A6); EKF-SLAM Filter (if D16 a) |

### M4. Live filter and visualisation

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[ ]` M4.1 | **slam_node** (⚠ `<depend>` on tf2_ros, visualization_msgs): the same core and queue as slam_replay, live. Publishes `/asv/slam/odom` (same format as ground truth), TF map → base_link, buoy markers with uncertainty ellipses, and the path. | On a bag played back in simulation time, its errors match slam_replay on the same bag within a stated percentage. `ros2 node info` lists no clean sensor topic (`/asv/imu/data`, `/asv/gnss/...`, `/asv/lidar/points`) and no `/asv/ground_truth/...` topic among slam_node's subscriptions (D3 b: the clean data sits on the normal names, so this check guards against reading it by accident). Marcell checks the RViz view. | Software Architecture; EKF-SLAM Filter; Integration and Time Synchronization |
| `[-]` M4.2 | Left out for now (D16). **Live resource check** (D16, D17; `design.md` §14): a small Python tool (standard library only) reads `/proc` for an allow-list of onboard processes (slam_node, follower and recorder now, perception nodes after M5; CPU from `schedstat`, memory as PSS, `design.md` §14.3) while a worst-case "board bag" (only what the boat would receive, through Task 2, a full mission long) plays at 1× with Gazebo stopped; in-node timing gives per-callback times. Then the same replay with the onboard nodes in a limited container (the `docker run` command in `design.md` §14.5; a command line, no `compose.yaml` change), stepping the limits down. The brief fixes the budgets (`design.md` §14.6). | Table per process: CPU core-seconds per second of data, per-callback p50/p99/max, PSS; the total cross-checked with `docker stats`. Limit table: the smallest core count and memory with no out-of-memory kill, no measurement later than the queue delay, and the budgets met. | Software Architecture; Discussion and Limitations (if D16 a) |

### M5. Real perception (optional: only if time allows, D8)

Marcell decided (D8 a, 2026-10-11) to build the filter on the fake detector first
and to add real detection only if time is left. So M5 is optional: skipped unless
Marcell asks, and M6 follows M4. Without M5, the fake detector's noise settings
are not calibrated against a real pipeline (`design.md` §5, §13).

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[ ]` M5.1 | (⚠ new package `asv_perception`; PCL if used, probably not installed: Dockerfile change) Lidar: level the points with roll/pitch (eq. 3.3, `design.md` §2), remove the boat's own hits, cluster, cluster centre → range and bearing from base_link (with optional centroid-bias correction). | Unit tests: the levelling test in `design.md` §2; a buoy on the boat's left gives a positive bearing from the upside-down lidar. | LiDAR Processing and Landmark Detection |
| `[ ]` M5.2 | Camera colour classification (HSV thresholds; ⚠ `<depend>` on cv_bridge, OpenCV). | Colour accuracy measured on a perception recording. | Camera-Based Classification |
| `[ ]` M5.3 | Lidar-camera fusion: project clusters into the image to give them a colour. | Fusion accuracy measured. | LiDAR-Camera Fusion |
| `[ ]` M5.4 | Measure the pipeline's error statistics against ground truth; set the fake detector to them; compare filter results with fake and real detections on the same bag. | Both sets of metrics reported side by side, with the agreed tolerance. | Experimental Scenarios; Discussion and Limitations |
| `[-]` M5.5 | Left out for now (D16). **Perception timing** (D16, D17): time per stage (levelling, clustering, colour, fusion) against points per scan and image size; the M4.2 check repeated with the perception nodes. | Table of stage times and the M4.2 tables with perception included. | Discussion and Limitations (if D16 a) |

### M6. Experiments

| Step | What | Done when | Thesis section |
|---|---|---|---|
| `[?]` M6.1 | **Sweep runner** (D10): factor table (`design.md` §11), M runs per setting, one folder per run, one results table for all runs. | One small sweep runs end to end with one command. | Experimental Scenarios |
| `[ ]` M6.2 | Studies (D9 a): GNSS modes and outages; IMU noise levels (also during outages; the brief checks how the CV7 datasheet defines its 1.5 °/h, `design.md` §11); detector noise; a small wave study (2-3 wave heights, one recording each). | Results table exists for every study. | Localization and Mapping Accuracy; Filter Consistency; Sensitivity Analysis; Environmental Conditions |
| `[ ]` M6.3 | Figures and App. B parameter tables generated by scripts from the run results and the YAML files (numbers only; captions and text are Marcell's). | One command regenerates every figure and table. | App. B Parameter Tables; Evaluation |

### M7. Optional

| Step | What | Done when |
|---|---|---|
| `[ ]` M7.1 | robot_localization baseline: `navsat_transform` with a local Cartesian frame (parameter `use_local_cartesian: true`; check the name in the Jazzy version), datum = the world's reference point, heading from `/asv/sensors/gnss/heading` turned into an IMU orientation message (not the IMU's own yaw). | Its errors are reported in the same results table as the EKF-SLAM runs, on the same bags. |
| `[?]` M7.2 | Coloured-noise filter variant (D6 b): only if D6 = a; with D6 b (recommended) it is part of M3.3b and this step goes away. | ANEES with the full corruptor profile compared with D6 a. |
| `[ ]` M7.3 | Steering with the filter's estimate (closed loop). With D5 C the pose comes from the filter and the twist from its inputs: the GNSS velocity rotated into the body frame with the estimated heading, and the yaw rate ω_m − b_g (no velocity state needed). | Task 1 completed on `/asv/slam/odom`; path difference to the ground-truth run reported. |
| `[ ]` M7.4 | Thrust-driven prediction with deliberately wrong drag numbers. | Errors compared with the main motion model. |
| `[ ]` M7.5 | Slowly moving buoy offsets in the fake detector, as a stand-in for mooring motion (`design.md` §5). | Shows whether the measurement noise absorbs the motion, as Static Landmarks claims. |
| `[ ]` M7.6 | Not planned: D18 a, live data is shown in RViz. Foxglove dashboard or a shore link (named in the old `milestone1_plan.md`, not needed for the thesis): view the route, status, map and sensor topics from a browser or a second computer. Foxglove connects through `foxglove_bridge` (a websocket server; a new dependency, ask first). ROS tools such as RViz on a second computer would also need ROS discovery opened beyond localhost (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `compose.yaml`; ask first). | Marcell sees the autonomy and SLAM topics live in the dashboard. |
| `[-]` M7.7 | Left out for now (D16; the boat computer choice is parked). **On-board replay** on a candidate computer (bought or borrowed): same commit, bag and seed as on the PC; build natively (ROS 2 Jazzy needs Ubuntu 24.04) or in the arm64 `ros:jazzy-ros-base` image. On a Jetson: fixed power mode (`nvpmodel`, the one the boat will use), at least 30 minutes, `tegrastats` logged (RAM, per-core CPU, GPU load, power, temperatures), once with GPU load (ZED depth, detector) (`design.md` §14.7). This is the only conclusive test for a board. | The M3.7, M3.8 and M4.2 tables from the board; filter metrics within a stated tolerance of the PC's. |

---

## 5. Thesis map

Where each part of the work is described. Section titles are those in the
current draft body.

| Thesis section | Steps | What the code provides |
|---|---|---|
| Planar Motion (eq. 3.3), Static Landmarks, Further Assumptions | see the table below | how each assumption is met or deliberately broken |
| Sensor Configuration | M2.6, D12 | sensor specs and mounts from `asv.yaml`, heading accuracy |
| Coordinate Frames and Transformations | M2.1, M2.8 | geodesy helper, frame tree, lever arms |
| Vehicle Motion Model, EKF-SLAM Formulation | M3.3a | D5 choice and its equations |
| Observation Models | M3.3b-d, M2.6 | GNSS position with lever arm (D15), heading α = π/2 − ψ, gyro, range-bearing |
| Noise Models | M2.5, M2.6, D6, D12 | noise equations, values with sources, Allan check |
| Data Association and Map Management (design and implementation) | M3.4 | gate, colour rule, tentative list |
| Toolchain | M0.3, M0.5 | versions, container |
| World and Course Elements | M2.7 | course, static buoys (D7) |
| Vehicle and Sensor Models | M2.2, M2.6 | sensor models and what Gazebo does not simulate |
| Ground Truth and Data Recording | M2.7, M2.8, M2.9, M3.5 | ground truth sources, recording profiles |
| Environmental Conditions | M6.2 | wave study (D9) |
| Software Architecture, App. C | M2.4, M2.6, M3.5, M4.1, M1.2-M1.6; topic names per D3 b | packages, nodes, topics, guidance code |
| (left out for now, D16) | M3.7, M4.2, M5.5 | CPU time and memory of the onboard code |
| LiDAR Processing, Camera-Based Classification, LiDAR-Camera Fusion | M5.1-M5.3 (only if time, D8) | perception pipeline; without M5 these sections have no implementation behind them |
| EKF-SLAM Filter | M3.3d, M4.1 | implementation |
| Integration and Time Synchronization | M0.2, M3.2 | time rules, measurement queue |
| Metrics, Experimental Scenarios | M3.6, M3.1, M1.2-M1.5, M1.7, M6.1 | metric definitions, how the trajectories are driven, factor table |
| Localization and Mapping Accuracy, Filter Consistency, Sensitivity Analysis | M6.2 | results |
| Discussion and Limitations | all | limitations list in `design.md` §13 |
| App. A Jacobian Derivations | M3.3a-d | Jacobians and their tests |
| App. B Parameter Tables | M6.3 | generated tables (numbers only) |

**Assumptions in the draft (Chapter 3.1) and what the simulation does with them**

| Assumption | In the simulation | Steps, decisions | Shown by |
|---|---|---|---|
| Planar motion, levelling (eq. 3.3) | roll and pitch, only if something needs them (M5 levelling, D5 a, D15 b), come with noise from a separate corruptor message (D11, open); the core with D5 C and D15 a does not need them; GNSS lever arm planar (D15 a) | D11, D15, M5.1 | levelling unit test; wave study |
| Static landmarks (eq. 3.5) | buoys static (D7 a); mooring motion not simulated; centroid bias optional | D7, M2.7, M3.1 | drift run; optional M7.5 |
| State (eq. 3.6) | x, y, θ plus the gyro bias b_g (D5 C), plus 2 GNSS slow-error states if D6 b | D5, D6, M3.3a | |
| A1 rigid mounting, known extrinsics | met exactly (extrinsics from `asv_frames.yaml`) | M2.8 | |
| A2 common time base | met: everything is stamped in simulation time; the queue orders by stamp | M0.2, M3.2 | queue unit test |
| A3 Gaussian white noise | deliberately broken: GNSS and IMU errors have slow Gauss-Markov parts. The gyro bias is in the state (D5 C); the GNSS slow error is treated as white with D6 a, or modelled in the state with D6 b (the white treatment then stays as a switch for this test) | D5, D6, M2.6 | ANEES with and without those parts |
| A4 GNSS availability | met in nominal runs; deliberately broken in outage runs | D9, M6.2 | outage study |
| A5 small heading uncertainty | met while the dual-antenna heading is available; broken during sky-view outages | D9, M6.2 | heading error and NEES during outages |
| A6 moderate map size | met (tens of buoys) | M2.7 | count in `buoys_gt.yaml` (the cost measurement M3.8 is left out for now, D16) |
| A7 unknown correspondences | met, and extended: an UNKNOWN colour uses the gate only; only coloured detections start landmarks | M3.4 | association unit tests |

---

## 6. Open checks on Marcell's machine

| Check | Blocks | Command or method |
|---|---|---|
| Installed packages (OpenCV, cv_bridge, rosbag2_cpp, PCL, yaml-cpp; numpy, scipy, matplotlib) | M3.5, M3.6, M5 | M0.3 |
| Gazebo's real-time factor when headless | M0.2 check; D10 (how long each recording takes) | read it from the Gazebo log or `gz stats` |
| Lidar sees no water (the generator hides it: `visibility_mask 7`) | M5.1 (only if time) | one scan over open water has no points near the water surface |
| How far floating buoys drift during a run | M2.7 | one 5-minute floating run |

---

## 7. Change log

**v1.4 (2026-10-14).** D5 decided C: state x, y, ψ, b_g plus buoys; GNSS
velocity and gyro are prediction inputs. M3.3a, M3.3c, M3.3d unblocked; the
corruptor also publishes `/asv/sensors/gnss/velocity`; M7.3 kept, with the odom
twist taken from the GNSS velocity rotated into the body frame. SWOT comparison
of the three candidates in `decision_notes.md`.

**v1.3 (2026-10-11).** Marcell's decisions:

- Decided: D1 a, D2 a, D3 b, D4 a, D7 a, D8 a, D9 a, D12 a, D13 a, D14 a,
  D15 a (raised again in the M3.3b brief), D16 (benchmarking left out for now),
  D17 a, D18 a (RViz). Open: D5, D6, D10 (after the speeds are measured) and
  the IMU message content (D11); background in the new `decision_notes.md`.
- D3 b (not the recommended option): the bridge keeps its names; realistic data
  goes to `/asv/sensors/...`. M2.3 is skipped; the topic rule and table
  (`design.md` §1, CLAUDE.md) and M7.1 changed; M4.1 checks slam_node's
  subscriptions.
- D8 a: the fake detector gives the true colour; M5 is optional (only if time).
- D16: M3.7, M3.8, M4.2, M5.5, M7.7 are `[-]`; the benchmark tool checks left
  M0.3; `design.md` §14 is kept for later. The boat-computer questions are parked.
- Steps whose decisions are all made changed from `[?]` to `[ ]`. M3.5 and M0.2
  now report the replay time and the headless real-time factor that D10 needs.
- From the research for the open decisions: the §6.1 motion-model table
  corrected (accelerometer biases required in a, tilt leak, b blind to speed
  changes, c's real weakness is the missing gyro-bias state); the real ArduPilot
  IMU topic described (`design.md` §1, §4.1, §14.8); the A3 row no longer
  assumes D6 a.

**v1.2 (2026-10-10).** After Marcell's push (commit 6b8be2b) and his question
about benchmarking the onboard code:

- The old docs said `mix()`, a teleop and 8 C++ tests exist; no C++ code was
  ever written. Fix-first item 2, M0.1, M1.2 (now writes `mix()` and sets up
  gtest), CLAUDE.md "Known state" and the layout are corrected. M1.0 and M1.1
  are marked done; M0.1 now also decides what happens to `CONTEXT.md`.
- `milestone1_plan.md` corrected (stale facts, simulation time, buoy radius in
  the gate check, symbols from the glossary) with a new section 10: open points
  for the M1.2 brief (`mix()` specification, reverse-thrust steering, on/off
  spin assist, the teardrop needs spin, calibrated test model, lookahead,
  d_end, parameter sources).
- Benchmarking the onboard code: decisions D16-D17 (and D18, Foxglove or a
  shore link, moved from the old M1 plan), steps M3.7, M3.8, M4.2,
  M5.5, M7.7, tool checks in M0.3, `design.md` §14, and a time-rule exception
  for benchmark clocks (`design.md` §3 rule 9).
- New: autonomy topics and thruster limits in `design.md` §1; guidance symbols
  in the glossary; optional M7.6 (Foxglove); `actuator_check.py` clock in M0.2;
  README in M0.5; the test baseline in CLAUDE.md.

**v1.1 (2026-10-10).** After a second review of v1:

- Milestone 1 Steps 2-6 now have rows (M1.2-M1.6), so the session start finds
  them; new status mark `[-]` for skipped steps.
- New decisions D14 (noise covariance letters) and D15 (GNSS lever arm with or
  without roll and pitch). D5 is explained in `design.md` §6.1; D5, D10 and D11
  are due earlier, because earlier steps depend on them.
- M2.1 and M2.2 swapped (geodesy helper first), so every "done when" can be
  checked when its step ends. M3.3 split into four parts; gyro update and
  new-landmark initialisation added.
- The Allan test rewritten (the old version could not pass reliably); the IMU
  sample timing corrected (8, 4, 4, 4 ms); the rule on who may read ground truth
  fixed (the follower and the replay's input stage may).
- `ws/data/` is git-ignored in M0.1; untracking the build output is a normal
  step (M0.4); M7 items have their own "done when".

**v1 (2026-10-09).** Replaced draft v0 after a review from four sides
(repository, ROS/Gazebo, thesis method, clarity): fix-first list, decision
table, small steps with checkable "done when", simulation time, recording
profiles, IMU orientation handling, evaluation details.
