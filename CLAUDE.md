# CLAUDE.md — RoboBoat ASV simulation

Context and rules for Claude Code. Read this first in every session.

## What this project is
Simulation environment for the BME Solar Boat Team's RoboBoat ASV (autonomous
surface vehicle): a catamaran with two steerable (azimuth) thrusters, Livox
Mid-360 lidar (upside down), ZED 2i stereo camera, Arducam AR0234 wide camera,
Pixhawk 6X IMU, Unicore UM982 dual-antenna RTK GNSS. The owner, Marcell, uses it
for his bachelor's thesis: GNSS-aided EKF-SLAM (map the buoys by colour, localise
the boat, and measure the accuracy and consistency of both against ground truth
under different sensor noise levels). The thesis experiments are simulation only.
The code that would run on the real boat (filter, perception, guidance) should
also build and run on the boat's onboard computer, which is not bought yet.
Benchmarking it is left out for now (decision D16).

Stack: Gazebo Harmonic + VRX (water, waves, buoyancy) + ROS 2 Jazzy, all inside
a Docker container. Host OS is NixOS.

## The main rule: Marcell understands every step before it happens
Marcell has to describe this work in his thesis, in his own words. Your job is to
build it **with** him so that he can explain every part. Nothing may be a black
box.

- **Explain first, then wait, then build**, for every step (the step protocol
  below).
- Briefs, reports and docs are short bullet notes and equations, not finished
  paragraphs.
- Don't give model answers to the "check your understanding" questions unless he
  asks after trying.

## Where the plans are
| File | What it is |
|---|---|
| `docs/master_plan.md` | What gets built, in which order; the decisions (section 3); thesis map |
| `docs/design.md` | How each part works (topics, frames, time, noise models, filter, evaluation) |
| `docs/glossary.md` | Symbols table and plain-language terms; add new ones here |
| `docs/decision_notes.md` | Background and options for the decisions that are still open |
| `docs/plans/TEMPLATE.md` | The layout every milestone plan follows |
| `docs/plans/mN_plan.md` | The steps of milestone N, with the brief and report of each step |
| `ws/src/asv_sim/docs/milestone1_plan.md` | Milestone 1 (its Step N = M1.N); also holds the M1 briefs and reports |
| `ws/src/asv_sim/docs/HOW_IT_WORKS.md` | The boat model generator; keep it accurate when changing `tools/asvgen/` |

The old chat summary `CONTEXT.md` (2026-10-07) was removed in M0.1; several of
its statements were wrong. The files in the table above are the source of truth.

**The thesis draft and Marcell's literature notes are NOT in this repository.**
Before writing an equation that belongs to the thesis, use the form in
`docs/design.md` and the symbols in the glossary's Symbols table. A new symbol is
marked "proposed" until Marcell accepts it. Cite thesis sections by title (and a
LaTeX label if Marcell gives one), never by a guessed equation number.

**At the start of a session:**
1. Run `git status` and `git log --oneline -3`.
2. Find the current step in `docs/master_plan.md`: a step marked `[~]` (resume it;
   its agreed brief is saved in the milestone plan), otherwise the first step, in
   table order, that is not marked `[x]` (done) or `[-]` (skipped) and is not
   optional (M0.5, M5, M7; Marcell may ask for those). If that step
   is `[?]`, ask Marcell for its decisions first.
3. Tell Marcell in two lines where things stand and what the next step is.

**When Marcell says `next`:** do the three checks above and post the brief of the
next step right away (in plan mode if it is active). Don't ask first.

## The step protocol
Work in the small steps of the master plan (M2.4, M3.1, ...). One step = one
commit. Steps and rows marked "Optional" (M0.5, M5 "only if time", section M7) are
skipped unless Marcell asks for them.

**Before a milestone.** Copy `docs/plans/TEMPLATE.md` to `docs/plans/mN_plan.md`:
one section per step from the master plan, each with its goal, its "done when"
and the decisions it needs. Marcell approves the milestone plan before the first
step starts. Exceptions: M0 needs no approval, its briefs and reports are just
logged in `docs/plans/m0_plan.md`; Milestone 1 keeps
`ws/src/asv_sim/docs/milestone1_plan.md` (it already has Steps 7 and 8 = M1.7,
M1.8).

**1. Brief (before touching any file).** Post it in chat, in plain language. Leave
out items that would only say "none". It should fit on one screen.

0. **In plain words**: one paragraph (4–8 sentences, everyday words, one
   concrete example): what will happen in this step, why, and what does not
   change. This is the one exception to "bullet notes, not paragraphs".
1. **Goal**: one sentence.
2. **Decisions needed**: open decisions (D-numbers) this step depends on, with the
   options and your recommendation. Never decide them yourself.
3. **Why**: what it is for, and which thesis section it feeds (by title).
4. **New ideas**: at most three concepts this step needs, each explained in one or
   two plain sentences (link to `docs/glossary.md`).
5. **What will change**: every file to be created or edited. Mark with ⚠ anything
   from the "Ask first" list below.
6. **How it works**: first in words, then the equations; a small diagram if it
   helps.
7. **How we will check it**: the tests and commands, and the numbers we expect.
8. **What I can't check**: anything only Marcell can see or do (the Gazebo window,
   RViz, keyboard input).

**Short form** for steps with no new idea and no equation (M0.1, M0.3, M0.4,
M0.5, M2.9, small fixes Marcell asks for): the plain-words paragraph, then goal,
open decisions, files (with ⚠ marks), how we check it, in about five lines. Still wait for his go.

**Progress bar** at the top of every brief (full and short form), built from
the marks in `docs/master_plan.md`: one square per step, grouped by milestone in
table order; 🟩 done `[x]`, 🟨 the step this brief is for, ⬜ not done yet.
Count only steps that will be done: leave out `[-]` steps and optional ones
(M0.5, M5, M7) unless Marcell asked for them; each sub-step (M3.3a, ...) is
one square. Below it: `<done> of <total> steps done: <xx>% completed!` (done ÷
total, rounded to a whole percent; the current step is not yet counted as done).
Example:
```
M0 🟩🟩🟨⬜ │ M1 🟩🟩⬜⬜⬜⬜⬜⬜⬜ │ M2 ⬜⬜⬜⬜⬜⬜⬜⬜ │ M3 ⬜⬜⬜⬜⬜⬜⬜⬜⬜ │ M4 ⬜ │ M6 ⬜⬜⬜
4 of 34 steps done: 12% completed!
```

If a step is too big to explain on one screen, propose splitting it into
sub-steps (M3.3a, M3.3b, ...).

Then **stop and wait** for Marcell to say go (or to change the brief). Do not
start building in the same turn. If Marcell asks a question, answer it before
anything else.

- If the brief has an open decision, "go" is not enough: ask Marcell to name his
  choice. Then record it (see "Decisions" below).
- When Marcell says go: save the agreed brief under the step in the milestone
  plan and set the step's mark in `docs/master_plan.md` to `[~]`.

**2. Build.** Do what the brief says, nothing more. Write the tests early and run
the relevant tests after every change, not only at the end. Small fixes needed to
reach the agreed goal (a compile error, a missing include) need no new brief.
Anything outside the brief, a surprise, a failing assumption, or a change to a
number that will appear in the thesis: stop, explain what you found and what you
propose, and wait again.

**3. Report.** Post in chat and save under the step in the milestone plan:

1. **What changed**: each file with one line on what it does now.
2. **How it was checked**: the commands and the relevant lines of their actual
   output (numbers, test counts). Never "should work".
3. **What could not be checked**, and how Marcell can check it.
4. **How to read the code**: where to start, and which function implements which
   equation.
5. **Numbers for the thesis**: parameter values and results, each with its source
   tag.
6. **Check your understanding**: two or three short questions Marcell can answer
   to see whether the step is clear (he may skip them).

Set the step's mark in `docs/master_plan.md` to `[x]`. If the step changed
anything listed under "Known state" below, update that section too.

**4. Commit** everything from the step in one commit: the code, the tests, the
milestone plan with brief and report, the master plan mark. Stage files by name;
never `git add -A` or `git add .`. Message: `M2.5: noise library` (step ID, colon,
short summary). A step's commit is found with `git log --grep "^M2.5:"`. Then
stop and summarise in two lines.

## Decisions
- The decisions are in `docs/master_plan.md` section 3. Marcell decided most
  of them on 2026-10-11 (D5 on 2026-10-14, D11 on 2026-10-21); the open ones
  (D6, D10) are explained in `docs/decision_notes.md`. Marcell decides; you explain
  the options and recommend one.
- Record his answer in the Status column: `decided: <option>, <date>: <his reason,
  in his words>`. When all decisions of a `[?]` step are made, change it to `[ ]`.
- If he picks an option that is not the recommended one, list the lines in the
  docs that change because of it, and update them (short-form step).
- A new open question becomes the next D-number in the table.

## How to explain
- Plain English, short sentences. Explain each term the first time, or link it to
  the glossary; add missing terms to the glossary.
- Say **why**, not only what. Name the alternatives you considered, one line each.
- Use the symbols in the glossary's Symbols table. Noise letters (D14 a):
  R = process noise covariance, Q = measurement noise covariance (Thrun), in
  docs, equations and comments. In code keep names without letters
  (`process_noise_cov`, `meas_noise_cov`).
- Every number has a unit and a source tag: [DS] datasheet, [EST] estimate (say
  why), [REPO] this repository, [SRC] Gazebo/ROS source code, [LIT] a paper (bib
  key), [MEASURED] measured in a run here.
- Separate what you checked from what you assume ("I ran X and got Y" vs. "I
  expect Y because ...").

## Conventions
Rules marked with a decision follow Marcell's choice as recorded in
`docs/master_plan.md` section 3. A rule tied to an open decision (D6, D10) is
raised in the brief.

- **C++** nodes: C++17, rclcpp. Nodes only do I/O; the logic (maths, models,
  filter) is ROS-free so it can be unit tested. It lives in the new `asv_core`
  package (D13 a), `include/asv_core/*.hpp`. The Milestone 1 guidance
  code stays in `include/asv_sim/`.
- **Comments**: explain non-obvious code (why, not only what); keep functions
  small. A function that implements a thesis equation says so (section title, and
  the LaTeX label if known) and maps code names to the symbols.
- **Tests**: gtest for C++, `unittest` for Python. Every Jacobian is checked
  against a numerical derivative. Never weaken or skip a test to make it pass.
  Current baseline (M2.9): 137 Python tests, run with `unittest`; `colcon test`
  for `asv_core` runs 57 gtests (`test_angles` 6, `test_geodesy` 7,
  `test_noise` 19, about 20 s, `test_imu_corruptor` 8,
  `test_gnss_corruptor` 17) and for
  `asv_sim` 133 (`test_mix` 9, `test_guidance` 15, `test_control` 27,
  `test_gate_monitor` 15, `test_follower` 11, `test_closed_loop` 7,
  `test_maneuver` 8, `test_maneuver_files` 3, `test_teleop` 9,
  `test_imu_corruption` 7, `test_gnss_corruption` 12, `test_buoy_map` 6,
  `test_ground_truth_tf` 4);
  `colcon test-result` counts 62 + 146 = 208, the extra eighteen are CTest's
  wrappers, one per gtest binary.
  Report both numbers after every step.
- **Python 3** for tools, launch files and offline analysis.
- **Frames**: world = ENU (x east, y north, z up), boat = base_link (x forward,
  y left, z up). map = world (same origin). Angles in radians, SI units. Every
  angle difference goes through one wrapping helper. Details: `docs/design.md` §2.
- **Extrinsics** (sensor positions) come from `config/asv_frames.yaml` or TF;
  never type them into code.
- **Topics**: everything under `/asv/...`; the table is in `docs/design.md` §1.
  D3 b: the bridge keeps the normal names, so the clean (perfect) sensor data is
  on `/asv/imu/data`, `/asv/gnss/...`, `/asv/lidar/points`; realistic data from
  the corruptor is under `/asv/sensors/...`; truth is under
  `/asv/ground_truth/...`. The filter (EKF core, slam_node, the filter stage of
  slam_replay) and any baseline read only `/asv/sensors/...` and
  `/asv/perception/...`, and those are their default topics. A node left on a
  normal name silently gets perfect data, so check every estimator's topics. The
  corruptor, the fake detector, the evaluation, the ground-truth TF node, the
  waypoint follower (it drives on ground truth), and slam_replay's input and
  logging stages read the clean and ground-truth topics on purpose.
- **Thrusters**: `/asv/thrusters/{left,right}/thrust` (std_msgs/Float64, N) and
  `/asv/thrusters/{left,right}/pos` (rad).
- **Steering sign**: a POSITIVE pod angle turns the boat RIGHT (pods are behind
  the centre of gravity). **Confirmed in Gazebo** (Step 1): pods at −0.4 rad →
  positive yaw rate (+0.15 rad/s) → boat turns LEFT. Rule for `asv_sim::mix()`
  (written in M1.2): it takes steer > 0 = turn left and outputs
  pod_angle = −steer. This holds while thrust is forward; with reverse thrust
  the pods turn the boat the other way (`milestone1_plan.md` section 10).
- **Ground truth**: `/asv/ground_truth/odom` (nav_msgs/Odometry, 50 Hz; pose in
  world, twist in the BODY frame: twist.linear.x = forward speed, twist.angular.z
  = yaw rate). The twist is smoothed by Gazebo: fine as controller feedback, not
  as true velocity in the evaluation.
- **Time** (`docs/design.md` §3): every launch file sets `use_sim_time` for all
  its nodes; `ros2 run` nodes get `--ros-args -p use_sim_time:=true`. Timers use
  the ROS clock (`rclcpp::create_timer(this, get_clock(), ...)`), never
  `create_wall_timer`. Measurement times come from `header.stamp`, never from
  `now()`; time steps are computed from stamps, never from the nominal rate; bags
  are recorded with `--use-sim-time`. One exception, only if benchmarking comes
  back (D16): benchmark timing (CPU and processing time) uses
  `std::chrono::steady_clock` and CPU-time clocks, only for logging
  (`docs/design.md` §3 rule 9, §14).
- **Randomness**: always seeded, one random stream per noise source.
- **Parameters** live in YAML files, not in code, each with a source tag.
- **Data** (bags, run results, figures) goes to `ws/data/` and is never
  committed. Never commit build output (`ws/build`, `ws/install`, `ws/log`).
- After changing `config/asv.yaml`: `python3 tools/make_asv.py` (writes the
  model, bridge and frames configs); no rebuild needed for those.

## Ask first
Marcell must say yes before you:
- change the generator (`tools/asvgen/`), the course generator, `docker/`,
  `compose.yaml` or `.gitignore`;
- add a dependency (apt package, pip package, new `<depend>` in `package.xml`);
- change a value in `config/asv.yaml` or the world file, or any number that will
  appear in the thesis;
- decide any open decision (`docs/master_plan.md` section 3);
- delete files (also `git rm --cached`), rewrite git history, or push.

## Honesty rules
- Never claim something works without running it. Show the command and output.
- Say clearly what could not be verified (e.g. the Gazebo window or RViz, which
  you can't see; keyboard input, which you can't type).
- If a test fails, show the failure and explain it.
- If you are not sure how Gazebo or ROS behaves, check the source or the docs, or
  say that it is unverified. Facts already checked are in `docs/design.md` §13.

## Running commands (from the host, in ~/thesis/docker-sym)
The container must be up: `docker compose up -d`. Run commands inside it with
`-T` (no TTY, required when not in an interactive terminal) and `bash -ic`
(interactive shell, so ~/.bashrc sources ROS and the workspace):
```
docker compose exec -T sim bash -ic "cd /ws && colcon build --symlink-install --packages-up-to <pkg>"
docker compose exec -T sim bash -ic "cd /ws && colcon test --packages-select <pkgs changed by the step> && colcon test-result --verbose"
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 -m unittest discover -s tests"
```
`--packages-up-to` builds the package and what it depends on, not all of VRX.
Anything that is timed (benchmarks, resource numbers) is built with
`--cmake-args -DCMAKE_BUILD_TYPE=Release` (`docs/design.md` §14.4).
Test every package the step changed (e.g. `asv_core asv_sim`), not only
`asv_sim`.

Simulation without a window (you cannot see the GUI), in the background:
```
docker compose exec -d sim bash -ic "ros2 launch asv_sim sim.launch.py gui:=false > /tmp/sim.log 2>&1"
docker compose exec -T sim bash -ic "timeout 10 ros2 topic hz /asv/ground_truth/odom"
docker compose exec -T sim bash -ic "pkill -f 'ros2 launch'; pkill -f 'gz sim'; pkill -f parameter_bridge; pkill -f static_transform_publisher; pkill -f 'lib/asv_sim/'"   # stop
```
Use `timeout` for anything that waits for messages, so commands never hang.

Recording with a profile (`nav`, `nav_lidar`, `perception`; `docs/design.md` §8);
the script stops the recorder with SIGINT so the bag is readable; then check it:
```
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/record.py --profile nav --name <name> --duration 60"
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/check_bag_rates.py /ws/data/bags/<name> --profile nav"
```
Other topic lists: `timeout -s INT 60 ros2 bag record --use-sim-time -o
/ws/data/bags/<name> --topics <topics>`.
Keyboard teleop (from M1.8 on) needs Marcell's own interactive terminal
(`docker compose exec sim bash`, then `ros2 run asv_sim teleop --ros-args -p
use_sim_time:=true`); you can't run or test the key input.

## Known state / caveats
Update this section in the same commit as the step that changes it.

- **`asv_core` (M2.1):** the ROS-free package (D13 a), library
  `asv_core::asv_core` (Eigen). `angles.hpp`: `wrap_angle()` and
  `yaw_from_quaternion()` (moved from `asv_sim`). `geodesy.hpp`: WGS84
  lat/lon/h ↔ ENU around the world's reference point (`LocalTangentPlane`),
  a copy of Gazebo's own conversion (gz-math 7.7.0 LOCAL2 ↔ SPHERICAL, Bowring
  one-pass inverse). Python twin `asv_sim/tools/geodesy.py` (also
  `read_geo_reference()` from the world SDF; heading must be 0). C++ and Python
  agree bit for bit on the shared points `asv_core/test/data/geodesy_points.csv`
  (regenerate with `python3 tools/geodesy.py --write-test-points ...`);
  round trip ≤ 3.3 nm.
- **C++ so far (M1.8):** the library `asv_sim_guidance` (links `asv_core`)
  holds `mix()` (`mix.hpp`), `Route` and `LosGuidance` (`guidance.hpp`),
  `HeadingControl` and `SpeedControl` and `course_angle()` (`control.hpp`), and `GateMonitor`
  (`gate_monitor.hpp`), and `Follower` (`follower.hpp`: the §5.7 state
  machine plus one step of the whole chain), all in `include/asv_sim/` with
  sources in `src/`; gtests `test_mix`, `test_guidance`,
  `test_control`, `test_gate_monitor`, `test_follower`, `test_closed_loop`
  (Task 1 through `Follower` on a test-only 3-DOF model,
  `test/boat_model.hpp`, which reads `asv.yaml` and `asv_report.json` with
  yaml-cpp).
  First node (M1.3): `waypoint_follower` (`src/waypoint_follower.cpp`, I/O
  only), parameters in `config/task1_route.yaml`, all required (no defaults
  in code); the boat values (`drag_linear`, `drag_quadratic`, `max_thrust`,
  `min_thrust`, `steer_limit`, `boat_length`, `boat_half_width`) are read
  from `asv.yaml` by `launch/task1.launch.py` (M1.4: `boat_params()`; one
  command `ros2 launch asv_sim task1.launch.py [gui:=false]
  [params_file:=...]`; stays up after FINISHED, stop with Ctrl-C). FINISHED and
  STOPPED are latched. First Gazebo run (starting gains): Task 1 done, 4/4
  PASS, 64.5 s, one thruster in reverse in 49.8 % of commands. M1.5: kept
  the starting gains (E2 a); `tools/analyze_run.py <bag>` recomputes gates,
  clearance, cross-track and thrust metrics from a bag (writes `metrics.json`,
  `trajectory.png`); 3 runs `ws/data/bags/task1_{1,2,3}` all pass criteria
  2-4 (64.50 s, closest buoy 0.671 m, reverse thrust about 50 % of RUNNING
  commands). M1.9: on a waypoint switch `Route::update()` measures s and e
  again on the new segment (before, LOS and d_end used the old segment's
  values for one tick); 3 new runs `ws/data/bags/task1_m19_{1,2,3}`: 64.60 s,
  closest buoy 0.674–0.675 m, all PASS (`AUTONOMY.md` §7).
  `mix()` takes throttle and spin as fractions of `max_thrust` and keeps the
  left/right difference on saturation (decision 10.1, 2026-10-18). The LOS
  lookahead is carried along the route (10.6 a); d_end is measured along the
  route and FINISHED uses the switch rule on the last segment (10.7 a).
  Heading control steers the course χ = ψ + atan2(v, max(u, 0.3)), not ψ
  (10.13 a); spin = K_s ψ̃ plus a ramp from ψ_spin = 0.3 rad (10.3 b,
  10.14 a, 10.16 b, K_s = 2.0); W0 is the fixed spawn (10.15 a). With this
  spin one thruster is in reverse much of the time (model: 63 % of steps);
  check reverse thrust in Gazebo (M1.3/M1.5). The throttle is ≥ 0 while
  |ψ̃| > ψ_turn = 0.3 rad (10.2 c); the speed PI has conditional-integration
  anti-windup. `GateMonitor` judges gate passes with the offset band of
  §5.6 (±0.3554 m for Task 1) and the clearance with the boat as a
  1.60 × 0.88 m rectangle (10.9 a); crossings up to 2 m beyond a buoy are
  FAIL_OUTSIDE (10.10 a); every buoy has its own radius (10.11 b).
  `wrap_angle()` is the one wrapping helper; since M2.1 it is in `asv_core`.
  M1.6: the as-built description (final equations, parameters, results,
  limitations) is `ws/src/asv_sim/docs/AUTONOMY.md`; `milestone1_plan.md`
  §2–6 are the history. Decision 10.16 is b (proportional spin).
  M1.7a: open-loop `maneuver_player` (`Maneuver`, `ManeuverPlayer` in
  `maneuver.hpp`; zero-order hold; start = first nonzero clock + 3 s; zero
  thrust after the end; status `/asv/maneuver/status`); run with
  `ros2 launch asv_sim maneuver.launch.py maneuver:={straight,zigzag,circle}
  [gui:=false]`; never together with the follower. Tables in
  `config/maneuvers/`, checked on the 3-DOF model by `test_maneuver_files`
  (≥ 0.5 m from the Task 1 buoys and from keep-out zones around Tasks 2, 3,
  5; the world has no shoreline). M1.7b: `tools/compare_runs.py` compares
  recorded runs on τ; open-loop Gazebo runs repeat to within one physics step
  (largest difference 4.18 mm at 1 m/s; circle ≤ 0.0104 mm), bags
  `ws/data/bags/maneuver_<name>_{1,2,3}`.
  M1.8: keyboard teleop `teleop` (`src/teleop_node.cpp`, I/O only; key
  logic `apply_key()` in `teleop.hpp`); reads the thruster limits from
  `asv.yaml` and its steps from `config/teleop.yaml` itself (T1 a, so
  `ros2 run` needs no parameter file); publishes zero on X, Ctrl-C, SIGTERM,
  SIGHUP. Gazebo keeps the last thruster command if a driver dies hard.
- **Noise library (M2.5):** `asv_core/noise.hpp`: `WhiteNoise` (σ = η/√dt,
  so ADEV = η/√τ; decided 2026-10-29), `TurnOnBias` (one draw per run),
  `GaussMarkov` (exact step for any dt, start drawn from N(0, σ_ss²)). Each
  noise source owns its own `RandomStream`, seeded with
  SplitMix64(global seed XOR FNV-1a(source name)); own polar normal draw,
  pinned values in `test_noise`. dt ≤ 0 throws. The §4.5 tests use 10 h
  (test 1) and 40 h (tests 2, 3) records, tolerance 5 % (worst over 10 seeds
  2.08 %). First user: the IMU corruptor (M2.6a).
- **Corruptor, IMU (M2.6a):** node `corruptor` (`src/corruptor_node.cpp`, I/O
  only; started by `sim.launch.py` since M2.6d, see below; alone
  `ros2 run asv_sim corruptor --ros-args -p use_sim_time:=true
  [-p noise_profile:=nominal]`). Reads
  `/asv/imu/data`, publishes `/asv/sensors/imu/data` (gyro + accel with white,
  turn-on and Gauss-Markov errors, same header, quaternion all zero,
  `orientation_covariance[0] = −1`, other covariances 0 = unknown) and the true
  bias b0 + b(k) on `/asv/ground_truth/imu_bias` (sensor_msgs/Imu). The first
  message and stamps that are not later than the last are dropped. Maths
  `asv_core/imu_corruptor.hpp`; glue `asv_sim/imu_corruption.hpp` (library
  `asv_sim_corruptor`). Values = `asv.yaml` × multipliers of
  `config/noise_profiles/<name>.yaml` (`nominal.yaml`: all 1, seed 1).
  Accelerometer density 1.079e-3 m/s²/√Hz (110 µg/√Hz, ±32 g; D11 q9 a; M2.5's
  `test_noise` still uses 70 µg/√Hz as a test value). No scale-factor or
  misalignment error (D11 q8 a). Live check (bag `ws/data/bags/m26a_check`):
  every clean stamp after the first corrupted, 200.02 Hz in sim time.
- **Corruptor, GNSS position (M2.6b):** the same node also reads
  `/asv/gnss/fix` and publishes `/asv/sensors/gnss/fix` (antenna fix + error
  ε_G = β_G + white per ENU axis, covariance diag of the total σ_w² + σ_ss²,
  DIAGONAL_KNOWN; STATUS_FIX standalone, GBAS_FIX with RTK; nothing in an
  outage), `/asv/ground_truth/gnss_status` (asv_msgs/GnssStatus, every epoch;
  `heading_available` set since M2.6c) and `/asv/ground_truth/gnss_error`
  (true slow error β_G, Vector3Stamped, frame world). Per-mode sizes in
  `asv.yaml` `unicore_um982_rtk.position_modes` (decided 2026-11-02: fixed
  5 + 6 mm, T_c 60 s; float 0.05 + 0.19 m, 60 s; standalone 0.3 + 1.47 m,
  300 s; per axis; vertical in the same ratios); the generator accepts the
  key and ignores it. Mode schedule = profile `gnss.schedule` (s after the
  first clean fix; nominal: rtk_fixed all run). Mode switch redraws the slow
  error; same mode after an outage steps it over the gap. Maths
  `asv_core/gnss_corruptor.hpp` (`GaussMarkov::redraw()` added to
  `noise.hpp`); glue `asv_sim/gnss_corruption.hpp`; `asv_sim` depends on
  `asv_msgs`. Live check (bag `ws/data/bags/m26b_check`): 329/329 clean
  epochs corrupted, 10.000 Hz sim time, white spread E/N/U 5.15/5.13/8.64 mm.
- **Corruptor, GNSS velocity and heading (M2.6c):** the node also reads
  `/asv/ground_truth/odom` and publishes `/asv/sensors/gnss/velocity`
  (TwistWithCovarianceStamped, frame world: primary antenna velocity =
  central difference of the clean fixes k−1, k+1, sent one epoch late with
  stamp t_k, + white σ_v = 0.03 m/s per ENU axis, `asv.yaml`
  `velocity_noise`; covariance linear σ_v², angular −1) and
  `/asv/sensors/gnss/heading` (GnssHeading: α = π/2 − ψ_odom + white σ_α =
  0.1°/1.657 m = 1.0532 mrad, in [0, 2π); waits for odometry up to its
  stamp). Both stop only in a sky-view outage; same in every other mode.
  The velocity is the antenna's (includes ω × R·l): M3.3a must handle it.
  Classes `asv_core::YawHistory`, `central_difference()`,
  `azimuth_from_yaw()`; glue `asv_sim::GnssCorruption`. Profile keys
  `gnss.velocity`, `gnss.heading` (multipliers). Live check (bag
  `ws/data/bags/m26c_check`, circle manoeuvre): velocity error std
  0.032/0.028/0.030 m/s, heading 1.10 mrad.
- **Corruptor in the launch, recording check (M2.6d):** `sim.launch.py`
  (so also task1 and maneuver) starts the corruptor; launch arguments
  `corruptor:=true|false` (default true), `noise_profile:=<name>` (default
  `nominal`; a command-line value reaches the included `sim.launch.py`).
  Test profile `config/noise_profiles/mode_check.yaml` (nominal sizes; fixed
  → float 20 s → standalone 40 s → outage 60 s → fixed 70 s).
  `tools/check_corruptor.py <bag> --profile <name>`: residuals (clean, true
  bias, true slow GNSS error and odometry truth taken away) against σ from
  `asv.yaml` × profile, within 3 standard errors; counts and schedule; exit 1
  on a failure. Bag `ws/data/bags/m26d_check` (circle, mode_check, 102.7 s):
  ALL PASS. Headless RTF 0.857 with the corruptor, 0.872 without (one 30 s
  measurement each). The stop command (Running commands) now also kills the
  static TF publishers and our nodes: leftovers piled up in M2.6d and a
  later bridge passed no data.
- **True buoy map (M2.7):** every buoy and vessel in the `rb26_task*` models
  is static (D7 a, D19 a; `make_roboboat2026_course.py --floating` brings
  back the floating course, only for drift runs). `tools/make_buoy_map.py`
  writes `config/buoys_gt.yaml` (generated, committed; 32 objects: 4
  Sur-Mark, 14 A-0, 5 A-2, 4 indicator, 5 `other` = vessels; names
  `<task>::<object>`; radius = largest horizontal radius, D20 a); rerun it
  after changing the course; a test fails if the file is stale. Node
  `buoy_map_publisher` (started by `sim.launch.py`; loader `buoy_map.hpp`)
  publishes it once, at the first nonzero sim time, as
  asv_msgs/LandmarkArray on `/asv/ground_truth/buoys` (transient_local,
  reliable, depth 1). Drift of the floating course in 305.7 s with the boat
  idle (`tools/measure_buoy_drift.py`): none horizontally (≤ 1.3e-14 m),
  heave ≤ 3.4 cm. `task1_route.yaml` still types its buoys by hand; a test
  checks they agree with the map within 1 mm.
- **TF tree and live view (M2.8):** one tree, root `world`: static
  world → map (identity, node `static_tf_map`), world → base_link from node
  `ground_truth_tf` (`src/ground_truth_tf_node.cpp`, I/O only; glue
  `transform_from_odometry()` in `ground_truth_tf.hpp`, library `asv_sim_tf`;
  copies pose, stamp and frame names of `/asv/ground_truth/odom`, 50 Hz),
  sensor frames static from `asv_frames.yaml`. `sim.launch.py` arguments
  `ground_truth_tf:=true|false` (default true; false for slam_node, M4.1,
  since base_link may have only one parent) and `rviz:=false|true` (default
  false; `rviz/asv.rviz`, fixed frame world: TF, clean lidar cloud,
  ground-truth odometry, follower route); both reach `sim.launch.py` through
  task1 and maneuver. Live check: TF equals the odometry at its stamps
  (position diff 0, quaternion ≤ 1.1e-16).
- **Recording profiles (M2.9):** `config/recording_profiles.yaml` (`nav` <
  `nav_lidar` < `perception`, via `extends`), `tools/record.py`,
  `tools/check_bag_rates.py` (count = rate × duration within 1 %). `nav` is
  the clean-only input of slam_replay (no `/asv/sensors/...`, D4 a) plus the
  true buoy map and the autonomy and manoeuvre topics (Q2 a). Measured: 8.2 MB,
  393 MB, 2.9 GB per sim minute. The ZED runs at 15.15 Hz (4 ms step grid),
  so camera rates are not checked. Status topics only publish on change: a
  recording started after the manoeuvre or route began has none.
- **`asv_msgs` (M2.4):** interface package (rosidl, depends on `std_msgs`
  only): `BuoyColour`, `BuoyDetection(Array)`, `Landmark(Array)`,
  `GnssHeading`, `GnssStatus` (was planned as SensorStatus; renamed, M2.4 B b);
  first user: the corruptor (M2.6b, `GnssStatus`).
  Detections carry no σ (A a: Q comes from the filter config) and no true ID.
  Every field has a `# [unit]` comment; `tests/test_msgs_documented.py` in
  `asv_sim` checks that and the constants (needs the built workspace).
- **Gazebo sensor noise off (M2.2):** `asv.yaml` `gazebo_sensor_noise: false`
  (required key, D2 a): the generator writes no `<noise>` into the lidar, IMU
  and NavSat sensors; camera pixel noise stays. So `/asv/imu/data`,
  `/asv/gnss/...` and `/asv/lidar/points` are exact. Before M2.2 Gazebo read
  the GNSS noise as degrees (about 890 m): don't use GNSS data recorded before
  M2.2. `tools/check_clean_gnss.py <bag>` compares each fix with ground truth
  + R(q)·l (lever arm from `asv_frames.yaml`); M2.2 run
  `ws/data/bags/m22_clean_gnss`: largest difference 3.6 nm.
- Gazebo's IMU orientation is perfect (no noise). Never feed it to a filter. The
  corruptor's IMU message (D11, 2026-10-21) is a raw 200 Hz gyro and accel
  with noise and biases, stamped at measurement time, without orientation; it
  never carries the true yaw. ArduPilot's processing (low-pass, 50 Hz hold,
  publish-time stamp) is left out (optional M7.8).
- Drag model (xU=4, xUU=6) matches Gazebo almost exactly (Step 1 actuator
  check): 10 N → 1.00 m/s, 20 N → 1.52 m/s, 40 N → 2.27 m/s, all <0.1%
  error. These are Gazebo's own numbers: fine for the controller's feedforward,
  but using them inside the filter would be an inverse crime.
  Sway drag yVV = 58 N/(m/s)² since M1.2e (10.12; was 4 from VRX, the boat
  slid sideways); the 3-DOF test model matches Gazebo within 1 % on a pod turn.
- Steering sign and differential thrust both confirmed (Step 1): negative pod
  angle → positive yaw rate (left turn), left−/right+ thrust → left turn.
- Simulation time (M0.2): `sim.launch.py` sets `use_sim_time` for all its nodes;
  `tools/sim_time_check.py` checks ROS-clock timers. Headless Gazebo runs at a
  real-time factor of about 0.85, not 1.0 (`docs/design.md` §13).

## Layout (project root = ~/thesis/docker-sym on the host)
```
CLAUDE.md                 this file
README.md                 overview and quick start (for people)
.gitmodules               VRX submodule URL (osrf/vrx)
docs/                     master plan, design reference, glossary, milestone plans
docker/Dockerfile, compose.yaml, setup-env.sh   container definition
ws/                       colcon workspace, mounted in the container at /ws
  data/                   bags, run results, figures (never committed; git-ignored)
  src/vrx/                VRX submodule @ 7609d1b (third party, do not modify)
  src/asv_sim/            simulator package (ament_cmake)
    config/asv.yaml       boat description: the single source of truth
    config/asv_*          GENERATED by tools/make_asv.py - never edit by hand
    models/asv/           GENERATED Gazebo model - never edit by hand
    models/rb26_*         RoboBoat 2026 course elements (generated by
                          tools/make_roboboat2026_course.py; static since M2.7)
    config/buoys_gt.yaml  GENERATED true buoy map (tools/make_buoy_map.py, M2.7)
    config/recording_profiles.yaml  topic lists of the recording profiles (M2.9)
    worlds/               roboboat_course.sdf (course + boat), course.sdf
    include/asv_sim/      ROS-free C++ (mix(), Route, LOS, control, gates, Follower, Maneuver, teleop keys) (from M1.2); imu_corruption.hpp, gnss_corruption.hpp (message glue, M2.6a/b/c); buoy_map.hpp (M2.7); ground_truth_tf.hpp (M2.8)
    src/                  C++ library sources (mix, guidance, control, gate monitor, follower, maneuver, teleop, imu_corruption, gnss_corruption, buoy_map, ground_truth_tf); nodes waypoint_follower, maneuver_player, teleop, corruptor, buoy_map_publisher, ground_truth_tf
    test/                 gtests for the C++ code                       (from M1.2)
    tools/                Python tools: make_asv.py (generator), studies, analysis, check_corruptor.py (M2.6d), make_buoy_map.py and measure_buoy_drift.py (M2.7), record.py and check_bag_rates.py (M2.9)
    tests/                Python tests of the generator and tools
    launch/               sim.launch.py (Gazebo + bridge + static TF + ground-truth TF + corruptor + buoy map [+ RViz]), task launch files
    rviz/                 asv.rviz: live view, fixed frame world (M2.8)
    docs/                 HOW_IT_WORKS.md (generator), milestone1_plan.md
  src/asv_core/           ROS-free maths (D13 a): angles, geodesy (from M2.1), noise (M2.5), IMU corruptor (M2.6a), GNSS corruptor (M2.6b, M2.6c); gtests in test/
  src/asv_msgs/           message types (from M2.4)
  src/asv_slam, asv_perception, asv_eval   (planned, see docs/design.md §12)
```
