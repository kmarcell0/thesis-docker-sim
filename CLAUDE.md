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
- **Never write thesis text**: no thesis paragraphs, no LaTeX thesis source, not
  even as an example. Briefs, reports and docs are short bullet notes and equations, not
  finished paragraphs. Scripts may generate figures and tables of numbers;
  captions and any sentence around them are Marcell's.
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
M0.5, M2.9, small fixes Marcell asks for): goal, open decisions, files (with
⚠ marks), how we check it, in about five lines. Still wait for his go.

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
  of them on 2026-10-11; the open ones (D6, D10, the IMU message content of
  D11) are explained in `docs/decision_notes.md`. Marcell decides; you explain
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
`docs/master_plan.md` section 3. A rule tied to an open decision (D6, D10, the
D11 message content) is raised in the brief.

- **C++** nodes: C++17, rclcpp. Nodes only do I/O; the logic (maths, models,
  filter) is ROS-free so it can be unit tested. It lives in the new `asv_core`
  package (D13 a), `include/asv_core/*.hpp`. The Milestone 1 guidance
  code stays in `include/asv_sim/`.
- **Comments**: explain non-obvious code (why, not only what); keep functions
  small. A function that implements a thesis equation says so (section title, and
  the LaTeX label if known) and maps code names to the symbols.
- **Tests**: gtest for C++, `unittest` for Python. Every Jacobian is checked
  against a numerical derivative. Never weaken or skip a test to make it pass.
  Current baseline (commit 6b8be2b): 28 Python tests, run with `unittest` only;
  `colcon test` runs no `asv_sim` tests until the first gtest is added in M1.2.
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
docker compose exec -T sim bash -ic "pkill -f 'ros2 launch'; pkill -f 'gz sim'; pkill -f parameter_bridge"   # stop
```
Use `timeout` for anything that waits for messages, so commands never hang.

Recording (profiles in `docs/design.md` §8); `timeout -s INT` stops the recorder
cleanly so the bag is readable:
```
docker compose exec -T sim bash -ic "timeout -s INT 60 ros2 bag record --use-sim-time -o /ws/data/bags/<name> --topics <topics>"
```
Keyboard teleop (from M1.8 on) needs Marcell's own interactive terminal
(`docker compose exec sim bash`, then `ros2 run asv_sim teleop --ros-args -p
use_sim_time:=true`); you can't run or test the key input.

## Known state / caveats
Update this section in the same commit as the step that changes it.

- **No C++ code exists yet** (commit 6b8be2b): `CMakeLists.txt` has no C++
  targets, and `mix()`, the teleop and the C++ tests that older docs mention were
  never written. M1.2 writes the first C++ code and the gtest setup. Check
  `git log` and the plans for the current state.
- **GNSS noise bug (until step M2.2):** `asv.yaml` gives GNSS noise in metres, but
  Gazebo applies it in degrees, so the simulated GNSS noise is about 890 m.
  Don't use GNSS data recorded before M2.2.
- Gazebo's IMU orientation is perfect (no noise). Never feed it to a filter. The
  corruptor's IMU message imitates the real boat's IMU topic (D11) and never
  carries the true yaw.
- Drag model (xU=4, xUU=6) matches Gazebo almost exactly (Step 1 actuator
  check): 10 N → 1.00 m/s, 20 N → 1.52 m/s, 40 N → 2.27 m/s, all <0.1%
  error. These are Gazebo's own numbers: fine for the controller's feedforward,
  but using them inside the filter would be an inverse crime.
- Steering sign and differential thrust both confirmed (Step 1): negative pod
  angle → positive yaw rate (left turn), left−/right+ thrust → left turn.
- Simulation time (M0.2): `sim.launch.py` sets `use_sim_time` for all its nodes;
  `tools/sim_time_check.py` checks ROS-clock timers. Headless Gazebo runs at a
  real-time factor of about 0.85, not 1.0 (`docs/design.md` §13).

## Layout (project root = ~/thesis/docker-sym on the host)
```
CLAUDE.md                 this file
docs/                     master plan, design reference, glossary, milestone plans
docker/Dockerfile, compose.yaml, setup-env.sh   container definition
ws/                       colcon workspace, mounted in the container at /ws
  data/                   bags, run results, figures (never committed; git-ignored)
  src/vrx/                VRX (third party, do not modify)
  src/asv_sim/            simulator package (ament_cmake)
    config/asv.yaml       boat description: the single source of truth
    config/asv_*          GENERATED by tools/make_asv.py - never edit by hand
    models/asv/           GENERATED Gazebo model - never edit by hand
    models/rb26_*         RoboBoat 2026 course elements (generated by
                          tools/make_roboboat2026_course.py)
    worlds/               roboboat_course.sdf (course + boat), course.sdf
    include/asv_sim/      ROS-free C++ (mix(), Milestone 1 guidance)   (from M1.2)
    src/                  C++ nodes (waypoint follower, ...)            (from M1.3)
    test/                 gtests for the C++ code                       (from M1.2)
    tools/                Python tools: make_asv.py (generator), studies, analysis
    tests/                Python tests of the generator
    launch/               sim.launch.py (Gazebo + bridge + static TF), task launch files
    docs/                 HOW_IT_WORKS.md (generator), milestone1_plan.md
  src/asv_core, asv_msgs, asv_slam, asv_perception, asv_eval   (planned, see docs/design.md §12)
```
