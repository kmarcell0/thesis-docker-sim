# RoboBoat ASV simulation · GNSS-aided EKF-SLAM

Simulation environment for the **BME Solar Boat Team**'s RoboBoat autonomous
surface vehicle (ASV), built for a bachelor's thesis on GNSS-aided EKF-SLAM:
map the course buoys by colour, localise the boat, and measure how accurate and
how consistent both are against ground truth under different sensor noise levels.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![VRX](https://img.shields.io/badge/VRX-7609d1b-0A7BBB)
![Docker](https://img.shields.io/badge/runs%20in-Docker-2496ED?logo=docker&logoColor=white)
![C++17 · Python 3](https://img.shields.io/badge/C%2B%2B17%20%C2%B7%20Python%203-informational)

> **Thesis source:** [kmarcell0/thesis-latex](https://github.com/kmarcell0/thesis-latex)

---

## What this repository aims to do

- **Simulate** the boat, the water and the RoboBoat 2026 course in Gazebo + VRX
  (waves, buoyancy, drag), with every sensor of the real boat.
- **Separate truth from measurement:** Gazebo gives perfect sensor data and the
  true pose; our own *corruptor* adds noise, biases, GNSS fix modes and outages,
  all seeded and set from one file.
- **Estimate:** an EKF-SLAM filter fuses GNSS position and velocity, the
  dual-antenna heading, the gyro and buoy detections (range, bearing, colour) to
  estimate the boat's pose and the buoy map.
- **Evaluate:** compare the estimate with ground truth: position and heading
  error, map error, and filter consistency (NEES / NIS).
- **Sweep:** record each trajectory once, then replay it through corruptor →
  detector → filter for many noise levels and seeds, so every setting sees the
  same motion and results repeat exactly.
- **Stay deployable:** the filter, perception and guidance code is plain C++17
  with ROS-free cores, so it can later run on the boat's onboard computer.

## The boat

<p align="center">
  <img src="ws/src/asv_sim/config/asv_layout.png" width="560"
       alt="Top and side view of the simulated boat with sensor positions and fields of view">
</p>

| Part | In the simulation |
|---|---|
| Hull | Catamaran, 2 × 1.60 m cylindrical hulls, 0.66 m apart; 18.2 kg in total |
| Propulsion | 2 steerable (azimuth) thrusters, +50 / −40 N, ±90° pod angle |
| Lidar | Livox Mid-360, mounted upside down, 10 Hz |
| Stereo camera | Stereolabs ZED 2i (4 mm lens) |
| Wide camera | Arducam AR0234, tilted 20° down |
| IMU | Pixhawk 6X |
| GNSS | Unicore UM982 dual-antenna RTK (position + heading), 1.66 m baseline |

All numbers come from [`asv.yaml`](ws/src/asv_sim/config/asv.yaml), the single
source of truth for the model; the Gazebo model, the bridge and the TF frames are
generated from it ([how](ws/src/asv_sim/docs/HOW_IT_WORKS.md)).

## How it fits together

```
 Gazebo + VRX  (perfect sensors, true pose)
   │ ros_gz_bridge
   ├──► /asv/ground_truth/odom ─────┬──────────────────────────────┐
   │                                │                              │
   ▼ /asv/{imu,gnss,lidar}/...      ▼                              │
 corruptor                   fake detector                         │
 (noise, biases, GNSS        (ground truth + true buoy map,        │
  modes and outages,          controlled noise, true colour)       │
  dual-antenna heading)             │                              │
   │ /asv/sensors/...               │ /asv/perception/buoys        │
   │                                ▼                              ▼
   └─────────────────────────► EKF-SLAM ──► /asv/slam/... ──► evaluation
                                    │                       (errors, NEES)
                                    ▼
                              RViz (live view)
```

The thesis numbers come from the offline path: a Gazebo run is recorded once as
a rosbag; `slam_replay` then runs corruptor → detector → filter over it for each
setting. Details: [`docs/design.md`](docs/design.md).

## Status

```
M0 🟩🟩🟩🟩🟩 │ M1 🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩 │ M2 🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩 │ M3 🟩⬜⬜⬜⬜⬜⬜⬜⬜ │ M4 ⬜ │ M6 ⬜⬜⬜
33 of 45 steps done: 73% completed!
```

| Milestone | What | State |
|---|---|---|
| M0 | Housekeeping: docs, simulation time, toolchain, repository | done |
| M1 | Drive the course on ground truth (guidance, waypoint follower, teleop) | done: Task 1 (3/3 runs pass), maneuver player, keyboard teleop |
| M2 | From perfect to realistic sensor data (geodesy, corruptor, buoy map) | done: Gazebo noise off, corruptor (IMU, GNSS position, velocity, heading, fix modes, outages), true buoy map, TF tree + RViz, recording profiles |
| M3 | The EKF-SLAM filter, offline (detector, filter, replay, evaluation) | in progress: fake detector done; next the measurement queue |
| M4 | Live filter and RViz view | planned |
| M5 | Real lidar + camera perception | optional |
| M6 | Experiments: noise sweeps, GNSS outages, waves | planned |

The step-by-step plan, with every decision and its reason, is in
[`docs/master_plan.md`](docs/master_plan.md).

## Quick start

Needs Docker with Compose and a GPU usable through `/dev/dri` (for the Gazebo
window and the camera sensors). Developed on NixOS; any Linux host should work.

```bash
# 1. Clone, including VRX (pinned submodule)
git clone --recurse-submodules https://github.com/kmarcell0/thesis-docker-sim.git
cd thesis-docker-sim            # already cloned? git submodule update --init

# 2. Write .env with your user, video and render group IDs, then start the container
./setup-env.sh
docker compose up -d

# 3. Build our packages (asv_sim and what it needs: asv_core, asv_msgs; not all of VRX)
docker compose exec -T sim bash -ic "cd /ws && colcon build --symlink-install --packages-up-to asv_sim"

# 4. Tests (Python, then the C++ gtests)
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 -m unittest discover -s tests"
docker compose exec -T sim bash -ic "cd /ws && colcon test --packages-select asv_core asv_sim && colcon test-result --verbose"
```

Run RoboBoat Task 1 autonomously (Gazebo + waypoint follower, one command;
how it works: [`AUTONOMY.md`](ws/src/asv_sim/docs/AUTONOMY.md)):

```bash
docker compose exec sim bash -ic "ros2 launch asv_sim task1.launch.py"              # with the Gazebo window
docker compose exec -d sim bash -ic "ros2 launch asv_sim task1.launch.py gui:=false > /tmp/task1.log 2>&1"
docker compose exec -T sim bash -ic "timeout 90 ros2 topic echo /asv/autonomy/status"   # WAITING → RUNNING i/9 → FINISHED
```

Record a run with a recording profile and check it (bags go to `ws/data/bags/`;
profiles `nav` < `nav_lidar` < `perception` in
[`recording_profiles.yaml`](ws/src/asv_sim/config/recording_profiles.yaml),
`docs/design.md` §8). Start the recording before the route or maneuver begins:
status topics only publish on change.

```bash
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/record.py --profile nav --name task1_1 --duration 160"
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/check_bag_rates.py /ws/data/bags/task1_1 --profile nav"   # counts = rate × duration
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/analyze_run.py /ws/data/bags/task1_1"   # metrics.json, trajectory.png
```

Open-loop maneuvers (`straight`, `zigzag`, `circle`; for repeatable test runs,
`AUTONOMY.md` §9) and a comparison of recorded runs:

```bash
docker compose exec -T sim bash -ic "ros2 launch asv_sim maneuver.launch.py maneuver:=circle gui:=false"
# each run: fresh Gazebo + a recording (odom, /asv/maneuver/status, thruster topics) into ws/data/bags/maneuver_circle_<k>
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/compare_runs.py --maneuver circle /ws/data/bags/maneuver_circle_{1,2,3}"   # compare.json, compare.png
```

Drive by keyboard (WASD; `AUTONOMY.md` §10). Only in your own interactive
terminal, never together with the follower or the maneuver player:

```bash
docker compose exec sim bash
ros2 run asv_sim teleop --ros-args -p use_sim_time:=true
```

Run the simulation. `sim.launch.py` starts Gazebo, the bridge, the static and
ground-truth TF, the corruptor (`/asv/sensors/...`), the true buoy map
(`/asv/ground_truth/buoys`) and the fake detector (`/asv/perception/buoys`):

```bash
# with the Gazebo window (if it does not open, allow X access on the host: xhost +local:)
docker compose exec sim bash -ic "ros2 launch asv_sim sim.launch.py"

# headless, in the background
docker compose exec -d sim bash -ic "ros2 launch asv_sim sim.launch.py gui:=false > /tmp/sim.log 2>&1"
docker compose exec -T sim bash -ic "timeout 10 ros2 topic hz /asv/ground_truth/odom"

# stop (also the static TF publishers and our nodes, so nothing piles up between runs)
docker compose exec -T sim bash -ic "pkill -f 'ros2 launch'; pkill -f 'gz sim'; pkill -f parameter_bridge; pkill -f static_transform_publisher; pkill -f 'lib/asv_sim/'"
```

Launch arguments of `sim.launch.py` (they also work on `task1.launch.py` and
`maneuver.launch.py`):

| Argument | Default | What it does |
|---|---|---|
| `gui:=true\|false` | `true` | Gazebo window |
| `rviz:=false\|true` | `false` | RViz live view (`rviz/asv.rviz`, fixed frame `world`) |
| `corruptor:=true\|false` | `true` | realistic sensor data on `/asv/sensors/...` |
| `noise_profile:=<name>` | `nominal` | `config/noise_profiles/<name>.yaml` (multipliers on `asv.yaml`, GNSS mode schedule, seed) |
| `fake_detector:=true\|false` | `true` | buoy detections from ground truth on `/asv/perception/buoys` |
| `detector:=<name>` | `nominal` | `config/detector/<name>.yaml` (range and bearing noise, misses, false alarms, seed) |
| `ground_truth_tf:=true\|false` | `true` | world → base_link from ground truth (off once the filter publishes it) |

Useful checks with the simulation running:

```bash
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/actuator_check.py"   # drag model, steering sign
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/sim_time_check.py"   # timers follow simulation time
```

Checks on a recorded bag:

```bash
# clean GNSS fix = true antenna position (Gazebo noise is off)
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/check_clean_gnss.py /ws/data/bags/<bag>"
# corruptor errors have the configured mean and spread (the bag needs the clean, /asv/sensors/... and ground-truth topics)
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/check_corruptor.py /ws/data/bags/<bag> --profile nominal"
```

After editing `asv.yaml`, regenerate the model:
`docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/make_asv.py"`.
After changing the course, regenerate the true buoy map:
`docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 tools/make_buoy_map.py"`
(a test fails if `buoys_gt.yaml` is stale).

## Repository layout

```
docs/                    master plan, design reference, glossary, step logs
docker/Dockerfile        container image (ROS 2 Jazzy + Gazebo Harmonic)
compose.yaml             container setup (GPU, X11, workspace mount)
setup-env.sh             writes .env for compose.yaml
ws/                      colcon workspace, mounted at /ws in the container
  src/vrx/               VRX (git submodule, osrf/vrx @ 7609d1b)
  src/asv_core/          ROS-free maths (Eigen only): angles, geodesy (lat/lon <-> ENU),
                         noise models, IMU and GNSS corruptor, fake detector; gtests
  src/asv_msgs/          message types: buoy detections, landmarks, GNSS heading and status
  src/asv_sim/           our simulator package
    config/asv.yaml      boat description (edit this)
    config/asv_*         generated from asv.yaml (bridge, frames, report, layout)
    config/buoys_gt.yaml generated true buoy map (32 course objects)
    config/noise_profiles/   corruptor profiles (nominal, mode_check)
    config/detector/     fake detector settings (nominal)
    config/recording_profiles.yaml  topic lists for record.py
    config/task1_route.yaml  Task 1 route, gains, buoys
    config/maneuvers/    open-loop maneuver tables (straight, zigzag, circle)
    config/teleop.yaml   teleop key steps
    models/, worlds/     generated boat model, RoboBoat 2026 course (static buoys)
    include/asv_sim/, src/   ROS-free guidance, control, maneuver and teleop logic,
                         message glue; nodes waypoint_follower, maneuver_player, teleop,
                         corruptor, buoy_map_publisher, ground_truth_tf, fake_detector
    test/                C++ gtests (incl. a closed-loop run on a 3-DOF model)
    launch/              sim.launch.py, task1.launch.py, maneuver.launch.py
    rviz/asv.rviz        live view
    tools/               model and buoy-map generators, studies, recording, checks,
                         analyze_run.py, compare_runs.py, geodesy.py
    tests/               Python tests
  data/                  bags, run results, figures (not in git)
```

Planned packages (`asv_slam`, `asv_eval`, `asv_perception`) are listed in
[`docs/design.md`](docs/design.md) §12.

## Documentation

| File | What is in it |
|---|---|
| [`docs/master_plan.md`](docs/master_plan.md) | What gets built and in which order; all decisions |
| [`docs/design.md`](docs/design.md) | Topics, frames, time rules, noise models, filter, evaluation |
| [`docs/glossary.md`](docs/glossary.md) | Symbols and plain-language terms |
| [`docs/decision_notes.md`](docs/decision_notes.md) | Background for the decisions still open |
| [`docs/plans/`](docs/plans/) | Milestone plans with the brief and report of every step (M0, M2 on) |
| [`docs/reviews/`](docs/reviews/) | External reviews (Gemini) of the plan and the code, checked point by point |
| [`ws/src/asv_sim/docs/milestone1_plan.md`](ws/src/asv_sim/docs/milestone1_plan.md) | Milestone 1: design history, decisions 10.x, briefs and reports of M1.x |
| [`ws/src/asv_sim/docs/AUTONOMY.md`](ws/src/asv_sim/docs/AUTONOMY.md) | M1 drivers as built (follower, maneuver player, teleop): equations, parameters, results |
| [`ws/src/asv_sim/docs/HOW_IT_WORKS.md`](ws/src/asv_sim/docs/HOW_IT_WORKS.md) | How the boat model generator works |
| [`CLAUDE.md`](CLAUDE.md) | Working rules for the AI coding assistant used in this project |

## Related

- **Thesis (LaTeX source):** <https://github.com/kmarcell0/thesis-latex>
- **VRX** (Virtual RobotX; water, waves, buoyancy, course models):
  <https://github.com/osrf/vrx>, pinned at commit `7609d1b`
- **BME Solar Boat Team**, Budapest University of Technology and Economics
