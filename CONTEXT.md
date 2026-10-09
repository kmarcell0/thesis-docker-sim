# RoboBoat ASV Simulation — Project Context

Last updated: 2026-10-07

## Project overview

Simulation environment for the **BME Solar Boat Team's RoboBoat ASV** — an
autonomous surface vehicle (catamaran) with two steerable azimuth thrusters.
Built for Marcell's MSc thesis on SLAM and localisation (EKF-based state
estimation).

**The boat** carries a Livox Mid-360 lidar (mounted upside-down), a ZED 2i
stereo camera, an Arducam AR0234 wide-angle camera, a Pixhawk 6X IMU, and a
Unicore UM982 dual-antenna RTK GNSS receiver. Total mass ~16 kg, fits the
RoboBoat 2026 size box (6 × 3 × 3 ft).

**The stack:**

- **Gazebo Harmonic** — physics, rendering, sensor simulation
- **VRX** (Virtual RobotX) — water surface, waves, buoyancy model
- **ROS 2 Jazzy** — middleware, topic transport, TF tree
- **Docker** — everything runs in a single container (`roboboat-sim:jazzy`);
  host OS is NixOS

**Thesis goal:** build and validate an EKF-based localisation pipeline using
simulated sensor data, then transfer it to the real boat. The simulation must
produce realistic-enough sensor data (lidar, cameras, IMU, GNSS) with known
ground truth for comparison.

---

## Current state (Steps 0–1 complete)

Two of seven implementation steps are done. The simulation environment works and
the boat's actuators are verified.

| Step | Status | What it proved |
|---|---|---|
| Step 0 — Package skeleton & launch | Done | colcon build clean, 8 C++ tests + 28 Python tests pass, headless sim starts, ground truth odometry arrives at ~50 Hz |
| Step 1 — Actuator check | Done | Drag model matches Gazebo within 0.1% (10 N → 1.00 m/s, 20 N → 1.52 m/s, 40 N → 2.27 m/s). Steering sign confirmed: negative pod angle → positive yaw rate (boat turns left). Differential thrust also confirmed. |
| Steps 2–6 | Not started | Guidance logic, ROS node, launch file, tuning, documentation |

**Key verified facts:**

- The drag model `X = -(4u + 6u|u|)` predicts Gazebo's steady-state speeds
  almost exactly, so feedforward thrust in the speed controller will be accurate
- Steering sign convention: a positive pod angle turns the boat right (pods are
  behind the CG); the `mix()` function takes `steer > 0 = turn left` and
  outputs `pod_angle = -steer`
- Ground truth odometry publishes pose in world frame (ENU) and twist in body
  frame (`twist.linear.x` = forward speed, `twist.angular.z` = yaw rate)

---

## Architecture

Everything runs inside a single Docker container (`roboboat-sim:jazzy`). The
host mounts `./ws` at `/ws` inside the container, shares the GPU via `/dev/dri`,
and uses `network_mode: host` so ROS 2 DDS works transparently between host and
container.

**Single source of truth:** `config/asv.yaml` describes the entire boat — hulls,
thrusters, sensors, drag coefficients. The generator `tools/make_asv.py`
(package `tools/asvgen/`) reads this file and produces:

- `models/asv/model.sdf` — the Gazebo model with all geometry, physics plugins,
  and sensors
- `config/asv_bridge.yaml` — the ros_gz_bridge configuration (which ROS 2
  topics map to which Gazebo topics)
- `config/asv_frames.yaml` — static TF frames for every sensor relative to
  `base_link`
- `config/asv_report.txt` — design analysis (mass budget, hydrostatics, sensor
  coverage)

Never edit the generated files by hand. Change `asv.yaml` and re-run the
generator.

**Launch:** `ros2 launch asv_sim sim.launch.py gui:=false` starts three things:

1. **Gazebo Harmonic** — loads `worlds/roboboat_course.sdf`, runs physics and
   sensors
2. **ros_gz_bridge** — translates 20 topics between Gazebo and ROS 2 (sensors
   out, thruster commands in)
3. **Static TF publishers** — one per sensor frame from `asv_frames.yaml`

**Project layout:**

```
docker/Dockerfile, compose.yaml        container definition
ws/                                     colcon workspace, mounted at /ws
  src/vrx/                              VRX (third party, do not modify)
  src/asv_sim/                          OUR package (ament_cmake)
    config/asv.yaml                     boat description (edit this)
    config/asv_*                        generated configs (do not edit)
    models/asv/                         generated Gazebo model
    tools/asvgen/                       generator package
    tests/                              28 Python tests for the generator
    launch/sim.launch.py                simulation launch file
    docs/                               documentation and plans
```

---

## Thruster control interface

The boat has two steerable azimuth thrusters (left and right), each controlled
by two ROS 2 topics. All use `std_msgs/msg/Float64`.

| Topic | What it controls | Units | Range |
|---|---|---|---|
| `/asv/thrusters/left/thrust` | Left propeller force | Newtons | -40 to +50 |
| `/asv/thrusters/right/thrust` | Right propeller force | Newtons | -40 to +50 |
| `/asv/thrusters/left/pos` | Left pod steering angle | Radians | -1.57 to +1.57 |
| `/asv/thrusters/right/pos` | Right pod steering angle | Radians | -1.57 to +1.57 |

**Signal path:** ROS 2 publisher → `ros_gz_bridge` (Float64 → gz.msgs.Double)
→ Gazebo plugins. The `Thruster` plugin applies propeller force; the
`JointPositionController` rotates the pod at up to 2 rad/s.

**Steering sign convention:** a negative pod angle makes the boat turn left
(positive yaw rate). The pods sit behind the centre of gravity, so pushing the
stern right swings the bow left. The `mix()` function takes `steer > 0 = turn
left` and outputs `pod_angle = -steer`.

**From the command line** (inside the container):

```bash
# Single message
ros2 topic pub --once /asv/thrusters/left/thrust std_msgs/msg/Float64 '{data: 20.0}'

# Sustained at 10 Hz (Ctrl+C to stop)
ros2 topic pub --rate 10 /asv/thrusters/left/thrust std_msgs/msg/Float64 '{data: 20.0}'
```

Ground truth feedback arrives on `/asv/ground_truth/odom`
(nav_msgs/Odometry).

---

## Upcoming work (Milestone 1, Steps 2–6)

Milestone 1 goal: the boat drives RoboBoat Task 1 (Evacuation Route & Return)
autonomously in Gazebo — through both entry gates, a turn, and back through the
same gates, with no human input after launch.

Full plan: `ws/src/asv_sim/docs/milestone1_plan.md`

### Step 2 — Guidance and control logic (pure C++, no ROS)

All maths in `include/asv_sim/guidance.hpp`: `Route` (polyline following,
cross-track error, segment switching), `LosGuidance` (line-of-sight desired
heading), `HeadingControl` (PD + spin assist for sharp turns), `SpeedControl`
(feedforward from the known drag model + PI correction), `GateMonitor` (detects
gate passages by line-segment intersection). Unit tests in
`test/test_guidance.cpp` including a closed-loop kinematic simulation of the
full route.

### Step 3 — The ROS node (`src/waypoint_follower.cpp`)

A thin node that subscribes to odometry, runs a 20 Hz timer, executes the state
machine (WAITING → RUNNING → FINISHED), and publishes the four thruster commands
plus autonomy status topics (`/asv/autonomy/path`, `/target`, `/status`,
`/cross_track_error`).

### Step 4 — One-command launch (`launch/task1.launch.py`)

Includes `sim.launch.py`, starts the waypoint follower with route parameters
from `config/task1_route.yaml` and drag/thrust values read from
`config/asv.yaml`.

### Step 5 — Tuning and evaluation

Run the full task in Gazebo, adjust gains in the YAML, verify gate passages and
buoy clearance. An analysis tool (`tools/analyze_run.py`) reads recorded bag
files and prints metrics. Success criteria: 3 consecutive runs with both gates
passed in both directions, 0.5 m minimum buoy clearance, no collisions.

### Step 6 — Documentation

`docs/AUTONOMY.md` explaining the route, algorithms, equations, chosen
parameters, and measured results.

### Key design decisions

- All control logic is pure C++ without ROS dependencies, so it can be unit
  tested without a running simulation
- The `odom_topic` parameter defaults to `/asv/ground_truth/odom` but can be
  swapped to an EKF output topic later without changing any code
- The waypoint route is defined in a YAML parameter file, not hardcoded
- `mix(throttle, steer, spin)` already exists and is tested — it converts
  high-level commands to the four thruster values

---

## Conventions and constraints

**Frames:** world = ENU (x east, y north, z up), boat = `base_link` (x forward,
y left, z up). Angles in radians, SI units everywhere.

**Code:**

- ROS nodes (when added): C++17, rclcpp. Logic in `include/asv_sim/*.hpp`
  without ROS dependencies; nodes only do I/O
- Tools, launch files, offline analysis: Python 3
- Topics: everything under `/asv/...`

**Working agreement:**

- Work in the small steps of the current plan
  (`docs/milestone1_plan.md`). Do one step at a time, then stop and summarise
- Run relevant tests after every change. Never claim something works without
  having run it
- Commit after each completed step
- Before adding a dependency or touching `docker/`, `compose.yaml`, or the
  generator (`tools/asvgen/`), ask first
- After changing `config/asv.yaml`: run `python3 tools/make_asv.py` (no rebuild
  needed)

**Running commands** (from the host, in the project root):

```bash
docker compose exec -T sim bash -ic "cd /ws && colcon build --symlink-install --packages-select asv_sim"
docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && python3 -m unittest discover -s tests"
docker compose exec -d sim bash -ic "ros2 launch asv_sim sim.launch.py gui:=false > /tmp/sim.log 2>&1"
docker compose exec -T sim bash -ic "pkill -f 'ros2 launch'; pkill -f 'gz sim'; pkill -f parameter_bridge"
```

Use `timeout` for anything that waits for messages, and `-T` (no TTY) for
non-interactive commands.
