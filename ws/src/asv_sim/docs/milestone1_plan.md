# Milestone 1 — Task 1 autonomously in simulation

**Goal:** the boat drives RoboBoat Task 1 (Evacuation Route & Return) by itself
in Gazebo: from the start, through both entry gates, a turn, and back out
through the same gates, using its real (simulated) thrusters. No keyboard, no
human input after launch.

**Why this first:** it closes the loop "the boat moves → the sensors produce
data → the data can be recorded". It is also the mandatory qualification task
of the competition. It uses Gazebo's ground-truth position for now; the
SLAM/EKF output replaces it later (same message type, a different topic).

**Scope note:** path planning and motor control are open taskforce subsystems.
The follower built here is a deliberately simple *test driver* with clean
interfaces (section 4), so that a colleague's planner or controller can
replace it later without touching anything else.

---

## 1. Success criteria (all must hold)

1. **One command** starts the simulation and the run: `ros2 launch asv_sim task1.launch.py`.
2. **Both gates, both directions:** the boat passes between both buoy pairs
   outbound and inbound, with at least 0.5 m clearance between its side and
   any buoy.
3. **No collision** with any buoy (minimum distance logged).
4. **It stops by itself** after the exit gates (thrust 0, status `FINISHED`).
5. **Repeatable:** 3 consecutive runs succeed.
6. **Observable:** the planned route, the current target point and the status
   are published as topics (visible in RViz now, Foxglove in Milestone 2).
7. **Tested and documented:** unit tests pass; `docs/AUTONOMY.md` explains the
   method with its equations and the tuned parameters.

## 2. The route

Course geometry (from `worlds/roboboat_course.sdf`): the boat spawns at (0, 0)
facing east; the entry gates are at x = 5 m and x = 13 m, each a red buoy at
y = +1.524 and a green buoy at y = −1.524 (Taylor Made Sur-Mark, radius
0.23 m). Task 2's channel starts at y = 6 (north), Task 3's gate is at x = 30
(south-east), so the turn goes clockwise through the space between them.

![route](img/task1_route.png)

| Waypoint | x [m] | y [m] | Speed towards it [m/s] | Role |
|---|---|---|---|---|
| W0 | 0 | 0 | – | start (spawn position) |
| W1 | 17 | 0 | 1.0 | straight out through both gates |
| W2 | 22 | 2.5 | 0.6 | set up the turn (swing slightly north) |
| W3 | 25 | 1 | 0.6 | teardrop turn, clockwise |
| W4 | 25.5 | −2 | 0.6 | |
| W5 | 23 | −4 | 0.6 | |
| W6 | 20 | −2.5 | 0.6 | |
| W7 | 17 | −0.5 | 0.6 | rejoin the centre line |
| W8 | 15 | 0 | 0.8 | aligned 2 m before gate 2 |
| W9 | −1 | 0 | 1.0 | out through both gates, stop |

Route length 57 m. Checked clearance of the centre line to every buoy, minus
the buoy radius and the boat's half width (0.44 m): 0.86 m at the gate buoys
(passing through the centre), 3.2 m to the Task 2 channel buoys, more to all
others. The waypoints live in a parameter file, not in code (section 6).

## 3. Architecture

```
 /asv/ground_truth/odom ──► waypoint_follower (C++ node) ──► /asv/thrusters/{left,right}/{thrust,pos}
 (later: your EKF output)        │                               (to Gazebo via the bridge)
                                 │ uses, pure C++ / no ROS:
                                 │   asv_sim::Route          route progression, cross-track error
                                 │   asv_sim::LosGuidance    desired heading (line of sight)
                                 │   asv_sim::HeadingControl steer + spin from heading error
                                 │   asv_sim::SpeedControl   throttle from speed error
                                 │   asv_sim::mix()          -> four thruster commands (exists)
                                 │   asv_sim::GateMonitor    detects gate passages
                                 ▼
          /asv/autonomy/path (nav_msgs/Path), /asv/autonomy/target (geometry_msgs/PointStamped),
          /asv/autonomy/status (std_msgs/String), /asv/autonomy/cross_track_error (std_msgs/Float64)
```

Same principle as the teleop: **all maths in plain C++ classes without ROS**
(unit tested, reusable), and a **thin node** for ROS I/O. The node runs a fixed
20 Hz control loop on a timer and always uses the latest odometry message.

## 4. Interfaces (the contract other subsystems can rely on)

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| in | `odom_topic` parameter, default `/asv/ground_truth/odom` | nav_msgs/Odometry | pose in world (ENU), twist in body frame |
| out | `/asv/thrusters/{left,right}/thrust` | std_msgs/Float64 | N |
| out | `/asv/thrusters/{left,right}/pos` | std_msgs/Float64 | pod angle, rad |
| out | `/asv/autonomy/path` | nav_msgs/Path | the planned route (latched: transient local) |
| out | `/asv/autonomy/target` | geometry_msgs/PointStamped | current line-of-sight target point |
| out | `/asv/autonomy/status` | std_msgs/String | `WAITING`, `RUNNING k/N`, `FINISHED`, `STOPPED: <reason>`, gate events |
| out | `/asv/autonomy/cross_track_error` | std_msgs/Float64 | m, for plotting |

## 5. Algorithms

Notation: boat position $\mathbf p = (x, y)$ and heading $\psi$ in the world
frame; forward speed $u$ and yaw rate $r$ in the body frame (from the odometry
twist). The heading comes from the orientation quaternion:
$\psi = \operatorname{atan2}\big(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2)\big)$.
Angle differences are always wrapped to $(-\pi, \pi]$.

### 5.1 Route progression (`Route`)
The route is a polyline $P_0 \dots P_N$; the boat follows segment
$k$ from $P_k$ to $P_{k+1}$, with direction $\alpha_k = \operatorname{atan2}(\Delta y, \Delta x)$
and length $L_k$. Expressed in the segment's frame:

$$s = (\mathbf p - P_k)\cdot\hat t, \qquad e = (\mathbf p - P_k)\cdot\hat n,
\qquad \hat t = (\cos\alpha_k, \sin\alpha_k),\ \hat n = (-\sin\alpha_k, \cos\alpha_k)$$

$s$ is the along-track progress, $e$ the cross-track error ($e > 0$: boat is
left of the path). **Switching:** go to the next segment when
$s \ge L_k - R_\text{switch}$ (the boat has reached or passed the waypoint
*along the path*). This "passed" test, unlike a pure distance circle, can never
make the boat circle a waypoint it missed by a little.

### 5.2 Line-of-sight guidance (`LosGuidance`)
The standard path-following law for marine vehicles (Fossen, *Handbook of
Marine Craft Hydrodynamics and Motion Control*): aim at a point a lookahead
distance $\Delta$ ahead on the path:

$$\psi_d = \alpha_k + \operatorname{atan2}(-e, \Delta)$$

Far from the path the boat heads almost straight back to it; on the path it
heads along it. $\Delta$ is the main tuning knob: small → tight tracking but
oscillation; large → smooth but corners are cut. Start with $\Delta = 3$ m
(about two boat lengths). The target point $P_k + (s + \Delta)\hat t$ is
published for visualisation.

### 5.3 Heading control (`HeadingControl`)
Heading error $\tilde\psi = \operatorname{wrap}(\psi_d - \psi)$, positive =
the desired heading is to the left. A PD law with the derivative taken on the
measured yaw rate (no kick when the target jumps at a waypoint switch):

$$\text{steer} = \operatorname{clamp}\big(K_\psi\,\tilde\psi - K_r\,r,\ \pm\text{max\_steer}\big)$$

Steering the pods only turns the boat while there is thrust, so for large
errors (sharp turns, slow speed) **spin assist** adds differential thrust:

$$\text{spin} = \begin{cases}\operatorname{clamp}(K_\text{spin}\,\tilde\psi,\ \pm\text{spin\_max}) & |\tilde\psi| > \psi_\text{spin}\\ 0 & \text{otherwise}\end{cases}$$

Signs follow `mix()`: steer > 0 and spin > 0 both turn the boat
counter-clockwise (left).

### 5.4 Speed control (`SpeedControl`)
Commanded speed: the waypoint's speed, reduced when the boat points the wrong
way and when it approaches the final waypoint:

$$u_\text{cmd} = \min\big(u_k \cdot \max(0, \cos\tilde\psi),\ K_\text{stop}\, d_\text{end}\big)$$

Throttle = **model-based feedforward + PI correction**. The simulated drag is
known exactly - it is the boat's own `hydrodynamics` in `config/asv.yaml`
(surge: $X = -(X_u + X_{uu}|u|)\,u$) - so the thrust needed to hold a speed is
known too, and the PI only corrects what the model gets wrong:

$$T_\text{ff}(u) = X_u u + X_{uu} u|u|, \qquad
\text{throttle} = \frac{T_\text{ff}(u_\text{cmd})}{2\,T_\text{max}} + K_p(u_\text{cmd}-u) + K_i\!\int(u_\text{cmd}-u)\,dt$$

clamped to $[-0.3, 1]$, with anti-windup (stop integrating while saturated).
Example: 1.0 m/s needs $4 + 6 = 10$ N in total, i.e. 5 N per thruster
(throttle 0.1 at 50 N max).

### 5.5 Allocation
`asv_sim::mix(throttle, steer, spin)` (already exists, unit tested) turns the
three values into the four thruster commands.

### 5.6 Gate monitor (`GateMonitor`)
Gates are given as buoy pairs (red, green). Each control step, the segment from
the previous to the current boat position is intersected with each gate
segment. On a crossing it reports: gate name, direction (outbound/inbound,
from the sign of the cross product), lateral offset from the gate centre, and
**pass/fail**: pass if the boat's side stays at least `gate_margin` (0.5 m)
inside both buoys. Also tracks the minimum distance to every buoy
(collision check). Results go to the status topic and the log.

### 5.7 Safety and states
`WAITING` (no odometry yet, or the start delay - 3 s, lets the boat settle on
the water) → `RUNNING k/N` → `FINISHED` (all thrust 0, pods straight).
`STOPPED: <reason>` from any state if the odometry is older than 0.5 s or the
run exceeds `max_run_time`: commands go to zero immediately. The node
publishes commands every cycle in every state, so "stopped" is actively
commanded, not just "no more messages".

## 6. Parameters (`config/task1_route.yaml`, ROS parameter file)

```yaml
waypoint_follower:
  ros__parameters:
    odom_topic: /asv/ground_truth/odom
    control_rate: 20.0          # Hz
    start_delay: 3.0            # s after the first odometry message
    max_run_time: 180.0         # s
    odom_timeout: 0.5           # s
    # route (W1..W9; W0 is wherever the boat starts)
    waypoints_x: [17.0, 22.0, 25.0, 25.5, 23.0, 20.0, 17.0, 15.0, -1.0]
    waypoints_y: [0.0, 2.5, 1.0, -2.0, -4.0, -2.5, -0.5, 0.0, 0.0]
    speeds:      [1.0, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.8, 1.0]
    switch_distance: 1.5        # m, R_switch
    lookahead: 3.0              # m, Delta
    # heading control (starting values - tuned in step 5)
    heading_kp: 1.2             # rad steer per rad error
    heading_kd: 0.6             # rad steer per rad/s yaw rate
    max_steer: 0.8              # rad
    spin_threshold: 0.8         # rad (~45 deg)
    spin_kp: 0.5
    spin_max: 0.4
    # speed control
    speed_kp: 0.3               # throttle per m/s error
    speed_ki: 0.05
    stop_gain: 0.5              # 1/s, K_stop
    # gates: name + red buoy (x, y) + green buoy (x, y)
    gate_names: [gate1, gate2]
    gate_coords: [5.0, 1.524, 5.0, -1.524, 13.0, 1.524, 13.0, -1.524]
    gate_margin: 0.5            # m between the boat's side and a buoy
    boat_half_width: 0.44       # m
```

The boat's own values - drag coefficients ($X_u$, $X_{uu}$) and `max_thrust` -
are **not** repeated here: the launch file reads them from `config/asv.yaml`
(single source of truth, as for everything else) and passes them as
parameters.

## 7. Implementation steps (for Claude Code - one at a time)

Each step ends with the listed check passing, a short summary, and a commit.

### Step 0 — Set up and verify the current state
- `git init` in the project root with a `.gitignore` for `ws/build/`,
  `ws/install/`, `ws/log/`, `.env`, `ws/src/vrx/` (third party; note its
  commit hash in the README instead).
- First real build of `asv_sim` (the C++ nodes were never built against real
  ROS - fix whatever the compiler reports, explain each fix).
- **Check:** `colcon build` clean, C++ tests (8) and Python tests (28) pass;
  headless sim starts; `/asv/ground_truth/odom` arrives at ~50 Hz.

### Step 1 — Actuator check (measure before controlling)
A small tool (`tools/actuator_check.py`) that commands fixed values with
the boat on the water and records the response from the odometry:
1. both thrusters 10, 20, 40 N, pods straight → steady forward speed;
   compare with the drag model's prediction ($X_u u + X_{uu}u^2 = T$);
2. 20 N with pods at −0.4 rad → yaw rate must be **positive** (turns left):
   **confirms the steering sign** (see CLAUDE.md);
3. spin: left −10 N, right +10 N → yaw rate positive.
- **Check:** a short table of measured vs predicted values in the step
  summary; the steering sign is confirmed or fixed in `mix()` and its test.

### Step 2 — Guidance and control logic (pure C++, no ROS)
`include/asv_sim/guidance.hpp` (+ `.cpp` if it gets long): `Route`,
`LosGuidance`, `HeadingControl`, `SpeedControl`, `GateMonitor`, angle
wrapping, yaw from quaternion. Unit tests in `test/test_guidance.cpp`:
- cross-track error sign and segment switching (incl. a missed waypoint);
- LOS heading on, left of and right of the path;
- angle wrapping across ±π (the turn passes through west = ±180°!);
- gate crossing: pass, fail (too close to a buoy), direction;
- **closed-loop logic test without Gazebo:** a simple kinematic boat model
  (speed with a first-order lag, yaw rate ∝ steer·u + spin) follows the Task 1
  route and passes both gates both ways. This catches logic errors in seconds;
  real dynamics are tuned in Gazebo (step 5).
- **Check:** all tests pass.

### Step 3 — The node (`src/waypoint_follower.cpp`)
Parameters (section 6), odometry subscription, 20 Hz timer, the state machine
(5.7), publishing commands + the four autonomy topics (`path` with
transient-local QoS so late viewers still get it).
- **Check:** builds without warnings; with the sim running headless and the
  node started by hand, the status goes `WAITING` → `RUNNING 1/9` and the
  boat's x position increases.

### Step 4 — One-command launch (`launch/task1.launch.py`)
Includes `sim.launch.py` (arguments passed through, e.g. `gui:=false`), starts
`waypoint_follower` with `config/task1_route.yaml` plus the drag and thrust
values read from `config/asv.yaml`.
- **Check:** `ros2 launch asv_sim task1.launch.py gui:=false` runs the
  whole task; the log shows the gate events and `FINISHED`.

### Step 5 — Tuning and evaluation
Run, read the log (gate offsets, minimum buoy distance, cross-track error,
time), adjust gains in the YAML, repeat. A small analysis tool
(`tools/analyze_run.py`) that reads a recorded run (`ros2 bag record` of
odometry + autonomy topics) and prints the metrics, so runs can be compared
objectively.
- **Check:** criteria 2–5 of section 1 met in 3 consecutive runs; final gains
  and results recorded.

### Step 6 — Documentation
`docs/AUTONOMY.md`: route, architecture, the equations of section 5, chosen
parameters and why, measured results (table from step 5), known limitations.
Update `README.md` with the run command.
- **Check:** Marcell has read it and can explain every part.

## 8. Known risks and open points

| Risk | Mitigation |
|---|---|
| C++ nodes fail to build on first try | Step 0 fixes them before anything new is added |
| Steering sign wrong | Step 1 measures it before any controller uses it |
| Turn too tight for the boat at 0.6 m/s | spin assist; if still too tight, widen the teardrop (move W3–W5 east, max x ≈ 27 to keep 3 m from Task 3) |
| Waves disturb tracking | start with the world's small default waves; increase `gain` later as a robustness test |
| Uncalibrated drag → feedforward off | the PI term corrects it; step 1 shows how far off the model is |
| Ground truth is "too perfect" | intended for now; Milestone 4+ swaps in the EKF output via `odom_topic` |

## 9. Out of scope (later milestones)
Foxglove dashboard and shore-link topics (Milestone 2–3), recording datasets
(Milestone 4), obstacle avoidance, detecting the gates with the sensors
instead of using known positions, other tasks.
