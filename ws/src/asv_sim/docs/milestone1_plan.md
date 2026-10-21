# Milestone 1 — Task 1 autonomously in simulation

**How this file is used (added 2026-10-10).** Step N here is step M1.N in
`docs/master_plan.md`, where its status mark lives. The step protocol in
`CLAUDE.md` applies (brief → Marcell's go → build → report → one commit
`M1.N: ...`); each step's brief and report are saved under the step below.
Steps 0 and 1 were done before the protocol existed (commits 9cc9fe7 and
f830b90). Section 10 lists open points in the Step 2 design that the M1.2 brief
must settle with Marcell. Symbols follow `docs/glossary.md` (the guidance
symbols there are "proposed" until Marcell accepts them).

**Goal:** the boat drives RoboBoat Task 1 (Evacuation Route & Return) by itself
in Gazebo: from the start, through both entry gates, a turn, and back out
through the same gates, using its real (simulated) thrusters. No keyboard, no
human input after launch.

**Why this first:** it closes the loop "the boat moves → the sensors produce
data → the data can be recorded". It is also the mandatory qualification task
of the competition. It uses Gazebo's ground-truth position; the filter output
can replace it in the optional step M7.3 (same message type, a different
topic).

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
3. **No collision** with any buoy (minimum distance logged, measured against
   the nominal buoy positions; see section 8).
4. **It stops by itself** after the exit gates (thrust 0, status `FINISHED`).
5. **Repeatable:** 3 consecutive runs succeed.
6. **Observable:** the planned route, the current target point and the status
   are published as topics (visible in RViz, which Marcell chose for live data:
   decision D18 a in the master plan).
7. **Tested and documented:** unit tests pass; `ws/src/asv_sim/docs/AUTONOMY.md`
   explains the method with its equations and the tuned parameters.

## 2. The route

**As built (M1.6):** the current route, equations and parameters are in
`AUTONOMY.md`; sections 2–6 below are the first proposal and its history.

Course geometry (from `worlds/roboboat_course.sdf`): the boat spawns at (0, 0)
facing east; the entry gates are at x = 5 m and x = 13 m, each a red buoy at
y = +1.524 and a green buoy at y = −1.524 (Taylor Made Sur-Mark, radius
0.23 m). Task 2's channel starts at y = 6 (north), Task 3's gate is at x = 30
(south-east), so the turn goes clockwise through the space between them.

![route](img/task1_route.png)

| Waypoint | x [m] | y [m] | Speed towards it [m/s] | Role |
|---|---|---|---|---|
| W0 | 0 | 0 | – | start (spawn position; fixed in the parameter file, also when the boat starts elsewhere: 10.15 a) |
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
 (optional M7.3: /asv/slam/odom)  │                               (to Gazebo via the bridge)
                                 │ uses, pure C++ / no ROS:
                                 │   asv_sim::Follower       state machine (5.7), runs the chain below (M1.3)
                                 │   asv_sim::Route          route progression, cross-track error
                                 │   asv_sim::LosGuidance    desired heading (line of sight)
                                 │   asv_sim::HeadingControl steer + spin from heading error
                                 │   asv_sim::SpeedControl   throttle from speed error
                                 │   asv_sim::mix()          -> four thruster commands (new, Step 2)
                                 │   asv_sim::GateMonitor    detects gate passages
                                 ▼
          /asv/autonomy/path (nav_msgs/Path), /asv/autonomy/target (geometry_msgs/PointStamped),
          /asv/autonomy/status (std_msgs/String), /asv/autonomy/cross_track_error (std_msgs/Float64)
```

**All maths in plain C++ classes without ROS** (unit tested, reusable), and a
**thin node** for ROS I/O. Since M1.3 one class, `Follower`
(`follower.hpp`), holds the whole per-step logic; the node and the
closed-loop test both drive it (S1 a, S2 a). The node runs a fixed 20 Hz control loop on a
ROS-clock timer (`rclcpp::create_timer(this, get_clock(), ...)`, so it runs in
simulation time; step M0.2 comes first) and always uses the latest odometry
message.

## 4. Interfaces (the contract other subsystems can rely on)

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| in | `odom_topic` parameter, default `/asv/ground_truth/odom` | nav_msgs/Odometry | pose in world (ENU), twist in body frame |
| out | `/asv/thrusters/{left,right}/thrust` | std_msgs/Float64 | N |
| out | `/asv/thrusters/{left,right}/pos` | std_msgs/Float64 | pod angle, rad |
| out | `/asv/autonomy/path` | nav_msgs/Path | the planned route (latched: transient local) |
| out | `/asv/autonomy/target` | geometry_msgs/PointStamped | current line-of-sight target point |
| out | `/asv/autonomy/status` | std_msgs/String | `WAITING`, `RUNNING 1/9` … `RUNNING 9/9` (segment number / n_w), `FINISHED`, `STOPPED: <reason>`, gate events |
| out | `/asv/autonomy/cross_track_error` | std_msgs/Float64 | m, for plotting |

## 5. Algorithms

**As built (M1.6):** final forms in `AUTONOMY.md` §5; this section keeps
the proposal and the "implemented form" notes in the order they were decided.

Notation: boat position $\mathbf p = (x, y)$ and heading $\psi$ in the world
frame; forward speed $u$ and yaw rate $\omega$ in the body frame (from the odometry
twist). The heading comes from the orientation quaternion:
$\psi = \operatorname{atan2}\big(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2)\big)$.
Angle differences are always wrapped to $(-\pi, \pi]$.

### 5.1 Route progression (`Route`)
The route is a polyline $W_0 \dots W_{n_w}$ ($n_w = 9$); the boat follows segment
$i$ from $W_i$ to $W_{i+1}$, with direction
$\gamma_i = \operatorname{atan2}(W_{i+1,y} - W_{i,y},\ W_{i+1,x} - W_{i,x})$
and length $\ell_i$. Expressed in the segment's frame:

$$s = (\mathbf p - W_i)\cdot\hat t, \qquad e = (\mathbf p - W_i)\cdot\hat n,
\qquad \hat t = (\cos\gamma_i, \sin\gamma_i),\ \hat n = (-\sin\gamma_i, \cos\gamma_i)$$

$s$ is the along-track progress, $e$ the cross-track error ($e > 0$: boat is
left of the path). **Switching:** go to the next segment when
$s \ge \ell_i - R_\text{switch}$ (the boat has reached or passed the waypoint
*along the path*). This "passed" test, unlike a pure distance circle, can never
make the boat circle a waypoint it missed by a little.
At most one switch per control step; after a switch, s and e are measured
again on the new segment (M1.9). On the last segment the same test ends
the route (FINISHED, decision 10.7 a). The distance left to W_{n_w} is measured
along the route: $d_\text{end} = (\ell_i - s) + \sum_{j>i} \ell_j$.

### 5.2 Line-of-sight guidance (`LosGuidance`)
The standard path-following law for marine vehicles (Fossen, *Handbook of
Marine Craft Hydrodynamics and Motion Control*): aim at a point a lookahead
distance $\Delta$ ahead on the path:

$$\psi_d = \gamma_i + \operatorname{atan2}(-e, \Delta)$$

Far from the path the boat heads almost straight back to it; on the path it
heads along it. $\Delta$ is the main tuning knob: small → tight tracking but
oscillation; large → smooth but corners are cut. Start with $\Delta = 3$ m
(about two boat lengths).

**Implemented form (decision 10.6 a):** the target point $\mathbf p_\text{LOS}$
is found by walking $\Delta$ along the route from the boat's projection
$W_i + s\,\hat t$, carrying what is left past $W_{i+1}$ onto the next
segments (never past $W_{n_w}$), and

$$\psi_d = \operatorname{atan2}\big(\mathbf p_{\text{LOS},y} - y,\ \mathbf p_{\text{LOS},x} - x\big).$$

While $\mathbf p_\text{LOS}$ is on segment $i$ this is exactly the formula
above; near a corner the target stays on the route. $\mathbf p_\text{LOS}$ is
published as `/asv/autonomy/target`.

### 5.3 Heading control (`HeadingControl`)
Heading error $\tilde\psi = \operatorname{wrap}(\psi_d - \psi)$, positive =
the desired heading is to the left. A PD law with the derivative taken on the
measured yaw rate (no kick when the target jumps at a waypoint switch):

$$\text{steer} = \operatorname{clamp}\big(K_\psi\,\tilde\psi - K_\omega\,\omega,\ \pm\text{max\_steer}\big)$$

Steering the pods only turns the boat while there is thrust, so for large
errors (sharp turns, slow speed) **spin assist** adds differential thrust:

$$\text{spin} = \begin{cases}\operatorname{clamp}(K_\text{spin}\,\tilde\psi,\ \pm\text{spin\_max}) & |\tilde\psi| > \psi_\text{spin}\\ 0 & \text{otherwise}\end{cases}$$

**Implemented form (decision 10.3 b):** a ramp that starts at zero at the
threshold, so spin has no jump:

$$\text{spin} = \operatorname{sign}(\tilde\psi)\,\operatorname{clamp}\big(K_\text{spin}(|\tilde\psi| - \psi_\text{spin}),\ 0,\ \text{spin\_max}\big)$$

With K_spin = 1.0 it reaches spin_max = 0.4 at |ψ̃| = 1.2 rad.

**Implemented form after M1.2f (decisions 10.13 a, 10.14 a, 10.16 b):**
the error is taken against the **course** χ (where the boat moves), not the
heading, with a low-speed guard u_min:

$$\chi = \operatorname{wrap}\big(\psi + \operatorname{atan2}(v,\ \max(u, u_\text{min}))\big), \qquad \tilde\psi = \operatorname{wrap}(\psi_d - \chi)$$

The D term stays on ω. Spin gets a proportional part K_s ψ̃ at every error
size (differential thrust turns the boat without the sideways push of the
aft pods), and the ramp starts at ψ_spin = 0.3 rad with K_spin = 2.0:

$$\text{spin} = \operatorname{clamp}\big(K_s\,\tilde\psi + \operatorname{sign}(\tilde\psi)\operatorname{clamp}(K_\text{spin}(|\tilde\psi| - \psi_\text{spin}),\ 0,\ \text{spin\_max}),\ \pm\text{spin\_max}\big)$$

The same ψ̃ feeds cos ψ̃ in u_cmd and the ψ_turn rule (5.4).

Signs follow `mix()`: steer > 0 and spin > 0 both turn the boat
counter-clockwise (left), **as long as the thrust is forward** (section 10.2).

### 5.4 Speed control (`SpeedControl`)
Commanded speed: the waypoint's speed, reduced when the boat points the wrong
way and when it approaches the final waypoint:

$$u_\text{cmd} = \min\big(u_i \cdot \max(0, \cos\tilde\psi),\ K_\text{stop}\, d_\text{end}\big)$$

($d_\text{end}$ is defined in the Step 2 brief; section 10.7.)

Throttle = **model-based feedforward + PI correction**. The simulated drag is
known exactly - it is the boat's own `hydrodynamics` in `config/asv.yaml`
(surge: $X = -(X_u + X_{uu}|u|)\,u$) - so the thrust needed to hold a speed is
known too, and the PI only corrects what the model gets wrong:

$$T_\text{ff}(u) = X_u u + X_{uu} u|u|, \qquad
\text{throttle} = \frac{T_\text{ff}(u_\text{cmd})}{2\,T_\text{max}} + K_P(u_\text{cmd}-u) + K_I\!\int(u_\text{cmd}-u)\,dt$$

clamped to $[-0.3, 1]$, with anti-windup (stop integrating while saturated).

**Implemented form (decision 10.2 c):** while $|\tilde\psi| > \psi_\text{turn}$
(0.3 rad) the lower limit is 0 instead of −0.3, so the pods always push forward
in a turn and the steering sign holds. $u_\text{cmd}$ is also kept ≥ 0 (just
past the end $d_\text{end} < 0$). Anti-windup by conditional integration: the
error is integrated unless the output is beyond a limit and the error pushes
further that way. The time step comes from the odometry stamps.
Example: 1.0 m/s needs $4 + 6 = 10$ N in total, i.e. 5 N per thruster
(throttle 0.1 at 50 N max).

### 5.5 Allocation (`mix()`, new in Step 2)
`asv_sim::mix(throttle, steer, spin)` turns the three values into the four
thruster commands. It does not exist yet (no C++ code has been written so far).
Proposed specification, to be agreed in the Step 2 brief (section 10.1):

- left thrust $= \operatorname{clamp}\big(T_\text{max}(\text{throttle} - \text{spin}),\ T_\text{min},\ T_\text{max}\big)$
- right thrust $= \operatorname{clamp}\big(T_\text{max}(\text{throttle} + \text{spin}),\ T_\text{min},\ T_\text{max}\big)$
- both pod angles $= \operatorname{clamp}(-\text{steer},\ \pm\text{steer\_limit})$

with $T_\text{max} = 50$ N, $T_\text{min} = -40$ N and steer_limit = 1.5708 rad
from `config/asv.yaml`. Gazebo's thruster plugin also clamps to −40…50 N.

### 5.6 Gate monitor (`GateMonitor`)
Gates are given as buoy pairs (red, green). Each control step, the segment from
the previous to the current boat position is intersected with each gate
segment. On a crossing it reports: gate name, direction (outbound/inbound,
from the sign of the cross product), lateral offset from the gate centre, and
**pass/fail**: pass if the boat's side stays at least `gate_margin` (0.5 m)
from the surface of both buoys, i.e.

$$|\text{offset}| \le \tfrac{1}{2}\,\text{gate width} - r_b - \text{boat\_half\_width} - \text{gate\_margin}
= 1.524 - 0.2286 - 0.44 - 0.5 = 0.355\ \text{m}$$

with the buoy radius $r_b$ = 0.2286 m (`models/rb26_surmark_red/model.sdf`).
Also tracks the minimum distance from the boat's side to the surface of every
buoy (collision check, same radius). Results go to the status topic and the
log.

**Implemented form (M1.2d, decisions 10.9 a, 10.10 a, 10.11 b;
`gate_monitor.hpp`).** Gate g has red buoy a and green buoy b, width
$w_g = |b - a|$, unit vector $\hat u = (b - a)/w_g$. Side value
$f(q) = (b - a) \times (q - a)$; a crossing happens when $f \ge 0$ changes
between the previous position $p^-$ and $p$ (half-open, so landing exactly on
the line counts once). Crossing point $q = p^- + t(p - p^-)$,
$t = f(p^-)/(f(p^-) - f(p))$; offset $o = w_g/2 - (q - a)\cdot\hat u$ (> 0
towards red). Crossings with $|o| > w_g/2 + $ `gate_capture` are ignored.
Direction OUTBOUND if $(b - a) \times (p - p^-) > 0$ (Task 1: eastward).
Result: FAIL_OUTSIDE if $|o| > w_g/2$; PASS if
$-(w_g/2 - r_\text{green} - b_w - d_m) \le o \le w_g/2 - r_\text{red} - b_w - d_m$
(each buoy its own radius); FAIL_CLOSE otherwise. Clearance to buoy j with
the boat as a rectangle $L \times 2b_w$ centred on base_link:
$d = R(-\psi)(c_j - p)$, $k$ = $d$ clamped to $[-L/2, L/2] \times [-b_w, b_w]$,
$c_j = |d - k| - r_j$; the smallest per buoy and overall are kept, and a
collision is flagged once $c_j \le 0$. Sampled once per control step.

### 5.7 Safety and states
`WAITING` (no odometry yet, or the start delay - 3 s, lets the boat settle on
the water) → `RUNNING 1/9` … `RUNNING 9/9` → `FINISHED` (all thrust 0, pods
straight; when it fires on the last segment is set in the Step 2 brief,
section 10.7). `STOPPED: <reason>` from any state if the odometry is older
than 0.5 s or the run exceeds `max_run_time`: commands go to zero immediately.
All durations are in simulation time: the odometry age is
`now() − odom.header.stamp` with the node on `use_sim_time`. The node
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
    # route W0..W9; W0 = the spawn, fixed (10.15 a)
    waypoints_x: [0.0, 17.0, 22.0, 25.0, 25.5, 23.0, 20.0, 17.0, 15.0, -1.0]
    waypoints_y: [0.0, 0.0, 2.5, 1.0, -2.0, -4.0, -2.5, -0.5, 0.0, 0.0]
    speeds:      [1.0, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.8, 1.0]
    switch_distance: 1.5        # m, R_switch
    lookahead: 3.0              # m, Delta
    # heading control (starting values - tuned in step 5)
    heading_kp: 1.2             # rad steer per rad error (K_psi)
    heading_kd: 0.6             # rad steer per rad/s yaw rate (K_omega)
    max_steer: 0.8              # rad
    spin_threshold: 0.3         # rad, psi_spin (10.14 a)
    spin_kp: 2.0                # 1/rad, K_spin (ramp, 10.3 b; 10.14 a): spin_max at 0.5 rad
    spin_max: 0.4
    spin_small_kp: 2.0          # 1/rad, K_s, proportional spin (10.16 b) [MEASURED
                                # in the M1.2f model sweep: 1.0-3.0 pass, 2.0 the middle]
    course_min_speed: 0.3       # m/s, u_min, low-speed guard of the course (M1.2f) [EST]
    # speed control
    speed_kp: 0.3               # throttle per m/s error
    speed_ki: 0.05
    stop_gain: 0.5              # 1/s, K_stop
    turn_threshold: 0.3         # rad, psi_turn: throttle >= 0 above it (10.2 c)
    min_throttle: -0.3          # lower throttle limit when not turning
    max_throttle: 1.0
    # buoys (world frame, nominal) [REPO task pose x model pose]; radius per
    # model: Sur-Mark 0.2286, A0 0.1015, A2 0.184 m (models/rb26_*/model.sdf).
    # Gate buoys + nearby buoys (10.11 b); from buoys_gt.yaml after M2.7.
    buoy_names: [gate1_red, gate1_green, gate2_red, gate2_green,
                 t2_ch1_red, t2_ch1_green, t3_red, t3_green]
    buoy_coords: [5.0, 1.524, 5.0, -1.524, 13.0, 1.524, 13.0, -1.524,
                  20.781, 6.0, 23.219, 6.0, 30.0, -4.476, 30.0, -7.524]
    buoy_radii: [0.2286, 0.2286, 0.2286, 0.2286, 0.1015, 0.1015, 0.184, 0.184]
    # gates: name + red and green buoy by name (only the Task 1 gates are checked)
    gate_names: [gate1, gate2]
    gate_red: [gate1_red, gate2_red]
    gate_green: [gate1_green, gate2_green]
    gate_margin: 0.5            # m between the boat's side and a buoy's surface (d_m)
    gate_capture: 2.0           # m beyond a buoy centre still reported, FAIL_OUTSIDE [EST] (10.10)
    # boat_half_width (0.44 m) and boat_length (1.60 m) are computed by the
    # launch file from asv.yaml (10.8 a), not set here.
```

The boat's own values - drag coefficients ($X_u$, $X_{uu}$), `max_thrust`,
`min_thrust` and `steer_limit` - are **not** repeated here: the launch file
reads them from `config/asv.yaml` (single source of truth, as for everything
else) and passes them as parameters.

## 7. Implementation steps (for Claude Code - one at a time)

Each step follows the step protocol in `CLAUDE.md`: brief, Marcell's go,
build, report, one commit `M1.N: ...`. The brief and the report are saved
under the step here.

### Step 0 — Set up and verify the current state  (M1.0, done)
Original plan: `git init` with a `.gitignore`, first build of `asv_sim`, check
build, tests and the headless sim.

**Report (from commit 9cc9fe7, 2026-10-05):** package skeleton
(`package.xml`, `CMakeLists.txt` with no C++ targets) and `launch/sim.launch.py`
(Gazebo with `gui:=false`, the bridge, 9 static TF publishers). `colcon build`
clean; 28 Python tests pass; there was no C++ code, so there were no C++ tests;
the headless sim publishes `/asv/ground_truth/odom` at about 46 Hz (Gazebo is
set to 50 Hz) and all 9 static TF frames.

### Step 1 — Actuator check (measure before controlling)  (M1.1, done)
A small tool (`tools/actuator_check.py`) that commands fixed values with
the boat on the water and records the response from the odometry:
1. total thrust 10, 20, 40 N (half per thruster), pods straight → steady
   forward speed; compare with the drag model's prediction
   ($X_u u + X_{uu}u^2 = T$);
2. 20 N total with pods at −0.4 rad → yaw rate must be **positive** (turns
   left): **confirms the steering sign** (see CLAUDE.md);
3. spin: left −10 N, right +10 N → yaw rate positive.

**Report (from commit f830b90 and CLAUDE.md):** 10 N → 1.00 m/s, 20 N →
1.52 m/s, 40 N → 2.27 m/s, all within 0.1 % of the drag model; pods at
−0.4 rad → +0.15 rad/s (left turn); left −10 N / right +10 N → +0.18 rad/s
(left turn). The tool times its phases with the wall clock; step M0.2 decides
whether it is switched to simulation time.

### Step 2 — Guidance and control logic (pure C++, no ROS)  (M1.2)
First C++ code in `asv_sim`: a library target and gtest in `CMakeLists.txt`,
⚠ new `<test_depend>ament_cmake_gtest</test_depend>` in `package.xml` (ask
first). `include/asv_sim/mix.hpp`: `mix()` (section 5.5).
`include/asv_sim/guidance.hpp` (+ `.cpp` if it gets long): `Route`,
`LosGuidance`, `HeadingControl`, `SpeedControl`, `GateMonitor`, angle
wrapping, yaw from quaternion. The brief settles the open points of section 10.
Unit tests in `test/test_mix.cpp` and `test/test_guidance.cpp`:
- `mix()`: steer > 0 gives pod angles < 0; spin > 0 gives left < right
  thrust; clamping to −40…50 N and ±1.5708 rad;
- cross-track error sign and segment switching (incl. a missed waypoint);
- LOS heading on, left of and right of the path;
- angle wrapping across ±π (the turn passes through west = ±180°!);
- gate crossing: pass, fail (too close to a buoy: the 0.355 m band of 5.6),
  direction;
- the rules the M1.2 brief chooses for section 10 (steering with reverse
  thrust, spin assist);
- **closed-loop logic test without Gazebo:** a simple kinematic boat model
  (speed with a first-order lag; turning either "yaw rate ∝ steer·u + spin" as
  first planned, or calibrated from Step 1 as proposed in 10.5, as the brief
  decides) follows the Task 1 route and passes both gates both ways. This
  catches logic errors in seconds; real dynamics are tuned in Gazebo (step 5).
- **Check:** all tests pass (`colcon test` now runs the gtests; the 28 Python
  tests still pass).

**Split (agreed 2026-10-18):** M1.2a `mix()` + C++/gtest setup; M1.2b angle
helpers, `Route`, `LosGuidance` (10.6, 10.7); M1.2c `HeadingControl`,
`SpeedControl` (10.2, 10.3); M1.2d `GateMonitor` (10.8); M1.2e closed-loop
logic test (10.4, 10.5). One commit each.

#### M1.2a Brief (agreed 2026-10-18)
- **Goal:** set up C++ and gtest in `asv_sim` and write `mix()`: (throttle,
  steer, spin) → left/right thrust [N] and pod angle [rad], clamped to the
  boat's limits.
- **Decisions:** 10.1 fractions of `max_thrust`, saturation b ("turning
  wins"); 10.2 c (not in `mix()`).
- **Why:** follower (M1.3), maneuver player (M1.7) and teleop (M1.8) all drive
  the boat through `mix()`. Experimental Scenarios; App. C.
- **New ideas:** thrust allocation; saturation (glossary).
- **Files:** ⚠ `package.xml` (`test_depend` `ament_cmake_gtest`),
  `CMakeLists.txt` (C++17, library `asv_sim_guidance`, gtest),
  `include/asv_sim/mix.hpp`, `src/mix.cpp`, `test/test_mix.cpp`; this plan,
  `docs/master_plan.md`, `docs/glossary.md`, `CLAUDE.md` (Known state, layout).
- **How it works:** limits T_max = 50 N, T_min = −40 N, steer_limit =
  1.5708 rad [REPO `asv.yaml`] come in a `ThrusterLimits` struct.
  Wanted T_L* = T_max(throttle − spin), T_R* = T_max(throttle + spin). Larger
  side above T_max → shift both down; smaller side below T_min → shift both up;
  difference above T_max − T_min → clip both. Then clamp each side. Pods:
  δ = clamp(−steer, ±steer_limit). Yaw moment (10.2):
  N = −0.70 (T_L + T_R) sin δ + 0.33 (T_R − T_L) cos δ. Example throttle 1.0,
  spin 0.4: wanted (30, 70) N → (10, 50) N, 40 N difference kept. The pod rate
  limit (2 rad/s) stays in Gazebo's plugin.
- **Check:** gtest `test_mix` (zero in/out, straight thrust, steer and spin
  signs with N > 0, clamps, saturation (10, 50) and (−40, 50)); Python 28 OK.
- **Can't check:** nothing GUI; `mix()` first drives the Gazebo boat in M1.3.

#### M1.2a Report (2026-10-18)
- **What changed**
  - `include/asv_sim/mix.hpp`: `ThrusterLimits`, `ThrusterCommand`, `mix()`
    declaration with the sign and unit conventions.
  - `src/mix.cpp`: `mix()`: wanted thrusts, shift to keep the difference,
    clamp, pod angle = clamp(−steer).
  - `test/test_mix.cpp`: 9 gtests.
  - `CMakeLists.txt`: C++17, `-Wall -Wextra -Wpedantic`, library
    `asv_sim_guidance`, install of headers and library, gtest `test_mix`.
  - `package.xml`: `<test_depend>ament_cmake_gtest</test_depend>`.
  - `docs/master_plan.md` (rows M1.2a–e), `docs/glossary.md` (saturation,
    thrust allocation), `CLAUDE.md` (Known state, test baseline, layout).
- **How it was checked**
  - `colcon build --symlink-install --packages-up-to asv_sim`:
    `Finished <<< asv_sim`; clean rebuild with `console_direct+`: 0 warnings.
  - `colcon test --packages-select asv_sim && colcon test-result --verbose`:
    `test_mix.gtest.xml: 9 tests, 0 errors, 0 failures`; summary
    `10 tests, 0 errors, 0 failures, 0 skipped` (the 10th is CTest's wrapper).
  - `python3 -m unittest discover -s tests`: `Ran 28 tests ... OK`.
- **Not checked:** `mix()` has not driven the Gazebo boat yet (M1.3). The
  test's yaw-moment helper is the steady formula of 10.2, not Gazebo.
- **How to read the code:** start at `mix.hpp` (conventions), then
  `mix.cpp` top to bottom: wanted T_L*, T_R* (§5.5) → shift (10.1 b) →
  clamp → δ = −steer. Tests in the same order as the brief.
- **Numbers:** T_max 50 N, T_min −40 N, steer_limit 1.5708 rad [REPO
  `asv.yaml`]; throttle 1.0 + spin 0.4 → (10, 50) N; spin ±1.0 →
  (−40, 50) / (50, −40) N [MEASURED gtest].
- **Check your understanding**
  1. Why does the boat turn less at full throttle with option a than with b?
  2. Throttle −1.0, spin 0.2: which side is shifted, and in which direction?
  3. Why does `mix()` never flip the pod angle when the thrust is negative?

#### M1.2b Brief (agreed 2026-10-18)
- **Goal:** angle wrapping, yaw from quaternion, `Route` (s, e, switching,
  remaining distance, finished) and `LosGuidance` (target point, ψ_d), ROS-free.
- **Decisions:** 10.6 a (lookahead carried onto the next segments);
  10.7 a (d_end along the route; FINISHED by the switch rule on the last
  segment).
- **Why:** the guidance half of the follower (M1.3); `SpeedControl` (M1.2c)
  needs d_end. Experimental Scenarios; App. C.
- **New ideas:** along-/cross-track (s, e); line-of-sight guidance; angle
  wrapping (glossary).
- **Files:** `include/asv_sim/angles.hpp`, `src/angles.cpp`,
  `include/asv_sim/guidance.hpp`, `src/guidance.cpp`, `test/test_angles.cpp`,
  `test/test_guidance.cpp`, `CMakeLists.txt`; this plan, `docs/master_plan.md`,
  `docs/glossary.md`, `CLAUDE.md`. No new dependency.
- **How it works:** wrap(a) = remainder(a, 2π), −π → +π, range (−π, π].
  ψ = atan2(2(q_w q_z + q_x q_y), 1 − 2(q_y² + q_z²)). `Route::update(p)`:
  s = (p − W_i)·t̂_i, e = (p − W_i)·n̂_i; s ≥ ℓ_i − R_switch → next segment (at
  most one switch per update); on the last segment the same test sets
  finished. d_end = (ℓ_i − s) + Σ_{j>i} ℓ_j. `LosGuidance`: walk Δ along the
  route from the projection W_i + s t̂_i (stop at W_{n_w}) → p_LOS;
  ψ_d = wrap(atan2(p_LOS − p)); equals γ_i + atan2(−e, Δ) on a straight
  segment. Bad input throws `std::invalid_argument`.
- **Check:** gtests `test_angles`, `test_guidance` (numbers on the Task 1
  route: e = ±0.5 at (5, ±0.5); switch at x = 15.5 not 15.4; missed waypoint
  (16, 3) switches; one switch per update; d_end = 57.21 m at the start;
  finished at x ≤ 0.5; ψ_d = ∓0.3218 rad at (5, ±1); carry-over target
  (17.894, 0.447) at (15, 0); ψ_d = ±2.976 across ±π on segment 8); `test_mix`
  still 9; Python 28 OK.
- **Can't check:** nothing GUI; whether Δ = 3 m and R_switch = 1.5 m suit the
  Gazebo boat is seen in M1.3/M1.5.

#### M1.2b Report (2026-10-18)
- **What changed**
  - `include/asv_sim/angles.hpp`, `src/angles.cpp`: `wrap_angle()` to
    (−π, π], `yaw_from_quaternion()`.
  - `include/asv_sim/guidance.hpp`, `src/guidance.cpp`: `Point2`; `Route`
    (s, e, switching, `remaining_distance()` = d_end, `finished()`);
    `LosGuidance` (p_LOS with the lookahead carried along the route, ψ_d).
  - `test/test_angles.cpp` (6 gtests), `test/test_guidance.cpp` (14 gtests).
  - `CMakeLists.txt`: the two sources in `asv_sim_guidance`, two gtests.
  - This plan (§5.1, §5.2 implemented form; 10.6, 10.7 decided),
    `docs/master_plan.md`, `docs/glossary.md` (R_switch, p_LOS, d_end; LOS
    entry), `CLAUDE.md` (Known state, test baseline).
- **How it was checked**
  - Clean rebuild (`--cmake-target clean`, then `colcon build
    --packages-select asv_sim --event-handlers console_direct+`): 0 lines
    matching "warning".
  - `colcon test --packages-select asv_sim && colcon test-result --verbose`:
    `test_angles` 6, `test_guidance` 14, `test_mix` 9 tests, 0 failures;
    summary `32 tests, 0 errors, 0 failures, 0 skipped` (29 gtests + 3 CTest
    wrappers).
  - `python3 -m unittest discover -s tests`: `Ran 28 tests ... OK`.
- **Not checked:** no Gazebo in this step; Δ = 3 m and R_switch = 1.5 m meet
  the real boat dynamics first in M1.3, tuning in M1.5.
- **How to read the code:** `angles.hpp` (two functions) → `guidance.hpp`
  (the contract of `Route` and `LosGuidance`) → `guidance.cpp`:
  `Route::update()` = §5.1 (s, e and the switch test), `remaining_distance()`
  = d_end (10.7 a), `LosGuidance::compute()` = §5.2 implemented form (the
  walk along the route, 10.6 a). Tests follow the brief's order.
- **Numbers:** route length 57.21 m from W0 = (0, 0) [MEASURED gtest, route of
  §2 REPO]; γ_1 = 0.4636 rad; ψ_d = ∓0.3218 rad at 1 m left/right of segment 0
  with Δ = 3 m; carry-over target (17.894, 0.447) m at (15, 0); FINISHED at
  x ≤ 0.5 m with R_switch = 1.5 m [EST starting values, `task1_route.yaml`
  plan §6].
- **Check your understanding**
  1. At p = (15, 0), why does the old target (18, 0) make the boat cut the
     corner at W1, and the new one not?
  2. Why can the switch test fire for a missed waypoint while a 1.5 m circle
     around W1 would not?
  3. Why is wrap(−π) mapped to +π, and where would two different values for
     the same direction cause trouble?

#### M1.2c Brief (agreed 2026-10-18)
- **Goal:** ROS-free `HeadingControl` (ψ̃, ω → steer, spin) and
  `SpeedControl` (u_ref, ψ̃, d_end, u, dt → throttle) with gtests.
- **Decisions:** 10.2 c (throttle ≥ 0 while |ψ̃| > ψ_turn, ψ_turn = 0.3 rad
  [EST]); 10.3 b (spin ramps from ψ_spin, K_spin = 1.0 [EST], spin_max 0.4
  reached at 1.2 rad).
- **Why:** the control half of the follower (M1.3). Experimental Scenarios;
  App. C.
- **New ideas:** PD on the measured yaw rate; feedforward + PI; anti-windup
  (glossary).
- **Files:** `include/asv_sim/control.hpp`, `src/control.cpp`,
  `test/test_control.cpp`, `CMakeLists.txt`; this plan (§5.3, §5.4, §6, §10),
  `docs/master_plan.md`, `docs/glossary.md`, `CLAUDE.md`. No new dependency.
- **How it works:** ψ̃ = wrap(ψ_d − ψ); steer = clamp(K_ψ ψ̃ − K_ω ω,
  ±max_steer); spin = sign(ψ̃)·clamp(K_spin(|ψ̃| − ψ_spin), 0, spin_max).
  u_cmd = min(u_ref·max(0, cos ψ̃), K_stop d_end); T_ff = X_u u_cmd +
  X_uu u_cmd|u_cmd|; throttle = clamp(T_ff/(2 T_max) + K_P(u_cmd − u) + K_I I,
  lower, 1) with lower = 0 if |ψ̃| > ψ_turn else −0.3; I integrates
  (u_cmd − u)·dt only when that does not push further into a limit; dt ≤ 0 →
  no integration; dt from stamps (caller). Bad parameters throw.
- **Check:** gtest `test_control` (steer sign, clamp, ω term −0.3 at ω = 0.5;
  wrapping across ±π; spin 0 below, continuous at, saturated above the
  threshold, odd; u_cmd cos factor and d_end cap; throttle 0.100 at 1.0 m/s;
  throttle 0 at ψ̃ = 1.0, u = 0.6; negative allowed when straight; anti-windup;
  I = 0.2 after 1 s of 0.2 m/s error; invalid parameters throw); other gtests
  unchanged; Python 28 OK.
- **Can't check:** nothing GUI; the gains meet a boat model in M1.2e and Gazebo
  in M1.3/M1.5. Weak spot to watch: for ψ_turn < |ψ̃| < ψ_spin the throttle can
  be 0 and spin not yet on, so nothing turns the boat.

#### M1.2c Report (2026-10-18)
- **What changed**
  - `include/asv_sim/control.hpp`, `src/control.cpp`: `HeadingControl`
    (ψ̃, steer, spin ramp) and `SpeedControl` (u_cmd, feedforward, PI with
    anti-windup, throttle ≥ 0 while turning).
  - `test/test_control.cpp` (18 gtests); `CMakeLists.txt`: the source in
    `asv_sim_guidance`, gtest `test_control`.
  - This plan: §5.3 and §5.4 implemented form, §6 (`spin_kp` 1.0, new
    `turn_threshold`, `min_throttle`, `max_throttle`), 10.3 decided;
    `docs/master_plan.md`, `docs/glossary.md` (ψ_spin, K_spin, ψ_turn, u_cmd,
    u_ref, K_stop, T_ff, K_P, K_I, I), `CLAUDE.md` (Known state, baseline).
- **How it was checked**
  - Clean rebuild (`--cmake-target clean`, then `colcon build --symlink-install
    --packages-up-to asv_sim --event-handlers console_direct+`): 0 lines
    matching "warning".
  - `colcon test --packages-select asv_sim && colcon test-result --verbose`:
    `test_mix` 9, `test_angles` 6, `test_guidance` 14, `test_control` 18,
    0 failures; summary `51 tests, 0 errors, 0 failures, 0 skipped` (47 gtests
    + 4 CTest wrappers).
  - `python3 -m unittest discover -s tests`: `Ran 28 tests ... OK`.
- **Not checked:** no Gazebo and no boat model in this step; whether the gains
  turn the boat through the teardrop is seen in M1.2e and M1.3/M1.5. The weak
  spot from the brief (throttle 0 and spin still off for
  ψ_turn < |ψ̃| < ψ_spin) is still open.
- **Small additions beyond the brief:** u_cmd is kept ≥ 0 (d_end < 0 just past
  the end would otherwise ask for a reverse speed); the throttle limits −0.3
  and 1 are parameters (`min_throttle`, `max_throttle`), not constants in code.
- **How to read the code:** `control.hpp` (the two contracts and their
  equations) → `control.cpp`: `HeadingControl::compute()` = §5.3 (steer) and
  10.3 b (spin); `SpeedControl::commanded_speed()` = u_cmd of §5.4,
  `feedforward_throttle()` = T_ff/(2 T_max), `update()` = PI + 10.2 c lower
  limit + anti-windup. Tests follow the same order.
- **Numbers:** throttle 0.100 at 1.0 m/s (10 N total) [MEASURED gtest, drag
  X_u = 4, X_uu = 6 REPO `asv.yaml`]; at ψ̃ = 1.0 rad, 0.6 m/s the raw throttle
  −0.0635 is held at 0 [MEASURED gtest]; spin 0.2 at 1.0 rad, 0.4 from 1.2 rad
  with K_spin = 1.0, ψ_spin = 0.8 rad, ψ_turn = 0.3 rad [EST starting values,
  plan §6].
- **Check your understanding**
  1. Why does the D term use the measured yaw rate ω and not the change of ψ̃?
     What happens at a waypoint switch otherwise?
  2. With ψ̃ = 1.0 rad the throttle is 0. What, if anything, still turns the
     boat, and how does that change at ψ̃ = 0.9 rad?
  3. In the anti-windup test, what would the throttle do after the error
     reverses if the integral had grown for the full 100 s?

#### M1.2d Brief (agreed 2026-10-19)
- **Goal:** ROS-free `GateMonitor`: gate crossings (gate, direction, offset,
  pass/fail) and the smallest clearance between the boat and every buoy.
- **Decisions:** 10.8 a, 10.9 a, 10.10 a, 10.11 b (all 2026-10-19).
- **Why:** judges the runs (success criteria 2 and 3, section 1); M1.2e and
  M1.3 use it. Experimental Scenarios; App. C.
- **New ideas:** gate (course), gate offset o, clearance c_j (glossary).
- **Files:** `include/asv_sim/gate_monitor.hpp`, `src/gate_monitor.cpp`,
  `test/test_gate_monitor.cpp`, `CMakeLists.txt`; this plan (5.6, 6, 10),
  `docs/master_plan.md`, `docs/glossary.md`, `CLAUDE.md`. No new dependency.
- **How it works:** `Buoy {name, c, r}`, `Gate {name, red, green}`,
  `BoatFootprint {L = 1.60, b_w = 0.44}`. `update(p, ψ)`: with a = red,
  b = green, f(q) = (b − a) × (q − a); a crossing happens when `f ≥ 0` changes
  between p⁻ and p (half-open); q = p⁻ + t(p − p⁻), t = f(p⁻)/(f(p⁻) − f(p));
  o = w_g/2 − (q − a)·û (> 0 towards red), ignored if |o| > w_g/2 + capture
  (2 m [EST]); OUTBOUND if (b − a) × (p − p⁻) > 0. PASS if
  −(w_g/2 − r_green − b_w − d_m) ≤ o ≤ w_g/2 − r_red − b_w − d_m (Task 1:
  ±0.3554 m); FAIL_OUTSIDE if |o| > w_g/2; FAIL_CLOSE otherwise. Clearance:
  d = R(−ψ)(c_j − p), k = clamp of d to the rectangle (±L/2, ±b_w),
  c_j = |d − k| − r_j; smallest per buoy and overall; collision once c_j ≤ 0.
  The first call only stores p.
- **Check:** gtest `test_gate_monitor`: one PASS outbound / inbound at y = 0;
  band edge 0.355 PASS, 0.356 FAIL_CLOSE (both sides); y = 2 FAIL_OUTSIDE,
  y = 4 no event; landing on the line counts once, three wiggles three
  events; clearance 0.8554 m heading east, 0.4954 m heading north at (5, 0);
  collision at (5, 1.0) (c = −0.1446); uneven band with mixed radii; Task 2/3
  buoys tracked; invalid arguments throw. Other gtests unchanged; Python 28 OK.
- **Can't check:** nothing GUI; buoy drift (nominal positions until M2.7);
  real passes in Gazebo come in M1.3/M1.5.

#### M1.2d Report (2026-10-19)
- **What changed**
  - `include/asv_sim/gate_monitor.hpp`, `src/gate_monitor.cpp`: `Buoy`,
    `Gate`, `BoatFootprint`, `GateEvent`; `GateMonitor` (crossings with
    direction, offset and PASS / FAIL_CLOSE / FAIL_OUTSIDE; clearance of the
    turned rectangle to every buoy, minimum per buoy, collision flag).
  - `test/test_gate_monitor.cpp` (15 gtests); `CMakeLists.txt`: the source in
    `asv_sim_guidance`, gtest `test_gate_monitor`.
  - This plan: §5.6 implemented form, §6 (per-buoy list with radii, gates by
    buoy name, `gate_capture`; boat size from `asv.yaml`), 10.8–10.11
    decided; `docs/master_plan.md`, `docs/glossary.md` (w_g, o, c_j, L, b_w,
    d_m; "Gate (course)", "Clearance"), `CLAUDE.md` (Known state, baseline,
    layout).
- **How it was checked**
  - Clean rebuild (`--cmake-target clean`, then `colcon build --symlink-install
    --packages-up-to asv_sim --event-handlers console_direct+`): 0 lines
    matching "warning".
  - `colcon test --packages-select asv_sim && colcon test-result --verbose`:
    `test_mix` 9, `test_angles` 6, `test_guidance` 14, `test_control` 18,
    `test_gate_monitor` 15, 0 failures; summary `67 tests, 0 errors,
    0 failures, 0 skipped` (62 gtests + 5 CTest wrappers).
  - `python3 -m unittest discover -s tests`: `Ran 28 tests ... OK`.
- **Not checked:** no Gazebo in this step; the monitor uses nominal buoy
  positions (drift is measured in M2.7); real passes are seen in M1.3/M1.5.
  Clearance is sampled once per control step, so a contact shorter than one
  step could be missed [EST: 5 cm of travel at 1 m/s, 20 Hz].
- **How to read the code:** `gate_monitor.hpp` (data types and the contract)
  → `gate_monitor.cpp`: `side_value()` = f(q); `update()` = crossing test,
  t, q, offset o, direction and the §5.6 band; `clearance()` = c_j
  (R(−ψ), clamp to the rectangle). Tests follow the brief's order.
- **Numbers:** gate band ±0.3554 m = 1.524 − 0.2286 − 0.44 − 0.5 [REPO
  course and `asv.yaml`; d_m = 0.5 m EST]; clearance to a gate buoy on the
  centre line 0.8554 m beside the boat, 0.4954 m off the bow [MEASURED gtest];
  capture 2 m [EST]; buoy radii 0.2286 / 0.1015 / 0.184 m [REPO
  `models/rb26_*/model.sdf`].
- **Check your understanding**
  1. Why does a step that ends exactly on the gate line, followed by a step
     off it, give one event and not two? Which comparison makes that so?
  2. The boat crosses gate 1 at o = 0 but with ψ = 60°. The band says PASS.
     What does the clearance check say, and why can the two disagree?
  3. Why is the band uneven for a gate with an A2 red and a Sur-Mark green buoy?

#### M1.2e Brief (agreed 2026-10-19)
- **Goal:** a gtest where a 3-DOF model boat, driven by the full M1 control
  chain at 20 Hz, follows the Task 1 route and passes both gates outbound and
  inbound.
- **Decisions:** 10.4 (stop and report on failure), 10.5 (3-DOF model);
  runs: nominal + 4 start offsets (±0.5 m sideways, ±0.3 rad heading); boat
  numbers read from `asv.yaml` + `asv_report.json` (yaml-cpp); the generator
  writes the whole-boat inertia (all 2026-10-19).
- **Why:** catches wiring, sign and logic errors before M1.3 and Gazebo.
  Experimental Scenarios; App. C.
- **New ideas:** 3-DOF manoeuvring model (no added mass: 0 in `model.sdf`);
  zero-order hold; explicit Euler in 4 ms sub-steps (glossary).
- **Files:** ⚠ `tools/asvgen/analysis.py` (`inertia_cog` in the report),
  ⚠ `tests/test_asvgen.py` (+1), ⚠ `docs/HOW_IT_WORKS.md`, ⚠ `package.xml`
  (`<test_depend>yaml_cpp_vendor</test_depend>`), regenerated
  `config/asv_report.*`; `test/boat_model.hpp`, `test/test_closed_loop.cpp`,
  `CMakeLists.txt`; this plan, `docs/master_plan.md`, `docs/glossary.md`,
  `CLAUDE.md`.
- **How it works:** every 50 ms Route → LOS → HeadingControl →
  SpeedControl (dt 0.05) → mix() → model, 4 ms sub-steps with the commands
  held → GateMonitor; end at FINISHED (+2 s, thrust 0) or 180 s. Pod angle
  rate ≤ 2 rad/s. T = T_L + T_R; X = T cos δ, Y = T sin δ,
  N = x_p T sin δ + 0.33 (T_R − T_L) cos δ; m(u̇ − vω) = X − (X_u + X_uu|u|)u;
  m(v̇ + uω) = Y − (Y_v + Y_vv|v|)v; I_z ω̇ = N − (N_r + N_rr|ω|)ω;
  ẋ = u cos ψ − v sin ψ, ẏ = u sin ψ + v cos ψ, ψ̇ = ω. About base_link (CoG
  1.8 cm aft, ignored [EST]).
- **Check:** model vs Step 1 within 10 % (1.00 m/s at 10 N; +0.15 rad/s for
  pods −0.4 rad at 20 N; +0.18 rad/s for ±10 N); 5 runs: gate1 OUT, gate2 OUT,
  gate2 IN, gate1 IN, all PASS, no collision, FINISHED < 180 s; report min
  clearance, max |e|, run time, max |v|. Other gtests unchanged; Python 29.
- **Can't check:** Gazebo effects (waves, buoyancy, roll/pitch, propeller
  spin-up); M1.3/M1.5 do that.
- **Committed together with M1.2f** (decided 2026-10-19, below).

#### M1.2f Brief (agreed 2026-10-19)
- **Goal:** heading control on the course χ instead of the heading ψ
  (10.13 a); spin assist from ψ_spin = 0.3 rad with K_spin = 2.0 (10.14 a);
  W0 fixed at the spawn (0, 0) in the parameter file (10.15 a).
- **Decisions (2026-10-19):** S1 low-speed guard: a ramp,
  χ = ψ + atan2(v, max(u, u_min)), u_min = 0.3 m/s [EST]; S2: a free
  function `course_angle()`, `HeadingControl` unchanged, the caller passes χ;
  S3: one combined commit for M1.2e and M1.2f (not the recommended option; the
  commit has both `M1.2e:` and `M1.2f:` lines so both greps find it).
- **Why:** M1.2e fails without these (crabbing on straights, weak corners,
  route shifted by off-spawn starts). Experimental Scenarios; App. C.
- **New ideas:** course χ; course-based LOS (Fossen) (glossary). The
  sideslip angle gets no symbol (β is already the buoy bearing).
- **Files:** `include/asv_sim/control.hpp`, `src/control.cpp`,
  `test/test_control.cpp`, `include/asv_sim/guidance.hpp` (W0 comment),
  `test/test_closed_loop.cpp`; this plan (§2, §5.3, §6, Step 3),
  `docs/master_plan.md`, `docs/glossary.md`, `CLAUDE.md`.
- **How it works:** χ = ψ + atan2(v, max(u, u_min)); ψ̃ = wrap(ψ_d − χ);
  steer = clamp(K_ψ ψ̃ − K_ω ω, ±max_steer) (D term stays on ω); the same ψ̃
  feeds spin, cos ψ̃ in u_cmd and the ψ_turn rule. v = odom twist.linear.y.
- **Check:** new `course_angle` gtests; the other gtests unchanged; M1.2e's
  5 closed-loop runs all PASS; on failure stop and report (10.4).
- **Can't check:** Gazebo behaviour (M1.3/M1.5).
- **Extended 2026-10-19 (10.16 b, K_s = 2.0):** proportional spin
  K_s ψ̃ in `HeadingControl` (new field `spin_small_kp`), because the
  offset runs still failed; tried first in a scratch copy (10.16).

#### M1.2e Report (2026-10-19)
- **What changed:**
  - `test/boat_model.hpp`: the 3-DOF model (surge, sway, yaw), pod rate
    limit, zero-order hold, explicit Euler; `load_boat_params()` reads
    `asv.yaml` and `asv_report.json` with yaml-cpp.
  - `test/test_closed_loop.cpp`: 5 model tests against the equations and
    Step 1; the Task 1 closed loop from the spawn and from 4 start offsets.
  - `CMakeLists.txt`, `package.xml`: the gtest and its
    `<test_depend>yaml_cpp_vendor</test_depend>`.
  - `tools/asvgen/analysis.py`: `mass.inertia_cog` (whole boat incl. pods)
    in the report; `tests/test_asvgen.py`: +1 test; `HOW_IT_WORKS.md`.
  - `config/asv.yaml`: yVV 4 → 58 (10.12); regenerated `model.sdf`,
    `asv_report.*`, `asv_bridge.yaml`, `asv_frames.yaml` (fingerprint only).
- **How it was checked:** `colcon test --packages-select asv_sim` →
  `colcon test-result`: 84 tests, 0 errors, 0 failures (78 gtests + 6 CTest
  wrappers); `test_closed_loop` 7/7. `python3 -m unittest discover -s
  tests`: 29 tests OK. Gazebo vs model after yVV = 58 (20 N, pods −0.4 rad):
  u = 1.396, v = −0.414, ω = 0.1499 [MEASURED Gazebo] vs 1.397, −0.413,
  0.1484 [MEASURED model]. Closed loop (with M1.2f): every run gate1 OUT,
  gate2 OUT, gate2 IN, gate1 IN, all PASS, no collision:

  | Run | finish | max \|e\| | max \|v\| | min clearance | gate offsets (m) |
  |---|---|---|---|---|---|
  | spawn | 64.55 s | 1.646 m | 0.253 m/s | 0.684 m | 0.000, 0.000, −0.155, +0.024 |
  | y +0.5 | 64.65 s | 1.640 m | 0.258 m/s | 0.683 m | +0.052, +0.074, −0.155, +0.023 |
  | y −0.5 | 64.50 s | 1.641 m | 0.252 m/s | 0.683 m | −0.052, −0.074, −0.155, +0.023 |
  | ψ +0.3 | 64.75 s | 1.648 m | 0.347 m/s | 0.683 m | −0.050, +0.085, −0.155, +0.023 |
  | ψ −0.3 | 64.60 s | 1.640 m | 0.347 m/s | 0.683 m | +0.050, −0.085, −0.154, +0.023 |

  Model vs Step 1: 10 N → u within 1e-4 of the drag root (1.00 m/s, Step 1
  1.00); pods −0.4 rad at 20 N → ω = 0.1484 rad/s (Step 1 0.15);
  ±10 N → ω within 1e-4 of the yaw balance 0.1755 rad/s (Step 1 0.18).
- **Differs from the brief:** sub-steps are 2.5 ms (20 per control step),
  not 4 ms; the brief's first run failed and led to 10.12–10.16.
- **What could not be checked:** waves, buoyancy, roll/pitch, propeller
  spin-up and reverse-thrust behaviour in Gazebo (M1.3/M1.5). The max |e| of
  1.65 m is in the teardrop turn, not near a gate.
- **How to read the code:** `test/boat_model.hpp` `BoatModel::step()` has
  the equations of the brief in its comment; `test_closed_loop.cpp`
  `run_task1()` is the 20 Hz loop (Route → LOS → course → HeadingControl,
  SpeedControl → mix() → model → GateMonitor).
- **Numbers for the thesis:** m = 18.191 kg, I_z = 3.68555 kg m² (whole
  boat; base_link alone 2.80801) [REPO `asv_report.json`]; yVV = 58
  N/(m/s)² [EST, C_d 0.8]; the Gazebo/model check above [MEASURED]; the
  closed-loop table [MEASURED, model].
- **Check your understanding:**
  1. Why does the model leave out added mass, and what would change in
     Gazebo if `model.sdf` had it?
  2. Why can this model check the controller's logic but not be used inside
     the filter?
  3. Why did the boat slide outward in every turn with yVV = 4?

#### M1.2f Report (2026-10-19)
- **What changed:**
  - `include/asv_sim/control.hpp`, `src/control.cpp`: `course_angle()`
    (χ with the low-speed guard); `HeadingGains::spin_small_kp` (K_s) and
    the spin law with the proportional part.
  - `test/test_control.cpp`: +6 `CourseAngle` tests, +3 proportional-spin
    tests, +1 invalid-K_s check; the M1.2c spin tests keep K_s = 0.
  - `include/asv_sim/guidance.hpp`: W0 comment (10.15 a).
  - `test/test_closed_loop.cpp`: W0 = (0, 0), χ fed to `HeadingControl`,
    ψ_spin 0.3, K_spin 2.0, K_s 2.0, u_min 0.3.
  - This plan: §2, §5.3 (implemented form), §6, Step 3, 10.16;
    `docs/glossary.md`: χ, u_min, K_s, course, non-minimum-phase, 3-DOF
    model, zero-order hold, explicit Euler.
- **How it was checked:** `test_control` 27/27; the M1.2e table above.
  The trials (χ̇ damping, K_s sweep) ran in a scratch copy in
  `ws/data/scratch/` (not committed); results in 10.16.
- **What could not be checked:** reverse thrust: one thruster is in reverse
  in about 63 % of the control steps, worst −23 N [MEASURED, model]; Gazebo
  may give less reverse thrust, and reverse thrust flips the pod steering
  (10.2). Check in M1.3/M1.5. The dip at K_s = 0.5 (2 fails, while 0.25
  passes) is not explained.
- **How to read the code:** `course_angle()` = the χ equation of §5.3;
  `HeadingControl::compute()` = the steer and spin equations of §5.3
  (K_s = `spin_small_kp`, K_spin = `spin_kp`, ψ_spin = `spin_threshold`).
- **Numbers for the thesis:** ψ_spin = 0.3 rad, K_spin = 2.0 /rad
  [EST, 10.14 a]; K_s = 2.0 /rad [MEASURED model sweep, 10.16]; u_min =
  0.3 m/s [EST]; the sweep table in 10.16 [MEASURED, model].
- **Check your understanding:**
  1. Why does steering on χ instead of ψ remove crabbing on a straight leg?
  2. Why does differential thrust damp the swing that the pods cause?
  3. What does the max(u, u_min) do when the boat drifts sideways while
     almost stopped?

### Step 3 — The node (`src/waypoint_follower.cpp`)  (M1.3)
Parameters (section 6), odometry subscription (u = twist.linear.x and
v = twist.linear.y, body frame, for the course χ: 5.3), 20 Hz ROS-clock timer, the
state machine (5.7), publishing commands + the four autonomy topics (`path`
with transient-local QoS so late viewers still get it). ⚠ New `<depend>`
entries `rclcpp`, `std_msgs`, `nav_msgs`, `geometry_msgs` (ask first). Needs
M0.2 (simulation time) done first.
- **Check:** builds without warnings; with the sim running headless and the
  node started by hand (`ros2 run asv_sim waypoint_follower --ros-args -p
  use_sim_time:=true --params-file <path>/task1_route.yaml`, plus the boat
  values that the launch file normally passes, section 6; the brief says how),
  the status goes
  `WAITING` → `RUNNING 1/9` and the boat's x position increases.

#### M1.3 Brief (agreed 2026-10-20)
- **Goal:** C++ node `waypoint_follower`: reads ground truth, runs the M1.2
  chain at 20 Hz in simulation time, commands the thrusters; plus
  `config/task1_route.yaml` (values as §6).
- **Decisions (2026-10-20):** S1 a: a ROS-free class `asv_sim::Follower`
  (`follower.hpp`) holds Route, LOS, the controllers, GateMonitor and the
  state machine; the node only converts messages. S2 a: `test_closed_loop`
  drives the model through `Follower` (same 4 PASS events expected; stop and
  report if more than late digits change). S3 a: STOPPED and FINISHED are
  latched. ⚠ New `<depend>` rclcpp, std_msgs, nav_msgs, geometry_msgs: yes.
- **Why:** closes the loop in Gazebo (M1.4 launch, M1.5 tuning, bags for
  M2). Experimental Scenarios; App. C.
- **New ideas:** ROS node callbacks (the odometry callback stores the latest
  message, a ROS-clock timer does the work; one thread); state machine;
  transient-local QoS for the path.
- **Files:** ⚠ `package.xml`; `CMakeLists.txt`; new `include/asv_sim/follower.hpp`,
  `src/follower.cpp`, `src/waypoint_follower.cpp`, `config/task1_route.yaml`,
  `test/test_follower.cpp`; `test/test_closed_loop.cpp`; this plan,
  `docs/master_plan.md`, `CLAUDE.md`, `docs/glossary.md`.
- **How it works:** each tick at sim time t: no odometry → WAITING;
  t − stamp > odom_timeout → STOPPED: odom timeout; t − t_first < start_delay
  → WAITING; t − t_run_start > max_run_time → STOPPED: time limit;
  route finished → FINISHED (zero thrust, pods straight); else RUNNING i/9:
  Route → LOS ψ_d → χ = course_angle(ψ, u, v, u_min) → HeadingControl →
  SpeedControl (Δt from odometry stamps; 0 for a repeated message) → mix().
  GateMonitor every tick with odometry. Commands, target and cross-track
  error every tick, status on change, path once (transient local, `world`).
  Boat values are required parameters (no default); M1.4's launch file reads
  them from `asv.yaml`.
- **Check:** build without warnings; 78 old gtests + `test_follower`;
  `test_closed_loop` output compared before/after; 29 Python tests; headless
  Gazebo with the node started by hand: status WAITING → RUNNING 1/9, x
  increases, `ros2 node info` shows only `/asv/ground_truth/odom` (+ `/clock`)
  as input, path received with transient-local QoS. Observation only: how far
  the run gets, reverse thrust share.
- **Can't check:** Gazebo window, RViz path. Unverified: whether Gazebo
  keeps the last thrust command when the node is killed.

#### M1.3 Report (2026-10-20)
- **What changed:**
  - `include/asv_sim/follower.hpp`, `src/follower.cpp`: `Follower` = the
    state machine of §5.7 plus one step of the control chain;
    `gate_event_text()` for the status line.
  - `src/waypoint_follower.cpp`: the node (parameters, latest odometry,
    20 Hz ROS-clock timer, publishes the topics of §4). Every parameter is
    required, no defaults in code.
  - `config/task1_route.yaml`: the parameters of §6.
  - `test/test_follower.cpp`: 11 gtests (states, latching, Δt from stamps,
    same chain as by hand, gate events, bad config).
  - `test/test_closed_loop.cpp`: drives the model through `Follower` (S2 a).
  - ⚠ `package.xml`: `<depend>` rclcpp, std_msgs, nav_msgs, geometry_msgs;
    `CMakeLists.txt`: `follower.cpp`, executable `waypoint_follower`,
    `test_follower`.
  - `docs/glossary.md`: callback/executor, state machine. This plan: §3.
- **How it was checked:**
  - `colcon build --packages-up-to asv_sim`: no warnings.
  - gtests: `test_mix` 9, `test_angles` 6, `test_guidance` 14,
    `test_control` 27, `test_gate_monitor` 15, `test_follower` 11,
    `test_closed_loop` 7 = 89 passed; `colcon test-result`: 96 tests,
    0 failures. Python: `Ran 29 tests ... OK`.
  - `test_closed_loop` before/after S2: every run still finishes with 4 PASS
    events and no collision. Changes: finish time +0.05 to +0.10 s, gate
    offsets at most 2 mm, max |e| at most 7 mm. For y±0.5 the closest buoy
    is a different one at the same value (0.683 → 0.682 m). Cause: the first
    step now has Δt = 0, and the monitor samples the pose at the start of
    the step instead of after it.
  - Node without the boat values: aborts with `Statically typed parameter
    'drag_linear' must be initialized.`
  - Headless Gazebo, node started by hand (`--params-file
    config/task1_route.yaml -p drag_linear:=4.0 -p drag_quadratic:=6.0
    -p max_thrust:=50.0 -p min_thrust:=-40.0 -p steer_limit:=1.5708
    -p boat_length:=1.60 -p boat_half_width:=0.44`): log `WAITING` →
    `RUNNING 1/9`; ground-truth x 8.91 m, then 15.46 m about 5 s later.
    `ros2 node info`: subscribers only `/asv/ground_truth/odom`, `/clock`,
    `/parameter_events`. Path received with transient-local QoS (10 poses).
  - Observation (bag `ws/data/bags/m1_3_check`, not committed): the whole
    Task 1 ran on the first try. gate1 OUT −0.000, gate2 OUT −0.000,
    gate2 IN −0.178, gate1 IN +0.020 m, all PASS. FINISHED after 64.50 s of
    simulation time in RUNNING. Closest buoy gate2_green 0.671 m, no
    collision. Max |e| 1.643 m. A thruster was in reverse in 49.8 % of the
    commands (model: about 63 %), smallest thrust −23.0 N. 1892 commands in
    96.7 s of simulation time (19.6 Hz).
- **What could not be checked:** the Gazebo window and the path in RViz
  (Marcell: start RViz, add Path on `/asv/autonomy/path` with Durability
  Transient Local, fixed frame `world`). The `WAITING` status is not in the
  bag (it was published before the recorder connected); the log shows it.
  Not checked: whether Gazebo keeps the last thrust command when the node is
  killed. Only one run so far; repeatability (3 runs) is M1.5.
- **How to read the code:** start at `Follower::step()` (`follower.cpp`).
  It has the §5.7 state order, then the chain Route → LOS (5.1, 5.2) →
  `course_angle()` → `HeadingControl` (5.3) → `SpeedControl` (5.4) → `mix()`
  (5.5). Then `WaypointFollower::tick()`: message in, `step()`, messages
  out. `to_nav()` maps the odometry to ψ, u, v, ω.
- **Numbers for the thesis:** Gazebo run, starting gains of §6: run time
  64.50 s, max |e| 1.643 m, closest buoy 0.671 m, gate offsets above
  [MEASURED, one run, 2026-10-20]. Model, same code: 64.60 s, 1.645 m,
  0.684 m [MEASURED model]. Reverse thrust 49.8 % of commands [MEASURED].
- **Check your understanding:**
  1. Why does the follower publish zero thrust every cycle in FINISHED,
     instead of just stopping publishing?
  2. Why is Δt taken from the odometry stamps and not from 1/control_rate?
  3. What would go wrong if STOPPED were not latched and the odometry came
     back after a gap of a few seconds?

### Step 4 — One-command launch (`launch/task1.launch.py`)  (M1.4)
Includes `sim.launch.py` (arguments passed through, e.g. `gui:=false`), starts
`waypoint_follower` with `config/task1_route.yaml` plus the drag, thrust and
steering values read from `config/asv.yaml`; sets `use_sim_time` for all nodes
(`SetParameter(name='use_sim_time', value=True)`).
- **Check:** `ros2 launch asv_sim task1.launch.py gui:=false` runs the
  whole task; the log shows the gate events and `FINISHED`.

#### M1.4 Brief (agreed 2026-10-20)
- **Goal:** `ros2 launch asv_sim task1.launch.py [gui:=false]
  [params_file:=...]` starts Gazebo, bridge, static TFs and
  `waypoint_follower`, the boat values read from `config/asv.yaml`.
- **Decisions (2026-10-20):** L1 b: arguments `gui` (passed to
  `sim.launch.py`) and `params_file` (default `config/task1_route.yaml`).
  L2 a: after FINISHED everything keeps running (stop with Ctrl-C); the
  follower keeps publishing zero thrust.
- **Why:** repeatable start for M1.5 and later recordings; success
  criterion 1 (§1); removes the hand-typed `-p` copies of `asv.yaml` (10.8 a).
- **New ideas:** OpaqueFunction (plain Python at launch time).
- **Files:** new `launch/task1.launch.py` (with a plain function
  `boat_params()`), new `tests/test_task1_launch.py`;
  `config/task1_route.yaml` (comment only); this plan, `docs/master_plan.md`,
  `CLAUDE.md`, `docs/glossary.md`. No CMake or `package.xml` change.
- **How it works:** `SetParameter(use_sim_time)`, include `sim.launch.py`,
  then `waypoint_follower` with `params_file` plus `boat_params(asv.yaml)`:
  drag_linear = xU, drag_quadratic = xUU, max_thrust, min_thrust,
  steer_limit, boat_length = hull.length, boat_half_width = hull.spacing/2 +
  hull.radius (same formulas as `test/boat_model.hpp`).
- **Check:** build without warnings; Python tests (29 + new); 89 gtests
  unchanged; headless run with `timeout -s INT` shows WAITING → RUNNING →
  four gate events → FINISHED; `ros2 param get` boat_half_width 0.44 and
  use_sim_time true.
- **Can't check:** the Gazebo window with `gui:=true`.

#### M1.4 Report (2026-10-20)
- **What changed:**
  - `launch/task1.launch.py`: new. Sets `use_sim_time`, includes
    `sim.launch.py` (`gui` passed through), starts `waypoint_follower` with
    `params_file` (default `config/task1_route.yaml`) plus `boat_params()`.
    `boat_params()` reads asv.yaml: drag_linear = xU, drag_quadratic = xUU,
    max_thrust, min_thrust, steer_limit, boat_length = hull.length,
    boat_half_width = hull.spacing/2 + hull.radius. A missing key raises.
  - `tests/test_task1_launch.py`: new, 2 unittests (the seven values from
    asv.yaml; a missing key raises KeyError).
  - `config/task1_route.yaml`: header comment only.
  - `docs/glossary.md`: launch file, OpaqueFunction.
- **How it was checked:**
  - `colcon build --symlink-install --packages-up-to asv_sim`: finished,
    no warnings; `task1.launch.py` linked into the install space.
  - Python: `Ran 31 tests ... OK` (29 + 2). `colcon test-result`: 96 tests,
    0 errors, 0 failures (89 gtests unchanged).
  - `ros2 launch asv_sim task1.launch.py gui:=false` (headless, one
    command): `ros2 param get` on `/waypoint_follower`: boat_half_width 0.44,
    drag_linear 4.0, use_sim_time True; `/asv_bridge` use_sim_time True.
  - Log: `WAITING` → `RUNNING 1/9`; gate1 OUTBOUND −0.002 m PASS, gate2
    OUTBOUND −0.004 m PASS, RUNNING 2/9 … 9/9, gate2 INBOUND −0.178 m PASS,
    gate1 INBOUND +0.021 m PASS, `FINISHED`, closest buoy gate2_green
    0.672 m, collision no.
- **What could not be checked:** the Gazebo window (`gui:=true`; Marcell:
  run `ros2 launch asv_sim task1.launch.py` in his terminal and watch the
  boat). The run time in simulation time: the log stamps are wall time
  (RUNNING → FINISHED 71.2 s wall; at a real-time factor of about 0.85 that
  fits M1.3's 64.5 s of simulation time, but it was not measured). M1.5's
  bag and `analyze_run.py` will give it. `params_file:=` with another file
  was not run (only the default).
- **How to read the code:** `task1.launch.py`: `generate_launch_description()`
  (arguments, use_sim_time, include), then `_launch_setup()` (reads asv.yaml,
  starts the node), then `boat_params()` (the seven values).
- **Numbers for the thesis:** none new. Second Gazebo run, starting gains:
  gate offsets above, closest buoy 0.672 m [MEASURED, one run, 2026-10-20];
  M1.3 gave 0.671 m.
- **Check your understanding:**
  1. Why are the boat values computed in the launch file instead of being
     written into `task1_route.yaml`?
  2. The follower and Gazebo start at the same moment. What stops the
     follower from steering before Gazebo publishes the first pose?
  3. If two parameter sources set the same name (params file and
     `boat_params()`), which one wins, and why does that not matter here?

### Step 5 — Tuning and evaluation  (M1.5)
Run, read the log (gate offsets, minimum buoy distance, cross-track error,
time), adjust gains in the YAML, repeat. A small analysis tool
(`tools/analyze_run.py`) that reads a recorded run and prints the metrics, so
runs can be compared objectively. Recording:
`timeout -s INT <s> ros2 bag record --use-sim-time -o /ws/data/bags/task1_<n>
--topics /clock /asv/ground_truth/odom /asv/autonomy/status
/asv/autonomy/cross_track_error /asv/autonomy/target
/asv/thrusters/left/thrust /asv/thrusters/right/thrust` (thrust topics added
in M1.5 for the reverse-thrust share).
- **Check:** criteria 2–5 of section 1 met in 3 consecutive runs; final gains
  and results recorded.

#### M1.5 Brief (agreed 2026-10-20)
- **Goal:** `tools/analyze_run.py` reads a recorded Task 1 bag and prints the
  metrics; with it, show criteria 2–5 (§1) in 3 consecutive runs.
- **Decisions (2026-10-20):** E1 a: gate crossings, offsets and clearance are
  recomputed in Python from `/asv/ground_truth/odom` and compared with the
  node's `GATE ...` status lines. E2 a: keep the §6 starting gains if the 3
  runs meet criteria 2–5; only record the results. E3 a: the tool also writes
  `trajectory.png` (route, buoys, driven path, gate crossings) next to the bag.
- **Why:** the M1.3/M1.4 numbers came only from the node's own log; an
  independent check of `GateMonitor`. Feeds "Experimental Scenarios"; the tool
  grows into `asv_eval` (M3.6).
- **New ideas:** rosbag2_py (ROS's Python bag reader; with `--use-sim-time`
  the stored times are simulation time); RMS cross-track error.
- **Files:** new `tools/analyze_run.py` (pure functions separate from bag I/O;
  writes `metrics.json` and `trajectory.png` into the bag folder, `ws/data/`),
  new `tests/test_analyze_run.py`; this plan, `docs/master_plan.md`,
  `CLAUDE.md`, `docs/glossary.md`. No CMake or `package.xml` change.
- **How it works:** buoys, gates, `gate_margin`, `gate_capture` from
  `task1_route.yaml`; L and b_w from `boat_params()` (`task1.launch.py`).
  Run time: first `RUNNING` → `FINISHED` (bag time). Gates and clearance: §5.6
  on consecutive odometry poses (50 Hz). Cross-track: max |e| and
  e_rms = sqrt(mean(e_k²)) over RUNNING. Thrust: reverse share = fraction of
  commands with min(F_L, F_R) < 0, smallest thrust, thrust 0 after FINISHED.
  Python offset minus node offset per gate event.
- **Check:** unittests on hand-made cases (the `test_gate_monitor` cases);
  Python 31 + new, gtests 89 unchanged; the tool on `m1_3_check` gives the
  node's offsets within a few mm; 3 headless runs `task1_1..3`, each 4/4 PASS,
  clearance ≥ 0.5 m, no collision, FINISHED with thrust 0.
- **Can't check:** the Gazebo window; the PNG is for Marcell to look at.

#### M1.5 Report (2026-10-20)
- **What changed:**
  - `tools/analyze_run.py`: new. Reads a bag (rosbag2_py), recomputes gate
    events and per-buoy clearance from `/asv/ground_truth/odom` with the §5.6
    equations (`gate_crossing()`, `clearance()`), compares offsets with the
    node's `GATE` lines, computes run time, max |e| and e_rms, reverse-thrust
    share, thrust after FINISHED; prints criteria 2–4, writes `metrics.json`
    and `trajectory.png` into the bag folder (or `--out`). Exit code 1 if a
    criterion fails. Boat size via `boat_params()` of `task1.launch.py`.
  - `tests/test_analyze_run.py`: new, 13 unittests (the `test_gate_monitor`
    gate and clearance cases, order check, RMS, reverse share, yaw, status
    parsing).
  - Step 5 recording command above: two thrust topics added.
  - `docs/glossary.md`: RMSE (cross-track RMS), rosbag2_py. `CLAUDE.md`:
    Known state, Python baseline 44.
- **How it was checked:**
  - Python: `Ran 44 tests ... OK` (31 + 13). `colcon test-result --all`:
    96 tests, 0 errors, 0 failures (89 gtests unchanged; no C++ change).
  - Tool on the M1.3 bag `m1_3_check`: run time 64.50 s; offsets −0.000,
    −0.000, −0.178, +0.020 m, all PASS; largest |recomputed − node| offset
    0.0005 m (the node prints 3 decimals); closest buoy gate2_green
    0.671 m; reverse share 49.8 % of 1290 RUNNING commands, smallest
    −23.0 N: the same numbers as the M1.3 report.
  - 3 headless runs (`task1.launch.py gui:=false` + `ros2 bag record
    --use-sim-time`, 160 s each), starting gains of §6:

    | Run | Time [s] | gate1 OUT | gate2 OUT | gate2 IN | gate1 IN | Closest buoy [m] | max \|e\| [m] | e_rms [m] | Reverse [%] | Min F [N] | Criteria 2–4 |
    |---|---|---|---|---|---|---|---|---|---|---|---|
    | task1_1 | 64.50 | −0.000 | −0.002 | −0.179 | +0.020 | 0.671 (gate2_green) | 1.645 | 0.514 | 50.4 | −22.9 | all OK |
    | task1_2 | 64.50 | −0.000 | −0.002 | −0.178 | +0.020 | 0.671 (gate2_green) | 1.645 | 0.514 | 50.2 | −23.0 | all OK |
    | task1_3 | 64.50 | −0.000 | −0.002 | −0.179 | +0.020 | 0.671 (gate2_green) | 1.645 | 0.514 | 50.4 | −23.0 | all OK |

    Gate offsets in m, all PASS (band ±0.355 m). Thrust after FINISHED 0.0 N
    in all runs. Node vs. recomputed offsets ≤ 0.0005 m. Criterion 5
    (3 consecutive runs) holds, so no gain change (E2 a).
- **What could not be checked:** the Gazebo window. `trajectory.png` in
  each bag folder (`ws/data/bags/task1_1/trajectory.png`): I opened it and
  it shows the route, buoys, path and the four crossings; Marcell should look
  at it too. The "% of all commands" share depends on how long the recorder
  ran (34.0 % in M1.3, 22.7 % here), so only the RUNNING share is meaningful.
- **How to read the code:** `main()` → `read_bag()` → `analyze()`.
  `gate_crossing()` = §5.6 crossing, offset o and band (same order as
  `GateMonitor::update()`); `clearance()` = c_j = |d − k| − r_j;
  `rms()` = e_rms; `reverse_share()`; `parse_status()` reads the node's lines.
- **Numbers for the thesis:** Task 1, starting gains, 3 Gazebo runs:
  run time 64.50 s, closest buoy 0.671 m, max |e| 1.645 m, e_rms 0.514 m,
  reverse thrust 50.2–50.4 % of RUNNING commands, smallest thrust −23.0 N,
  gate offsets above [MEASURED, 2026-10-20]. Runs differ by at most 1 mm in
  the offsets: the simulation is nearly, not exactly, repeatable.
- **Check your understanding:**
  1. Why is it useful that the tool recomputes the gate offsets instead of
     reading them from the node, if both use the same equations?
  2. Odometry comes at 50 Hz, the monitor in the node samples at 20 Hz. Which
     one can give a smaller closest-buoy distance, and why?
  3. Why is the reverse share counted only over the RUNNING commands?

### Step 6 — Documentation  (M1.6)
`ws/src/asv_sim/docs/AUTONOMY.md`: route, architecture, the equations of
section 5, chosen parameters and why, measured results (table from step 5),
known limitations. Create or update `README.md` (repo root) with the run
command.
- **Check:** Marcell has read it and can explain every part.

#### M1.6 Brief (agreed 2026-10-20)
- **Goal:** one as-built reference `docs/AUTONOMY.md` for the Task 1
  autonomy, and a current `README.md` with the Task 1 run command.
- **Choices (2026-10-20):** A a: §2–6 of this plan stay as the decision
  history, with a pointer to AUTONOMY.md (nothing deleted). B: the guidance
  symbols stay "proposed" (decided when App. C is written); AUTONOMY.md marks
  them so. C: no run figure committed; AUTONOMY.md gives the command that
  regenerates `trajectory.png`.
- **Why:** success criterion 7 (§1). §2–6 mix the first proposal with
  "implemented form" patches and have stale lines (§5.5 "does not exist
  yet", §5.3 K_spin = 1.0); decision 10.16 is labelled "a" in §5.3, §6, the
  glossary and `task1_route.yaml`, but b (proportional spin) was built. Feeds
  App. C and Experimental Scenarios.
- **Files:** new `docs/AUTONOMY.md` (bullets and equations only: run,
  route, architecture, final equations next to their functions, parameter
  table with source tags and reasons, M1.5 results, limitations, symbols);
  `README.md` (status, Task 1 commands, layout, docs table); this plan
  (pointers, 10.16 label); `config/task1_route.yaml` (comment only);
  `docs/glossary.md` (10.16 label); `docs/master_plan.md`; `CLAUDE.md`. No
  code or number change.
- **Check:** script: every parameter value in AUTONOMY.md equals
  `task1_route.yaml` / `asv.yaml`; equations re-read against the sources;
  links resolve; tests unchanged (Python 44, gtests 89 / 96); the README
  Task 1 command run once headless reaches FINISHED.
- **Can't check:** whether the doc is clear to Marcell (the step's "done
  when"); GitHub rendering.

#### M1.6 Report (2026-10-20)
- **What changed:**
  - `docs/AUTONOMY.md`: new. Run command; route table; architecture and
    topics; the final equations of `Route`, `LosGuidance`, `course_angle()`,
    `HeadingControl`, `SpeedControl`, `mix()`, `GateMonitor`, `Follower`,
    each next to its function; parameter table (value, unit, tag, reason,
    decision); M1.5 results and how to reproduce them; known limitations.
    Guidance symbols marked "proposed" (choice B).
  - `README.md`: status bar and milestone table; Task 1 run, record and
    analyse commands; gtest command; layout with `include/ src/ test/`,
    `task1.launch.py`, `task1_route.yaml`; AUTONOMY.md in the docs table.
  - This plan: "as built" pointers at §2 and §5 (choice A a); 10.16 a → b in
    §5.3 and §6. `config/task1_route.yaml`: the same label fix (comment
    only). `docs/glossary.md`: 10.16 a → b (2 places).
- **How it was checked:**
  - Script (scratchpad, not committed): all 40 values in AUTONOMY.md (30
    parameters + W0–W9) equal `task1_route.yaml` / `asv.yaml`: "40 checked,
    0 bad"; b_w = 0.4400 m, PASS band 0.3554 m.
  - Results table against `ws/data/bags/task1_{1,2,3}/metrics.json`: run time
    64.5 s, node offsets −0.0/−0.002/−0.179(−0.178)/+0.02 m, closest buoy
    gate2_green 0.6705/0.6711/0.6706 m, max |e| 1.6447/1.6451/1.6452 m.
  - Equations re-read against `guidance.hpp`, `control.hpp`, `mix.cpp`,
    `gate_monitor.hpp`, `follower.hpp/.cpp` (χ, not ψ, goes into
    `HeadingControl::compute()`: `follower.cpp` line 107-108).
  - Relative links resolve (`ls`).
  - Python `Ran 44 tests ... OK`; `colcon test-result --all`: 96 tests, 0
    errors, 0 failures (gtests 6+7+27+11+15+14+9 = 89).
  - README command `task1.launch.py gui:=false` run once headless: `GATE gate1
    OUTBOUND offset -0.000 m PASS`, `gate2 OUTBOUND -0.002 PASS`, `gate2
    INBOUND -0.179 PASS`, `gate1 INBOUND +0.020 PASS`, then `FINISHED`.
- **What could not be checked:** whether the doc is clear to Marcell (the
  step's "done when": he reads it and can explain every part); GitHub
  rendering of the equations.
- **Found, not changed (outside the brief):** the C++ comments still say
  "10.16 a" (`control.hpp` 3×, `control.cpp`, `test_control.cpp`,
  `test_closed_loop.cpp`); a comment-only fix, waits for Marcell's yes.
- **How to read the code:** start at AUTONOMY.md §3 (the chain), then §5 in
  order; each subsection names the function that implements it.
- **Numbers for the thesis:** none new; AUTONOMY.md §6–7 collect the values
  with their tags (M1.5 results [MEASURED, 2026-10-20]).
- **Check your understanding:**
  1. Why does the heading error use the course χ, while the D term uses the
     yaw rate ω?
  2. In `mix()`, which quantity survives saturation, and why that one?
  3. Why are X_u and X_uu fine in the speed feedforward but not in the
     filter?

### Step 7 — Maneuver player  (M1.7)
Plays a timed list of (t, throttle, steer, spin) from YAML through `mix()`,
open loop, timed in simulation time; waits for the first `/clock` message
before starting. For simple, repeatable test runs (straight line, circle,
zig-zag).
- **Check:** the same maneuver run twice; the largest difference between the
  two true trajectories is measured and reported.

Split on 2026-10-20 into M1.7a (the player) and M1.7b (repeatability in
Gazebo, own brief).

#### M1.7a Brief (agreed 2026-10-20)
- **Goal:** `ros2 launch asv_sim maneuver.launch.py maneuver:=circle
  [gui:=false]` plays a timed (t, throttle, steer, spin) table through
  `mix()`, open loop, in simulation time.
- **Decisions (2026-10-20):** P1 split a/b; P2 zero-order hold between rows
  (not linear interpolation); P3 after the last row the player keeps
  publishing zero thrust and `DONE` until Ctrl-C (same as the follower, L2 a).
- **Why:** repeatable test runs with known inputs (straight, circle,
  zig-zag) for recordings (M2.9) and filter studies; Experimental Scenarios.
  Differs from the follower: the inputs are fixed, the path is not.
- **New ideas:** open loop; zero-order hold (glossary).
- **Files:** new `include/asv_sim/maneuver.hpp`, `src/maneuver.cpp`
  (`Maneuver`, in `asv_sim_guidance`), `test/test_maneuver.cpp`,
  `test/test_maneuver_files.cpp` (each maneuver on the 3-DOF model),
  `src/maneuver_player.cpp` (I/O only), `launch/maneuver.launch.py`,
  `config/maneuvers/{straight,circle,zigzag}.yaml`; edits `CMakeLists.txt`
  (no new dependencies), `docs/design.md` §1, `docs/glossary.md`,
  `AUTONOMY.md`, this plan, `docs/master_plan.md`, `CLAUDE.md`.
- **How it works:** t₀ = first nonzero ROS time + `start_delay`
  (`design.md` §3 rule 5), τ = t − t₀; c(τ) = c_k for t_k ≤ τ < t_{k+1};
  c = 0 for τ < 0 and τ ≥ t_N (the last row must be all zero). Every 20 Hz
  tick: `mix(throttle, steer, spin, limits)` → the four thruster topics;
  `/asv/maneuver/status`: `WAITING`, `RUNNING k/N`, `DONE`; the start time is
  logged. The table is checked at start (t_0 = 0, strictly increasing, equal
  lengths, last row zero). Each maneuver drives east along y = 0 through both
  Task 1 gates, then the pattern east of gate 2 (where the Task 1 teardrop
  turns). Values [EST], set on the 3-DOF model.
- **Check:** build clean; gtests 89 + new; Python 44 unchanged; on the model
  every maneuver keeps ≥ 1.0 m [EST] from all Task 1 buoys; one headless
  Gazebo run per maneuver goes WAITING → RUNNING → DONE, `use_sim_time` true.
- **Can't check:** the Gazebo window.

Changed during the build (2026-10-20, Marcell): **Q4 a**, clearance 0.5 m
for every buoy and zone instead of 1.0 m. A pass through a Task 1 gate
centre gives 1.524 − 0.2286 − 0.44 = 0.855 m, so 1.0 m was impossible.
**Q5 B1**, the circle is a spin in place followed by pods-only left circles
west of the spawn. The world has no shoreline, so the area check uses
keep-out zones around Tasks 2, 3 and 5 instead of a free box. Inside the
first box no circle fitted: pods-only radius is about 10 m [MEASURED on the
model].

#### M1.7a Report (2026-10-20)
1. **What changed**
   - `include/asv_sim/maneuver.hpp`, `src/maneuver.cpp`: `Maneuver` (checks
     the table, zero-order hold `active_row()`, `command_at()`) and
     `ManeuverPlayer` (start on the first nonzero clock + `start_delay`,
     states WAITING/RUNNING/DONE, `mix()`), in `asv_sim_guidance`.
   - `src/maneuver_player.cpp`: node, I/O only; 20 Hz ROS-clock timer, four
     thruster topics, `/asv/maneuver/status`, logs the start time.
   - `launch/maneuver.launch.py`: `maneuver:=<name>`, `gui`; limits from
     `asv.yaml` through `boat_params()` of `task1.launch.py`.
   - `config/maneuvers/{straight,zigzag,circle}.yaml`: the three tables.
   - `test/test_maneuver.cpp` (8 gtests), `test/test_maneuver_files.cpp`
     (3 gtests: each file on the 3-DOF model; buoys and keep-out zones).
   - `CMakeLists.txt`; `docs/design.md` §1 (status topic); `docs/glossary.md`
     (open/closed loop; zero-order hold extended); `AUTONOMY.md` §9;
     `docs/master_plan.md` (M1.7 split into M1.7a, M1.7b); `CLAUDE.md`.
2. **How it was checked**
   - `colcon build --packages-up-to asv_sim`: no warnings.
   - `colcon test-result`: 109 tests, 0 failures (100 gtests: the 89 before
     plus `test_maneuver` 8, `test_maneuver_files` 3; 9 CTest wrappers).
   - `python3 -m unittest discover -s tests`: Ran 44 tests, OK.
   - Model check [MEASURED, 3-DOF model, no waves]:

     | maneuver | end (x, y) m | closest buoy | closest zone |
     |---|---|---|---|
     | straight | (26.10, 0.00) | gate1_red 0.855 m | task3 3.72 m |
     | zigzag | (24.81, 0.42) | gate1_red 0.855 m | task2 4.66 m |
     | circle | (3.01, 3.93), x −13.2..6.3, y 0..21.0 | gate1_red 2.177 m | task2 9.26 m |

   - Headless Gazebo, one run each (`ros2 launch asv_sim maneuver.launch.py
     maneuver:=<name> gui:=false`): all three logged `start at t = 3.052 s`,
     WAITING → RUNNING 1/N … N/N → DONE; `ros2 param get /maneuver_player
     use_sim_time` → True. Last true position (taken a few seconds after
     DONE, so only a rough comparison): zigzag (24.99, 0.45) m, circle
     (3.65, 4.58) m. Straight: RUNNING → DONE took 40.2 s wall for 25 s
     simulated (real-time factor about 0.62 in this run).
3. **What could not be checked:** the Gazebo window (`gui:=true`), and
   whether the boat touches a buoy in Gazebo (no clearance analysis of the
   Gazebo runs yet: M1.7b). Marcell: `ros2 launch asv_sim maneuver.launch.py
   maneuver:=circle` and watch the spin in place and the circles.
4. **How to read the code:** start at `maneuver.hpp`.
   `Maneuver::active_row()` is the zero-order hold c(τ) = c_k for
   t_k ≤ τ < t_{k+1}; `ManeuverPlayer::step()` sets t_start = first t > 0 +
   start_delay, τ = t − t_start, and calls `mix()`. The node
   (`maneuver_player.cpp`) only copies parameters in and commands out.
5. **Numbers for the thesis:** maneuver tables in `AUTONOMY.md` §9 [EST];
   `start_delay` 3 s, 20 Hz [EST]; clearance limit 0.5 m [EST, Q4 a];
   keep-out zones [REPO course files]; model results above [MEASURED, model].
6. **Check your understanding**
   1. Why does the circle maneuver start with a spin in place instead of
      turning right away?
   2. If the player's timer fires 30 ms late in one run, which part of the
      path changes, and why does the follower not have this problem?
   3. Why does the last row of every table have to be all zero?

#### M1.7b Brief (agreed 2026-10-20)
- **Goal:** `tools/compare_runs.py <bag> <bag> [...]` gives the path
  difference between recorded runs of the same maneuver; every maneuver
  recorded three times in headless Gazebo; numbers reported.
- **Decision (2026-10-20):** Q1 3 runs per maneuver (three pairs; a range,
  not one number).
- **Why:** shows whether one recording per maneuver stands for "the" run;
  Experimental Scenarios. Also checks buoy clearance in Gazebo (M1.7a only
  checked it on the model).
- **Files:** new `tools/compare_runs.py`, `tests/test_compare_runs.py`;
  `tools/analyze_run.py` (`read_bag(path, topics=None)`, default unchanged);
  this plan, `AUTONOMY.md` §9, `docs/master_plan.md`, `CLAUDE.md`. No C++,
  no new dependency.
- **How it works:** per bag: poses from `/asv/ground_truth/odom` header
  stamps; t_s = bag time of the first `RUNNING` on `/asv/maneuver/status`;
  τ = t − t_s. Run B linearly interpolated at run A's τ (ψ unwrapped first).
  e(τ) = ‖p_B(τ) − p_A(τ)‖, e_ψ(τ) = wrap(ψ_B − ψ_A). Per pair over
  τ ∈ [0, t_N + 10 s]: max e, RMS e, e(t_N), max |e_ψ|; per run: closest
  buoy; thruster command change times compared between runs. Writes
  `compare.json`, `compare.png` to `ws/data/runs/maneuver_<name>/`.
- **Recording:** `maneuver.launch.py gui:=false` in the background,
  `ros2 bag record --use-sim-time` (odom, maneuver status, four thruster
  topics) into `ws/data/bags/maneuver_<name>_<k>`; fresh Gazebo per run.
- **Check:** unit tests (identical runs → e = 0; 0.1 m shift → 0.1 m; time
  offset interpolated right; ψ across ±π); Python 44 + new; gtests 100
  unchanged. Expected [EST, unverified]: 0 to a few dm, growing with τ,
  largest for the circle.
- **Can't check:** nothing visual is needed.

#### M1.7b Report (2026-10-20)
1. **What changed**
   - `tools/compare_runs.py`: lines runs up on τ (first `RUNNING`),
     interpolates run B at run A's samples, prints e and e_ψ per pair (mm in
     the printout, m in `compare.json`), closest buoy per run, command
     timing per pair; writes `compare.json`, `compare.png`.
   - `tests/test_compare_runs.py`: 12 tests (wrap/unwrap, start, track,
     interpolation, identical runs → 0, 0.1 m shift → 0.1 m, offset
     sampling, heading across ±π, no overlap, command changes).
   - `tools/analyze_run.py`: `read_bag(path, topics=None)`; default list
     unchanged.
   - `AUTONOMY.md` §9 (result line), `docs/master_plan.md`, `CLAUDE.md`.
2. **How it was checked**
   - `python3 -m unittest discover -s tests`: Ran 56 tests, OK (44 + 12).
     No C++ change: gtests 100 as in M1.7a.
   - A bag compared with itself: e = 0 exactly.
   - Nine headless recordings, fresh Gazebo each (`ws/data/bags/
     maneuver_{straight,zigzag,circle}_{1,2,3}`); every run logged
     `start at t = 3.052 s` and reached DONE; the circle bags cover
     265.8 s of simulation time (τ up to t_N + 10 s = 155 s needed).
   - Results [MEASURED], τ ∈ [0, t_N + 10 s]:

     | maneuver | pair | max e (mm) | RMS e (mm) | e(t_N) (mm) | max abs e_ψ (rad) |
     |---|---|---|---|---|---|
     | straight | 1-2 | 0 | 0 | 0 | 0 |
     | straight | 1-3 | 4.0007 | 3.8797 | 4.0001 | 6.4e-6 |
     | straight | 2-3 | 4.0007 | 3.8797 | 4.0001 | 6.4e-6 |
     | zigzag | 1-2 | 4.1823 | 3.8993 | 4.1821 | 7.0e-4 |
     | zigzag | 1-3 | 1.2438 | 0.5084 | 1.2116 | 7.0e-4 |
     | zigzag | 2-3 | 3.9999 | 3.8913 | 3.9990 | 6.5e-6 |
     | circle | 1-2 | 0.0104 | 0.0090 | 0.0083 | 1.3e-7 |
     | circle | 1-3 | 0.0033 | 0.0032 | 0.0031 | 3.1e-8 |
     | circle | 2-3 | 0.0097 | 0.0084 | 0.0080 | 1.2e-7 |

   - In every pair the recorded command values and times (bag time − t_s)
     are identical (largest offset 0.000 s).
   - Closest buoy, all runs: straight and zigzag gate2_green 0.855 m (the
     gate-centre pass), circle gate1_red 2.202 m.
   - Where the differences come from (checked on the τ series):
     - one physics step: straight run 3 and zigzag run 2 differ from the
       others by a constant 4.00 mm from τ = 0.008 s on, = 1 m/s × 4 ms
       (one physics step [REPO world step 0.004 s]): the first command
       reached the physics one step earlier or later. Zigzag runs 1 and 3
       are identical until τ = 27.008 s (the last steering change), then
       differ by up to 1.24 mm.
     - before the start: at τ = −3 s, with zero thrust at the spawn,
       some runs already differ by up to 6.7 µm. Cause not determined.
3. **What could not be checked:** why the step offset happens (the
   recorder sees the same command times; the offset is between the bridge
   and the physics step, not visible in the bag). The plots
   (`ws/data/runs/maneuver_*/compare.png`) are for Marcell to look at.
4. **How to read the code:** `compare_runs.main()` → `analyze_runs()`
   (per bag: `start_time()`, `track()`, clearances, `command_changes()`)
   → `compare_pairs()` → `compare()`, which implements
   e(τ) = ‖p_B(τ) − p_A(τ)‖ with `interpolate()`, and e_ψ = wrap(ψ_B − ψ_A).
5. **Numbers for the thesis:** the table above [MEASURED, 3 runs per
   maneuver, headless Gazebo, world default waves]. Largest difference
   4.18 mm over 39 s (zigzag); circle ≤ 0.0104 mm over 155 s.
6. **Check your understanding**
   1. Why is a constant 4 mm offset a time shift and not a different path?
   2. The circle runs did not get the step offset. Would a 4 ms shift in
      the circle give a larger or smaller e than in the straight run?
   3. Why are the recorded command times identical while the paths differ?

### Step 8 — Keyboard teleop (WASD)  (M1.8)
C++, calls `mix()`. W/S change the throttle level, A/D the steer level and Q/E
the spin level, in steps; Space sets everything to zero. Levels stay set after
the key is released (a terminal sends no key-release events). Publishes zero
on exit. The key-to-command logic is a ROS-free function with unit tests; the
node only reads keys and publishes. Started only by Marcell in his own
terminal (`ros2 run asv_sim teleop --ros-args -p use_sim_time:=true`), never
from a launch file. Never runs together with the follower or player.
- **Check:** unit tests pass; Marcell drives the boat with it (Claude Code
  can't press keys).

#### M1.8 Brief (agreed 2026-10-20)
- **Goal:** `ros2 run asv_sim teleop --ros-args -p use_sim_time:=true` in
  Marcell's own terminal drives the boat with W/S/A/D/Q/E/Space through
  `mix()`.
- **Decisions (2026-10-20):** language C++ (one `mix()` for follower, player
  and teleop; a Python teleop would need a second copy of it). **T1 a**: the
  node reads the thruster limits from `asv.yaml` and its key steps from
  `config/teleop.yaml` itself (yaml-cpp, package share directory), because a
  launched node gets no keyboard; ⚠ new `<depend>`s `yaml_cpp_vendor` (was
  only `test_depend`) and `ament_index_cpp`.
- **Why:** driving by hand to check the boat by eye and to place it before a
  recording; a tool, no thesis section.
- **New ideas:** terminal raw mode (glossary).
- **Files:** new `include/asv_sim/teleop.hpp`, `src/teleop.cpp` (key logic,
  ROS-free, in `asv_sim_guidance`), `src/teleop_node.cpp` (I/O only),
  `config/teleop.yaml`, `test/test_teleop.cpp`; edits `CMakeLists.txt`,
  ⚠ `package.xml`, `AUTONOMY.md`, `docs/glossary.md`, `README.md`, this plan,
  `docs/master_plan.md`, `CLAUDE.md`.
- **How it works:** keys (either case) W/S throttle ± step, A/D steer ± step
  (A = left, steer > 0), Q/E spin ± step (Q = left, spin > 0), Space all zero,
  X or Ctrl-C quit; other keys ignored. Levels clamped: throttle ∈
  [min_thrust / max_thrust, 1], steer ∈ [−steer_limit, steer_limit], spin ∈
  [−1, 1]; `mix()` handles the thrust saturation (10.1 b). Every 20 Hz
  ROS-clock tick: read all waiting keys (non-blocking), publish `mix(levels)`.
  In raw mode Ctrl-C is a byte (ISIG off), so zero is published before
  `rclcpp::shutdown()`; SIGTERM/SIGHUP also publish zero and restore the
  terminal. Gazebo holds the last command: a crash leaves the boat moving
  (caveat in `AUTONOMY.md`). Steps [EST]: throttle 0.1, steer 0.1 rad, spin
  0.1.
- **Check:** build clean; gtests 100 + new `test_teleop`; Python 56
  unchanged; smoke test in a headless sim under `script` (a pseudo-terminal)
  with keys fed: "www" gives 15 N per thruster, "x" gives 0.
- **Can't check:** real keyboard driving and the Gazebo window.

#### M1.8 Report (2026-10-20)
1. **What changed**
   - `include/asv_sim/teleop.hpp`, `src/teleop.cpp`: `TeleopLevels`,
     `TeleopSteps`, `apply_key()` (one key → new levels, clamped; quit flag).
   - `src/teleop_node.cpp`: raw terminal (`RawTerminal`), keys read in the
     main loop, 20 Hz ROS-clock timer publishes `mix(levels)`; zero on X,
     Ctrl-C, SIGTERM, SIGHUP (own signal handlers, so zero goes out before
     `rclcpp::shutdown()`).
   - `config/teleop.yaml`: steps 0.1 / 0.1 rad / 0.1 [EST], `control_rate` 20 Hz.
   - `test/test_teleop.cpp`: 9 gtests.
   - `CMakeLists.txt` (library source, `teleop` executable, test; yaml-cpp
     `find_package` moved out of the test block), `package.xml` (`<depend>`
     `ament_index_cpp`, `yaml_cpp_vendor`; the `test_depend` dropped).
   - Docs: `AUTONOMY.md` §10, glossary "Terminal raw mode", README M1 row,
     `CLAUDE.md` baseline and Known state.
2. **How it was checked**
   - `colcon build --packages-up-to asv_sim`: finished, no warnings.
   - `colcon test-result`: `119 tests, 0 errors, 0 failures, 0 skipped`
     (109 gtests = 100 + 9 `test_teleop`, plus 10 CTest wrappers).
   - Python: `Ran 56 tests ... OK`.
   - Smoke test, headless sim, keys fed through `script` (a pseudo-terminal):
     "www" then "x". Status line went 0 → 5 → 10 → 15 N per side → 0;
     `/asv/thrusters/left/thrust`: 38 × 0.0, 69 × 15.000000000000002, then
     2 × 0.0 (right the same); output `zero thrust published, teleop stopped`,
     exit code 0.
   - SIGTERM test: "ww", then `pkill -TERM`: 72 × 10.0 then 0.0, same message.
3. **What could not be checked:** driving with a real keyboard and watching
   the boat in the Gazebo window. Marcell: `docker compose exec sim bash`,
   `ros2 launch asv_sim sim.launch.py &` (or the GUI in another terminal),
   then `ros2 run asv_sim teleop --ros-args -p use_sim_time:=true`.
4. **How to read the code:** start at `apply_key()` (`src/teleop.cpp`, a
   switch on the key, then the clamps), then `main()` in `teleop_node.cpp`
   (the loop: read keys → `handle_key()` → `spin_once()`; on exit
   `publish_zero()`). The equations are only `mix()` (§5.5).
5. **Numbers for the thesis:** none (a tool). Steps 0.1 / 0.1 rad / 0.1
   [EST]; reverse throttle limit −40 / 50 = −0.8 [REPO asv.yaml].
6. **Check your understanding**
   1. Why does the node read the keys in the main loop and not in the timer
      callback?
   2. What would happen to the zero command if rclcpp's own Ctrl-C handler
      were left on?
   3. Why does one S after many W presses give 0.9 and not more?

### Step 9 — Waypoint-switch fix  (M1.9)
Found by the 2026-10-20 review (A1, A2): on the tick of a waypoint switch,
`Route::update()` left s and e of the old segment in place, so LOS, d_end and
the published e used them on the new segment for one tick. Marcell
(2026-10-21): fix now and rerun Task 1.
- **Check:** a gtest on the switch tick; all tests; 3 headless Task 1 runs.

#### M1.9 Brief (agreed 2026-10-21)
- **Goal:** on the switch tick, LOS and d_end use s and e measured on the new
  segment.
- **Why:** one wrong ψ_d per switch (8 in Task 1; LOS target 2–4 m ahead,
  d_end 2–17 m too small, as computed in the critique); the Task 1 numbers in
  the thesis should come from correct code. Experimental Scenarios; App. C.
- **Files:** `src/guidance.cpp` (`Route::update()`), `guidance.hpp` (comment),
  `test/test_guidance.cpp` (new `SwitchTickUsesNewSegment`), this section and
  one sentence in §5.1, `AUTONOMY.md` §7 (new numbers), `CLAUDE.md` (Known
  state, test baseline 109 → 110), `docs/master_plan.md` (mark). Old M1.5
  reports stay as history.
- **How:** after `++segment_`, s = (p − W_{i+1})·t̂_{i+1},
  e = (p − W_{i+1})·n̂_{i+1}; s is then usually slightly negative (correct for
  LOS, which clamps with max(0, s + Δ), and for d_end). At most one switch per
  update stays (segment 3→4 is 3.04 m < 2 R_switch).
- **Check:** gtest: from segment 0, p = (16, 0) switches to segment 1 with
  s = −0.894 m, e = +0.447 m, target W1 + 2.106 t̂₁, and target, d_end, e
  equal to a second `update(p)`; fails before the fix. All gtests (110) and
  56 Python tests. 3 headless runs `ws/data/bags/task1_m19_{1,2,3}` +
  `analyze_run.py`: 4/4 PASS, FINISHED, closest buoy ≥ 0.5 m; numbers expected
  within mm–cm of 64.50 s / 0.671 m (expectation); larger change → stop and
  ask.
- **Can't check:** the Gazebo window.

#### M1.9 Report (2026-10-21)
Commit: `git log --grep "^M1.9:"`.
1. **What changed**
   - `src/guidance.cpp`: new private `Route::measure(p)` (s and e on the
     current segment); `update()` calls it, and again right after
     `++segment_`.
   - `include/asv_sim/guidance.hpp`: `measure()` declared; `update()` comment
     says s and e always belong to `segment()`.
   - `test/test_guidance.cpp`: new `Route.SwitchTickUsesNewSegment`.
   - §5.1 one sentence; `AUTONOMY.md` §5.1 note and §7 table (new runs, old
     numbers kept as "before the fix"); `CLAUDE.md` Known state and test
     baseline; `docs/master_plan.md` mark.
2. **How it was checked**
   - New test before the fix: FAILED (target (19.618, −2.246) on the switch
     tick vs. (18.883, 0.942) one tick later). After the fix: passes.
   - gtests: test_guidance 15, total 110 passed; `colcon test-result --all`:
     `120 tests, 0 errors, 0 failures, 0 skipped`. Python: `Ran 56 tests ... OK`.
   - 3 headless runs (`task1.launch.py gui:=false` + the Step 5 recording
     command, 160 s), `tools/analyze_run.py`, all criteria 2–4 OK:

     | Run | Time [s] | gate1 OUT | gate2 OUT | gate2 IN | gate1 IN | Closest buoy [m] | max \|e\| [m] | e_rms [m] | Reverse [%] | Min F [N] |
     |---|---|---|---|---|---|---|---|---|---|---|
     | task1_m19_1 | 64.60 | −0.000 | −0.002 | −0.175 | +0.022 | 0.674 (gate2_green) | 1.668 | 0.518 | 50.0 | −22.7 |
     | task1_m19_2 | 64.60 | −0.000 | −0.002 | −0.175 | +0.021 | 0.675 (gate2_green) | 1.668 | 0.518 | 50.0 | −22.7 |
     | task1_m19_3 | 64.60 | −0.000 | −0.002 | −0.175 | +0.021 | 0.675 (gate2_green) | 1.669 | 0.518 | 50.0 | −22.7 |

   - Change against M1.5: time +0.10 s (2 control ticks), closest buoy
     +3–4 mm, max |e| +2.3 cm, e_rms +4 mm, gate2 IN offset +4 mm; node vs.
     recomputed offsets ≤ 0.0005 m. Inside the brief's "mm–cm".
   - Note: my first run script left `ros2 launch` processes alive between runs
     (`pkill -INT` did not end them), so runs 2 and 3 recorded no odometry the
     first time; rerun with a hard cleanup (`ws/data/m19_runs.sh`, not
     committed). Only the clean runs are in the table.
3. **What could not be checked:** the Gazebo window (all runs headless).
4. **How to read the code:** `Route::measure()` is the §5.1 projection
   (s = (p − W_i)·t̂_i, e = (p − W_i)·n̂_i); `Route::update()` is the switch
   rule; `LosGuidance::compute()` and `remaining_distance()` are unchanged and
   now always see the current segment's s.
5. **Numbers for the thesis** [MEASURED, bags `ws/data/bags/task1_m19_{1,2,3}`]:
   Task 1 run time 64.60 s; closest buoy 0.674–0.675 m (gate2_green); max |e|
   1.668–1.669 m; e_rms 0.518 m; reverse thrust 50.0 % of RUNNING commands;
   gate offsets in the table. Parameters unchanged (§6).
6. **Check your understanding**
   1. Why is s usually negative right after a switch, and why is that fine for
      LOS and d_end?
   2. Why does `update()` not test for a second switch after measuring on the
      new segment?
   3. Why did the run time change by exactly a multiple of 0.05 s?

## 8. Known risks and open points

| Risk | Mitigation |
|---|---|
| No C++ code existed before Step 2 (found 2026-10-10; older notes said `mix()` existed) | Step 2 writes `mix()` and sets up gtest before anything builds on it |
| Steering sign wrong | Step 1 measured it (confirmed); the reverse-thrust case is open (10.2) |
| Turn too tight for the boat at 0.6 m/s | pods alone turn with a radius of about 6.4–7 m, the teardrop needs about 2.4–3.2 m (10.4), so spin assist is the main way to turn, not a fallback. Widening the teardrop (move W3–W5 east, max x ≈ 27 to keep 3 m from Task 3) adds roughly 1 m of radius [EST], so spin assist stays needed |
| Waves disturb tracking | start with the world's small default waves; increase `gain` later as a robustness test |
| Uncalibrated drag → feedforward off | Step 1 showed the drag model matches Gazebo within 0.1 %; the PI term corrects the rest |
| Ground truth is "too perfect" | intended (master plan idea 3); the optional M7.3 swaps in `/asv/slam/odom` via `odom_topic` |
| Buoys float freely (no mooring), gate checks use nominal positions | distances are logged against nominal positions; the true buoy drift is measured in M2.7 |

## 9. Out of scope (later milestones)
Recording datasets (master plan M2.9), the filter (M3–M4), detecting the gates
with the sensors instead of using known positions (M5, only if time), obstacle avoidance,
other tasks. A Foxglove dashboard and shore-link topics were planned here for
Milestones 2–3; Marcell decided on 2026-10-11 to use RViz for live data (D18 a),
so they are only the optional, unplanned step M7.6.

## 10. Open points for the Step 2 brief (found 2026-10-10)

Found while checking this plan against the master plan and the repository
(commit 6b8be2b). Numbers are [EST] from the drag model unless marked. The
M1.2 brief lists each point with options and a recommendation; Marcell
decides.

1. **`mix()` specification.** Section 5.5 is a proposal. Still open: what
   happens when throttle ± spin saturates (keep the spin difference and lower
   both, or clip each side), and whether throttle and spin are fractions of
   `max_thrust` (as proposed) or newtons.
   **decided 2026-10-18:** fractions of `max_thrust`; on saturation keep the
   left/right difference and lower both sides ("turning wins", option b).
2. **Steering with reverse thrust.** The yaw moment of the pods is
   $-0.70\,(T_L + T_R)\sin\delta + 0.33\,(T_R - T_L)\cos\delta$
   ($\delta$ = pod angle; pods 0.70 m behind base_link, which is close to
   the centre of gravity, and 0.33 m to each side [REPO `asv.yaml`]). With
   negative mean thrust the pods turn the boat the wrong way. This happens in normal operation: with $u_\text{cmd} = u_i
   \cos\tilde\psi$ and $K_P = 0.3$, at 0.6 m/s the throttle is negative for
   $|\tilde\psi| \gtrsim 0.62$ rad (36°), before the integral term (−0.011 at
   0.7 rad, −0.063 at 1.0 rad), and every teardrop corner turns by 53–65°.
   Below the spin threshold (0.8 rad) nothing else turns the boat. Options:
   a) steer = 0 when throttle ≤ 0; b) pod angle = −steer·sign(throttle) in
   `mix()`; c) throttle ≥ 0 while turning (|ψ̃| above a threshold).
   Recommended: c (the pods always push forward, so the sign rule always holds;
   a and b switch at throttle ≈ 0 and can chatter). Needs a unit test.
   **decided 2026-10-18:** c (built in SpeedControl, M1.2c); `mix()` keeps
   pod angle = −steer.
3. **Spin assist is on/off.** At the threshold, $K_\text{spin}\psi_\text{spin}
   = 0.5 \cdot 0.8 = 0.4$ = spin_max, so spin jumps from 0 to ±0.4 (±20 N per
   side) and can chatter around |ψ̃| = 0.8 rad. Proposed: spin =
   sign(ψ̃)·clamp(K_spin(|ψ̃| − ψ_spin), 0, spin_max), with a new starting
   value for K_spin, and a unit test that spin is continuous at the threshold.
   **decided 2026-10-18:** b, ramp from the threshold with K_spin = 1.0 [EST]
   (no reason given); with 10.2 c the turn threshold is ψ_turn = 0.3 rad
   [EST] (M1.2c).
4. **The teardrop needs spin.** The four loop corners (W2–W5) turn the heading
   by 233°. Spread over the path from W2 to W6 (12.95 m) that is a mean turning
   radius of about 3.2 m; over W2 to W5 (9.6 m) about 2.4 m. So the boat needs
   about 0.19–0.25 rad/s at 0.6 m/s. With the pods alone at max_steer 0.8 rad
   the steady turning radius is about 6.4 m if the speed PI holds 0.6 m/s, or
   about 7 m if the thrust stays at the feedforward value (the boat then slows
   to about 0.47 m/s). So spin assist is part of every loop corner (risk
   table).
   **decided 2026-10-19:** if the closed-loop model test (M1.2e) fails the
   teardrop with the §6 starting gains, stop and report; gain or route changes
   are a separate decision (no reason given).
5. **Calibrated test model for Step 2.** Instead of "yaw rate ∝ steer·u +
   spin" (an arbitrary gain that can pass with any controller), use the
   steady-state yaw balance $n_R\,\omega + n_{RR}\,\omega|\omega|$ = yaw
   moment of point 2, with $n_R = n_{RR} = 32$ [REPO `asv.yaml`]. It reproduces
   Step 1: 0.148 rad/s for the pod test (measured 0.15) and 0.175 rad/s for
   the differential test (measured 0.18). Optionally a first-order lag on ω.
   **decided 2026-10-19:** a 3-DOF model (surge, sway, yaw) with Gazebo's
   own mass, inertia and drag, read from `asv.yaml` and `asv_report.json`
   (yaml-cpp, new `<test_depend>`); the generator writes the whole-boat inertia
   into the report. No reason given. Correction found while planning: the
   2.81 kg m² in `model.sdf` is base_link alone; the pods are separate links.
6. **Lookahead vs. short segments.** Δ = 3 m is about as long as the loop
   segments (2.06–3.61 m; only W7–W8 is shorter than 3 m), and with R_switch = 1.5 m each loop
   segment is followed for only about 0.6–2 m. At a switch the target point
   $W_i + (s + \Delta)\hat t$ lies on the segment's extension, up to about
   1.4 m off the route. Options: carry the leftover lookahead onto the next
   segment (target stays on the route; recommended), or a smaller Δ in the
   loop.
   **decided 2026-10-18:** a, carry the lookahead onto the next segments
   (M1.2b).
7. **d_end and FINISHED.** $d_\text{end}$ is not defined. As a straight line
   to W9 it would slow the start, because the boat spawns 1 m from W9 =
   (−1, 0). Proposed: $d_\text{end}$ = remaining distance along the route
   ($\ell_i - s$ plus the later segments). Also say when FINISHED fires: on
   the last segment when $s \ge \ell_8 - R_\text{switch}$ (x ≤ 0.5 m), or at
   W9.
   **decided 2026-10-18:** a, $d_\text{end}$ along the route; FINISHED on the
   last segment when $s \ge \ell_8 - R_\text{switch}$ (M1.2b).
8. **Parameter sources.** `boat_half_width` (0.44 m = 0.66/2 + 0.11 from
   `asv.yaml`) should be computed by the launch file like the drag values;
   `gate_coords` and `buoy_radius` repeat the world and model files and can
   come from the ground-truth buoy file once M2.7 exists. Every parameter gets
   a source tag (CLAUDE.md): [EST] for the starting gains, [REPO] for the
   geometry.
   **decided 2026-10-19:** a, the launch file computes `boat_half_width`
   (0.66/2 + 0.11 = 0.44 m) and `boat_length` (1.60 m) from `asv.yaml`; gates
   and buoys (with one radius each) are in `task1_route.yaml` [REPO] until
   M2.7, then come from `buoys_gt.yaml` (no reason given).
9. **Boat shape in the gate monitor (found 2026-10-19).** Section 5.6 used a
   circle of radius `boat_half_width`, which misses the bow and stern (hulls
   1.60 m long). Options: a) offset band for gate pass/fail, the 1.60 × 0.88 m
   rectangle turned by ψ for the clearance; b) circle for both; c) rectangle
   for both. **decided 2026-10-19:** a (no reason given).
10. **Crossing outside the buoys (found 2026-10-19).** Section 5.6 only
    counted crossings between the buoy centres, so going round a buoy gave no
    event. Options: a) report crossings of the extended gate line up to
    `gate_capture` (2 m [EST]) beyond each buoy, FAIL_OUTSIDE when outside;
    b) only between the buoys. **decided 2026-10-19:** a (no reason given).
11. **Buoys in the clearance check (found 2026-10-19).** Options: a) the 4
    gate buoys; b) also the nearby Task 2 channel gate 1 and Task 3 gate
    buoys, typed into `task1_route.yaml`. **decided 2026-10-19:** b (no
    reason given). These buoys have other radii (A0 0.1015 m, A2 0.184 m
    [REPO `models/rb26_*/model.sdf`]), so every buoy has its own radius.
12. **Sideways drag (found 2026-10-19, M1.2e).** The closed-loop test
    failed: in every turn the boat slid outward (|e| up to 7–10 m). The
    3-DOF model and Gazebo agree: 20 N, pods −0.4 rad → u = 1.282, v = −1.252,
    ω = 0.1498 in Gazebo [MEASURED] vs 1.285, −1.250, 0.1484 in the model.
    Cause: `yVV` = 4 (copied from VRX roboboat01, uncalibrated) is no larger
    than the surge drag. Options: a) a physically based sway drag in
    `asv.yaml`; b) keep it and steer more with spin; c) both.
    **decided 2026-10-19:** a, "If the current number is only an estimate,
    then a is the clear choice"; yVV = ½ ρ C_d A_lat = ½·1000·0.8·0.146
    ≈ 58 N/(m/s)² (A_lat = 2 hulls × 1.60 m × draft 0.0455 m [REPO];
    C_d = 0.8 [EST, range 0.6–1.0, literature value not yet checked]).
    yV = 4, nR = nRR = 32 and the surge drag are unchanged.
    Check after regenerating: Gazebo u = 1.396, v = −0.414, ω = 0.1499
    [MEASURED] vs model 1.397, −0.413, 0.1484.
13. **Crabbing on straight legs (found 2026-10-19, M1.2e).** A small pod
    angle turns the nose towards the path but pushes the boat sideways away
    from it; the heading error stays small while |e| grows (0.42 m from a
    0.5 m start). Options: a) course feedback, χ = ψ + atan2(v, u) instead of
    ψ in the heading controller (Fossen, course-based LOS); b) integral LOS;
    c) nothing, tune in M1.5. **decided 2026-10-19:** a (no reason given).
14. **Too little turning force in corners (found 2026-10-19, M1.2e; the weak
    spot of M1.2c).** For ψ_turn < |ψ̃| < ψ_spin the throttle drops to about
    0.04 and spin is still off (ω ≈ 0.05 rad/s). Options: a) ψ_spin = 0.3 rad
    (= ψ_turn) and K_spin = 2.0 [EST]; b) a minimum throttle while turning;
    c) leave it to M1.5. **decided 2026-10-19:** a (no reason given).
15. **W0 for off-spawn starts (found 2026-10-19, M1.2e).** With W0 = the
    start point, a start 0.5 m to the side puts the route itself 0.35 m off
    the gate 1 centre. Options: a) W0 is the fixed spawn (0, 0), in the
    parameter file; b) W0 = the start point. **decided 2026-10-19:** a (no
    reason given). M1.2f builds 10.13–10.15; M1.2e then runs again.
16. **Cross-track swing after an offset start (found 2026-10-19, M1.2f).**
    With 10.13–10.15 built, the spawn run passed but the four offset runs
    crossed gate 2 outbound at ±0.58 m (FAIL_CLOSE, band ±0.355 m): e swung
    +0.65 → −0.58 → +0.59 m with a period of about 11 s [MEASURED, model].
    Cause: the aft pods first push the boat sideways the wrong way
    (non-minimum-phase). A sweep of Δ (3/4/5 m), K_ω (0.6/1.2) and K_ψ
    (0.8/1.2) found nothing better; with ψ instead of χ every run failed and
    collided. Options: a) D term on the course rate χ̇; b) proportional spin
    K_s ψ̃ at every error size; c) smaller start offsets (weakens the test);
    d) leave it to M1.5. a was tried in a scratch copy and was worse (9–18
    of 20 gate events failed). b: K_s = 0 / 0.25 / 0.5 / 1.0 / 1.5 / 2.0 / 3.0
    gave 4 / 0 / 2 / 0 / 0 / 0 / 0 failed events, worst offset 0.583 / 0.353 /
    0.374 / 0.283 / 0.196 / 0.155 / 0.171 m [MEASURED, model]. **decided
    2026-10-19:** b with K_s = 2.0 ("a, K_s = 2.0, build it", picking
    K_s = 2.0 from the list 2.0 / 1.0 / none; no reason given). Risk: one thruster
    runs in reverse in about 63 % of the control steps (39 % before), worst
    about −23 N; in Gazebo reverse thrust may be weaker and flips the pod
    steering (10.2) — check in M1.3/M1.5.
