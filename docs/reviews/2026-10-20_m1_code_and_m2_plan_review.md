# Milestone 1 Code and Milestone 2+ Plan Review (2026-10-20)

Review of the Gazebo Harmonic + VRX + ROS 2 Jazzy simulation codebase for bachelor thesis: GNSS-aided EKF-SLAM on an autonomous catamaran.

> **Scope Note:** Points already raised in `docs/reviews/2026-10-18_gemini_master_plan_review.md` (such as map consistency definition, D5 C outage motion kinematics, filter initialisation $x_0/P_0$, GNSS-only baseline, sweep divergence policy, and general date schedules) are omitted here to focus on new, unresolved technical issues.

---

## Part A: State of the Milestone 1 Code (C++ and Python)

### 1. Severe guidance glitch on segment transition
- **File & Function:** `ws/src/asv_sim/src/guidance.cpp`, `Route::update`
- **Quote:**
  ```cpp
  if (along_ >= length_[segment_] - switch_distance_) {
    if (segment_ + 1 < length_.size()) {
      ++segment_;
      // s and e on the new segment are computed on the next call.
  ```
- **Severity:** High
- **Issue:** When a waypoint switch occurs, `segment_` is incremented, but `along_` and `cross_` retain the coordinates from the *previous* segment ($s \approx L_{\text{old}} - R_{\text{switch}}$). Immediately following this call, `Follower::step` (`ws/src/asv_sim/src/follower.cpp`) invokes `los_.compute(route_, nav->p)`. In `LosGuidance::compute`, `i` is the new segment index, but `route.along_track()` is the old segment's along-track coordinate ($L_{\text{old}} - R_{\text{switch}}$). As a result, $d = \text{along} + \Delta$ projects forward from that large offset on the new segment, causing the lookahead target to skip far down the new segment (or past it) for exactly one control cycle before snapping back on the next tick. In the same tick, `Route::remaining_distance` computes `length_[segment_] - along_`, subtracting the old along-track coordinate from the new segment's length, which can produce a negative distance and cause an abrupt deceleration glitch.
- **Concrete Fix:** Immediately upon incrementing `segment_`, recompute `along_` and `cross_` against `waypoints_[segment_]` and `tangent_[segment_]` before exiting `Route::update`.

---

### 2. Guidance unit test gap on segment transition step
- **File & Function:** `ws/src/asv_sim/test/test_guidance.cpp`, `advance_to_segment`
- **Quote:**
  ```cpp
  void advance_to_segment(Route & route, std::size_t segment)
  {
    for (std::size_t k = 1; k <= segment; ++k) {
      route.update(kTask1[k]);
    }
    ASSERT_EQ(route.segment(), segment);
  }
  ```
- **Severity:** Medium
- **Issue:** None of the unit tests in `test_guidance.cpp` invoke `los.compute()` or inspect `remaining_distance()` on the exact call where `route.update()` trips a segment switch. In `advance_to_segment`, waypoints are advanced in a loop, and test cases (such as `EqualsFossenFormOnAStraightSegment`) invoke a secondary `route.update(p)` before testing LOS, which masked Bug 1 during test execution.
- **Concrete Fix:** Add a unit test feeding a point that crosses the switch threshold ($s \ge L - R_{\text{switch}}$), followed immediately in the same cycle by `los.compute()` and `remaining_distance()`, verifying that `out.target` and remaining distance transition smoothly.

---

### 3. Conflation of course error and heading error in speed control and feedback
- **File & Function:** `ws/src/asv_sim/src/follower.cpp`, `Follower::step`
- **Quote:**
  ```cpp
  const double chi = course_angle(nav->heading, nav->u, nav->v, config_.course_min_speed);
  const HeadingCommand hc = heading_.compute(target.desired_heading, chi, nav->yaw_rate);
  const double u_cmd = speed_.commanded_speed(
    config_.speeds[route_.segment()], hc.heading_error, route_.remaining_distance());
  ```
- **Severity:** Medium
- **Issue:** `chi` (course angle $\chi$) is passed as the `heading` parameter into `HeadingControl::compute`. Inside, `cmd.heading_error = wrap_angle(desired_heading - heading)`, meaning `hc.heading_error` is actually the *course error* $\tilde{\chi} = \psi_d - \chi$, not physical heading error $\tilde{\psi}$. Passing this into `speed_.commanded_speed` ($\cos(\tilde{\chi})$) and `SpeedControl::update` ($|\tilde{\chi}| > \psi_{\text{turn}}$) means forward speed throttling is governed by trajectory sideslip rather than physical bow alignment. Additionally, in `HeadingControl::compute`, the D-term is $-K_\omega \omega$ (damping physical yaw rate $\dot{\psi}$), mixing course P-action with heading-rate D-action without structural separation.
- **Concrete Fix:** Explicitly separate physical heading error $\tilde{\psi} = \text{wrap}(\psi_d - \psi)$ from course error $\tilde{\chi} = \text{wrap}(\psi_d - \chi)$ in `Follower::step`. Feed $\tilde{\psi}$ into `speed_.commanded_speed` and `speed_.update` turn thresholds, while retaining $\tilde{\chi}$ for the course steering law.

---

### 4. Unbounded / unhandled dt under clock jitter or pauses
- **File & Function:** `ws/src/asv_sim/src/follower.cpp`, `Follower::step`
- **Quote:**
  ```cpp
  const double dt = control_stamp_ ? nav->stamp - *control_stamp_ : 0.0;
  control_stamp_ = nav->stamp;
  const double throttle = speed_.update(u_cmd, nav->u, hc.heading_error, dt);
  ```
- **Severity:** Medium
- **Issue:** In `waypoint_follower.cpp`, the control step runs on an independent 20 Hz ROS timer pulling the latest cached `odom_`. When timer jitter samples the same message twice, `nav->stamp == *control_stamp_`, causing `dt = 0.0` (skipping integration), followed by a doubled step on the next tick. If Gazebo pauses or stutters, `dt` can span multiple seconds, causing a massive leap in `integral += error * dt` in `SpeedControl::update`. Furthermore, if an out-of-order stamp arrives, `dt < 0.0` skips integration but updates `control_stamp_` anyway.
- **Concrete Fix:** Clamp `dt` to a valid operating envelope:
  ```cpp
  if (dt <= 0.0 || dt > 2.0 / config_.control_rate) { dt = 0.0; }
  ```
  Only update `control_stamp_ = nav->stamp` when `nav->stamp > *control_stamp_`.

---

### 5. Hardcoded frame assumptions hindering M2/M3 estimator integration
- **File & Function:** `ws/src/asv_sim/src/waypoint_follower.cpp`, `WaypointFollower::to_nav` and `kWorldFrame`
- **Quote:**
  ```cpp
  constexpr char kWorldFrame[] = "world";
  ...
  return asv_sim::NavState{
    rclcpp::Time(m.header.stamp).seconds(),
    {m.pose.pose.position.x, m.pose.pose.position.y},
    asv_sim::yaw_from_quaternion(q.w, q.x, q.y, q.z),
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z};
  ```
- **Severity:** Medium
- **Issue:** When switching from ground truth to `/asv/slam/odom` or external baselines, the estimator publishes in frame `map`, not `world`. `kWorldFrame` is hardcoded as the frame ID for published path and targets. Moreover, `to_nav` assumes `m.twist.twist.linear.x/y` is body-frame surge $u$ and sway $v$ without verifying `child_frame_id == "base_link"`. If a baseline or filter variant outputs world-frame twist, `to_nav` will feed world velocities directly into body speed and course controllers.
- **Concrete Fix:** Make the reference frame a ROS parameter (defaulting to `world`, switchable to `map`). In `to_nav()`, assert that `m.child_frame_id == "base_link"`; if not, project linear velocities into body coordinates using heading $\psi$.

---

### 6. Inconsistent parameter ingestion architecture across M1 nodes
- **File & Function:** `ws/src/asv_sim/src/waypoint_follower.cpp`, `load_config` vs `ws/src/asv_sim/src/teleop_node.cpp`
- **Quote (`teleop_node.cpp`):**
  ```cpp
  const YAML::Node thr = YAML::LoadFile(config + "/asv.yaml")["thrusters"];
  ```
- **Severity:** Low
- **Issue:** Three disparate parameter loading mechanisms exist across M1: `waypoint_follower` declares 25+ ROS parameters and unpacks flattened arrays; `teleop_node` reads YAML files directly from share via `yaml-cpp`; and `maneuver_player` reads flat vectors via ROS parameters. In M2.1 (`asv_core`), having no unified ROS-free configuration struct and YAML parser will complicate headless replays (`slam_replay` in M3.5) and evaluation runners.
- **Concrete Fix:** Provide plain C++ configuration structs with `yaml-cpp` deserializers in `asv_core` for boat limits, routes, and course definitions so offline test harnesses, `slam_replay`, and ROS nodes use identical configuration models.

---

### 7. Duplicate timestamp division-by-zero vulnerability in Python evaluation
- **File & Function:** `ws/src/asv_sim/tools/compare_runs.py`, `interpolate`
- **Quote:**
  ```python
  k = bisect.bisect_left(taus, tau)
  if taus[k] == tau:
      return trk[k][1:]
  a, b = trk[k - 1], trk[k]
  w = (tau - a[0]) / (b[0] - a[0])
  ```
- **Severity:** Low
- **Issue:** If a recorded rosbag contains two odometry messages with identical simulation timestamps (common with multi-rate publishers or paused Gazebo clocks), `poses = sorted(poses)` can yield consecutive samples where $b[0] == a[0]$. If `tau` falls across such an interval, `w = (tau - a[0]) / (b[0] - a[0])` triggers `ZeroDivisionError`.
- **Concrete Fix:** Filter duplicate timestamps during extraction in `analyze_run.py:odom_poses`: skip appending any pose whose timestamp matches the preceding pose.

---

## Part B: The Plan for Milestone 2 Onward

### 1. Package boundary and type dependency clash between `asv_core` and `asv_msgs`
- **File & Section:** `docs/design.md`, §12 Packages & `docs/master_plan.md`, M2.1, M2.4, M3.1
- **Quote (`design.md`):**
  ```markdown
  asv_core (new, D13) | ament_cmake, no ROS | geodesy, noise models, fake detector logic, measurement queue, EKF-SLAM...
  ```
- **Severity:** High
- **Issue:** `asv_core` is defined as completely ROS-free, depending only on Eigen. However, `design.md` §12 places the "fake detector logic" and "measurement queue" inside `asv_core`. In M2.4, `asv_msgs` defines ROS 2 IDL messages (`BuoyDetectionArray`, `LandmarkArray`). If `asv_core` implements fake detector logic and measurement queuing, it cannot use `asv_msgs` types without violating the "no ROS" requirement.
- **Concrete Fix:** Define canonical, ROS-free C++ structs in `asv_core` (e.g., `asv::core::BuoyDetection`, `asv::core::Landmark`, `asv::core::MeasurementQueue<T>`). In `asv_sim` and `asv_slam`, implement lightweight adapter functions that convert between `asv_msgs` and `asv_core` structs.

---

### 2. Clean GNSS velocity generation mechanism and UM982 specification
- **File & Section:** `docs/design.md`, §1 Table & §4.2
- **Quote (`design.md`):**
  ```markdown
  /asv/sensors/gnss/velocity | geometry_msgs/TwistWithCovarianceStamped (ENU) | 10 Hz | corruptor | realistic (D5 C: prediction input)
  ```
- **Severity:** Medium
- **Issue:** D5 C requires GNSS velocity to drive EKF prediction. However, Gazebo Harmonic's NavSat sensor plugin does not produce velocity (it only outputs `sensor_msgs/NavSatFix`). The corruptor must therefore synthesize clean GNSS velocity from `/asv/ground_truth/odom` (by transforming ground truth linear velocity to ENU and applying Doppler noise). Consequently, clean bag recordings (M2.9 `nav` profile) *must* record `/asv/ground_truth/odom` to enable replay. Furthermore, the UM982 Doppler velocity noise is still marked `[DS, to check]` in `design.md` §4.2.
- **Concrete Fix:** Document in `design.md` §1 and §4.2 that the corruptor derives GNSS velocity from `/asv/ground_truth/odom`, ensure `/asv/ground_truth/odom` is included in the M2.9 `nav` recording profile, and confirm the UM982 velocity noise parameter before implementing M2.6.

---

### 3. Step ordering: unblocking M2.4 and M2.6 via decision D11
- **File & Section:** `docs/master_plan.md`, §4 M2.4 & M2.6
- **Quote (`master_plan.md`):**
  ```markdown
  [?] M2.4 | asv_msgs package ... separate roll/pitch message only if the D11 choice needs one.
  ```
- **Severity:** Medium
- **Issue:** M2.4 (`asv_msgs`) and M2.6 (corruptor node) are marked `[?]` waiting for decision D11 (IMU message content: A vs B vs C). Because M2.6 corruptor publishes custom messages (`asv_msgs/GnssHeading`, `asv_msgs/SensorStatus`), leaving D11 open stalls message definition and corruptor development.
- **Concrete Fix:** Formally close D11 with Option A (nominal = ideal: white noise + turn-on bias only). Since D5 C is planar, no separate roll/pitch message is required. Move M2.4 immediately after M2.1 so message interfaces are stable before implementing node publishers.

---

### 4. Concrete scope pruning for a single bachelor student
- **File & Section:** `docs/master_plan.md`, §3 D6, D11 & §4 M3.5, M5
- **Quote (`master_plan.md`):**
  ```markdown
  [?] M3.3b | EKF core, part 2 (D6...) ... M3.5 | slam_replay ... <depend> on rosbag2_cpp
  ```
- **Severity:** Medium
- **Issue:** Beyond general workload concerns, several specific components add disproportionate complexity without advancing the core research hypothesis:
  1. D6 b introduces an augmented 2-state Gauss-Markov estimator that significantly complicates the EKF formulation and tuning.
  2. D11 A proposes digital filter emulation (4 Hz low-pass, 50 Hz hold, publish delay) in the corruptor.
  3. M3.5 specifies a custom C++ `rosbag2_cpp` replay engine with complex type support and deserialization.
  4. M5 specifies a full point-cloud clustering and camera HSV fusion pipeline.
- **Concrete Fix:**
  - **D11:** Commit strictly to Option A, nominal = ideal (white noise + bias only). Drop ArduPilot software delay/hold emulation.
  - **D6:** Lock in Option a (covariance inflation per GNSS mode) for the core filter. Postpone Option b (augmented state) to optional M7.2.
  - **M5:** Officially designate M5 (real perception) as out-of-scope; use the fake detector exclusively for all thesis evaluation runs.
  - **M3.5 Replay:** Consider whether reading bags via Python (`rosbag2_py`) or serializing preprocessed sensor streams is faster to implement than a bespoke C++ `rosbag2_cpp` replay node.
