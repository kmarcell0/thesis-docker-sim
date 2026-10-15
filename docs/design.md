# Design reference

This file explains **how** each part of the plan in [`master_plan.md`](master_plan.md)
works. Symbols and terms are explained in [`glossary.md`](glossary.md).

Source tags for numbers: **[DS]** datasheet, **[EST]** estimate (needs a source or
a reason), **[REPO]** this repository, **[SRC]** Gazebo or ROS source code,
**[LIT]** a paper (bib key), **[MEASURED]** measured in a run here.

Decisions D1-D18 are in the master plan, section 3. Marcell decided most of them
on 2026-10-11; what follows from them is written here as rules. Where a design
still depends on an open decision (D6, D10, the IMU message content of D11),
the recommended option is described and the decision ID is given; that part is a
proposal, not a rule, until Marcell decides.

---

## 1. Topics

Rule (what it protects: the filter must never see perfect data):

- **Read only `/asv/sensors/...` and `/asv/perception/...`:** the filter (the EKF
  core, slam_node, and the filter stage of slam_replay) and any baseline. Their
  default topic names point there. They never read the clean sensor topics
  (`/asv/imu/data`, `/asv/gnss/...`, `/asv/lidar/points`) or
  `/asv/ground_truth/...`.
- **Read the clean and ground-truth topics on purpose:** the corruptor, the fake detector, the evaluation, the
  ground-truth TF node, the waypoint follower while it drives on ground truth
  (master plan idea 3), and slam_replay's input stage (it
  runs the corruptor and fake detector itself) and its logging of true values.

D3 b (decided 2026-10-11): the bridge keeps its normal names, so the clean data
from Gazebo is on `/asv/imu/data`, `/asv/gnss/fix`, `/asv/lidar/points`, and the
corruptor publishes realistic data under `/asv/sensors/...`. The cameras are not
corrupted by our node, so they keep their names. The price of D3 b: a node left on
a normal name gets perfect data without any warning. Two guards: the filter's
default topic names are the `/asv/sensors/...` ones, and the M4.1 check confirms
with `ros2 node info` that none of slam_node's subscriptions is a clean sensor
topic or a `/asv/ground_truth/...` topic. On the real boat, the drivers' topics (e.g.
ArduPilot's `/ap/...`) are remapped to the `/asv/sensors/...` names. For the IMU
a rename is not enough: ArduPilot publishes it in a NED/FRD frame, with the
orientation quaternion's components in the wrong order, no covariances and
publish-time stamps [SRC AP_DDS], so the real boat needs a small converter node
(future work, not in this plan).

| Topic | Type | Rate | Published by | Kind |
|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | every physics step | bridge | time |
| `/asv/ground_truth/odom` | nav_msgs/Odometry | 50 Hz [REPO] | bridge (Gazebo OdometryPublisher) | ground truth |
| `/asv/ground_truth/buoys` | asv_msgs/LandmarkArray (latched) | once | ground-truth tool (M2.7) | ground truth |
| `/asv/ground_truth/gnss_status` | asv_msgs/SensorStatus | 10 Hz | corruptor | ground truth (true fix mode, outage flag) |
| `/asv/imu/data` | sensor_msgs/Imu | 200 Hz [REPO] | bridge | clean after M2.2 (orientation perfect); until then it carries Gazebo's IMU noise |
| `/asv/gnss/fix` | sensor_msgs/NavSatFix | 10 Hz [REPO] | bridge | clean |
| `/asv/gnss/secondary/fix` | sensor_msgs/NavSatFix | 10 Hz [REPO] | bridge | clean (only for checks) |
| `/asv/lidar/points` | sensor_msgs/PointCloud2 | 10 Hz [REPO] | bridge | clean |
| `/asv/sensors/imu/data` | sensor_msgs/Imu | 200 Hz | corruptor | realistic; what it carries is D11 (§4.1) |
| `/asv/sensors/gnss/fix` | sensor_msgs/NavSatFix | 10 Hz | corruptor | realistic |
| `/asv/sensors/gnss/heading` | asv_msgs/GnssHeading | 10 Hz | corruptor | realistic |
| `/asv/sensors/gnss/velocity` | geometry_msgs/TwistWithCovarianceStamped (ENU) | 10 Hz | corruptor | realistic (D5 C: prediction input) |
| `/asv/sensors/lidar/points` | sensor_msgs/PointCloud2 | 10 Hz | corruptor | realistic (only needed for real perception, M5) |
| `/asv/zed/left/image_raw`, `.../camera_info` (and right, depth) | sensor_msgs/Image, CameraInfo | 15 / 10 Hz [REPO] | bridge | camera |
| `/asv/cam_wide_front/image_raw`, `.../camera_info` | sensor_msgs/Image, CameraInfo | 30 Hz [REPO] | bridge | camera |
| `/asv/perception/buoys` | asv_msgs/BuoyDetectionArray | 10 Hz | fake detector or perception | measurements |
| `/asv/slam/odom` | nav_msgs/Odometry | filter rate | slam_node | estimate |
| `/asv/slam/map` | asv_msgs/LandmarkArray | 10 Hz | slam_node | estimate |
| `/asv/slam/markers`, `/asv/slam/path` | visualization_msgs/MarkerArray, nav_msgs/Path | 10 Hz | slam_node | display |
| `/asv/thrusters/{left,right}/{thrust,pos}` | std_msgs/Float64 | controller rate | follower, player or teleop | commands |
| `/asv/autonomy/path` | nav_msgs/Path (latched: transient local) | once | waypoint follower (M1.3) | display |
| `/asv/autonomy/target` | geometry_msgs/PointStamped | controller rate | waypoint follower | display |
| `/asv/autonomy/status` | std_msgs/String | on change | waypoint follower | state (`WAITING`, `RUNNING 1/9` … `RUNNING 9/9`, `FINISHED`, `STOPPED: <reason>`, gate events) |
| `/asv/autonomy/cross_track_error` | std_msgs/Float64 (m) | controller rate | waypoint follower | for plots |

Thruster command limits [REPO `asv.yaml`]: thrust −40 N to +50 N per thruster,
pod angle ±1.5708 rad (±90°), pod turn rate at most 2 rad/s. `mix()` clamps
the thrust and the pod angle to these values; they reach it as parameters taken
from `asv.yaml`, not typed into the code. Gazebo enforces the turn rate
(`cmd_max` in `model.sdf`).

`/asv/slam/odom` follows the same contract as ground truth, so the waypoint
follower can switch to it later: `header.frame_id = map`, `child_frame_id =
base_link`, twist in the **body** frame (twist.linear.x = forward speed). If the
state has no velocity (D5 C, decided 2026-10-14), so slam_node fills the twist
from its inputs: the GNSS velocity rotated into the body frame with the
estimated heading, and the yaw rate ω_m − b_g. The follower can then switch to
it (optional M7.3).

The ground-truth twist is fine as feedback for the controller (M1 uses it), but
not as true velocity for evaluation: Gazebo smooths it (§10).

## 2. Frames

| Frame | Meaning | Who publishes the transform |
|---|---|---|
| `world` | Gazebo's world frame (the thesis's 𝒲): east-north-up (ENU), origin at the world's reference point (`<spherical_coordinates>` in `roboboat_course.sdf`: 27.367256° N, 82.451120° W, i.e. `longitude_deg` −82.451120; elevation 0; heading 0) [REPO]. Ground truth uses it. | (root) |
| `map` | The filter's frame. Identical to `world`: same origin, same axes. | static identity transform world → map (launch file) |
| `base_link` | Boat body (ℬ): x forward, y left, z up; origin midway between the hulls, at hull-axis height, at mid hull length [REPO]. | **exactly one** node: the ground-truth TF node (world → base_link) when the filter is off, or slam_node (map → base_link) when it is on. Never both: a frame may have only one parent. |
| sensor frames (𝒮) | all frames in `config/asv_frames.yaml`, e.g. `lidar_link`, `zed_link`, `zed_left_optical`, `zed_right_optical`, `cam_wide_front_link`, `cam_wide_front_optical`, `imu_link`, `gnss_primary_link`, `gnss_secondary_link` | static, from `config/asv_frames.yaml` [REPO] |

Rules:

- All sensor mounting positions (*extrinsics*) are read from `asv_frames.yaml`
  (or TF). They are never typed into code. Key values [REPO]: GNSS primary antenna
  (−0.76, −0.33, 0.185) m, secondary (0.76, 0.33, 0.185) m, IMU (−0.12, 0, 0.19)
  m, lidar (0.25, 0, 0.62) m mounted upside down (roll = π).
- The filter converts latitude/longitude to local ENU with the **world's
  reference point** and the WGS84 ellipsoid (geodesy helper, M2.1). It never
  uses "the first fix" as the origin, because that would shift everything by the
  antenna lever arm (about 0.83 m).
- Angles: yaw ψ is counter-clockwise from east, in (−π, π]. The GNSS heading is an
  azimuth α, clockwise from north. They are related by ψ = π/2 − α, followed by
  wrapping (thesis, Planar Motion). Every angle difference goes through one
  wrapping helper.
- **Levelling** (thesis eq. 3.3, level frame ℋ): first move the point into the
  body frame, then tilt the whole body-frame point:
  p_ℋ = R_y(θ)·R_x(φ)·(R_ℬ𝒮·p_𝒮 + t_ℬ𝒮), as in the draft. Range and bearing are
  then taken from the base_link origin. Keep t_ℬ𝒮 inside the brackets: adding it
  after the tilt would misplace lidar points (0.62 m above base_link) by about
  5.4 cm at 5° tilt. Unit test: the lidar origin (p_𝒮 = 0) with 5° roll and no
  pitch maps to (0.25, −0.054, 0.618) m.
- The upside-down lidar: in `lidar_link`, y and z point the other way than in
  `base_link`; R_ℬ𝒮 from `asv_frames.yaml` takes care of it.

## 3. Time

1. Every node in simulation or replay runs with `use_sim_time:=true`. Each launch
   file sets it for all its nodes at once (`SetParameter`); a node started with
   `ros2 run` gets `--ros-args -p use_sim_time:=true`.
2. Timers use the node's ROS clock: `rclcpp::create_timer(this, get_clock(),
   period, callback)` in C++, `node.create_timer(...)` in Python. Never
   `create_wall_timer`: it always runs on the computer's clock, whatever
   `use_sim_time` says.
3. **When** something was measured always comes from `header.stamp`. The node's
   clock only drives timers.
4. The corruptor copies the input stamp to its output. Time steps (dt), bias
   updates and outage schedules are computed from stamps, never from the nominal
   rate. (The IMU's samples land on 4 ms physics steps, so its intervals repeat
   8, 4, 4, 4 ms: 200 Hz on average, but not a steady 5 ms [SRC gz-sensors
   `Sensor.cc`, REPO world step 0.004 s].)
5. The maneuver player waits for the first `/clock` message and schedules
   everything relative to the first simulation time it sees.
6. Bags are always recorded with `ros2 bag record --use-sim-time ...`, so the
   recording time is simulation time. (rosbag2 writes nothing before the first
   `/clock`.) Thruster commands have no timestamp of their own (std_msgs/Float64),
   so their recording time is their only time.
7. Never run `ros2 bag play --clock` while Gazebo is running (two clocks). When
   the bag already contains `/clock`, play it without `--clock`.
8. slam_replay and slam_node process measurements in `header.stamp` order through
   the measurement queue (§7), not in the order they arrive.
9. **One exception: benchmark timing** (§14; left out for now, D16). CPU time and processing time are
   measured with `std::chrono::steady_clock` (wall time) and CPU-time clocks
   (`CLOCK_THREAD_CPUTIME_ID`, `getrusage`), because simulation time says nothing
   about how long the computer worked. These values are only logged; they never
   change an output, a timer or a dt.

## 4. The corruptor (sensor error models)

The corruptor turns clean data into realistic data. Its maths lives in the
ROS-free package `asv_core` (D13 a) so it can be tested offline and reused by
slam_replay. Nominal values come from `asv.yaml` `sensor_specs` (D2 a keeps them
there). A noise profile (`asv_sim/config/noise_profiles/<name>.yaml`) only holds
multipliers, schedules, overrides and the seed.

### 4.1 IMU

- **White noise** per sample: σ = η / √dt, with η the noise density and dt from
  the stamps. Nominal [DS, `asv.yaml`]: gyro η = 6.632e-5 rad/s/√Hz, accelerometer
  η = 6.865e-4 m/s²/√Hz. At 200 Hz that is 9.4e-4 rad/s (0.054 °/s) and 9.7e-3
  m/s² (0.99 mg), the same as `asv_report.json`. Pending (`decision_notes.md`
  D11): the accelerometer density at ArduPilot's ±32 g range is 110 µg/√Hz
  (1.079e-3 m/s²/√Hz, DS-000577 Table 2), awaiting Marcell's OK; and TDK's own
  definition of the density gives a per-sample σ √2 smaller than this rule. The
  M2.5 brief settles the convention; the η²·f_fuse rule below follows it.
- The clean IMU topic still carries Gazebo's own IMU noise until M2.2 switches
  it off, so M2.2 comes before the IMU corruptor is tested.
- **Turn-on bias**: one draw per run [EST, `asv.yaml`]: gyro σ = 0.0017 rad/s,
  accelerometer σ = 0.02 m/s². It stays constant for the whole run. (Gazebo's IMU
  uses one variable for both biases, so when its wandering bias is switched on,
  the turn-on bias slowly fades away [SRC gz-sensors `GaussianNoiseModel.cc`].
  Ours keeps the two separate, so the turn-on bias stays. The thesis should state
  which model is used.)
- **Wandering bias**: a first-order *Gauss-Markov* process, b(k+1) =
  e^(−dt/T_c)·b(k) + σ_ss·√(1 − e^(−2dt/T_c))·w(k), with w a standard normal draw.
  Steady-state σ_ss [EST, `asv.yaml`]: gyro 0.0002 rad/s (about 41 °/h),
  accelerometer 0.005 m/s²; T_c = 300 s.
- **What the message carries** (D11, open; options in `decision_notes.md`).
  Marcell's direction: it should "follow real world scenarios", i.e. imitate the
  real boat's ArduPilot IMU topic. Proposal (option A): gyro and accelerometer
  with the noise above, then ArduPilot's processing as noise-profile switches
  (low-pass 4 Hz gyro / 10 Hz accel with Rover defaults, values held at the 50 Hz
  loop, stamp = publish time [SRC ArduPilot AP_DDS, AP_InertialSensor]; levels
  ideal / tuned / Rover default). Orientation is not published
  (`orientation_covariance[0] = −1`, "not available"), because the real
  message's yaw is EKF3's, which with GPS-for-yaw contains the same dual-antenna
  heading the filter already uses. Roll and pitch get their own message only if
  something needs them (D5 a, D15 b, M5); their error is [EST], since no
  published ArduPilot attitude accuracy was found. Whatever D11 decides, no
  output carries the true yaw (unit test).
- **Filter noise from the noise density**: if the stream is low-passed and held,
  the filter's gyro noise must come from the density and the rate it fuses at
  (variance = η²·f_fuse), not from the spread of the samples, and repeated
  samples are skipped.
- The corruptor logs every true bias value, so trajectory.csv can hold them
  whatever D5 decides.

### 4.2 GNSS position

The clean fix from Gazebo is the exact position of the **antenna** (not of
base_link). The corruptor converts it to local ENU metres, adds the error in
metres, converts back to latitude/longitude, and publishes a NavSatFix.

| Mode | White σ (horizontal) | Slowly varying error (Gauss-Markov σ_ss, T_c) | Covariance the receiver reports | Source |
|---|---|---|---|---|
| RTK fixed | 0.008 m (+1 ppm) | [EST, to choose in M2.6] | (0.008 m)² | [DS] Unicore UM982 brief, in `asv.yaml` |
| RTK float | [EST] decimetre level | [EST] | [EST] | [LIT] RTK literature (gnss-aided-navigation-notes) |
| Standalone | 1.5 m RMS | [EST]: most of the standalone error varies slowly | (1.5 m)² | [DS] Unicore brief (single point 1.5 m H RMS); `asv.yaml` tags it [EST] |
| Outage | no message published | | | schedule in the noise profile |

- Vertical noise is applied too (1.5 cm RTK [DS]), even though the planar filter
  ignores height.
- The fix quality reaches the filter **only through `position_covariance`** (m²,
  ENU, `COVARIANCE_TYPE_DIAGONAL_KNOWN`), as assumption A4 says. NavSatStatus
  can't express RTK fixed vs float. The true mode goes to
  `/asv/ground_truth/gnss_status` for evaluation only.
- At a mode switch the slowly varying error gets a new draw. That reproduces the
  position jump seen when RTK comes back [LIT `zhang2026impact`].
- How the filter treats this correlated error is decision D6. Two points for
  the M2.6 brief (`decision_notes.md` questions 3 and 5): each mode's 1σ total
  should be split into a white and a slow part (not both at full size), and the
  reported covariance should follow one convention (recommended: the total).
  Whether the slow error gets a new draw when GNSS returns in the same mode after
  an outage is also open.
- D5 C: the corruptor also publishes a noisy GNSS (Doppler) velocity in ENU
  (geometry_msgs/TwistWithCovarianceStamped, `/asv/sensors/gnss/velocity`), from
  ground-truth position differences plus noise. The UM982 velocity accuracy is
  still to be looked up [DS, to check]. In a sky-view outage it stops with the
  position.

### 4.3 Dual-antenna heading

- Output (`asv_msgs/GnssHeading`): azimuth α of the bow in rad, clockwise from
  north, in [0, 2π); its σ; a flag `baseline_fixed`. The 23.47° mounting offset of
  the antenna baseline is assumed to be configured in the receiver, so α is
  already the bow's azimuth.
- α = π/2 − ψ_true (wrapped) + noise, with ψ_true interpolated to the GNSS epoch
  stamp.
- σ (D12 a): 0.1° RMS at 1 m baseline [DS], divided by the 1.657 m baseline
  [REPO] = 0.060°.
- Availability is separate from the position mode: heading stays up when only
  the RTK corrections from the shore base are lost (the two antennas don't need
  the base). It drops during a sky-view outage.
- Optional: a constant error in the configured mounting offset.

### 4.4 Lidar

- Range noise per point along its ray: σ = 0.02 m at 10 m [DS]. Optional angular
  noise (< 0.15° [DS]) and random point dropout.
- Only matters for real perception (M5, which is optional: only if time allows,
  D8). The fake detector does not use the point cloud.
- The simulated lidar can't see the water (the generator sets `visibility_mask 7`
  [REPO]), so there are no water or spray returns. That is a limitation.

### 4.5 Noise library and its tests

- Random numbers: one stream per noise source, seeded from (global seed, source
  name). Changing one sensor's settings then leaves every other sensor's noise
  unchanged ("common random numbers"). The normal distribution is implemented in
  the library rather than taken from `std::normal_distribution`, whose output
  differs between compilers.
- **Tests** (unit tests, no Gazebo, fixed seeds). τ is the Allan averaging time;
  T_c is the Gauss-Markov correlation time from §4.1.
  1. *White noise alone*: the overlapping Allan deviation at τ = 0.1, 1 and 10 s
     equals η/√τ.
  2. *Gauss-Markov alone*, with a short test correlation time (e.g. T_c = 5 s; the
     code is the same for any T_c) and a long record (e.g. 10 h at 200 Hz): the
     sample standard deviation is σ_ss, the lag-k autocorrelation is
     e^(−k·dt/T_c), and the Allan deviation at about 10 values of τ between
     0.5·T_c and 4·T_c follows the analytic curve
     σ_A²(τ) = (σ_ss²·T_c²/τ²)·[2τ/T_c − 3 + 4e^(−τ/T_c) − e^(−2τ/T_c)].
  3. *Both together, nominal values*: the Allan deviation at τ = 1, 10 and 30 s
     equals the analytic total √(η²/τ + σ_A²(τ)). At τ = 1 s the bias part is
     small (it adds 1.0 % for the gyro, 5.7 % for the accelerometer); from τ = 10 s
     on it is larger than the white-noise part (+72 % and +254 %), so an
     implementation that forgets the bias can't pass.
  - Tolerances: about 5 %; the M2.5 brief fixes them from the spread over a few
    seeds. Nothing tests the position of the hump on its own: the curve is almost
    flat around its peak (at τ ≈ 1.89·T_c, height 0.617·σ_ss), so its position
    scatters a lot.
  - The 3 h Allan plot with nominal values (T_c = 300 s) is a figure for the
    thesis, not a pass/fail test. See the glossary entry "Allan deviation" for why
    our curve has a hump and no flat "bias instability" floor.
- These tests prove that the corruptor implements its model. They do not prove
  the model matches a real Pixhawk; the thesis must not present them that way.

## 5. Fake detector

Makes buoy measurements from ground truth (pose interpolated to the detection
time) and `buoys_gt.yaml`.

- **Range and bearing** (lidar part): 360° around the boat, 10 Hz. Maximum range
  per buoy class from `asv_report.json` [REPO], where the lidar hits a buoy with at
  least 2 beams: A-0 4.75 m, A-2 10.75 m, Sur-Mark 18.75 m.
- **Colour** (camera part, D8 a): the buoy's true colour from `buoys_gt.yaml`,
  but only inside a camera's field of view and range [REPO]:
  ZED 72° wide, up to 8.9 / 16.2 / 20.1 m (A-0 / A-2 / Sur-Mark, 20 px wide);
  wide camera 85°, up to 5.3 / 9.6 / 12.0 m. Elsewhere the colour is UNKNOWN.
- Measurement: range r and bearing β from the base_link origin, in the level
  frame, with noise σ_r, σ_β [EST]. Misses with probability P_miss; false alarms
  at a rate λ per scan; colour mix-ups with a confusion probability (default 0,
  so a seen colour is the true colour); optional centroid bias of π·r_b/4 towards
  the boat (thesis, Static Landmarks).
- Other vessels in the course are reported as class "other". The filter must not
  map them.
- The true buoy ID is logged by slam_replay for evaluation, but it is not part of
  the message the filter sees.
- The 2-beam and 20-px thresholds are generator assumptions (`HOW_IT_WORKS.md`
  §14). The fake detector's errors have exactly the shape the filter assumes, so
  its results are optimistic (an inverse crime). M5.4 would calibrate it against
  the real pipeline; M5 is optional (D8), so without it the detector settings
  stay estimates, and the thesis says so.
- Optional (M7): a small random offset per buoy that changes slowly, as a stand-in
  for mooring motion, to test the draft's claim that mooring motion is absorbed
  by the measurement noise.

## 6. The filter (EKF-SLAM)

Lives in the ROS-free package `asv_core` (D13 a); `slam_node` and
`slam_replay` are thin wrappers.

- **Notation** (D14 a, decided 2026-10-11): Thrun's letters in docs, equations
  and comments: R = process noise covariance, Q = measurement noise covariance.
  Code keeps names without letters (`process_noise_cov`, `meas_noise_cov`), as
  Marcell asked. R also names the rotation matrices in eq. 3.3 (R_x, R_y, R_ℬ𝒮);
  which one gets another letter is Marcell's choice in the thesis. Symbols are in
  the glossary's Symbols table; a new symbol is marked "proposed" until Marcell
  accepts it.
- **State** (D5 C, decided 2026-10-14): x, y, ψ, the gyro bias b_g and the buoys
  (4 + 2N); plus 2 GNSS slow-error states if D6 b (§4.2). Landmarks are 2D points with a colour label.
- **Prediction** (D5 C): inputs are the GNSS velocity (v_E, v_N, world frame) and
  the gyro rate ω_m: x += v_E·dt, y += v_N·dt, ψ += (ω_m − b_g)·dt, b_g as a slow
  random walk; buoys unchanged. Without a GNSS velocity (outage) the last value
  is held with a growing uncertainty (rule set in the M3.3a brief). The input
  noise enters as process noise. Process noise from a documented continuous-time model,
  discretised (Van Loan method, [LIT `vanloan1978computing`]).
- **GNSS position update** (D15 a, decided 2026-10-11; Marcell wants to discuss
  it again in the M3.3b brief): z = ENU position of the antenna (2D).
  - D15 a (planar, as the draft's Planar Motion section says): h(ξ) = [x, y]ᵀ +
    C(ψ)·l_xy, with C(ψ) the 2D rotation by ψ and l_xy = (−0.76, −0.33) m. Roll and
    pitch move the antenna (0.185 m above base_link) by 1.3-1.9 cm at 5° roll or
    pitch alone and up to 2.8 cm when both are 5° (depending on the signs), which
    is more than the RTK noise; that error is covered by
    the measurement covariance and stated as a limitation.
  - D15 b (3D lever arm): h(ξ) = [x, y]ᵀ + [R_z(ψ)·R_y(θ)·R_x(φ)·l]₁,₂ with
    l = (−0.76, −0.33, 0.185) m and φ, θ from the corruptor's separate roll/pitch
    message (made only if something needs it, D11, §4.1) treated as known inputs. Then attitude errors also enter the GNSS update, and the draft's
    Planar Motion section must say so.
  - Either way the Jacobian includes the ∂/∂ψ term from the lever arm.
    Measurement covariance: per D6 (open). With D6 a, NavSatFix covariance ×
    an inflation factor per GNSS mode; with D6 b (recommended), only the white
    part, the slow part being two extra states (`decision_notes.md`).
- **Heading update**: z = α; h(ξ) = wrap(π/2 − ψ); the innovation is wrapped.
- **Gyro**: with D5 C an input, not an update (the D5 b form would be z = yaw
  rate, h = ω + b_g). Strictly, the gyro's
  z-axis measures the body rate, not ψ̇; with roll and pitch of a few degrees they
  differ slightly. The M3.3c brief states which form is used.
- **Buoy update**: z = (r, β); d = m_j − [x, y]ᵀ, q = dᵀd,
  h = [√q, wrap(atan2(d_y, d_x) − ψ)]ᵀ (planned for Observation Models).
- **New landmark** (initialisation): m_new = [x, y]ᵀ + r·[cos(ψ + β), sin(ψ + β)]ᵀ.
  Its covariance and its cross-covariance with the rest of the state come from the
  Jacobians of this formula with respect to the pose and to (r, β). These Jacobians
  are tested like all others.
- **Data association** (thesis A7): nearest neighbour with a Mahalanobis gate (χ²,
  2 degrees of freedom: 5.99 at 95 %, 9.21 at 99 %). A known colour must match
  the landmark's colour; an UNKNOWN colour uses the gate only. Only coloured buoy
  detections may start a new landmark. New landmarks are tentative until seen
  n_conf times; tentative ones that are not confirmed are dropped.
- **Numerics**: Joseph-form covariance update, covariance kept symmetric. H is
  sparse (a buoy update touches only the pose and that buoy), so the update is
  written with P·Hᵀ and rank-2 terms; written with full matrices it costs N³
  instead of N² (§14.8). M3.8 is left out (D16); the M3.5 replay time shows it
  instead: more than about 30 s per 5-min bag suggests full-matrix code.

### 6.1 Choosing the motion model (D5: decided C, 2026-10-14)

Decided: **C** = option c below plus a gyro-bias state, with the GNSS velocity
as input (comparison in `decision_notes.md`). The table is kept as the record of
the options.

|  | a) IMU-driven | b) Constant body velocity and turn rate (CTRV-type; recommended) | c) Pose only (eq. 3.6 as drafted) |
|---|---|---|---|
| State besides the buoys | x, y, ψ, u, v, b_g, b_ax, b_ay (8 + 2N; the accelerometer biases are needed: 0.02 m/s² unestimated gives 36 m in 60 s) | x, y, ψ, u, v, ω, b_g (7 + 2N) | x, y, ψ (3 + 2N) |
| What drives the prediction | measured accelerations and yaw rate (inputs) | the current velocity estimate; changes are absorbed by the process noise | measured yaw rate and GNSS Doppler velocity (inputs) |
| Gyro | input | measurement | input |
| Accelerometer | used (gravity removed with roll and pitch) | not used | not used |
| During a sky-view GNSS outage, no mapped buoy in view (mapped buoys bound the drift for every option) | dead reckoning on the IMU; drifts with the remaining accelerometer bias and with roll/pitch errors (g·Δθ: 0.1° gives 0.017 m/s²); follows speed changes | coasts on the last velocity estimate; heading from the gyro with its bias estimated; blind to speed changes (a 1 m/s slowdown: about 8 m after 10 s) | holds the last speed (Doppler is gone too); no gyro-bias state, so 0.0017 rad/s gives 5.8° and 4.6 m sideways after 60 s at 1.5 m/s |
| Change to the draft's eq. 3.6 | state grows; "attitude errors only enter through the projection" no longer holds | state grows | none |
| Extra corruptor output | roll/pitch with an error model (D11; [EST], it drives a's outage results) | none | GNSS velocity |
| What the IMU sweep (§11) can test | gyro and accelerometer | gyro only | gyro only |
| Work | most | medium | least |

With b, the accelerometer is not used at all, so the IMU sweep and any argument
that "the consumer IMU is good enough" cover only the gyro. Choose a if the
accelerometer study matters to the thesis. Numbers and a middle option (b plus
the accelerometer as a measurement of forward acceleration) are in
`decision_notes.md`.

Predicting the motion from the commanded thrust and the drag model is not an
option for the main results: the drag numbers are Gazebo's own (an inverse
crime, see glossary).

## 7. Measurement queue

- Measurements wait in a buffer sorted by `header.stamp` for a fixed delay L (e.g.
  0.2 s, longer than the slowest message's arrival delay). Then they are applied
  in stamp order, with the prediction carried forward to each stamp.
- A measurement that arrives later than L is counted and reported, not applied
  to the wrong pose.
- slam_replay and slam_node use the same class, so offline and live results
  match.

## 8. Recording profiles

Data rates are calculated from the message sizes and rates in `asv.yaml`. Check
them with `ros2 bag info` (M2.9).

| Profile | Topics | Approx. rate |
|---|---|---|
| `nav` | `/clock`, ground-truth odom, clean IMU, both clean GNSS fixes, thruster commands | about 0.1 MB/s (about 7 MB per minute) |
| `nav_lidar` | `nav` + clean lidar | about 6.5 MB/s (about 0.4 GB per minute) |
| `perception` | `nav_lidar` + ZED left image + camera_info | about 48 MB/s (about 3 GB per minute) |
| everything | all bridged topics incl. all cameras and the depth cloud | about 190-225 MB/s (over 10 GB per minute): don't |

Sweeps with the fake detector need only `nav`. Real perception (M5, only if
time) needs `perception`. Data goes to `ws/data/bags/` (git-ignored from M0.1 on). For
nav-only runs, Gazebo still renders every camera because the bridge always
subscribes to them; marking those bridge entries `lazy` (ros_gz_bridge option)
speeds up headless runs. This is optional, and changing the generated bridge file
means changing the generator or writing a modified copy in the launch file.

## 9. slam_replay and the run folder

slam_replay reads a bag with `rosbag2_cpp` and runs corruptor → detector → filter
in stamp order, with no ROS timing at all. One run writes one folder:

```
ws/data/runs/<sweep>/<run_id>/
  manifest.yaml     git commit + "dirty" flag, asvgen fingerprint, bag name and size,
                    command line, seed
  config.yaml       full merged configuration (noise profile, detector, filter)
  trajectory.csv    t, true pose, estimated pose, pose covariance,
                    true biases, (estimated velocity and bias, if in the state)
  map.csv           landmark id, colour, x, y, covariance, sightings, tentative/confirmed
  innovations.csv   t, sensor, landmark id, true buoy id, innovation, S, NIS, accepted
  metrics.json      the numbers from §10
```

Test: the same config and seed twice give an identical `metrics.json`. Results are
reproducible within the same container image.

## 10. Evaluation

All in Python (`asv_eval`), from the run folder.

- **Ground truth at the right time**: interpolate ground truth to each estimate's
  stamp (linear for position, shortest way round for yaw). Ground truth comes at
  50 Hz; using the nearest sample instead can be 10 ms off, about 2 cm at 2 m/s,
  more than the RTK noise. Ground-truth velocity, if needed, comes from
  differences of ground-truth positions, not from the odom twist (Gazebo smooths
  it over 10 samples).
- **Localisation error** (accuracy): position error in m and heading error in rad
  (wrapped), reported separately. **No trajectory alignment**: GNSS puts the
  estimate in the world frame, and an alignment would hide exactly the error we
  want to measure. Per run: RMSE over time. Over runs: mean ± 95 % confidence
  interval; also the mean error (bias) and the spread (precision).
- **Map error**: match estimated landmarks to true buoys one to one (*Hungarian
  assignment*) with a cutoff distance c: a pair further apart than c counts as one
  missed and one spurious landmark. "Missed" counts only buoys that came within
  sensor range at least once. Report RMSE of matched pairs, missed and spurious
  counts, and colour accuracy. Optional: OSPA [LIT Schuhmacher, Vo & Vo, IEEE TSP
  2008].
- **Consistency (NEES)**: ε = eᵀP⁻¹e with e = [x, y, wrapped ψ] (n = 3). Averaged
  over M runs at each time step (ANEES). 95 % band: [χ²_{Mn}(0.025)/M,
  χ²_{Mn}(0.975)/M]. The band holds per time step, so even a perfect filter is
  outside it at about 5 % of the steps.

  | M runs | 95 % band for ANEES (n = 3) |
  |---|---|
  | 1 | 0.22 - 9.35 |
  | 20 | 2.02 - 4.16 |
  | 50 | 2.36 - 3.72 |

- **Matched check** (M3.6): the corruptor runs a white-noise-only profile (no
  Gauss-Markov errors, no turn-on biases, no centroid bias, no GNSS mode switches),
  the filter's measurement noise equals that noise, and its process noise is tuned
  once on a separate run and then frozen. Pass: the ANEES is inside its band at
  about 95 % of the time steps (at least 90 %).
- **NIS** per sensor, with its degrees of freedom: GNSS position 2, heading 1,
  buoy 2. NIS of gated buoy updates is cut off by the gate and biased low; compute
  it also on fake-detector runs with the true buoy IDs. Optional: test GNSS
  innovations for correlation over time.

## 11. Sweeps

`asv_eval` sweep runner: a sweep YAML lists the settings and the number of runs M
(D10). Vary one factor at a time from nominal, plus one crossed block (IMU level ×
GNSS outage).

| Factor | Levels |
|---|---|
| GNSS mode | RTK fixed, float, standalone; outages of [EST] 10 / 30 / 60 s |
| IMU | nominal, 2×, 5× (white noise and bias together), CV7-like bias (below). With D5 C only the gyro matters (§6.1). If D11 A: also the processing level ideal / tuned / Rover default. |
| Detector | σ_r and σ_β × 1, 2, 5; P_miss; false-alarm rate; colour error |
| Waves (D9) | wave gain 0 / 0.1 / 0.3 (each needs a new recording) |

- **CV7-like level** [EST, the rule must be stated in the thesis]: keep T_c =
  300 s and choose σ_ss so that the hump of our Allan curve (0.617·σ_ss) is as
  high as the CV7's Allan floor. The CV7 datasheet gives 1.5 °/h (7.3e-6 rad/s).
  If that number is the bias instability B (floor = 0.664·B), σ_ss ≈ 1.08·B ≈
  7.8e-6 rad/s; if it is the floor itself, σ_ss ≈ B/0.617 ≈ 1.2e-5 rad/s. Which
  one applies is checked in the datasheet in the M6.2 brief. For comparison, the
  nominal σ_ss = 2e-4 rad/s is about 41 °/h.
- Filter tuning rule, stated per run: the filter's noise settings = inflation ×
  nominal, kept fixed while the true noise changes ("mismatched"). The matched
  check (§10) is separate.

Responses: position RMSE, heading RMSE, map RMSE, missed/spurious counts, ANEES,
NIS. One results table (CSV) for all runs; figures are made from it.

## 12. Packages, configuration and dependencies

Package names follow D13 a (decided 2026-10-11).

| Package | Kind | Contains | Depends on |
|---|---|---|---|
| `asv_core` (new, D13) | ament_cmake, no ROS | geodesy, noise models, fake detector logic, measurement queue, EKF-SLAM, data association, ellipse helper; gtest | Eigen |
| `asv_msgs` (new) | ament_cmake + rosidl | BuoyDetection(Array), Landmark(Array), GnssHeading, SensorStatus; a roll/pitch message only if D11 needs one | |
| `asv_sim` (exists) | ament_cmake | world, models, generator, launch, guidance (M1), follower, player, teleop, corruptor node, fake detector node, ground-truth tools | asv_core, asv_msgs |
| `asv_perception` (new, only if time: D8) | ament_cmake | lidar clustering, colour classification, fusion | asv_core, asv_msgs |
| `asv_slam` (new) | ament_cmake | slam_node, slam_replay | asv_core, asv_msgs (not asv_sim) |
| `asv_eval` (new) | ament_python | metrics, sweep runner, figures, App. B tables | asv_msgs |

Custom messages get their own package because that is ROS 2 best practice: the
real-boat code and the Python tools can then use them without the simulator. (An
ament_python package such as asv_eval can't define messages itself.)

Configuration map:

| What | Where |
|---|---|
| boat, sensor mounts, datasheet noise values | `asv_sim/config/asv.yaml` (then `make_asv.py`) |
| course elements | `asv_sim/tools/make_roboboat2026_course.py` |
| course layout, waves | `asv_sim/worlds/roboboat_course.sdf` (hand-edited; launch arguments later) |
| noise profiles | `asv_sim/config/noise_profiles/<name>.yaml` |
| fake detector settings | `asv_sim/config/detector/<name>.yaml` |
| filter settings | `asv_slam/config/<name>.yaml` |
| sweeps | `asv_eval/config/sweeps/<name>.yaml` |
| recordings, run results, figures | `ws/data/{bags,runs,figures}/` (git-ignored from M0.1 on) |

Installed in the container (confirm with M0.3): Eigen and gtest (Dockerfile);
rosbag2_cpp (part of `ros:jazzy-ros-base`); yaml-cpp (through
`yaml_cpp_vendor`, very likely); cv_bridge and OpenCV very likely (dependencies of
`rqt_image_view`); PCL probably not. Python libraries for evaluation (numpy,
scipy, matplotlib) are not checked yet. Ask before adding anything.

## 13. Facts to rely on, and limitations

Checked facts (gz-sensors / gz-sim source code and the repository):

- Gazebo applies NavSat horizontal noise in degrees (the SDF spec says metres;
  open issue gz-sensors #325). The bridge's NavSatFix always says "covariance
  unknown" and "fix".
- Gazebo's dynamic bias only works for the IMU (and force/torque). The lidar gets
  range noise only. Camera noise only changes pixel brightness. There is no
  dual-antenna heading sensor.
- The IMU's orientation output is perfect (no noise), in ENU.
- The NavSat sensor reports the antenna's position, not base_link's.
- Ground truth is published at 50 Hz; its twist is a 10-sample moving average of
  finite differences.
- The IMU's samples land on 4 ms physics steps (intervals 8, 4, 4, 4 ms).
- The lidar can't see the VRX water (`visibility_mask 7`).
- All buoys and vessels in the course float freely (buoyancy plugin, no mooring);
  only the dock parts are static.
- The pod steering angle follows commands at no more than 2 rad/s, and the actual
  angle is not published to ROS.

Measured in this repository (M0.2, 2026-10-15, `tools/sim_time_check.py`):

- `sim.launch.py` sets `use_sim_time` for all its nodes (bridge and 9 static TF
  publishers, checked with `ros2 param get`).
- Headless real-time factor (world asks for 1.0): 0.848 [MEASURED, 30 s
  simulated in 35.4 s wall; this PC, no other load]. Not measured with the GUI.
- A 20 Hz ROS-clock timer fires 20.03 per simulated second at RTF 0.848 and
  20.00 at RTF 0.493; a wall timer fires 23.60 and 40.57 (= 20 / RTF).
- The RTF can be changed at runtime, without touching the world file:
  `gz service -s /world/roboboat_course/set_physics --reqtype gz.msgs.Physics
  --reptype gz.msgs.Boolean --req 'real_time_factor: 0.5, max_step_size: 0.004'`.
- `tools/actuator_check.py` now times its phases in simulation time; rerun gives
  the Step 1 table unchanged (1.00 / 1.52 / 2.27 m/s, +0.150 and +0.176 rad/s).

Limitations to state in the thesis (Discussion and Limitations):

- No water or spray returns in the lidar; no multipath or reflections in GNSS.
- Buoys are static in the experiments (D7 a); mooring motion is not simulated.
- Fake-detector results are optimistic, because their errors have the shape the
  filter assumes. Without real perception (M5 is optional, D8), the detector's
  noise settings are estimates, not measured from a real pipeline.
- Monte Carlo spread covers sensor noise only, for the recorded trajectories.
- IMU sample timing is quantised to the physics step.
- With D15 a, the GNSS lever arm ignores roll and pitch (1.3-2.8 cm at 5° tilt).
- The noise models are verified (Allan and statistics tests), not validated
  against real hardware.

## 14. Benchmarking the onboard code

**Left out for now** (D16, 2026-10-11: "Leave out benchmarking for now, first lets
get a working sim environment"). The steps that use this section (M3.7, M3.8,
M4.2, M5.5, M7.7) are marked `[-]`. The section is kept for later; nothing in the
current plan depends on it.

Purpose: CPU time, memory and timing of the code that would run on the boat, to
help choose the boat computer (and, with D16 a, to back assumption A6). Steps:
M0.3 (tools present), M3.7, M3.8, M4.2, M5.5, M7.7. These numbers come from
desk research and test programs on a cloud x86 machine (2026-10-10), not from
Marcell's PC; every "[EST]" is unmeasured here.

### 14.1 What counts as onboard

| Group | Processes | In the benchmark |
|---|---|---|
| Onboard, measurable here | perception nodes (M5), measurement queue + EKF-SLAM (slam_node), waypoint follower, rosbag2 recorder, static TF publishers (negligible) | yes |
| Onboard, **not** measurable here | sensor drivers (livox_ros_driver2, ZED SDK and its ROS wrapper incl. GPU depth, AR0234 camera driver, Micro XRCE-DDS Agent for the Pixhawk, UM982 driver), any neural-network detector, all GPU work (the container has no NVIDIA runtime) | only on the board (M7.7) |
| Sim-only | Gazebo, ros_gz_bridge, corruptor, fake detector, ground-truth tools, evaluation, RViz, `ros2 bag play` | excluded; shown separately for context |

slam_replay runs the bag reader, corruptor, fake detector and filter in one
process, so its replay speed is a filter-only throughput bound, and its peak
memory includes the sim-only stages (label it so in `resources.json`). Onboard
memory comes from slam_node in M4.2.

### 14.2 Clocks

- Durations and latency: `std::chrono::steady_clock`. CPU time:
  `clock_gettime(CLOCK_THREAD_CPUTIME_ID)` per thread, `getrusage(RUSAGE_SELF)`
  per process. Never the ROS clock: with `use_sim_time` it is simulation time
  (§3 rule 9).
- `ros2 topic delay` is the CLI node's clock minus `header.stamp`, and `ros2
  topic hz` also uses the node's clock: with simulation time neither measures
  compute time [SRC ros2cli jazzy `delay.py`, `hz.py`].
- Latency inside a node: log (header.stamp, steady time at receive, at dequeue
  from the queue, before publish).

### 14.3 What the numbers mean

- **CPU load**: report core-seconds of CPU per second of data, plus per-callback
  p50, p99 and max (from in-node timing). These compare across runs.
- **/proc sampling**: `utime`/`stime` in `/proc/<pid>/stat` count in ticks of
  1/100 s, so a 0.1 s sample can only read 0, 10, 20, ... %. Use
  `/proc/<pid>/task/<tid>/schedstat` (nanoseconds; field 1 = time on a CPU,
  field 2 = time waiting for one, a direct "too few cores" signal), or 1 s
  windows [SRC kernel `sched-stats.rst`].
- **Peak memory**: `VmHWM` and `ru_maxrss` are the peak resident set, which
  includes shared libraries (rclcpp, DDS). Adding them up over processes counts
  those twice, and the peaks don't happen at the same moment. Threads share one
  memory, so there is no per-thread memory figure. For "how much RAM the boat
  needs": sum PSS (`/proc/<pid>/smaps_rollup`) at the same moment, or step the
  memory limit down until the out-of-memory kill (M4.2) [SRC proc(5)].
- **Container numbers**: the sim container is one cgroup, so `memory.peak`,
  `cpu.stat` and `docker stats` include Gazebo. `memory.peak` also counts page
  cache (large when reading bags; it needs kernel 5.19 or newer); `docker stats`
  subtracts inactive file cache and has no peak [SRC kernel `cgroup-v2.rst`,
  Docker docs]. Use them as a cross-check, or in an onboard-only container.
- Optional: writing `5` to `/proc/self/clear_refs` resets `VmHWM` and
  `ru_maxrss`, so start-up spikes can be left out (checked on a Linux test
  machine, not on Marcell's).
- On a Jetson, CPU and GPU share the same RAM: ZED depth and a detector come
  out of the same budget [SRC NVIDIA CUDA for Tegra].

### 14.4 Run protocol

- Release build: `colcon build ... --cmake-args -DCMAKE_BUILD_TYPE=Release`
  (the normal build has no build type, so no optimisation). No `-march=native`
  for numbers compared across machines; the same test program gave 4-21 µs per
  EKF update depending on flags [MEASURED on a cloud x86 core].
- Warm-up: drop the first seconds. At least 5 repeats; report median and
  spread, p99 rather than only max. A hand-written timing loop must use its
  result, or the compiler may remove the work.
- Record in `resources.json`: CPU model (`lscpu -e`), frequency governor,
  cores used, compiler and flags, build type, RMW, executor thread count,
  docker limits, commit.
- Cores: one logical CPU per physical core
  (`/sys/devices/system/cpu/cpuN/topology/core_cpus_list`); on a hybrid Intel
  CPU, one core type only. Set the executor's thread count explicitly: the
  default thread count follows `hardware_concurrency()`, which ignored a
  restricted CPU set in a sandbox test [EST for Docker].
- Scene: the worst case, not the average. The "board bag" goes through Task 2
  (the most course elements) and lasts a full mission, so map growth and leaks
  show up (memory slope, queue length over time).
- The sim lidar is a regular 40 × 500 grid; the real Mid-360 scan pattern
  differs, so perception timings are "sim geometry" until repeated on a real
  recording.

### 14.5 Imitating a smaller computer (M4.2)

```
docker run --rm --network=host --ipc=host -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$PWD/ws:/ws" --cpuset-cpus=<cores> --memory=<RAM> --memory-swap=<same RAM> \
  roboboat-sim:jazzy bash -ic "..."
```
(The image from `compose.yaml`; it already runs as the user `dev`. Run from
the project root, like `docker compose`.)

- Same image, a command line, no `compose.yaml` change. `--ipc=host` is
  required: Fast DDS (the Jazzy default) uses shared memory between processes
  on one host, and containers with separate shared memory can't talk [SRC Fast
  DDS docs]. `--memory-swap` equal to `--memory`, or the container may swap
  [SRC Docker docs]. `ros2 bag play` stays in the normal container.
- `--cpuset-cpus` (whole cores) rather than `--cpus` (a time quota per 100 ms
  that throttles in bursts) [SRC kernel `sched-bwc.rst`].
- Imitates: core count and RAM. Does **not** imitate: per-core speed, ARM vs
  x86, caches, memory bandwidth, GPU, power, heat. The CPU frequency can only be
  capped on the NixOS host (`/sys` is read-only in the container), and a capped
  x86 core is still not an ARM core.

### 14.6 Budgets (proposals [EST], decided in the M4.2 brief)

- One lidar scan every 100 ms (10 Hz): perception + fusion + filter update p99
  at most 50 ms.
- End-to-end p99 below the queue delay L.
- Sustained use of the cores given at most 50-70 %: the boat adds drivers,
  recording and the Pixhawk link, which the PC can't measure.

### 14.7 From PC numbers to a board

- Rough scale only: Geekbench 6 single-core, single uploads (about ±20-40 %): Jetson
  Orin Nano Super about 810, Orin NX 16 GB about 877, AGX Orin 682-998,
  Raspberry Pi 5 748-904, Intel N150 1294, desktop Core Ultra 7 265K 3340
  [SRC Geekbench browser, Geerling sbc-reviews; the ±20-40 % is EST]. An Orin core is roughly a
  quarter of a current desktop core [EST]. Better: run the same M3.8 program on
  both machines.
- Conclusive: the on-board replay (M7.7). On a Jetson: fix and record the power
  mode (`nvpmodel`, the one the boat will use), run at least 30 minutes, log
  `tegrastats` (per-core CPU and MHz, GPU, RAM, power per rail in mW,
  temperatures), and run once with GPU load (ZED depth, detector) [SRC NVIDIA
  Jetson docs; soak length EST].

### 14.8 Expected load [EST, desk research]

- **EKF** (D5 b, N = 50 buoys): 7 + 2N = 107 state values, P ≈ 92 kB. A
  structure-aware update (P − K(PHᵀ)ᵀ) costs about 4·107² ≈ 46 kflop, a dense
  Joseph update about 4·107³ ≈ 4.9 Mflop. Test program on one cloud x86 core:
  about 4-21 µs vs 1.3-2.3 ms per update; a Joseph form written for the sparse
  H costs a few times the plain one, still quadratic [EST, not timed]. With
  10-30 buoy detections per scan [EST] plus GNSS, heading and gyro, about
  320-520 updates per second: at most about 1 % of a core vs about 0.4-1.2
  cores for the updates alone. **How the filter is written matters more than the board**; M3.8's slope
  checks it.
- **Lidar** levelling + voxel grid + clustering of a 20 000-point scan: about
  2 ms on one x86 core (own test code, not PCL).
- **Drivers**: the ZED SDK needs an NVIDIA GPU (compute capability ≥ 7.5;
  Jetson Orin or Thor) [DS Stereolabs]; without it the ZED 2i is a plain USB
  camera with no SDK depth. The AR0234 sends only uncompressed YUY2: about
  35 MB/s at 960 × 600 @ 30 fps [DS Arducam]. No published CPU figure was found
  for livox_ros_driver2.
- **Recording**: about 48 MB/s for the `perception` profile in simulation; the
  real ZED wrapper publishes 32-bit images unless `enable_24bit_output` is set
  (about 55 MB/s for the left image alone at 720p, 15 fps) [SRC Stereolabs
  docs]. A V30 SD card guarantees only 30 MB/s, so record to NVMe.
- **Gyro rate on the real boat**: ArduPilot's DDS IMU topic is published at most
  every 6 ms (about 167 Hz) but only gets new data at the main loop rate, 50 Hz
  by default on a Rover, so it repeats values unless `SCHED_LOOP_RATE` is raised;
  Rover's default low-pass filters (4 Hz gyro, 10 Hz accel) add about 56 and
  22 ms of delay [SRC ArduPilot AP_DDS, AP_InertialSensor]. See D11.
