# M2 plan: From perfect to realistic sensor data

One section per step of `docs/master_plan.md` M2. Each step's brief and report
are saved here, so this file becomes the logbook of the milestone. Notes are
short bullets and equations, not finished paragraphs. The Brief, Report and
"Marcell's notes" parts (`TEMPLATE.md`) are added under a step when it starts.

Status: approved on 2026-10-21

Before M2.1: M1.9 (waypoint-switch fix, review 2026-10-20 A1/A2), logged in
`ws/src/asv_sim/docs/milestone1_plan.md`.

Decisions settled for M2 (all in `master_plan.md` §3): D1 a, D2 a, D3 b, D4 a,
D5 C, D7 a, D11 (raw IMU, no ArduPilot processing, 2026-10-21), D12 a, D13 a.
None of the M2 steps waits for a decision.

Order: M2.1 → M2.2 → M2.4 → M2.5 → M2.6a–d → M2.7 → M2.8 → M2.9 (table order).
M2.2 needs M2.1 (geodesy for its check); M2.6 needs M2.2, M2.4 and M2.5.

---

## M2.1 Geodesy helper and the `asv_core` package

**Goal:** a ROS-free package `asv_core` with latitude/longitude ↔ local
east/north/up conversion, the same as Gazebo's, in C++ and Python.
**Done when:** unit tests: the reference point maps to (0, 0, 0); a point 100 m
east and 100 m north converts there and back within 1 mm; C++ and Python give
the same numbers (same test points, difference ≤ 1e-9 m).
**Decisions needed:** none (D13 a decided).

- ⚠ First new package (`ament_cmake`, `<depend>` on Eigen, gtest).
- Reference point read from the world SDF (`<spherical_coordinates>`), WGS84;
  the brief checks Gazebo's own conversion in gz-math source [SRC].
- `wrap_angle()` moves from `asv_sim` to `asv_core` (CLAUDE.md Known state);
  `asv_sim` then depends on `asv_core`.
- `asv_core` keeps its own plain structs; nodes convert to and from `asv_msgs`
  (review 2026-10-20 B1; one line in `design.md` §12).

---

## M2.2 Switch Gazebo's IMU, NavSat and lidar noise off

**Goal:** Gazebo publishes perfect IMU, GNSS and lidar data; camera noise stays.
**Done when:** the regenerated `model.sdf` has no `<noise>` in the IMU, NavSat
and lidar sensors and still has it in the cameras. On a short recording, each
clean GNSS fix (converted with the M2.1 helper) equals the true antenna position
(ground truth interpolated to the fix stamp + lever arm rotated by the true
roll, pitch, yaw); the report gives the largest difference (expected a few mm).
**Decisions needed:** none (D1 a, D2 a decided).

- ⚠ Generator change (`tools/asvgen/`) and new `asv.yaml` key
  `gazebo_sensor_noise: false`; update `HOW_IT_WORKS.md`.
- Ends the "GNSS noise ≈ 890 m" bug (master plan §2 item 1).

---

## M2.3 Topic names

Skipped `[-]` (D3 b, 2026-10-11). Nothing to do.

---

## M2.4 `asv_msgs` package

**Goal:** the project's own message types, each field documented with unit and
frame.
**Done when:** the package builds; every field has a comment with its unit and
frame; a Python check (`ros2 interface show`) lists all types.
**Decisions needed:** none (D11 decided 2026-10-21: no roll/pitch message).

- ⚠ New package and dependencies (`rosidl_default_generators`,
  `std_msgs`, `geometry_msgs`).
- Types: BuoyDetection(Array) with an UNKNOWN colour, Landmark(Array),
  GnssHeading (`design.md` §4.3), SensorStatus. GNSS velocity uses
  geometry_msgs/TwistWithCovarianceStamped (no new type).

---

## M2.5 Noise library

**Goal:** white noise, turn-on bias and Gauss-Markov bias in `asv_core`, one
seeded random stream per noise source, time steps from timestamps.
**Done when:** the three tests in `design.md` §4.5 pass (white alone;
Gauss-Markov alone with a short T_c; both together at τ = 1, 10, 30 s against
the analytic total), at the tolerance fixed in the brief.
**Decisions needed:** none.

- The brief settles the density convention (σ = η/√dt, or TDK's √2 smaller;
  `decision_notes.md` D11) and the test tolerances (from a few seeds).
- Normal distribution implemented in the library, not `std::normal_distribution`
  (`design.md` §4.5).

---

## M2.6 Corruptor (proposed split into a–d)

Too big for one screen, so split. Common rules: nominal values from
`asv.yaml` `sensor_specs`; noise profiles
(`asv_sim/config/noise_profiles/<name>.yaml`) hold only multipliers, schedules,
overrides and the seed; the maths lives in `asv_core`, the node only does I/O;
every true bias is logged; no output carries the true yaw.

### M2.6a IMU
**Goal:** `/asv/sensors/imu/data`: gyro and accelerometer at 200 Hz with white
noise, turn-on bias and Gauss-Markov bias, stamped at measurement time, no
orientation (`orientation_covariance[0] = −1`) (D11, 2026-10-21).
**Done when:** unit tests: error mean and spread match the profile; the
orientation field is marked unavailable; no output contains the true yaw.
**Decisions needed:** D11 questions 8 (per-run scale/misalignment error:
nominal or own study) and 9 (accelerometer density 110 µg/√Hz in `asv.yaml`,
⚠ value change) are raised in this brief. With D5 C the filter does not use the
accelerometer.

### M2.6b GNSS position
**Goal:** `/asv/sensors/gnss/fix`: antenna position + white and slowly varying
error per mode (RTK fixed / float / standalone), outages from a schedule,
covariance per mode; true mode on `/asv/ground_truth/gnss_status`.
**Done when:** unit tests per mode; a mode switch redraws the slow error; no
message during an outage.
**Decisions needed:** none (D6 is the filter's side, M3.3b). The brief
proposes the white/slow split of each mode's 1σ and the covariance convention
(`design.md` §4.2, `decision_notes.md` questions 3 and 5).

### M2.6c GNSS velocity and dual-antenna heading
**Goal:** `/asv/sensors/gnss/velocity` (ENU, from ground-truth position
differences + noise, D5 C) and `/asv/sensors/gnss/heading` (α = π/2 − ψ + noise,
σ = 0.060°, D12 a).
**Done when:** unit tests: velocity and heading errors have the configured
spread; heading wraps into [0, 2π); both stop in a sky-view outage, heading
stays up when only RTK corrections are lost.
**Decisions needed:** none. The brief looks up the UM982 velocity accuracy
[DS] (still "to check" in `design.md` §4.2).

### M2.6d Corruptor node and recording check
**Goal:** one node publishes all of the above live from the clean topics.
**Done when:** on a recording, corrupted − clean has the configured mean and
spread per sensor; `ros2 node info` shows the expected inputs and outputs.
**Decisions needed:** none.

- Update `HOW_IT_WORKS.md` §5.4 and §14 (heading now emulated by the corruptor,
  `design.md` §4.3). Lidar range noise is left for M5 (optional).

---

## M2.7 True buoy map

**Goal:** `buoys_gt.yaml` (name, colour, class, radius, position) from the
course files, published on a latched topic; buoys made static.
**Done when:** unit test: Task 1 gate 1 red buoy at (5.0, 1.524) m; every buoy
include in the regenerated `rb26_task*` models has `<static>true</static>`;
drift of floating buoys (largest movement per buoy in 5 min) written down.
**Decisions needed:** none (D7 a decided).

- ⚠ Course generator change (`make_roboboat2026_course.py`).
- Marcell checks in the Gazebo window that static buoys sit at the waterline.

---

## M2.8 Frames and TF

**Goal:** one connected TF tree: static world → map identity, world →
base_link from ground truth (when the filter is not running); an RViz config
with the lidar cloud in the world frame.
**Done when:** `ros2 run tf2_tools view_frames` shows one connected tree;
Marcell checks RViz.
**Decisions needed:** none.

---

## M2.9 Recording profiles

**Goal:** profiles `nav`, `nav_lidar`, `perception` (`design.md` §8).
**Done when:** for each profile, `ros2 bag info` counts equal rate × simulated
duration within 1 % (IMU 200, GNSS 10, ground truth 50 Hz; lidar 10 Hz).
**Decisions needed:** none. Short-form brief.
