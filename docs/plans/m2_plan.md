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

### Brief (agreed on 2026-10-28)

1. **Goal:** new ROS-free package `asv_core` with WGS84 latitude/longitude ↔
   ENU conversion that does what Gazebo does, in C++ and Python; `wrap_angle()`
   moves into it.
2. **Decisions:** none (D13 a). Layout choices taken as recommended:
   - Python copy in `asv_sim/tools/geodesy.py` (no `ament_cmake_python`).
   - `yaw_from_quaternion()` moves to `asv_core` with `wrap_angle()`.
3. **Why:** the filter turns each GNSS fix into world-frame metres
   (Coordinate Frames and Transformations); the corruptor adds GNSS error in
   metres and converts back (Noise Models); origin = the world's reference
   point, not the first fix (`design.md` §2). M2.2's check needs our maths to
   equal Gazebo's, or it measures our maths instead of the simulator.
4. **New ideas:**
   - ECEF route: lat/lon/h → Earth-centred XYZ → minus the reference point's
     XYZ → rotate into east/north/up. Exact, no flat-Earth approximation.
   - Bowring's closed-form inverse: XYZ → lat/lon in one pass, as Gazebo does
     (error far below 1 mm near the surface); copied to match Gazebo.
   - Proposed symbols (φ is roll): φ_G, λ_G, h_G; N_G(φ_G) prime-vertical
     radius; a, b, e², e'².
5. **What changes:** ⚠ new package `ws/src/asv_core/` (⚠ `<depend>eigen</depend>`,
   gtest); `geodesy.hpp/.cpp`, `angles.hpp/.cpp`, `test_geodesy.cpp`,
   `test_angles.cpp` (moved, ⚠ `git mv` out of `asv_sim`);
   `test/data/geodesy_points.csv`. `asv_sim`: ⚠ `<depend>asv_core</depend>`,
   includes in `control.cpp`, `guidance.cpp`, `waypoint_follower.cpp`,
   `CMakeLists.txt`. New `asv_sim/tools/geodesy.py`, `tests/test_geodesy.py`.
   Docs: `design.md` §12, §13; glossary; this plan; master plan mark;
   CLAUDE.md.
6. **How it works** [SRC: gz-sim8 `Util.cc` `sphericalCoordinates()` calls
   `PositionTransform(world pos, LOCAL2, SPHERICAL)`; gz-math 7.7.0
   `SphericalCoordinates.cc`; LOCAL is the legacy frame with x flipped]. Our
   heading is 0, so the heading rotation is the identity.
   a = 6378137.0 m, b = 6356752.314245 m, e² = 1 − b²/a², e'² = a²/b² − 1.
   - N_G = a / √(1 − e² sin²φ_G)
   - X = (N_G + h_G) cos φ_G cos λ_G, Y = (N_G + h_G) cos φ_G sin λ_G,
     Z = ((b²/a²) N_G + h_G) sin φ_G
   - [e n u]ᵀ = R(φ_0, λ_0)(p − p_0),
     R = [[−sin λ_0, cos λ_0, 0], [−sin φ_0 cos λ_0, −sin φ_0 sin λ_0, cos φ_0],
     [cos φ_0 cos λ_0, cos φ_0 sin λ_0, sin φ_0]]
   - inverse: p = p_0 + Rᵀ[e n u]ᵀ; ρ = √(X² + Y²), θ = atan(Z a / (ρ b)),
     φ_G = atan((Z + e'² b sin³θ) / (ρ − e² a cos³θ)), λ_G = atan2(Y, X),
     h_G = ρ / cos φ_G − N_G(φ_G)
   - C++: `GeoPoint{lat_deg, lon_deg, alt}`, `LocalTangentPlane(ref)` with
     `to_enu()` / `to_geo()` (Eigen::Vector3d). No SDF parsing in C++; the
     reference comes as a parameter, read by Python `read_geo_reference()`.
7. **Checks:** gtest + Python: reference → (0, 0, 0) within 1e-9 m;
   100 m E + 100 m N round trip within 1 mm; 1e-5° north of the reference gives
   n = M·Δφ (meridian radius, independent formula) within 1 mm; every CSV row
   within 1e-9 m in both languages. Python: `read_geo_reference()` of
   `roboboat_course.sdf` = (27.367256, −82.451120, 0.0); error on heading ≠ 0,
   non-WGS84 surface or non-ENU orientation. Baselines reported (asv_sim gtests
   drop the moved `test_angles`).
8. **Can't check:** agreement with a running Gazebo; that is M2.2.

### Report (2026-10-28)
Commit: the one whose message starts with `M2.1:` (`git log --grep "^M2.1:"`).

1. **What changed**
   - `ws/src/asv_core/` (new): `package.xml` (eigen, gtest), `CMakeLists.txt`
     (library `asv_core::asv_core`, exported); `geodesy.hpp/.cpp`
     (`ecef_from_geo()`, `geo_from_ecef()`, `LocalTangentPlane`);
     `angles.hpp/.cpp` and `test/test_angles.cpp` (moved from `asv_sim`,
     namespace `asv_core`); `test/test_geodesy.cpp`;
     `test/data/geodesy_points.csv` (14 shared points, 2 references × 7 offsets).
   - `asv_sim`: `package.xml` and `CMakeLists.txt` depend on and link
     `asv_core`; `control.cpp`, `guidance.cpp`, `waypoint_follower.cpp`
     call `asv_core::wrap_angle()` / `asv_core::yaw_from_quaternion()`.
   - `asv_sim/tools/geodesy.py`: same maths, `read_geo_reference()`, CSV
     writer; `asv_sim/tests/test_geodesy.py`.
   - Docs: `design.md` §12 (asv_core contents), §13 (Gazebo's conversion
     [SRC]); glossary (φ_G, λ_G, h_G, φ_0, λ_0, h_0, p_0, N_G, a, b, e², e'²,
     proposed; Bowring entry); `AUTONOMY.md` and `README.md` paths;
     CLAUDE.md (baseline, Known state, Layout).
2. **How it was checked**
   - Source: gz-math 7.7.0 is installed (`GZ_MATH_VERSION_FULL "7.7.0"`, header
     has `LOCAL2 = 5`); the conversion was compared line by line with
     gz-math7 `SphericalCoordinates.cc` (`PositionTransform`,
     `UpdateTransformationMatrix`) and gz-sim8 `Util.cc:738`
     (`PositionTransform(xyzPose.Pos(), LOCAL2, SPHERICAL)`).
   - `colcon build --symlink-install --packages-up-to asv_sim`:
     `Summary: 2 packages finished [16.2s]`, no warnings.
   - `colcon test --packages-select asv_core asv_sim`; `colcon test-result`:
     `Summary: 128 tests, 0 errors, 0 failures, 0 skipped` (asv_core 15 =
     `test_angles` 6 + `test_geodesy` 7 + 2 wrappers; asv_sim 113 = 104 + 9).
     A stale `test_angles` result in `build/asv_sim` first made it 134; it was
     removed from the build folder.
   - `python3 -m unittest discover -s tests`: `Ran 67 tests ... OK`
     (56 + 11 new).
   - One-off program over the 14 shared points [MEASURED]:
     `max |C++ - Python| = 0.000e+00 m`, `max round-trip error = 3.313e-09 m`.
   - Plausibility: 100 m E + 100 m N at u = 0 lies 1.571 mm above the
     ellipsoid (CSV), as d²/(2R) = 20000/(2·6.37·10⁶) m ≈ 1.57 mm predicts.
3. **What could not be checked:** that a running Gazebo's clean fixes convert
   back to the true antenna position. That is M2.2's check.
4. **How to read the code:** start at `asv_core/include/asv_core/geodesy.hpp`
   (each function lists its equation), then `src/geodesy.cpp`:
   `ecef_from_geo()` = forward formula with N_G; `LocalTangentPlane`
   constructor = p_0 and R; `to_enu()` = R(p − p_0); `to_geo()` =
   p_0 + Rᵀ·enu then `geo_from_ecef()` (Bowring). `tools/geodesy.py` mirrors
   this function by function.
5. **Numbers for the thesis**
   - WGS84 a = 6378137.0 m, b = 6356752.314245 m [SRC gz-math 7.7.0].
   - Reference point 27.367256° N, −82.451120° E, 0 m, heading 0 [REPO
     `roboboat_course.sdf`].
   - Round-trip error ≤ 3.3 nm up to 5.8 km from the reference; C++ and
     Python identical [MEASURED].
6. **Check your understanding**
   - Why does a point 100 m east of the reference, put on the ellipsoid, come
     out with u < 0 rather than u = 0?
   - What would go wrong in M2.2 if we used gz-math's LOCAL frame instead of
     LOCAL2?
   - Why is the reference point a parameter of the C++ code, not read from the
     SDF there?

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
