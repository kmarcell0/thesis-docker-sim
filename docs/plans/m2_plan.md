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

### Brief (agreed on 2026-10-29)

**1. Goal**
Gazebo publishes clean IMU, GNSS and lidar data; camera pixel noise stays.

**2. Decisions needed**
No D-numbers open (D1 a, D2 a decided). One small design choice inside the brief:
- **Key required or optional?** a) required top-level bool `gazebo_sensor_noise`
  (missing key = clear error; recommended: a forgotten key can't silently bring the
  noise back, which is exactly D2's worry). b) optional, default `true` (old behaviour).

Marcell (2026-10-29): **a**, required key.

**3. Why**
- Every recording after M2.2 must have usable "clean" data; the corruptor and the
  evaluation build on it. Feeds *Vehicle and Sensor Models*.
- The recording check also proves the GNSS chain end to end (Gazebo NavSat → geodesy
  helper → ENU) before any filter relies on it.

**4. New ideas (glossary: "Clean/realistic", "Lever arm")**
- **Clean vs realistic data**: Gazebo gives the perfect measurement; our own node
  adds the errors, with correct units, seeds and logged true biases.
- **Lever arm l**: the antenna isn't at base_link. Antenna position =
  base_link position + R(φ,θ,ψ)·l, l = (−0.76, −0.33, 0.185) m [REPO asv.yaml].
- **Interpolating ground truth to a stamp**: ground truth comes at 50 Hz, fixes at
  10 Hz; we take the truth at the fix's own `header.stamp`.

**5. What will change**
- ⚠ `ws/src/asv_sim/config/asv.yaml`: new top-level key `gazebo_sensor_noise: false`
  (datasheet noise values stay, the corruptor will read them).
- ⚠ `tools/asvgen/config.py`: the key in `SECTIONS` as `BOOL`.
- ⚠ `tools/asvgen/sensors.py`: `build_imu`, `_navsat` (both GNSS antennas) and
  `build_lidar` leave out `<noise>` when the switch is off; cameras and the stereo
  camera ignore the switch. The flag is passed from `build_sensors` (the
  `BUILDERS` dispatch, line ~274) via `cfg`.
- Regenerated with `python3 tools/make_asv.py`: `models/asv/model.sdf`,
  `config/asv_*` (report gets a `gazebo_sensor_noise` line).
- `tests/test_asvgen.py`: switch off → no `<noise>` in imu/navsat/lidar, cameras keep
  it; switch on → noise as before; missing key → ConfigError.
- New `tools/check_clean_gnss.py <bag>` (reuses `analyze_run.read_bag`,
  `geodesy.read_geo_reference` + `LocalTangentPlane.to_enu`) and
  `tests/test_check_clean_gnss.py` (interpolation and lever-arm maths on hand-made data).
- `docs/HOW_IT_WORKS.md` (§5.3 IMU, §5.4 GNSS, lidar section, config reference): the switch.
- `docs/master_plan.md` (mark, §2 item 1 "fixed"), `docs/plans/m2_plan.md` (brief,
  report), `CLAUDE.md` Known state (drop the 890 m caveat; new test baseline).

**6. How it works**
Generator: `if cfg['gazebo_sensor_noise']: add <noise>` for the three sensor types.
Without `<noise>`, gz-sensors adds nothing (to verify in gz-sensors source, not assume).

Check tool, per fix k at stamp t_k, for each antenna (primary l, secondary l₂):
- truth: p(t_k) linear between the two odom samples around t_k, q(t_k) by slerp
- expected antenna position: a_k = p(t_k) + R(q(t_k))·l
- measured: m_k = to_enu(lat_k, lon_k, alt_k)
- error e_k = m_k − a_k; report max and RMS of |e_horizontal| and |e_up|.

**7. How we will check it**
- `grep -c '<noise' model.sdf` inside imu / navsat / lidar sensors = 0; camera sensors
  still have it (the new unittest does this; also shown by hand).
- Python unittest: 67 + new tests, all pass. `colcon test` for `asv_sim` (104 gtests;
  `test_closed_loop` reads the regenerated `asv_report.json`) and `asv_core` (13).
- Headless run: `maneuver.launch.py maneuver:=circle gui:=false` (turning, so the
  lever-arm rotation matters), bag `ws/data/bags/m22_clean_gnss` with `/clock`,
  ground-truth odom, both fixes, IMU; ~60 s sim → ~600 fixes per antenna.
- Expected: largest error a few mm or less (geodesy round trip ≤ 3.3 nm; fixes every
  100 ms land on 20 ms truth samples). One physics step (4 ms) of timing offset at
  2 m/s would show as ~8 mm, so that size of error points to a stamp offset.
  If it's larger: stop and report, don't tune.

**8. What I can't check**
- Nothing visual here. Not checked in this step: that the lidar points and IMU are
  exactly noise-free (only the SDF shows it); lidar `range/resolution` 0.01 may still
  quantise ranges (unverified; I'll look in gz-sensors source and say).


### Report (2026-10-29)

**1. What changed**
- `config/asv.yaml`: new required key `gazebo_sensor_noise: false` (datasheet noise values unchanged).
- `tools/asvgen/config.py`: the key is validated as a bool; missing → error.
- `tools/asvgen/sensors.py`: `build_lidar`, `build_imu`, `_navsat` (both antennas) write `<noise>` only if the switch is on; `NOISE_SWITCHED` lists the types; cameras unchanged.
- `tools/asvgen/analysis.py`: report line "Gazebo noise ON/OFF ..." and `gazebo_sensor_noise` in `asv_report.json`.
- Regenerated `models/asv/model.sdf` (−105 lines: the noise blocks), `model.config`, `config/asv_*` (new fingerprint 39342ea90832).
- `tools/check_clean_gnss.py` (new): per fix, truth interpolated to the stamp + R(q)·l vs. the fix in ENU.
- Tests: `test_asvgen.py` +3 (switch off, switch on, key required), `test_check_clean_gnss.py` (new, 9).
- Docs: `HOW_IT_WORKS.md` §4.1, §5 table note, §5.1, §5.3, §5.4, new §5.5, §12, §14, §16; `design.md` §13 (new fact); `master_plan.md` §2 item 1; `CLAUDE.md` known state and baseline.

**2. How it was checked**
- Sensors in the regenerated `model.sdf` (count of `<noise>`): lidar 0, imu 0, gnss_primary 0, gnss_secondary 0, zed_left 1, zed_right 1, cam_wide_front 1, zed_depth 0 (it never had one).
- gz-sensors 8.2.2 source (the version in the container's `gz_sensors_vendor`): `Lidar.cc`, `ImuSensor.cc`, `NavSatSensor.cc` make a noise model only when the SDF noise type is not NONE, else the value passes through; lidar `RangeResolution()` is never used to quantise ranges [SRC].
- `python3 -m unittest discover -s tests`: `Ran 79 tests ... OK` (was 67).
- `colcon test --packages-select asv_core asv_sim` + `colcon test-result`: `128 tests, 0 errors, 0 failures` (13 + 104 gtests, unchanged).
- Headless run `maneuver.launch.py maneuver:=circle gui:=false`, bag `ws/data/bags/m22_clean_gnss`: 66.4 s, 664 fixes per antenna, 3321 ground-truth samples; boat up to 0.98 m/s, |roll| ≤ 0.0063 rad, |pitch| ≤ 0.0158 rad.
- `tools/check_clean_gnss.py`:
  ```
  /asv/gnss/fix (gnss_primary_link, lever arm [-0.76, -0.33, 0.185] m): 664 fixes, 664 stamped exactly on a truth sample
    horizontal max 3.556e-09 m, RMS 1.891e-09 m; vertical max 2.792e-09 m, RMS 8.879e-10 m
  /asv/gnss/secondary/fix (...[0.76, 0.33, 0.185] m): 664 fixes, 664 stamped exactly on a truth sample
    horizontal max 3.424e-09 m, RMS 1.838e-09 m; vertical max 2.539e-09 m, RMS 8.767e-10 m
  largest difference: 3.556e-09 m
  ```
  Same fixes compared without the lever arm: 0.83 m, so the check is sensitive to it.

**3. What could not be checked**
- The interpolation path never ran on real data: every fix stamp lands on a ground-truth sample (100 ms vs 20 ms). It is covered by unit tests only.
- The IMU and lidar outputs were not compared with truth in a run; only the SDF and the gz-sensors source show they are noise-free. Marcell can look at `/asv/imu/data` at rest in a GUI run if he wants a visual check.
- The circle runs at about 1 m/s, so a one-physics-step (4 ms) stamp offset would have shown as about 4 mm, not 8 mm as in the brief (2 m/s).

**4. How to read the code**
- Start at `sensors.expand()`: it passes `gazebo_noise` only to the types in `NOISE_SWITCHED`.
- `build_imu`, `_navsat`, `build_lidar`: the `... if gazebo_noise else None` (E() skips None).
- `check_clean_gnss.py`: `pose_at()` = p(t_k), q(t_k) (linear + `slerp()`); `rotate()` = R(q)·l; `antenna_errors()` = e_k = m_k − a_k; `summary()` = max/RMS.

**5. Numbers for the thesis**
- Clean GNSS fix vs. true antenna position: largest 3.6 nm horizontal, 2.8 nm vertical, 664 fixes per antenna, circle maneuver ~1 m/s [MEASURED, bag `m22_clean_gnss`]. The size is the geodesy round trip (≤ 3.3 nm, M2.1), not a sensor error.
- GNSS lever arms: primary (−0.76, −0.33, 0.185) m, secondary (0.76, 0.33, 0.185) m [REPO `asv_frames.yaml`].
- Gazebo NavSat applies horizontal noise in degrees (0.008 → ~890 m) [SRC, gz-sensors #325].

**6. Check your understanding**
1. Why is the key required instead of having a default value?
2. If the fix stamps were one physics step (4 ms) late, about how large would the error be on this run, and in which direction?
3. Why does dropping the lever arm give ~0.83 m and not exactly |l| = 0.85 m?

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
  GnssHeading (`design.md` §4.3), GnssStatus. GNSS velocity uses
  geometry_msgs/TwistWithCovarianceStamped (no new type).

### Brief (agreed on 2026-10-29)

**Summary (in plain words)**
Our programs will soon send each other things ROS has no ready-made type for:
"a red buoy 6 m away, 20° to the left", "the buoy map", "the two GNSS antennas
say the bow points 87° from north", "the GNSS is in RTK fixed mode". ROS programs
exchange data only in agreed shapes called messages. This step defines those
shapes, nothing else: a new package `asv_msgs` with 7 small `.msg` files, every
field commented with unit and frame. At build time ROS turns them into C++ and
Python code. No sensor, filter or simulation logic changes; the corruptor
(M2.6), buoy map (M2.7), fake detector (M3.1) and filter (M3) use them later.

**1. Goal:** a new interface package `asv_msgs` with the project's own message
types, every field documented with unit and frame.

**2. Decisions needed** (no D-number open; two small design choices)
- A. Noise σ in a buoy detection? a) no, the filter takes Q from its config
  (recommended); b) `range_std`, `bearing_std` per detection.
- B. Name of the status message? a) keep `SensorStatus` (recommended);
  b) rename to `GnssStatus`.

Marcell (2026-10-29): **A a** (no σ), **B b** (`GnssStatus`; changes
`design.md` §1 and §12, `master_plan.md` M2.4, this section).

**3. Why:** corruptor, ground-truth map, fake detector and filter need typed
messages; an own package is usable on the real boat without the simulator.
Feeds: Software Architecture.

**4. New ideas:** rosidl / interface package (glossary); constants in a message
(`uint8 RED=1`).

**5. What will change:** ⚠ new package `ws/src/asv_msgs/` (deps
`rosidl_default_generators`, `rosidl_default_runtime`, `std_msgs`, all already
in the container); 7 `.msg` files; new Python test
`asv_sim/tests/test_msgs_documented.py`; docs (`design.md` §1/§12, this plan,
`master_plan.md`, `CLAUDE.md`).

**6. How it works (fields)**
- `BuoyColour`: `uint8 value`; UNKNOWN=0, RED=1, GREEN=2, YELLOW=3, BLACK=4.
- `BuoyDetection`: `range` m, `bearing` rad (base_link origin, level frame, CCW
  from x forward, (−π, π]); `BuoyColour colour`; `object_class` BUOY=0 /
  OTHER=1. No true ID, no σ.
- `BuoyDetectionArray`: Header (measurement time, `base_link`), list; also
  published when empty.
- `Landmark`: id, name (GT), colour, model (GT), radius m, x, y m in map,
  2×2 covariance m² row-major (zeros for GT), sightings, confirmed.
- `LandmarkArray`: Header (`map`), list.
- `GnssHeading`: Header (GNSS epoch), azimuth α rad clockwise from north,
  [0, 2π), α = π/2 − ψ; `azimuth_std` rad; `baseline_fixed`.
- `GnssStatus` (GT only): Header; position_mode NO_FIX=0 / STANDALONE=1 /
  RTK_FLOAT=2 / RTK_FIXED=3; `sky_view_outage`, `corrections_lost`,
  `heading_available`.

**7. How we will check it:** `colcon build` of `asv_msgs`; `ros2 interface show`
for all 7 types; new unittest (every field commented with unit/frame; Python
import; constant values). Python tests 79 → ~81; gtests unchanged.

**8. What I can't check:** nothing visual in this step.

### Report (2026-10-29)

**1. What changed**
- `ws/src/asv_msgs/` (new): `package.xml` (rosidl generators, runtime,
  `std_msgs`; member of `rosidl_interface_packages`), `CMakeLists.txt`
  (`rosidl_generate_interfaces` for the 7 files).
- `msg/BuoyColour.msg`, `BuoyDetection.msg`, `BuoyDetectionArray.msg`,
  `Landmark.msg`, `LandmarkArray.msg`, `GnssHeading.msg`, `GnssStatus.msg`:
  fields as in the brief; a file comment says what the message is for and its
  frame; every field ends with `# [unit] meaning`.
- `asv_sim/tests/test_msgs_documented.py` (new, 5 tests): the 7 planned types
  exist; every field has a `# [unit]` comment; spatial types name their frame;
  generated Python constants and defaults.
- Docs: `design.md` §1 (GnssStatus) and §12 (message summary); `glossary.md`
  (message constants); `master_plan.md` M2.4; `CLAUDE.md` known state, layout,
  baseline.

**2. How it was checked**
- `colcon build --symlink-install --packages-select asv_msgs`:
  `Finished <<< asv_msgs [8.01s]`.
- `ros2 interface package asv_msgs`: lists all 7 types; `ros2 interface show`
  prints the field comments.
- `python3 -m unittest discover -s tests`: `Ran 84 tests ... OK` (was 79).
- The unit regex rejects `float64 range` and `float64 range  # metres`, accepts
  `float64 range  # [m] ok`.
- `colcon test --packages-select asv_msgs asv_core asv_sim` + `colcon
  test-result`: `128 tests, 0 errors, 0 failures` (13 + 104 gtests, unchanged;
  asv_msgs has no tests of its own).

**3. What could not be checked:** no message is published yet; the first real
use is M2.6 (corruptor) and M2.7 (true map).

**4. How to read the code:** start with `BuoyDetection.msg` (the filter's main
input), then `Landmark.msg`. `CMakeLists.txt` is the list of files rosidl turns
into code. No equations here; `GnssHeading.msg` states α = π/2 − ψ (§4.3).

**5. Numbers for the thesis:** none (constants are labels, not parameters).

**6. Check your understanding**
1. Why is a scan with no buoys still published as an empty array?
2. Why does `BuoyDetection` have no true buoy ID, and where does the evaluation
   get it from?
3. Why must the filter never subscribe to `GnssStatus`?

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

### Brief (agreed on 2026-10-29)

**Summary (in plain words)**
Since M2.2 Gazebo gives perfect sensor readings. Real sensors have three kinds
of error: a small random jitter on every reading (white noise), a fixed offset
chosen when the sensor powers on (turn-on bias), and an offset that slowly
drifts over minutes (Gauss-Markov bias). Example: a gyro lying still should
read 0; a real one reads 0.0012, 0.0003, −0.0008 rad/s … around an offset that
slowly wanders. This step writes the small C++ "dice" that produce these three
errors; M2.6 adds them to the clean data. Each sensor gets its own seeded dice:
the same seed always gives exactly the same noise, and changing the GNSS noise
never changes the IMU noise. Statistics tests prove the dice match the maths.
No node, no topic, no Gazebo run, no value in `asv.yaml` changes.

**1. Goal:** white noise, turn-on bias and Gauss-Markov bias in `asv_core`,
one seeded random stream per noise source, time steps from timestamps.

**2. Decisions needed** (two design choices, no D-number)
- A. Density η → per-sample σ: a) σ = η/√Δt (recommended; ADEV = η/√τ,
  matches `design.md` §4.1 and `asv_report.json`); b) σ = η/√(2Δt) (TDK's
  one-sided definition, √2 smaller).
- B. Gauss-Markov start: a) random draw from N(0, σ_ss²) (recommended,
  stationary from t = 0); b) zero (too-good IMU for about T_c = 300 s).

Marcell (2026-10-29): **A a**, **B a**.

**3. Why:** the error model of every simulated sensor (IMU M2.6a, GNSS
M2.6b/c). Feeds: Noise Models; Vehicle and Sensor Models.

**4. New ideas:** noise density η (per √Hz, per-sample σ depends on Δt from
stamps); exact Gauss-Markov step (keeps σ_ss whatever Δt is; IMU intervals are
8, 4, 4, 4 ms); common random numbers (one stream per source, seeded from
(global seed, source name)).

**5. What will change:** `asv_core/include/asv_core/noise.hpp`, `src/noise.cpp`
(new); `test/test_noise.cpp`, `test/allan.hpp` (new, test-only overlapping
Allan deviation); `asv_core/CMakeLists.txt`; docs `design.md` §4.1/§4.5,
`glossary.md`, this plan, `master_plan.md`, `CLAUDE.md`. No new dependency.

**6. How it works:** measured = true + b₀ + b_k + n_k.
- White: n_k = σ_w·w_k, σ_w = η/√Δt_k.
- Turn-on: b₀ ~ N(0, σ₀²), one draw per run.
- Gauss-Markov: φ_k = e^(−Δt_k/T_c), b_{k+1} = φ_k·b_k + σ_ss·√(1−φ_k²)·w_k,
  b at start drawn from N(0, σ_ss²).
- Random numbers: `std::mt19937_64` (output fixed by the C++ standard), seeded
  with SplitMix64(seed XOR FNV-1a(name)); own Marsaglia polar normal draw.
- Δt ≤ 0 throws.

**7. How we will check it:** same seed and name → same numbers, different
names → different streams; Allan helper on a ramp gives a·τ/√2; the three
tests of `design.md` §4.5. Tolerance 5 % with one fixed seed if the largest
error over 10 seeds is ≤ 2.5 %, otherwise stop and show the numbers. Expected:
`asv_core` gtests 13 → about 21; `asv_sim` 104 and Python 84 unchanged.

**8. What I can't check:** that the model matches a real Pixhawk (the tests
only prove the code implements the model).

**During the build** (2026-10-29): with 10 h records the worst error over 10
seeds was above 2.5 % (test 2 autocorrelation at 5 s 3.66 %, ADEV 2.74 %;
test 3 at τ = 30 s 4.54 % gyro, 4.67 % accel). Marcell chose **a)**: tests 2
and 3 use 40 h records (worst 2.08 %), test 1 stays 10 h, tolerance 5 %.

### Report (2026-10-29)

**1. What changed**
- `asv_core/include/asv_core/noise.hpp`, `src/noise.cpp` (new):
  `stream_seed()`, `RandomStream` (mt19937_64 + polar normal draw),
  `WhiteNoise`, `TurnOnBias`, `GaussMarkov`.
- `asv_core/test/allan.hpp` (new, test-only): overlapping Allan deviation,
  analytic Gauss-Markov Allan variance, mean, std, autocorrelation.
- `asv_core/test/test_noise.cpp` (new, 18 gtests): streams (seed, repeat,
  pinned values, N(0,1) moments), white noise (σ, bad input, integral variance
  with 8/4/4/4 ms steps), turn-on bias, Gauss-Markov (bad input, zero size,
  stationary start, uneven steps), Allan helper on a ramp, `design.md` §4.5
  tests 1–3 (gyro and accel), and a check that test 3 would catch a missing
  bias.
- `asv_core/CMakeLists.txt`: `src/noise.cpp`, `test_noise` (timeout 300 s).
- Docs: `design.md` §4.1 (convention, stationary start), §4.5 (as built,
  tolerances, record lengths); `glossary.md` (noise density, common random
  numbers, Gauss-Markov); `decision_notes.md` (convention settled);
  `master_plan.md`; `CLAUDE.md` (known state, baseline).

**2. How it was checked**
- Tolerance sweep (scratch program, 10 seeds, worst |relative error|):
  10 h: test 1 0.28 / 0.70 / 1.96 %; test 2 σ 1.50 %, autocorrelation 0.67 /
  3.66 %, ADEV 2.74 %; test 3 gyro 0.64 / 1.72 / 4.54 %, accel 0.66 / 2.41 /
  4.67 %. 40 h: test 1 0.08 / 0.31 / 0.70 %; test 2 0.77 %, 0.23 / 1.43 %,
  1.12 %; test 3 gyro 0.35 / 0.70 / 1.78 %, accel 0.42 / 1.03 / 2.08 %.
- Extra statistics tests, 10 seeds: white integral 0.89 %, Gauss-Markov with
  uneven steps σ 0.76 % and correlation 0.38 % (tolerance 5 %); turn-on spread
  1.25 %, Gauss-Markov start spread 1.07 % (tolerance 2.5 %).
- First run: `WhiteNoise.IntegralVarianceWithUnevenSteps` failed (ratio 1.121).
  This was a test bug: in the 8/4/4/4 ms pattern 1 s is 200 samples, not 250
  (√1.25 = 1.118). Fixed in the test, and the same lag in the Gauss-Markov
  uneven test. The library was not changed.
- Pinned values: the same with `-O0` and `-O2`.
- `colcon test --packages-select asv_core asv_sim` + `colcon test-result`:
  `147 tests, 0 errors, 0 failures, 0 skipped` (asv_core 13 + 18 = 31 gtests,
  asv_sim 104 unchanged, 12 CTest wrappers). `test_noise` takes 19.4 s
  (debug build).
- `python3 -m unittest discover -s tests`: `Ran 84 tests ... OK` (unchanged).

**3. What could not be checked:** whether a real Pixhawk follows this model;
nothing uses the library yet (first user M2.6a).

**4. How to read the code:** start with `noise.hpp` (the equations are in the
comments), then `noise.cpp`: `WhiteNoise::sample()` = n_k = σ_w·w_k with
σ_w = η/√dt (`design.md` §4.1); `TurnOnBias` constructor = b₀ ~ N(0, σ₀²);
`GaussMarkov::step()` = b_{k+1} = φ_k·b_k + σ_ss·√(1−φ_k²)·w_k;
`RandomStream::normal()` = polar method. In the tests, `allan.hpp`
`allan_deviation()` and `gauss_markov_allan_variance()` are the §4.5 formulas.

**5. Numbers for the thesis**
- Per-sample gyro σ at 200 Hz: 9.379e-4 rad/s (η = 6.632e-5 rad/s/√Hz [DS],
  σ = η/√dt).
- Test tolerance 5 %, record lengths 10 h / 40 h at 200 Hz; worst errors over
  10 seeds above [MEASURED].

**6. Check your understanding**
1. Why does the white noise per sample get larger when the IMU steps get
   shorter, and what stays the same?
2. Why does the Gauss-Markov push use √(1−φ²) and not a fixed size?
3. Why does each noise source have its own stream instead of one stream per
   sensor?

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

#### Brief (agreed on 2026-10-29)

**Summary (in plain words)**
Since M2.2 Gazebo's IMU is perfect. In M2.5 we wrote the noise "dice"; this
step uses them. A new node, `corruptor`, reads every clean IMU message, adds
three errors on each axis and publishes the result as the "realistic" IMU.
Example: a clean yaw rate of 0.100 rad/s may come out as 0.1021 (0.0017
turn-on offset + a slowly drifting offset + a little jitter). It removes the
orientation (a real raw IMU does not measure heading) and publishes the true
offsets on a separate ground-truth topic, so the evaluation can later compare
them with the filter's estimate. IMU only for now: GNSS joins in M2.6b/c, the
recording check is M2.6d. No change to Gazebo, the world or the filter.

**1. Goal:** `/asv/sensors/imu/data`: gyro and accelerometer with white noise,
turn-on bias and Gauss-Markov bias, the original stamp, no orientation.

**2. Decisions needed** (D11 questions 8 and 9)
- Q8 per-run scale-factor and misalignment error: a) leave out now, a
  limitation (recommended); b) a switch, off in nominal; c) on in nominal.
- Q9 ⚠ accelerometer density in `asv.yaml` 70 → 110 µg/√Hz (±32 g range,
  DS-000577 Table 2): a) yes (recommended); b) keep 70 µg/√Hz.

Marcell (2026-10-29): **Q8 a**, **Q9 a** (asked first what "the filter does not
use the accelerometer" means: with D5 C the prediction uses GNSS velocity and
gyro yaw rate only, so the change affects the parameter table, not results).

**3. Why:** the filter (M3) must never see perfect data. Feeds: Vehicle and
Sensor Models; Noise Models; App. B Parameter Tables.

**4. New ideas:** noise profile (`noise_profiles/nominal.yaml`: seed and
multipliers gyro/accel × white/turn-on/drift, 1 = datasheet, 0 = off; T_c
override; nominal values stay in `asv.yaml`); "not available" fields
(`orientation_covariance[0] = −1`; the other covariances all zero = unknown,
the filter takes its noise from its own config); true-bias log (b₀ + b_GM per
axis on `/asv/ground_truth/imu_bias`, sensor_msgs/Imu, same stamp).

**5. What will change:** `asv_core`: `imu_corruptor.hpp/.cpp` (ROS-free, 3
axes × 2 sensors × 3 parts = 18 named streams like `imu.gyro_z.white`),
`test/test_imu_corruptor.cpp`, `CMakeLists.txt`. `asv_sim`:
`include/asv_sim/imu_corruption.hpp` + src (profile merge with `asv.yaml`,
clean Imu message → corrupted one), `src/corruptor_node.cpp` (I/O only),
`config/noise_profiles/nominal.yaml`, `test/test_imu_corruption.cpp`,
`CMakeLists.txt`, ⚠ `package.xml` `<depend>sensor_msgs</depend>`. ⚠
`asv.yaml` (Q9 a) + `make_asv.py`. Docs: `design.md` §1, §4.1, `glossary.md`,
`decision_notes.md`, this plan, `master_plan.md`, `CLAUDE.md`.

**6. How it works:** per axis i: measured_i = true_i + b₀,i + b_GM,i(k) +
n_i(k); n = η/√Δt · w with Δt from consecutive stamps; b₀ ~ N(0, σ₀²) once per
run; b_GM(k+1) = φ·b_GM(k) + σ_ss·√(1−φ²)·w, φ = e^(−Δt/T_c). The first
message (no Δt) and messages with Δt ≤ 0 are dropped (warning). The
accelerometer keeps gravity as Gazebo gives it (specific force).

**7. How we will check it:** `asv_core` gtests: same seed → same output;
output − logged bias has mean ≈ 0 and spread η/√Δt (5 %; gyro at 4 ms
1.049e-3 rad/s); turn-on spread over seeds = σ₀; multiplier 0 switches a part
off exactly. `asv_sim` gtests: two inputs differing only in yaw give
bit-identical outputs; `orientation_covariance[0] = −1`; stamp copied; first
message dropped; profile merge. Live: headless sim + `ros2 run asv_sim
corruptor`, `ros2 topic hz` ≈ 200 Hz on both topics. Counts: `asv_core` 31 →
about 39, `asv_sim` 104 → about 110, Python 84.

**8. What I can't check:** that these errors match a real Pixhawk 6X.

#### Report (2026-10-29)

**1. What changed**
- `asv_core/include/asv_core/imu_corruptor.hpp`, `src/imu_corruptor.cpp`
  (new): `ImuCorruptor::process()` = measured = true + b₀ + b(k) + n(k) on
  3 axes of gyro and accelerometer; returns the true bias too; drops the first
  sample and stamps that are not later than the last (integer ns, no state
  change).
- `asv_core/test/test_imu_corruptor.cpp` (new, 8 gtests); `CMakeLists.txt`.
- `asv_sim/include/asv_sim/imu_corruption.hpp`, `src/imu_corruption.cpp`
  (new, library `asv_sim_corruptor`): `imu_error_params()` (asv.yaml ×
  profile), `profile_seed()`, `corrupt_imu()` (clean Imu → data + bias
  messages).
- `asv_sim/src/corruptor_node.cpp` (new): node `corruptor`, I/O only.
- `asv_sim/config/noise_profiles/nominal.yaml` (new): seed 1, all multipliers 1.
- `asv_sim/test/test_imu_corruption.cpp` (new, 7 gtests); `CMakeLists.txt`;
  ⚠ `package.xml` `<depend>sensor_msgs</depend>`.
- ⚠ `asv.yaml`: `accel_noise_density` 6.865e-4 → 1.079e-3 (Q9 a); regenerated
  with `make_asv.py`: only the fingerprint lines and the report's IMU line
  changed (0.99 mg → 1.555 mg per sample); no physics change.
- Docs: `design.md` §1 (new topic `/asv/ground_truth/imu_bias`), §4.1
  (density, as built, left-out scale/misalignment), §4.5 (note: its
  accelerometer numbers use the M2.5 value); `glossary.md` (noise profile,
  "not available" field, specific force); `decision_notes.md` (D11 q8 a, q9 a);
  `master_plan.md`; `CLAUDE.md`; `HOW_IT_WORKS.md` §5 (report number 1.55 mg).

**2. How it was checked**
- First `test_imu_corruptor` run: 2 failures in
  `ErrorMinusBiasIsWhiteNoiseOfTheProfile` (ratio 0.913). Test bug: the
  "4 ms steps" filter `i % 4 != 0` also took the 8 ms step (it ends at
  `i % 4 == 1`). Fixed by selecting on the step's own dt; library unchanged.
- `colcon test --packages-select asv_core asv_sim` + `colcon test-result`:
  `164 tests, 0 errors, 0 failures, 0 skipped` (asv_core 31 + 8 = 39,
  asv_sim 104 + 7 = 111, 14 CTest wrappers). `test_imu_corruptor` 2.4 s.
- `python3 -m unittest discover -s tests`: `Ran 84 tests ... OK`.
- Live (headless sim + `ros2 run asv_sim corruptor`): log `profile 'nominal',
  seed 1; gyro eta 6.632e-05, b0 0.0017, b_ss 0.0002; accel eta 0.001079,
  b0 0.02, b_ss 0.005; T_c 300 s`, no warnings. `ros2 node info /corruptor`:
  subscribes `/asv/imu/data`, publishes `/asv/sensors/imu/data`,
  `/asv/ground_truth/imu_bias`. `ros2 topic hz` ≈ 184 Hz (wall clock,
  real-time factor below 1).
- 22 s bag `ws/data/bags/m26a_check` (script `ws/data/m26a_check.py`, not
  committed): 4335 corrupted stamps = 4335 clean stamps in the same window,
  0 missing, 0 extra; 200.02 Hz in sim time; steps 4 and 8 ms; every message
  `orientation_covariance[0] = −1`, quaternion 0. Corrupted − clean − bias on
  4 ms steps: gyro std 1.0445e-3 / 1.0494e-3 / 1.0582e-3 rad/s (expected
  1.049e-3), accel 1.7099e-2 / 1.6973e-2 / 1.6548e-2 m/s² (expected
  1.706e-2; n = 3251 each, so about ±1.2 % from chance; z is −3.0 %).
  Clean accel z mean 9.798 m/s² (specific force).

**3. What could not be checked:** that the model matches a real Pixhawk 6X;
the corruptor is not in a launch file yet (M2.6d).

**4. How to read the code:** start with `imu_corruptor.hpp` (equations in the
header comment), then `ImuCorruptor::process()` and `corrupt()` in
`imu_corruptor.cpp` (b = b₀ + b(k) with b(k) stepped by dt; measured = true + b
+ n). Then `imu_corruption.cpp`: `imu_error_params()` (asv.yaml × profile),
`corrupt_imu()` (builds the output from the new numbers only). The node
`corruptor_node.cpp` only wires topics.

**5. Numbers for the thesis**
- Accelerometer density 1.079e-3 m/s²/√Hz (110 µg/√Hz at ±32 g) [DS
  DS-000577 Table 2]; per sample at 200 Hz 1.55 mg [REPO asv_report.json].
- Per-sample σ on 4 ms steps: gyro 1.049e-3 rad/s, accel 1.706e-2 m/s² [REPO,
  σ = η/√dt]; measured in Gazebo 1.0445–1.0582e-3 rad/s and
  1.6548–1.7099e-2 m/s² [MEASURED, 22 s].
- Nominal profile: seed 1, all multipliers 1 [REPO].

**6. Check your understanding**
1. Why does the corruptor drop the very first IMU message?
2. Two runs differ only in the true yaw, yet the outputs are bit-identical.
   Why does that prove the true yaw cannot leak?
3. Why can the accelerometer density change without changing any filter
   result (D5 C)?

### M2.6b GNSS position
**Goal:** `/asv/sensors/gnss/fix`: antenna position + white and slowly varying
error per mode (RTK fixed / float / standalone), outages from a schedule,
covariance per mode; true mode on `/asv/ground_truth/gnss_status`.
**Done when:** unit tests per mode; a mode switch redraws the slow error; no
message during an outage.
**Decisions needed:** none (D6 is the filter's side, M3.3b). The brief
proposes the white/slow split of each mode's 1σ and the covariance convention
(`design.md` §4.2, `decision_notes.md` questions 3 and 5).

#### Brief (agreed on 2026-11-02)

**Summary (in plain words)**
Since M2.2 Gazebo's GNSS fix is perfect (the exact antenna position). A real
UM982 fix jitters a little every epoch (white part) and sits a bit off for
minutes (slow part); both depend on the mode: RTK fixed (cm), RTK float (dm),
standalone (m), or no fix in a sky-view outage. The `corruptor` node (M2.6a)
now does this for GNSS: each clean fix gets the error of the current mode and
goes out on `/asv/sensors/gnss/fix` with the covariance a receiver would
report. The mode over time comes from a schedule in the noise profile, so an
outage experiment (D9 a) is just another profile. The true mode and the true
slow error go to ground-truth topics. Velocity and heading: M2.6c; launch file
and recording check: M2.6d. No change to Gazebo, the world or the filter.

**1. Goal:** `/asv/sensors/gnss/fix` = antenna fix + white and slow error per
mode, no message in an outage, covariance per mode; true mode on
`/asv/ground_truth/gnss_status`.

**2. Decisions needed** (brief questions; D6 itself stays open)
- Q1 numbers per mode, per axis 1σ (vertical scaled by the same ratios):

  | Mode | total H | white H | slow σ_ss H | T_c | total V |
  |---|---|---|---|---|---|
  | rtk_fixed | 0.008 m [DS] | 0.005 | 0.006 | 60 s | 0.015 m [DS] |
  | rtk_float | 0.20 m [EST] | 0.05 | 0.19 | 60 s | 0.40 m [EST] |
  | standalone | 1.5 m [EST] | 0.3 | 1.47 | 300 s | 2.5 m [EST] |

  +1 ppm left out (≤ 1 mm within 1 km of the base).
- Q2 (D6 q5) reported covariance: a) total white² + slow² (recommended); b) white only.
- Q3 (D6 q3) same mode after an outage: a) slow error keeps evolving across
  the gap (exact Gauss-Markov step, recommended); b) redraw if gap > T_gap;
  c) always redraw.
- Q4 "0.8 cm horizontal RMS": a) per axis, σ_E = σ_N = 8 mm (recommended);
  b) 2D, 5.7 mm per axis.

Marcell (2026-11-02): **Q1 the table**, **Q2 a**, **Q3 a**, **Q4 a**.

**3. Why:** the filter must never see a perfect fix; GNSS modes and outages are
core (D9 a); D6 only matters with a real slow error. Feeds: Vehicle and Sensor
Models; Noise Models; Ground Truth and Data Recording.

**4. New ideas:** mode schedule (list in the profile, times from the first
clean fix; nominal = rtk_fixed all run); white/slow split of one datasheet 1σ;
redraw at a mode switch (fresh N(0, σ_ss²) draw, the position jump when RTK
returns [LIT zhang2026impact]).

**5. What will change:** `asv_core`: `gnss_corruptor.hpp/.cpp`,
`GaussMarkov::redraw()` in `noise.hpp/.cpp`, `test_gnss_corruptor.cpp`, one
test in `test_noise.cpp`, `CMakeLists.txt`. `asv_sim`: `gnss_corruption.hpp/.cpp`,
`corruptor_node.cpp`, `noise_profiles/nominal.yaml`, `test_gnss_corruption.cpp`,
`CMakeLists.txt`, ⚠ `package.xml` `<depend>asv_msgs</depend>`. ⚠ `asv.yaml`
(per-mode table), ⚠ generator `tools/asvgen/config.py` (accept the keys),
`make_asv.py` rerun. Docs: `design.md` §1, §4.2, `glossary.md`,
`decision_notes.md`, this plan, `master_plan.md`, `CLAUDE.md`.

**6. How it works:** per epoch, mode m from the schedule; outage → no fix
(status still published). Else per axis j ∈ {E, N, U}: e_j = s_m,j(k) +
n_m,j(k), n ~ N(0, σ_w²) per epoch (GNSS spec is per epoch, not a density),
s = Gauss-Markov(σ_ss, T_c) stepped by Δt from stamps; mode switch → redraw of
the new mode's s; same mode after a gap → s stepped over the gap (Q3 a). The
error is added in an ENU frame centred on the clean fix (axes differ from the
world's by d/R_E: 24 µm at 100 m and 1.5 m error). Covariance diag(σ_w² +
σ_ss²), DIAGONAL_KNOWN; status standalone → STATUS_FIX, float/fixed →
STATUS_GBAS_FIX. True slow error on `/asv/ground_truth/gnss_error`
(geometry_msgs/Vector3Stamped, ENU, m). The first fix is published; stamps not
later than the last are dropped.

**7. How we will check it:** `asv_core` gtests per mode (white spread 5 %,
slow spread over seeds, total covariance, redraw at a switch, outage, gap rule,
same seed, multiplier 0); `asv_sim` gtests (corrupted − clean in world ENU = e
within 0.1 mm, schedule errors, status flags, covariance). Live: ≈ 10 Hz on
fix and status; short bag white spread ≈ 5 mm. Counts: `asv_core` 39 → about
49, `asv_sim` 111 → about 118, Python 84.

**8. What I can't check:** that the float and standalone numbers match a real
UM982.

#### Report (2026-11-02)

**1. What changed**
- `asv_core/include/asv_core/gnss_corruptor.hpp`, `src/gnss_corruptor.cpp`
  (new): `GnssMode`, `GnssSchedule` (mode over time), `GnssCorruptor::process()`
  = per ENU axis ε_G = β_G + σ_w·w; redraw at a mode switch, step over the gap
  in the same mode; reports the total σ.
- `asv_core/noise.hpp/.cpp`: `GaussMarkov::redraw()` and `sigma_ss()`.
- `asv_core/test/test_gnss_corruptor.cpp` (new, 11 gtests); one new test in
  `test_noise.cpp`; `CMakeLists.txt`.
- `asv_sim/include/asv_sim/gnss_corruption.hpp`, `src/gnss_corruption.cpp`
  (new, in library `asv_sim_corruptor`): `gnss_error_params()`,
  `gnss_schedule()`, `corrupt_gnss()` (NavSatFix → fix + GnssStatus + slow
  error).
- `asv_sim/src/corruptor_node.cpp`: subscribes `/asv/gnss/fix`; publishes
  `/asv/sensors/gnss/fix`, `/asv/ground_truth/gnss_status`,
  `/asv/ground_truth/gnss_error`.
- `asv_sim/config/noise_profiles/nominal.yaml`: `gnss:` block (white 1, slow
  1, schedule rtk_fixed from 0).
- `asv_sim/test/test_gnss_corruption.cpp` (new, 7 gtests); `CMakeLists.txt`;
  ⚠ `package.xml` `<depend>asv_msgs</depend>`.
- ⚠ `asv.yaml`: `unicore_um982_rtk.position_modes` (Q1 table). ⚠ generator
  `tools/asvgen/config.py`: new kind `mapping`, optional `position_modes` for
  `gnss`/`gnss_dual`; `tests/test_asvgen.py` +1 test. `make_asv.py` rerun:
  only the fingerprint lines of the 6 generated files changed
  (80b8183fdba9 → 301b46547e66).
- Docs: `design.md` §1 (new topic `/asv/ground_truth/gnss_error`), §4.2
  (table, model, as built); `glossary.md` (σ_w, ε_G, β_G proposed; mode
  schedule, white/slow split, redraw); `decision_notes.md` (D6 q3, q5
  answered); `HOW_IT_WORKS.md` (the key); `master_plan.md`; `CLAUDE.md`.

**2. How it was checked**
- `colcon build --packages-up-to asv_sim`: no warnings.
- `colcon test --packages-select asv_core asv_sim` + `colcon test-result`:
  `185 tests, 0 errors, 0 failures, 0 skipped` (asv_core 39 + 12 = 51:
  `test_gnss_corruptor` 11, `test_noise` 19; asv_sim 111 + 7 = 118; 16 CTest
  wrappers). All new tests passed on the first run. `test_gnss_corruptor` 5.6 s.
- `python3 -m unittest discover -s tests`: `Ran 85 tests ... OK`.
- Live (headless sim + `ros2 run asv_sim corruptor`): log `GNSS rtk_fixed
  white 0.005, slow 0.006 m; schedule: rtk_fixed@0.000000s`; `ros2 node info
  /corruptor` shows `/asv/gnss/fix` in and the three new topics out; `ros2
  topic hz` 8.6 Hz wall clock (real-time factor about 0.86).
- 33 s bag `ws/data/bags/m26b_check` (script `ws/data/m26b_check.py`, not
  committed): 329 corrupted fixes = 329 clean fixes in the same window, 0
  missing, 0 extra; status stamps = fix stamps; 10.000 Hz sim time; mode 3
  (RTK fixed) throughout. Corrupted − clean − β_G in world ENU (Python
  geodesy): mean −0.27 / −0.24 / +0.13 mm, std 5.15 / 5.13 / 8.64 mm
  (expected 5.0 / 5.0 / 9.4 mm; with n = 329 the std scatters ±3.9 %, so up
  is −8 %, about two of those). Covariance diag 6.1e-5, 6.1e-5, 2.16e-4 m²
  (= 7.81² mm², 14.70² mm²), type 2 (DIAGONAL_KNOWN), status 2 (GBAS_FIX).

**3. What could not be checked:** that the numbers match a real UM982 (the
split, float, standalone and T_c values are estimates); the outage and mode
switches were only checked in unit tests, not live (the nominal profile has
none; M2.6d or the M6 profiles will show them in Gazebo).

**4. How to read the code:** start with `gnss_corruptor.hpp` (the equations
in the header comment), then `GnssCorruptor::process()`: schedule → mode;
outage returns early; `same_mode ? slow.step(dt) : slow.redraw()` is the Q3
rule; `error = slow + white_sigma * normal()` is ε_G = β_G + σ_w·w. Then
`gnss_corruption.cpp`: `gnss_error_params()` (asv.yaml × profile),
`corrupt_gnss()` (ENU error → lat/lon with `LocalTangentPlane`, covariance,
status). The node only wires topics.

**5. Numbers for the thesis**
- Per-axis 1σ white/slow/T_c: RTK fixed 0.005 / 0.006 m / 60 s (v 0.0094 /
  0.0113 m), RTK float 0.05 / 0.19 m / 60 s (v 0.10 / 0.38 m), standalone
  0.3 / 1.47 m / 300 s (v 0.5 / 2.45 m) [EST, `asv.yaml`; RTK fixed totals
  8 mm / 15 mm DS].
- Reported totals: 7.81 mm, 0.196 m, 1.500 m horizontal per axis [REPO].
- Live white spread RTK fixed 5.15 / 5.13 / 8.64 mm, n = 329 [MEASURED].

**6. Check your understanding**
1. Why is the GNSS white noise not divided by √dt, while the IMU's is?
2. GNSS drops out for 2 s in RTK fixed and comes back in RTK fixed. Roughly
   how similar is the slow error before and after (T_c = 60 s)? And after a
   10-minute outage?
3. Why does the filter only learn the mode through `position_covariance`?

### M2.6c GNSS velocity and dual-antenna heading
**Goal:** `/asv/sensors/gnss/velocity` (ENU, from ground-truth position
differences + noise, D5 C) and `/asv/sensors/gnss/heading` (α = π/2 − ψ + noise,
σ = 0.060°, D12 a).
**Done when:** unit tests: velocity and heading errors have the configured
spread; heading wraps into [0, 2π); both stop in a sky-view outage, heading
stays up when only RTK corrections are lost.
**Decisions needed:** none. The brief looks up the UM982 velocity accuracy
[DS] (still "to check" in `design.md` §4.2).

#### Brief (agreed on 2026-11-02)

**Summary (in plain words)**
The filter (D5 C) moves the boat with the gyro and the GNSS velocity, and
corrects its heading with the dual-antenna heading. Gazebo gives neither in a
usable form (its NavSat velocity is lost in the bridge; it has no dual-antenna
heading). The `corruptor` builds both: the true antenna velocity + UM982
velocity noise (0.03 m/s) on `/asv/sensors/gnss/velocity`, and the true yaw ψ
turned into the receiver's azimuth α = π/2 − ψ + noise (0.060°) on
`/asv/sensors/gnss/heading`. Both stop in a sky-view outage; the heading stays
up when only the RTK corrections are lost. No change to Gazebo, the world or
the filter; launch file and recording check are M2.6d.

**1. Goal:** noisy antenna velocity (ENU) and azimuth α in [0, 2π), both
following the M2.6b mode schedule; `GnssStatus.heading_available` set.

**2. Decisions needed** (brief questions, no D-number)
- Q1 true velocity: a) central difference of the clean antenna fixes,
  (p_{k+1} − p_{k−1}) / (t_{k+1} − t_{k−1}), published one epoch late with
  stamp t_k (rec.); b) backward difference, no delay, lags half an epoch
  (≈ 0.02 m/s in turns); c) odometry pose ± 20 ms + lever arm.
- Q2 true heading: a) yaw ψ from `/asv/ground_truth/odom` at the fix stamp
  (rec.); b) azimuth of (secondary − primary) clean fix − 23.47° mounting angle
  (real tilt error ≈ 0.08° at 5° roll).
- Q3 velocity noise "0.03 m/s RMS" [DS UM982 product brief]: a) per axis
  E, N, U (U [EST]), white, same in every mode except outage (rec.); b) 2D,
  0.021 m/s per axis.
- Q4 heading noise: a) white only, σ_α = 0.1°/1.657 m = 0.060° (D12 a) (rec.);
  b) white + slow multipath part [EST].

Marcell (2026-11-02): **Q1 a**, **Q2 a**, **Q3 a**, **Q4 a**.

**3. Why:** D5 C needs the velocity as a prediction input; the heading makes ψ
and the gyro bias observable; outage runs (D9 a) must lose both. Feeds:
Vehicle and Sensor Models; Sensor Configuration; Noise Models.

**4. New ideas:** central difference (slope over both neighbours belongs to
t_k; a one-sided slope belongs to t_k − Δt/2); azimuth (from north, clockwise)
vs yaw (from east, counter-clockwise): α = π/2 − ψ wrapped; heading
availability is separate from the position mode.

**5. What will change:** `asv_core`: `gnss_corruptor.hpp/.cpp` (velocity and
heading noise, streams `gnss.velocity_{e,n,u}`, `gnss.heading`;
`central_difference()`, `azimuth_from_yaw()`, a yaw buffer), tests.
`asv_sim`: `gnss_corruption.hpp/.cpp`, `corruptor_node.cpp` (subscribes
`/asv/ground_truth/odom`, two new publishers), `nominal.yaml` (`gnss.velocity`,
`gnss.heading` multipliers), tests. ⚠ `asv.yaml`: `velocity_noise: 0.03` [DS]
(heading σ from the existing `heading_accuracy_1m` and the antenna positions).
⚠ generator `tools/asvgen/config.py`: accept the key. Docs: `design.md` §1,
§4.2, §4.3; glossary (σ_v, σ_α proposed); plans; `CLAUDE.md`.

**6. How it works:** v̂_k = (p_{k+1} − p_{k−1}) / (t_{k+1} − t_{k−1}) from the
clean antenna fixes in world ENU; v_k = v̂_k + σ_v·w, published when fix k+1
arrives, stamp t_k, frame `world`; covariance linear diag σ_v², angular −1
(not measured). It is the antenna's velocity (includes ω × R·l, up to
≈ 0.25 m/s at 0.3 rad/s: M3.3a must handle it). α_k = wrap_[0,2π)(π/2 −
ψ(t_k) + σ_α·w), ψ from ground-truth odometry at t_k; `azimuth_std` = σ_α,
`baseline_fixed` = true, frame `base_link`. Outage → neither, and
`heading_available` = false.

**7. How we will check it:** gtests: error spread = σ (5 %); α in [0, 2π) at
the wrap points; central difference exact on a parabola, better than the
one-sided slope on a circle; outage → none, standalone → heading; same seed;
multiplier 0 → truth. Live: both at ≈ 10 Hz sim time, heading error spread
≈ 1 mrad. Counts `asv_core` 51 → about 58, `asv_sim` 118 → about 123.

**8. What I can't check:** whether the real UM982 errors are white and this
size; whether "0.03 m/s RMS" is per axis, 2D or 3D.

#### Report (2026-11-02)

**1. What changed**
- `asv_core/gnss_corruptor.hpp/.cpp`: `GnssCorruptor` also draws the
  velocity noise σ_v·w per ENU axis (streams `gnss.velocity_{e,n,u}`) and the
  azimuth noise σ_α·w (stream `gnss.heading`) for every epoch with a fix;
  none in an outage. New `azimuth_from_yaw()`, `central_difference()`,
  `YawHistory`. Test: 6 new tests in `test_gnss_corruptor` (17 now).
- `asv_sim/gnss_corruption.hpp/.cpp`: `gnss_error_params()` also reads
  `velocity_noise` and σ_α = `heading_accuracy_1m` / baseline; new class
  `GnssCorruption` (position as before, velocity one epoch late, headings that
  wait for the odometry yaw). Test: 5 new tests in `test_gnss_corruption` (12 now).
- `corruptor_node.cpp`: subscribes `/asv/ground_truth/odom`; publishes
  `/asv/sensors/gnss/velocity` and `/asv/sensors/gnss/heading`.
- `noise_profiles/nominal.yaml`: `gnss.velocity: 1.0`, `gnss.heading: 1.0`.
- `CMakeLists.txt`: `asv_sim_corruptor` links nav_msgs.
- ⚠ `asv.yaml`: `unicore_um982_rtk.velocity_noise: 0.03` [DS].
- ⚠ generator `tools/asvgen/config.py`: accepts `velocity_noise` (number);
  `tests/test_asvgen.py` +1 test. `make_asv.py` rerun: 6 generated files, one
  line each (the fingerprint).
- Docs: `design.md` §1, §4.2 (velocity), §4.3 (as built); `glossary.md` (σ_v,
  σ_α proposed; "Central difference"; azimuth in the corruptor);
  `HOW_IT_WORKS.md` (the new key; §5.4/§14 stay for M2.6d); `CLAUDE.md`.

**2. How it was checked**
- `colcon test --packages-select asv_core asv_sim`, `colcon test-result`:
  `Summary: 196 tests, 0 errors, 0 failures, 0 skipped` (asv_core 57 gtests,
  asv_sim 123, + 16 CTest wrappers).
- `python3 -m unittest discover -s tests`: `Ran 86 tests ... OK`.
- First run: one failure, `VelocityIsTheCentralDifferenceOneEpochLate`
  missed the brief's 1e-6 m/s by up to 1.1e-5 m/s. Cause: the velocity is
  formed in an ENU frame centred on the antenna, 141 m from the world origin,
  tilted by d / R_E = 2.2e-5 rad (the same effect as M2.6b's 24 µm). Tolerance
  set to 5e-5 m/s from that geometry (0.1 % of σ_v).
- Live: `maneuver.launch.py maneuver:=circle gui:=false` + corruptor nominal,
  bag `ws/data/bags/m26c_check` (43.9 s), `ws/data/m26c_check.py`
  (truth independent of the corruptor: antenna velocity from odometry
  p + R(q)·l, central difference at ±20 ms; heading π/2 − yaw(odom)):
  - `ros2 topic hz /asv/sensors/gnss/heading`: 10.000 Hz (sim time).
  - 439 clean fixes; 436 velocities and 436 headings, all on clean stamps;
    missing: heading at 0.0, 0.1, 0.2 s, velocity at 0.0, 0.1 s (most likely
    the recorder subscribing late; 0 "dropped" warnings in the node log) and
    the last epoch (needs the next fix).
  - Velocity error mean / std E −0.0005 / 0.0319, N +0.0013 / 0.0279,
    U +0.0003 / 0.0300 m/s (expected 0 / 0.03; std scatter ±3.4 % at n = 436);
    antenna speed 0.06–1.02 m/s.
  - Heading error mean −2.1e-6, std 1.101e-3 rad = 0.0631° (expected
    1.053e-3, +4.5 %); azimuth range 0.002–6.275 rad (wrap exercised);
    `azimuth_std` 1.0532e-3. Velocity covariance diag 0.0009 ×3, −1 ×3.
  - `heading_available` true on all 439 status messages.

**3. What could not be checked**
- Whether the real UM982 velocity and heading errors are white and this
  size, and whether "0.03 m/s RMS" is per axis.
- Outage and standalone live: only unit tests (the nominal profile has
  neither). M2.6d's recording check can use an outage profile.

**4. How to read the code**
- `asv_core/gnss_corruptor.hpp` header comment: v = v_true + σ_v w,
  α = wrap(π/2 − ψ + σ_α w). `GnssCorruptor::process()` draws the noise;
  `azimuth_from_yaw()` is α = π/2 − ψ wrapped; `central_difference()` is v̂_k.
- `asv_sim::GnssCorruption::on_fix()`: velocity of the previous epoch;
  `ready_headings()`: α = `azimuth_from_yaw(ψ − noise)` once the yaw is known.

**5. Numbers for the thesis**
- σ_v = 0.03 m/s per ENU axis [DS UM982 product brief; per axis and U [EST]].
- σ_α = 0.1° [DS] / 1.657 m [REPO] = 0.0603° = 1.0532 mrad (D12 a).
- Live: velocity error std 0.0319 / 0.0279 / 0.0300 m/s, heading error std
  1.101 mrad [MEASURED, `m26c_check`].
- Truth error of the central difference: v ω² Δt²/6 ≈ 0.2 mm/s at 1.5 m/s,
  0.3 rad/s (vs ≈ 22 mm/s one-sided) [REPO, test].

**6. Check your understanding**
1. Why does the velocity come out one epoch late, and why is that harmless
   for slam_replay?
2. Why does the heading keep coming when the RTK corrections are lost, but
   not in a sky-view outage?
3. The boat turns at 0.3 rad/s. Why does the GNSS velocity then differ from
   the velocity of base_link, and by roughly how much?

### M2.6d Corruptor node and recording check
**Goal:** one node publishes all of the above live from the clean topics.
**Done when:** on a recording, corrupted − clean has the configured mean and
spread per sensor; `ros2 node info` shows the expected inputs and outputs.
**Decisions needed:** none.

- Update `HOW_IT_WORKS.md` §5.4 and §14 (heading now emulated by the corruptor,
  `design.md` §4.3). Lidar range noise is left for M5 (optional).

#### Brief (agreed on 2026-11-02)

**Summary (in plain words)**
The corruptor already makes all realistic sensor data (IMU, GNSS position,
velocity, heading), but it is started by hand and was only checked live with
RTK fixed all run. This step starts it with every simulation launch, records
one run in which the GNSS goes fixed → float → standalone → outage → fixed,
and adds a tool that compares each noisy topic with the clean one and checks
that the errors have the size `asv.yaml` says (e.g. gyro 1.05e-3 rad/s, GNSS
5 mm in RTK fixed, 0.3 m in standalone). It also fixes two stale lines in
`HOW_IT_WORKS.md` about the heading. No change to the noise maths, Gazebo, the
world or the filter.

**1. Goal:** the corruptor starts with the simulation; one recording shows
that every sensor's error has the configured mean and spread, in every GNSS
mode.

**2. Decisions needed** (brief questions, no D-number)
- Q1 where the node starts: a) in `sim.launch.py`, on by default (rec.);
  b) in `sim.launch.py`, off by default; c) separate launch file.
- Q2 the check tool: a) committed with tests (rec.); b) throw-away script.
- Q3 live check of modes: a) new test profile `mode_check.yaml` (rec.);
  b) nominal only.

Marcell (2026-11-02): **Q1 a**, **Q2 a**, **Q3 a**.

**3. Why:** the live filter (M4.1) needs `/asv/sensors/...` in every run; the
mode switches and outages had only unit tests. Feeds: Vehicle and Sensor
Models; Ground Truth and Data Recording.

**4. New ideas:** launch argument (`name:=value` on `ros2 launch`); standard
error (mean scatters by σ/√n, std by σ/√(2(n−1))); independent truth (the
check computes σ and the truth itself, never from the corruptor's own fields).

**5. What will change:** `launch/sim.launch.py` (args `corruptor`, default
true, and `noise_profile`, default nominal; the node);
`config/noise_profiles/mode_check.yaml` (new: fixed 0 s → float 20 s →
standalone 40 s → outage 60 s → fixed 70 s, seed 1); `tools/check_corruptor.py`
and `tests/test_check_corruptor.py` (new); docs: `HOW_IT_WORKS.md` §5.4, §14,
`design.md` §1/§4, `corruptor_node.cpp` header, glossary, `CLAUDE.md`, plans.

**6. How it works:** residuals per sensor, everything known removed:
- IMU (4 ms steps): e = ω̃ − ω − b̂ (accel the same); σ = η/√Δt: gyro
  1.049e-3 rad/s, accel 1.706e-2 m/s².
- GNSS position, per mode and ENU axis: e = ENU(fix̃) − ENU(fix) − β_G;
  σ_w h/v fixed 5/9.4 mm, float 0.05/0.10 m, standalone 0.3/0.5 m.
- Velocity: ṽ − v_ant (odom p + R(q)·l, central difference ±20 ms); σ_v =
  0.03 m/s.
- Heading: wrap(α̃ − (π/2 − ψ_odom)); σ_α = 1.053 mrad.
- Pass: |mean| ≤ 3σ/√n and |s/σ − 1| ≤ 3/√(2(n−1)). Counts: one output per
  clean stamp (except the first); nothing in the outage; status = schedule.

**7. How we will check it:** build; `colcon test asv_core asv_sim` (196
unchanged); Python unittest (86 + new). Headless default launch: `ros2 node
info /corruptor` (3 in, 7 out), `noise_profile` = nominal, real-time factor.
`maneuver.launch.py maneuver:=circle gui:=false noise_profile:=mode_check`,
about 90 s sim time recorded to `ws/data/bags/m26d_check`, then
`check_corruptor.py`.

**8. What I can't check:** the spread of the slow GNSS error and of the IMU
turn-on bias (one draw per run or mode; only the unit tests cover it);
whether the real sensors behave like this.

#### Report (2026-11-02)

**1. What changed**
- `asv_sim/launch/sim.launch.py`: starts the corruptor; launch arguments
  `corruptor` (default true) and `noise_profile` (default nominal).
- `asv_sim/config/noise_profiles/mode_check.yaml` (new): test profile, nominal
  sizes, GNSS fixed 0 s → float 20 s → standalone 40 s → outage 60 s →
  fixed 70 s, seed 1.
- `asv_sim/tools/check_corruptor.py` (new): residuals per sensor and GNSS
  mode against σ from `asv.yaml` × profile; message counts; mode schedule;
  exit 1 on any failure.
- `asv_sim/tests/test_check_corruptor.py` (new, 21 tests).
- `asv_sim/src/corruptor_node.cpp`: header comment only (how it is started).
- Docs: `HOW_IT_WORKS.md` §5.4 and §14 (heading emulated by the corruptor);
  `design.md` §4 (launch, recording check); glossary (independent truth,
  launch argument, standard error); `CLAUDE.md` (Known state, baseline,
  layout, and the stop command now also kills `static_transform_publisher`
  and our nodes, see 2).

**2. How it was checked**
- `colcon build --symlink-install --packages-up-to asv_sim`: 3 packages
  finished.
- `colcon test --packages-select asv_core asv_sim`, `colcon test-result`:
  `Summary: 196 tests, 0 errors, 0 failures, 0 skipped` (no C++ change).
- `python3 -m unittest discover -s tests`: `Ran 107 tests ... OK` (86 + 21).
- Default headless launch: `ros2 node info /corruptor` subscribes
  `/asv/gnss/fix`, `/asv/ground_truth/odom`, `/asv/imu/data` (+ `/clock`),
  publishes the 7 topics (`gnss_error`, `gnss_status`, `imu_bias`,
  `sensors/gnss/{fix,heading,velocity}`, `sensors/imu/data`);
  `noise_profile` = nominal, `use_sim_time` = True.
- Real-time factor (sim/wall over about 30 s wall): 0.857 with the
  corruptor, 0.872 with `corruptor:=false`. One measurement each, so the
  difference is not shown to be real.
- Surprise on the way: the old stop command left the
  `static_transform_publisher` nodes running (27 after three launches); with
  them the next launch's bridge passed no data (`/clock` empty, Gazebo itself
  running). Fixed by killing them too; the stop command in `CLAUDE.md` now
  does that.
- `maneuver.launch.py maneuver:=circle gui:=false noise_profile:=mode_check`
  (`ros2 param get /corruptor noise_profile`: mode_check, so the argument
  reaches the included file), bag `ws/data/bags/m26d_check` (102.7 s,
  28 MiB), `check_corruptor.py --profile mode_check`: `ALL PASS`, exit 0.
  - Counts: IMU 0 missing / 0 extra (corrupted and bias); GNSS status,
    fix, gnss_error, velocity, heading 0 / 0; `heading_available` agrees.
    1023 statuses, 923 fixes: 100 outage epochs = 10 s.
  - Switches: rtk_fixed 0.5, rtk_float 20.1, standalone 40.1, outage 60.1,
    rtk_fixed 70.1 s; first clean fix at 0.1 s: schedule PASS.
  - Residual std / expected (n): gyro 0.998 / 0.998 / 1.004, accel
    0.999 / 0.987 / 0.990 (15337 each, 4 ms steps; expected 1.0486e-3 rad/s,
    1.7060e-2 m/s²); GNSS rtk_fixed E/N/U 1.038 / 1.027 / 1.001 (522),
    rtk_float 0.981 / 0.919 / 1.032 (200), standalone 0.928 / 0.992 / 1.014
    (200); velocity 0.994 / 0.996 / 1.025 (923); heading 1.023 (923). All
    means inside 3σ/√n.

**3. What could not be checked**
- The spread of the slow GNSS error and of the IMU turn-on bias: one draw
  per run or mode (unit tests M2.5, M2.6b only).
- The tool runs 38 mean/std tests on a full recording: even with a correct
  corruptor, about 1 run in 10 is expected to fail one of them by chance
  (each about 1 in 370). A single near-miss FAIL means "run again with
  another seed", not a bug.
- Whether the real sensors behave like this.

**4. How to read the code**
- Start at the docstring of `tools/check_corruptor.py` (the four residual
  equations and the pass rule), then `main()` top to bottom.
- `expected_sigmas()` = Python twin of `imu_error_params()` /
  `gnss_error_params()`; `residual_stats()` = pass rule;
  `imu_residuals()` e = ω̃ − ω − b̂; `gnss_residuals()` e = ENU(fix̃) − β_G;
  `velocity_residuals()` e = ṽ − v_ant; `heading_residuals()`
  e = wrap(α̃ − (π/2 − ψ)); `schedule_matches()` the mode order and spacing.
- Launch: section 4 of `_launch_setup()` in `sim.launch.py`.

**5. Numbers for the thesis**
- Per 4 ms IMU sample: gyro 1.0486e-3 rad/s, accel 1.7060e-2 m/s² [REPO,
  η/√Δt]; measured 1.0462–1.0531e-3 rad/s, 1.6830–1.7049e-2 m/s² [MEASURED,
  `m26d_check`].
- GNSS white σ per axis, measured / configured: fixed 5.19 / 5.13 / 9.41 mm
  vs 5 / 5 / 9.4 mm; float 0.049 / 0.046 / 0.103 m vs 0.05 / 0.05 / 0.10 m;
  standalone 0.279 / 0.297 / 0.507 m vs 0.3 / 0.3 / 0.5 m [MEASURED vs REPO
  `asv.yaml`].
- Velocity 0.0298 / 0.0299 / 0.0308 m/s vs 0.03; heading 1.078 mrad vs
  1.0532 mrad [MEASURED vs REPO].
- Headless RTF 0.857 with the corruptor [MEASURED, one 30 s window].

**6. Check your understanding**
1. Why does the tool subtract `/asv/ground_truth/gnss_error` before it
   checks the GNSS spread, and what would the std of the raw error be in
   standalone mode without that?
2. In rtk_float the N std came out 8 % low and still passed. Would 8 % low
   pass for the IMU gyro? Why is the allowed band different?
3. Why must the expected σ not be read from the fix's covariance field?

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

### Brief (agreed on 2026-11-03)

```
M0 🟩🟩🟩🟩🟩 │ M1 🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩 │ M2 🟩🟩🟩🟩🟩🟩🟩🟩🟨⬜⬜ │ M3 ⬜⬜⬜⬜⬜⬜⬜⬜⬜ │ M4 ⬜ │ M6 ⬜⬜⬜
29 of 45 steps done: 64% completed!
```

#### Context
Fake detector (M3.1) and evaluation (M3.6) need the true buoy positions. D7 a
(decided 2026-10-11): buoys made static, positions read from the course files.
Today every buoy and vessel floats freely (design.md §13), so its true position
is not known after a few seconds.

#### Summary (plain words)
Write a list of every buoy in the course: name, colour, class, size, and its
exact x/y in the world. A small Python tool computes it from the files that
place the buoys (task pose in the world file + buoy pose inside the task
model). The buoys are then nailed in place (static) so the list stays true for
the whole run. A small node publishes the list once on a "latched" topic, so a
node that starts later still gets it. Before nailing them down, one 5-minute
run with the floating buoys measures how far they wander: the number goes into
the thesis limitations. The boat, sensors and filter don't change.

#### Decisions (new D19, D20): decided a and a, 2026-11-03

- **D19 — what goes into the map and becomes static.**
  a) (recommended) all floating course objects: Sur-Marks, A-0, A-2, indicator
     buoys (class `indicator`), vessels (class `other`, colour yellow/black).
     design.md §5 already says the detector reports vessels as "other", so it
     needs their positions. 27 buoys + 5 vessels = 32 entries.
  b) only Sur-Marks, A-0, A-2 static and listed (23 buoys); indicators and
     vessels keep floating and are not in the map.
- **D20 — meaning of `radius`.**
  a) (recommended) largest horizontal radius of the body: A-0 0.1015, A-2 0.184,
     Sur-Mark 0.2286 m (same as task1_route.yaml today); non-round objects:
     circumscribed circle (indicator 0.323, vessel 0.65 m). Fix the comment in
     `Landmark.msg` ("at the waterline" → "largest horizontal").
  b) radius where the water cuts the body: A-0 0.0878, A-2 0.1388 m (spheres sit
     with their equator above water); lidar hits mostly above water, so smaller
     than what it sees.

#### What will change
- new `tools/make_buoy_map.py`: reads `worlds/roboboat_course.sdf` + generated
  `models/rb26_task*/model.sdf` + element model geometry; composes 2D poses
  (x, y, yaw); writes `config/buoys_gt.yaml` (generated, committed; scoped names
  like `task1_entry_exit::gate1_red_port`, id = index).
- new `tools/measure_buoy_drift.py`: reads Gazebo's pose topic
  (`/world/roboboat_course/pose/info`, gz CLI; to be confirmed it lists nested
  models) at start and after 300 s sim time; largest movement per object.
- ⚠ `tools/make_roboboat2026_course.py`: `static=True` on buoy (and with D19 a,
  vessel) includes; regenerate `models/rb26_task{1,2,3,5}*/model.sdf`.
- new `include/asv_sim/buoy_map.hpp`, `src/buoy_map.cpp` (ROS-free YAML loader,
  yaml-cpp already a dependency), node `src/buoy_map_publisher.cpp` (I/O only,
  `/asv/ground_truth/buoys`, asv_msgs/LandmarkArray, QoS transient_local +
  reliable, depth 1, published once after the first nonzero clock).
- `CMakeLists.txt` (new lib source, node, gtest), `launch/sim.launch.py` (start
  the node, `use_sim_time`).
- tests: `tests/test_make_buoy_map.py`, `test/test_buoy_map.cpp`.
- D20 a: `asv_msgs/msg/Landmark.msg` comment.
- docs: design.md §13 (buoys static now), master_plan D19/D20 + mark,
  m2_plan brief/report, CLAUDE.md Known state + layout, glossary ("latched /
  transient local", "static model").

#### How it works
world pose of a buoy = task pose ∘ local pose (planar):
x_w = x_T + cos ψ_T · x_l − sin ψ_T · y_l
y_w = y_T + sin ψ_T · x_l + cos ψ_T · y_l
(ψ_T = task yaw; Task 2 is rotated by 1.5708 rad.)
Colour and class come from the element model name (`rb26_a2_yellow` → A-2,
yellow); radius from that model's collision geometry.

#### Build order
1. `make_buoy_map.py` + Python tests (on the current files).
2. Drift run with the CURRENT floating buoys: headless sim, boat idle, 300 s sim
   time; `measure_buoy_drift.py` → table (largest movement per object).
3. Generator change, regenerate, check `git diff` adds only `<static>` tags.
4. C++ loader + gtest, node, launch.
5. Live check of the latched topic; sim log free of buoyancy-plugin errors.

#### How we will check it
- Python: gate 1 red = (5.0, 1.524) m (±1e-9); Task 2 rotation example
  ch_gate1_red ≈ (20.781, 6.000); every buoy include in regenerated rb26_task*
  has `<static>true</static>`; committed yaml == freshly generated; task1_route
  `buoy_coords` agree with the map within 1 mm.
- gtest `test_buoy_map`: loads the yaml, count, gate 1 red, bad colour throws.
- Live: `ros2 topic echo --once --qos-durability transient_local
  /asv/ground_truth/buoys` started AFTER the sim → 32 (or 23) landmarks.
- Report Python and colcon test counts (baseline 107 / 57 + 123).

#### What I can't check
Marcell, in the Gazebo window: static buoys sit at the waterline (not floating
above / sunk), look the same as before.

### Report (2026-11-03)

**What changed**
- `tools/make_buoy_map.py` (new): world task pose ∘ local pose → `config/buoys_gt.yaml`
  (generated, committed): 32 objects, class/colour from the element model name,
  radius from its collision shapes (D20 a).
- `config/buoys_gt.yaml` (new, generated).
- `tools/make_roboboat2026_course.py` ⚠: every object in a task model is static
  (`STATIC_OBJECTS`); `--floating` restores the old course for drift runs.
- `models/rb26_task{1,2,3,5}*/model.sdf`: regenerated; the diff only adds
  `<static>true</static>` to 32 includes; no element model changed.
- `tools/measure_buoy_drift.py` (new): reads `/world/roboboat_course/pose/info`
  twice, composes nested poses, prints drift per object; checks links stay at
  their model origin.
- `include/asv_sim/buoy_map.hpp`, `src/buoy_map.cpp` (new, ROS-free):
  `parse_buoy_map()`, `colour_code()`; library `asv_sim_buoy_map`.
- `src/buoy_map_publisher.cpp` (new node, I/O only): `/asv/ground_truth/buoys`,
  LandmarkArray, transient_local + reliable, depth 1, once at the first nonzero
  sim time.
- `launch/sim.launch.py`: starts `buoy_map_publisher`. `CMakeLists.txt`: library,
  node, gtest.
- `asv_msgs/msg/Landmark.msg`: comments of `name`, `model` (classes incl.
  indicator, other), `radius` (D20 a). No field changed.
- Tests (new): `tests/test_make_buoy_map.py` (13), `tests/test_measure_buoy_drift.py`
  (4), `test/test_buoy_map.cpp` (6).
- Docs: master_plan D19, D20, mark; design.md §1 row, §13; glossary (static model,
  r_b); CLAUDE.md known state, baseline, layout.

**How it was checked**
- `python3 -m unittest discover -s tests` → `Ran 124 tests ... OK` (107 + 17).
- `colcon test --packages-select asv_core asv_msgs asv_sim` →
  `Summary: 203 tests, 0 errors, 0 failures, 0 skipped`; asv_core 57 gtests,
  asv_sim 129 (`test_buoy_map` 6).
- `make_buoy_map.py` → `32 objects {'Sur-Mark': 4, 'A-0': 14, 'indicator': 4,
  'other': 5, 'A-2': 5}`; gate 1 red (5.0, 1.524) m; ch_gate1_red
  (20.781000, 5.999996) m.
- Diff check: removed lines == added lines with `<static>true</static>` taken out.
- Drift run (floating, before regeneration; boat idle; sim time 8.680 → 314.380 s):
  horizontal ≤ 1.28e-14 m for all 32, vertical |Δz| ≤ 0.0340 m
  (`ws/data/m27_drift/drift.json`). Link poses relative to their model: 0.0 m (41 links).
- Static run: plugin loads for 32 objects, no error; 37.3 s: horizontal 0.0,
  vertical 0.0, distance from map ≤ 1.9e-14 m (`static_check.json`).
- Latched topic: `ros2 topic echo --once --qos-durability transient_local` about
  30 s after launch → 32 landmarks, stamp 0.100 s, frame `map`, entry 0 =
  gate1_red_port (5.0, 1.524), colour 1, Sur-Mark, r 0.2286.

**What could not be checked**
- The Gazebo window: static buoys at the waterline, looking as before. Marcell:
  `ros2 launch asv_sim sim.launch.py` and look at Task 1 and Task 3.
- Drift with the boat touching buoys or with steeper waves: not measured.
- `ros2 topic echo` with default QoS also got the map (Jazzy's CLI adapts to the
  publisher's durability); a node with default QoS still would not.

**How to read the code**
- Start at `make_buoy_map.py` `build_map()`: `compose()` is
  x_w = x_T + cos ψ_T x_l − sin ψ_T y_l, y_w = y_T + sin ψ_T x_l + cos ψ_T y_l;
  `horizontal_radius()` is D20 a.
- `buoy_map.cpp` `parse_buoy_map()`: checks; `buoy_map_publisher.cpp`
  `publish_once()`: map → message.
- `measure_buoy_drift.py` `world_positions()`: p_world = p_task + R(q_task) p_local.

**Numbers for the thesis**
- 32 objects: 4 Sur-Mark, 14 A-0, 5 A-2, 4 indicator, 5 vessels [REPO].
- Radii (D20 a) [REPO]: Sur-Mark 0.2286, A-0 0.1015, A-2 0.184, indicator 0.3233,
  vessel 0.65 m.
- Drift of free-floating buoys, 305.7 s, waves gain 0.1, period 4 s, steepness 0,
  boat idle: horizontal 0 (≤ 1.3e-14 m), heave ≤ 3.4 cm [MEASURED].

**Check your understanding**
1. Why does Task 2's rotation matter for the map, but not Task 1's?
2. Why did the floating buoys not move sideways at all, and what would make them?
3. Why must the topic be "latched", and what must a subscriber do to get it?

---

## M2.8 Frames and TF

**Goal:** one connected TF tree: static world → map identity, world →
base_link from ground truth (when the filter is not running); an RViz config
with the lidar cloud in the world frame.
**Done when:** `ros2 run tf2_tools view_frames` shows one connected tree;
Marcell checks RViz.
**Decisions needed:** none.

### Brief (agreed on 2026-11-03)

Marcell first asked whether the live view is needed before the thesis
(2026-11-03); options: skip to M7, TF only, keep. He chose: keep as briefed.
Choices A a and B a taken as recommended.

```
M0 🟩🟩🟩🟩🟩 │ M1 🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩🟩 │ M2 🟩🟩🟩🟩🟩🟩🟩🟩🟩🟨⬜ │ M3 ⬜⬜⬜⬜⬜⬜⬜⬜⬜ │ M4 ⬜ │ M6 ⬜⬜⬜
30 of 45 steps done: 67% completed!
```

#### Summary (in plain words)
ROS keeps a "family tree" of coordinate frames (TF): which frame sits where relative
to which. Today the tree has two disconnected pieces: `base_link` with its sensors
(static, from `asv_frames.yaml`), and nothing that says where `base_link` is in
`world`. So RViz cannot draw a lidar point "in the world", and nothing connects
`map` (the filter's frame) to `world`. This step adds the two missing links:
`world → map` (identity: same origin, same axes) and `world → base_link` taken from
ground-truth odometry, 50 times per second. Then one RViz config shows the lidar
cloud, the true boat pose and the follower's route in the world frame. Nothing in the
corruptor, follower or sensor data changes; this is only for viewing and for later
steps (fake detector M3.1, slam_node M4.1) that need the tree.

#### 1. Goal
One connected TF tree (world → map, world → base_link → sensors) and an RViz config
that shows the lidar cloud in the world frame.

#### 2. Decisions needed
No open D-number. Two layout choices, taken as recommended unless Marcell says otherwise:
- **A. Who publishes world → base_link:** (a, rec.) our own small C++ node
  `ground_truth_tf` (odom → TF, copies stamp and frame names), can be switched off
  with a launch argument so slam_node can take over in M4.1; (b) bridge Gazebo's
  OdometryPublisher TF topic (`/asv/ground_truth/pose_v`) onto `/tf`: no code, but
  the model comment says "topic only, NOT TF" on purpose, it can't be unit tested
  and switching it off means editing the bridge config (generator).
- **B. How RViz starts:** (a, rec.) launch argument `rviz:=false|true` in
  `sim.launch.py` (default false, so headless runs and my runs don't open windows);
  (b) a separate `rviz.launch.py`.

#### 3. Why
- Thesis: Coordinate Frames and Transformations (world/map/base_link/sensor frames);
  Ground Truth and Data Recording (truth pose also as TF).
- M3.1 fake detector and M5 perception need sensor poses in world; M4.1 slam_node
  publishes map → base_link and needs `map` hanging under `world`.
- Rule from design.md §2: base_link has exactly one parent → only one of
  ground_truth_tf / slam_node may run; the launch argument enforces that.

#### 4. New ideas
- **TF tree:** frames as a tree; each edge is a transform published by one node;
  any two frames in one tree can be related. (glossary "Frame, TF, TF tree")
- **Static vs dynamic transforms:** static ones (world→map, sensor mounts) are sent
  once, latched on `/tf_static`; dynamic ones (world→base_link) are stamped and
  streamed on `/tf`; a lookup at time t interpolates between stamps.

#### 5. What will change
- `ws/src/asv_sim/include/asv_sim/ground_truth_tf.hpp` + `src/ground_truth_tf.cpp`:
  `transform_from_odometry(const nav_msgs::msg::Odometry&)` → TransformStamped
  (message glue, testable without a running ROS graph).
- `ws/src/asv_sim/src/ground_truth_tf_node.cpp`: node, I/O only: subscribe
  `/asv/ground_truth/odom`, broadcast on `/tf`.
- `ws/src/asv_sim/test/test_ground_truth_tf.cpp`: gtest.
- `ws/src/asv_sim/CMakeLists.txt`: library source, node, test, install `rviz/`.
- ⚠ `ws/src/asv_sim/package.xml`: `tf2_ros` from `exec_depend` to `depend`;
  new `exec_depend rviz2` (both already installed in the container).
- `ws/src/asv_sim/launch/sim.launch.py`: static world → map identity; node
  `ground_truth_tf` (arg `ground_truth_tf:=true|false`, default true); RViz (arg
  `rviz:=false|true`).
- `ws/src/asv_sim/rviz/asv.rviz`: fixed frame `world`; Grid, TF, PointCloud2
  `/asv/lidar/points`, Odometry `/asv/ground_truth/odom`, Path `/asv/autonomy/path`.
- `ws/src/asv_sim/tests/test_rviz_config.py`: config parses; fixed frame `world`;
  lidar topic present.
- Docs: design.md §2 (who publishes, launch args), §1 topic table (`/tf`,
  `/tf_static`), glossary if a term is missing, CLAUDE.md Known state + Layout +
  test baseline, master plan mark, m2_plan brief + report.

#### 6. How it works
- Odometry pose = T_𝒲ℬ (pose of base_link in world): position p, orientation q.
  TF edge world → base_link carries exactly that: translation = p, rotation = q,
  header.stamp = odom stamp, frame_id = odom.header.frame_id (`world`),
  child_frame_id = odom.child_frame_id (`base_link`). Twist is ignored.
- A lidar point: p_𝒲 = T_𝒲ℬ · T_ℬ𝒮 · p_𝒮 (T_ℬ𝒮 static from asv_frames.yaml; RViz
  composes this itself).
```
world ──(static, identity)──> map            (M4.1: map → base_link instead of the line below)
  └──(ground_truth_tf, 50 Hz)──> base_link ──(static)──> lidar_link, imu_link, gnss_*, zed_*, cam_*
```

#### 7. How we will check it
- gtest `test_ground_truth_tf`: values, stamp and frame names copied; twist not used.
- Python test for the RViz file.
- Build; `colcon test` asv_sim (expect 129 + new) and asv_core (57 unchanged);
  Python unittest (124 + new).
- Headless run: `ros2 run tf2_tools view_frames` → one tree, root `world`, 12 frames
  (world, map, base_link + 9 sensor frames); `ros2 run tf2_ros tf2_echo world base_link`
  equals the odom pose at the same moment (within one 20 ms odom period of motion;
  boat idle → equal to < 1 mm); `tf2_echo world lidar_link` z ≈ base_link z + 0.62 m;
  `ros2 topic hz /tf` ≈ 50 Hz sim time.
- `ground_truth_tf:=false` → view_frames shows base_link not connected to world.

#### 8. What I can't check
- The RViz window: Marcell runs `ros2 launch asv_sim sim.launch.py rviz:=true`
  (or task1 with it) and checks that the lidar points of the buoys sit where the
  buoys are and stay put while the boat moves (if TF were wrong they would move
  with the boat).

### Report (2026-11-03)

**What changed**
- `asv_sim/include/asv_sim/ground_truth_tf.hpp`, `src/ground_truth_tf.cpp`:
  `transform_from_odometry()`: odometry pose → TransformStamped, stamp and
  frame names copied, twist ignored (library `asv_sim_tf`).
- `src/ground_truth_tf_node.cpp`: node `ground_truth_tf`, I/O only:
  `/asv/ground_truth/odom` in, `/tf` (world → base_link) out.
- `test/test_ground_truth_tf.cpp`: 4 gtests (stamp/frames, exact pose copy,
  twist not used, other frame names pass through).
- `launch/sim.launch.py`: static world → map (`static_tf_map`), node
  `ground_truth_tf` (arg `ground_truth_tf`, default true), RViz (arg `rviz`,
  default false); helper `_is_true()` also used for `corruptor`.
- `rviz/asv.rviz`: fixed frame world; Grid (1 m), TF, PointCloud2
  `/asv/lidar/points`, Odometry `/asv/ground_truth/odom`, Path
  `/asv/autonomy/path` (transient local); top-down view.
- `tests/test_rviz_config.py`: 5 tests (fixed frame, lidar topic, TF display,
  every topic bridged or ours, route subscribed transient local).
- `CMakeLists.txt` (library, node, test, install `rviz/`); ⚠ `package.xml`
  (`tf2_ros` → `depend`, `exec_depend rviz2`).
- Docs: `design.md` §1 (`/tf`, `/tf_static` rows), §2 (who publishes, launch
  args, live view); glossary (static vs dynamic transform); CLAUDE.md.

**How it was checked**
- `colcon test --packages-select asv_core asv_sim`, `colcon test-result`:
  `Summary: 208 tests, 0 errors, 0 failures, 0 skipped` (asv_core 57 gtests,
  asv_sim 133 = 129 + 4; 62 + 146 with the CTest wrappers).
- `python3 -m unittest discover -s tests`: `Ran 129 tests ... OK` (124 + 5).
- Headless `sim.launch.py`, `ros2 run tf2_tools view_frames`: one tree, root
  `world`, 12 frames; edges world → map, world → base_link, base_link → 6
  sensor/mount frames, zed_link → 2 optical, cam_wide_front_link → optical;
  base_link rate 50.216 Hz.
- rclpy check (TF buffer lookup at each odometry stamp, 120 stamps): position
  difference 0.000 m, largest quaternion component difference 1.1e-16.
  world → map = (0, 0, 0), q = (0, 0, 0, 1).
- `lidar_link` z − `base_link` z = 0.6209 m (0.62 m mount; the extra ~1 mm is
  the boat's small pitch/roll at rest acting on the 0.25 m forward offset).
- `ros2 topic hz /tf --use-sim-time`: average 50.000 Hz.
- `task1.launch.py gui:=false ground_truth_tf:=false`: no `ground_truth_tf`
  node; view_frames has no world → base_link edge (base_link cut off), so the
  argument reaches the included `sim.launch.py`.

**What could not be checked**
- The RViz window. Marcell: `ros2 launch asv_sim task1.launch.py rviz:=true`
  (in `docker compose exec sim bash`); the lidar points on the buoys sit on
  the buoys and stay still in the world while the boat moves and turns; the
  green route line runs through the gates; the TF axes ride on the boat.

**How to read the code**
- Start at `ground_truth_tf.hpp` (T_𝒲ℬ: odometry pose = transform world →
  base_link), then `ground_truth_tf_node.cpp` (subscribe, convert, send).
- `sim.launch.py` sections 3 (static TF incl. world → map), 3b (ground-truth
  TF), 6 (RViz).
- p_𝒲 = T_𝒲ℬ · T_ℬ𝒮 · p_𝒮 is composed by tf2 / RViz, not by our code.

**Numbers for the thesis**
- world → base_link at 50 Hz [MEASURED M2.8] (= odometry rate, 50 Hz [REPO]).
- Lidar mount 0.62 m above base_link [REPO `asv_frames.yaml`].

**Check your understanding**
1. Why may `ground_truth_tf` and slam_node not run at the same time, and what
   would RViz show if they did?
2. If world → base_link were wrong by a constant yaw, how would the buoys'
   lidar points look in RViz while the boat turns?
3. Why is world → map static, but world → base_link sent 50 times a second?

---

## M2.9 Recording profiles

**Goal:** profiles `nav`, `nav_lidar`, `perception` (`design.md` §8).
**Done when:** for each profile, `ros2 bag info` counts equal rate × simulated
duration within 1 % (IMU 200, GNSS 10, ground truth 50 Hz; lidar 10 Hz).
**Decisions needed:** none. Short-form brief.

### Brief (agreed on 2026-11-05)

**In plain words:** recording a run into a bag is like a video recorder: you
choose which channels (topics) to save. Too few and the filter can't be replayed
later; everything and the cameras fill the disk with over 10 GB per minute. This
step fixes three channel lists (profiles): `nav` (clean IMU, clean GNSS, ground
truth, thruster commands, about 7 MB per minute; for all filter experiments),
`nav_lidar` (+ lidar) and `perception` (+ ZED image, for the optional real
perception), each started with one command, and a checker that proves nothing
was lost: e.g. a 60 s run must hold 200 × 60 = 12 000 IMU messages, within 1 %.
Nothing in the simulation, corruptor or boat changes; slam_replay (M3.5) will
later read these `nav` bags.

- **Goal:** profiles `nav`, `nav_lidar`, `perception`; counts = rate ×
  simulated time within 1 %.
- **Choices (2026-11-05):** Q1 a: profiles in `config/recording_profiles.yaml`,
  started with `tools/record.py`; the checker reads the same file. Q2 a: `nav`
  also records `/asv/autonomy/{status,cross_track_error,target}`,
  `/asv/maneuver/status` and `/asv/ground_truth/buoys` (a few kB per minute;
  `analyze_run.py` works on `nav` bags).
- **Files:** new `config/recording_profiles.yaml`, `tools/record.py`,
  `tools/check_bag_rates.py`, `tests/test_recording_profiles.py`; edited
  `docs/design.md` §8 (measured sizes), CLAUDE.md (recording command, Known
  state), this plan, `docs/master_plan.md`.
- **Check:** Python unittests; headless sim + `maneuver:=circle`, record each
  profile, checker: IMU 200, GNSS 10 (both fixes), ground truth 50, lidar 10 Hz
  within 1 %.
- Out of scope: `lazy` bridge entries (generator change, ask first).

### Report (2026-11-05)

**What changed**
- `asv_sim/config/recording_profiles.yaml`: the three profiles; per topic the
  checked rate (Hz) or `null`; `extends` chains nav < nav_lidar < perception.
- `asv_sim/tools/record.py`: `profile_topics()` resolves a profile;
  `record_command()` builds `ros2 bag record --use-sim-time -o ... --topics ...`;
  `--duration` (wall s) then SIGINT; `--print` shows the command only.
- `asv_sim/tools/check_bag_rates.py`: counts and duration from `metadata.yaml`,
  `check_counts()`: |count − f·D| ≤ 1 % of f·D; prints MB per sim second;
  exit 1 on a failure.
- `asv_sim/tests/test_recording_profiles.py`: 8 tests (nesting, nav has what
  replay needs and no `/asv/sensors/...`, every topic bridged or named in a
  node's source, rates = asv.yaml / generator, cycles and duplicates, command,
  checker pass/fail on a fake `metadata.yaml`).
- `docs/design.md` §8 (commands, measured sizes), §13 (camera on the step grid);
  CLAUDE.md (recording commands, Known state, layout, test baseline).

**How it was checked**
- `python3 -m unittest discover -s tests`: `Ran 137 tests ... OK` (129 + 8).
- `colcon test --packages-select asv_sim`: 133 gtests, `colcon test-result`
  Summary 208 tests, 0 errors, 0 failures (asv_core unchanged, 62 of them).
- Headless sim, `maneuver:=circle`, `record.py` 60 / 60 / 30 wall s, then
  `check_bag_rates.py`:

| Profile | D (sim s) | odom 50 | IMU 200 | fix 10 | secondary 10 | lidar 10 | MB per sim min |
|---|---|---|---|---|---|---|---|
| nav | 53.66 | +0.01 % | +0.01 % | −0.10 % | −0.10 % | – | 8.2 |
| nav_lidar | 51.19 | +0.02 % | 0.00 % | +0.02 % | +0.02 % | +0.02 % | 392.6 |
| perception | 24.16 | +0.02 % | +0.02 % | +0.18 % | +0.18 % | −0.23 % | 2899.1 |

  All PASS. `/asv/ground_truth/buoys`: 1 message (latched topic recorded).
- Surprise (stopped and asked): ZED `camera_info` +1.01 % first failed. The
  header stamps show no loss: frames 64/68 ms apart, 15.15 Hz in sim time
  (4 ms physics step; 66.7 ms is not a whole number of steps). Decision
  2026-11-05: camera rates recorded but not checked.

**What could not be checked:** nothing needs the GUI. Status topics publish
only on change, so they are empty when recording starts after the manoeuvre or
route; start the recorder first if they are needed.

**How to read the code:** `record.py` `profile_topics()` → `check_bag_rates.py`
`read_metadata()` → `check_counts()` → `main()`.

**Numbers for the thesis**
- Recording sizes: 8.2 MB, 393 MB, 2.9 GB per simulated minute [MEASURED M2.9].
- ZED effective rate 15.15 Hz (asv.yaml 15 Hz) [MEASURED M2.9]; lidar exactly
  10 Hz; physics step 4 ms [REPO world SDF].

**Check your understanding**
1. Why does `nav` record the clean IMU and GNSS and not `/asv/sensors/...`?
2. Why is a 100 ms lidar exact on the 4 ms grid but a 66.7 ms camera not?
3. Why is the duration taken in sim time, not wall time?
