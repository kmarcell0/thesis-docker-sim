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
