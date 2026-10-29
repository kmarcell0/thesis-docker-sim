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
