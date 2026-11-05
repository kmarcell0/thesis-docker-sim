# M3 plan: The filter, offline

One section per step of `docs/master_plan.md` M3. Each step's brief and report
are saved here, so this file becomes the logbook of the milestone. Notes are
short bullets and equations, not finished paragraphs. The Brief, Report and
"Marcell's notes" parts (`TEMPLATE.md`) are added under a step when it starts.

Status: approved on 2026-11-05

Decisions settled for M3 (all in `master_plan.md` §3): D4 a, D5 C, D8 a,
D13 a, D14 a, D15 a (raised again in the M3.3b brief, as Marcell asked).
Open: D6 (before M3.3b), D10 (before M3.6; M3.5 measures the replay speed it
needs). M3.7 and M3.8 are left out (D16).

Order: M3.1 → M3.2 → M3.3a → M3.3b → M3.3c → M3.3d → M3.4 → M3.5 → M3.6
(table order). M3.3a–d need M3.2 only for M3.5; M3.4 needs M3.3d; M3.5 needs
M3.1–M3.4; M3.6 needs M3.5.

---

## M3.1 Fake detector

**Goal:** buoy measurements (range, bearing, colour) made from ground truth and
the true buoy map, with seeded noise, misses, false alarms and colour mix-ups.
**Done when:** unit tests: only buoys in lidar range (and, for the colour, in a
camera's view and range) are reported; noise mean and σ, miss, false-alarm and
mix-up rates match the settings within 3 standard errors; the node publishes
`/asv/perception/buoys` at 10 Hz in sim time.
**Decisions needed:** none (D8 a).

### Brief (agreed on 2026-11-05)

0. **In plain words:** fake the buoy sightings from the true pose and the true
   map, 10 times a second. Lidar range limit per class (2 beams), colour only
   inside a camera's view and 20-px range, else UNKNOWN. Example: red Sur-Mark
   12 m ahead → "12.04 m, 0.01 rad, red"; A-0 at 6 m → nothing (limit
   4.75 m). Sensors, corruptor and Gazebo unchanged; one new topic.
1. **Goal:** seeded fake detector (ROS-free logic + thin node) publishing
   `BuoyDetectionArray` at 10 Hz from ground truth.
2. **Decisions (Marcell, 2026-11-05):**
   - A (scan timing): **odom grid**: a scan happens at the ground-truth odom
     message whose stamp reaches the next 0.1 s mark; that message's pose is
     used exactly (no interpolation; `design.md` §5 updated).
   - B (nominal settings, all [EST]): **clean-ish**: σ_r 0.10 m, σ_β 0.02 rad,
     P_miss 0, λ 0, mix-ups 0, centroid bias off, vessel lidar range 18.75 m.
     Misses and false alarms are switched on in M6 sweeps.
3. **Why:** measurements with exactly known error for the filter (M3.3d,
   M3.4); separates filter error from perception error (D8 a). Thesis:
   Experimental Scenarios.
4. **New ideas:** false alarm (Poisson count, mean λ per scan); centroid bias
   (points even across the buoy's width → average π·r_b/4 nearer than the
   centre); inverse crime (noise has the filter's assumed shape → optimistic).
5. **What changes:** new `asv_core` `fake_detector.hpp/.cpp`,
   `test_fake_detector.cpp`; new `asv_sim` `fake_detector_node.cpp`,
   `config/detector/nominal.yaml`; edits: both `CMakeLists.txt`,
   `sim.launch.py` (`fake_detector:=true`), `design.md` §5, glossary,
   CLAUDE.md Known state. No new dependencies.
6. **How it works:** pose p = (x, y), ψ from odom; buoy p_b, r_b from the map.
   - lidar sees the buoy if ‖p_b − p_lidar‖ ≤ limit(class) [REPO
     `asv_report.json`]: A-0 4.75, A-2 10.75, Sur-Mark 18.75 m;
   - colour known if inside a camera's hfov (ZED 72°, wide 85°) and its 20-px
     range (ZED 8.9/16.2/20.1, wide 5.3/9.6/12.0 m);
   - r̃ = ‖p_b − p‖ − c·(π/4)·r_b + n_r, β̃ = wrap(atan2(Δy, Δx) − ψ + n_β),
     n_r ~ N(0, σ_r²), n_β ~ N(0, σ_β²);
   - miss with P_miss, colour swap with p_c, Poisson(λ) false alarms per scan;
     one seeded stream per effect; occlusion ignored (limitation).
7. **How we check it:** gtests (visibility, bearing sign, wrap near ±π, rates
   and moments within 3 SE over ~10⁵ samples, same seed → same output); live
   `ros2 topic hz /asv/perception/buoys` ≈ 10.0 Hz sim time, Task 1 gate buoys
   seen at the spawn.
8. **What I can't check:** nothing visual in this step.

### Report (2026-11-05)
Commit: the one whose message starts with `M3.1:` (`git log --grep "^M3.1:"`).

1. **What changed**
   - `asv_core/include/asv_core/fake_detector.hpp`, `src/fake_detector.cpp`:
     `FakeDetector` (visibility, noise, misses, false alarms, mix-ups, bias;
     detections sorted by measured bearing) and `ScanGrid` (scan times).
   - `asv_core/noise.hpp/.cpp`: `RandomStream::uniform()` made public (the
     detector needs uniform draws; the same bits as before, so the pinned
     `test_noise` values are unchanged).
   - `asv_core/test/test_fake_detector.cpp`: 16 gtests.
   - `asv_sim/include/asv_sim/fake_detection.hpp`, `src/fake_detection.cpp`:
     glue (library `asv_sim_detector`): `detector_setup()`,
     `detector_objects()`, `pose_from_odometry()`, `detection_message()`.
   - `asv_sim/src/fake_detector_node.cpp`: node `fake_detector`, I/O only.
   - `asv_sim/config/detector/nominal.yaml`: settings (M3.1 B).
   - `asv_sim/test/test_fake_detection.cpp`: 5 gtests on the repo files.
   - `asv_sim/launch/sim.launch.py`: starts the node; arguments
     `fake_detector:=true|false` and `detector:=<name>` (the second is not
     in the brief; it mirrors `noise_profile`).
   - Both `CMakeLists.txt`; `design.md` §5; glossary (σ_r, σ_β, P_miss, λ,
     p_c; "False alarm, miss"; why π/4 under "Centroid bias"); CLAUDE.md.
   - Not in the brief: the map has class `indicator` (r_b 0.32 m), which
     `asv_report.json` has no limits for. The detector file maps it (and
     `other`) to the Sur-Mark limits (`class_limits_as`, [EST]). **Marcell
     to confirm** (it is a thesis number).
2. **How it was checked**
   - `colcon build --packages-up-to asv_sim`: clean, no warnings.
   - `colcon test --packages-select asv_core asv_sim`: asv_core
     `Summary: 79 tests, 0 errors, 0 failures` (73 gtests: 57 + 16
     `test_fake_detector`); asv_sim `Summary: 152 tests, 0 errors, 0
     failures` (138 gtests: 133 + 5 `test_fake_detection`). Total 231.
   - `python3 -m unittest discover -s tests`: `Ran 137 tests ... OK`.
   - Statistics (seeded, 10⁵ scans, all within 3 standard errors): range and
     bearing mean and σ, miss rate (P_miss 0.2), false-alarm count, mean
     range and bearing (λ 0.5), mix-up rate (p_c 0.1) and the share of each
     wrong colour; same seed gives identical output; misses do not change
     the range of the detections that are kept.
   - Live (`sim.launch.py gui:=false`, boat at the spawn):
     `ros2 topic hz /asv/perception/buoys --use-sim-time` →
     `average rate: 10.000, min: 0.100s max: 0.100s std dev: 0.00000s`.
     One message (stamp 25.200 s): 4 detections, the Task 1 gate
     Sur-Marks: green at −0.298 / −0.137 rad, red at 0.130 / 0.327 rad;
     ranges 5.158, 13.093, 13.100, 5.068 m (true 5.229, 13.088, 13.088,
     5.229 m; differences ≤ 1.6 σ_r). Log: `map with 32 objects received`.
3. **What could not be checked:** detections while the boat moves were not
   recorded live (the gtests cover pose and yaw); nothing visual in this step.
   Marcell can look with `ros2 topic echo /asv/perception/buoys` during
   `task1.launch.py`.
4. **How to read the code:** start at `fake_detector.hpp` (the equations
   are in its header comment), then `FakeDetector::scan()`: visibility
   (`hypot` against `lidar_range`), r̃ and β̃ (same lines as the equations),
   `colour_seen()`, `mixed_up()`, `add_false_alarms()` (Poisson by
   multiplying uniforms). `ScanGrid::due()` = decision A. The node only wires
   `on_map()` and `on_odom()`.
5. **Numbers for the thesis**
   - σ_r 0.10 m, σ_β 0.02 rad, P_miss 0, λ 0, p_c 0, centroid bias off,
     false alarms up to 18.75 m [EST, `detector/nominal.yaml`].
   - Lidar limits A-0 4.75, A-2 10.75, Sur-Mark 18.75 m; camera limits ZED
     8.9 / 16.2 / 20.1 m, wide 5.3 / 9.6 / 12.0 m [REPO `asv_report.json`];
     indicator and vessels = Sur-Mark limits [EST].
   - Scan rate 10.000 Hz, stamps on the 0.1 s grid [MEASURED].
6. **Check your understanding**
   1. Why are the detections sorted by bearing and not sent in map order?
   2. Why are the range and bearing noise drawn even for a buoy that is
      then missed?
   3. Why is the lidar limit measured from the lidar but the range from
      base_link?

---

## M3.2 Measurement queue

**Goal:** one class that releases measurements in stamp order after a fixed
delay L, shared by slam_replay and slam_node (D13 a, `design.md` §7).
**Done when:** shuffled input (within L) gives identical output; a message
later than L is counted, not applied.
**Decisions needed:** none.

---

## M3.3a EKF core, part 1: state and prediction

**Goal:** state x, y, θ, b_g + buoys; prediction from GNSS velocity and gyro
rate minus b_g, with process noise (D5 C, D14 a; `design.md` §6, §6.1).
**Done when:** Jacobians match a numerical derivative; a hand-made straight run
predicts the expected pose and covariance growth.
**Decisions needed:** none. Note: the GNSS velocity is the antenna's (includes
ω × R·l, M2.6c); this step handles it.

---

## M3.3b EKF core, part 2: GNSS position update

**Goal:** GNSS position update with the antenna lever arm.
**Done when:** Jacobian test; a hand-made scenario converges to the true
position.
**Decisions needed:** D6 (coloured GNSS error: white with enlarged covariance,
or two extra states); D15 raised again (planar or 3D lever arm).

---

## M3.3c EKF core, part 3: heading update

**Goal:** dual-antenna heading update; makes b_g observable.
**Done when:** Jacobian tests; wrapped innovations near ±π give the right sign.
**Decisions needed:** none.

---

## M3.3d EKF core, part 4: buoy update and new landmarks

**Goal:** range-bearing update and new-landmark initialisation with
cross-covariances.
**Done when:** Jacobian tests; a hand-made scenario with two buoys gives the
expected map.
**Decisions needed:** none.

---

## M3.4 Data association and map management

**Goal:** Mahalanobis gate, colour rule (known colour must match; UNKNOWN uses
the gate only), tentative landmarks promoted after n_conf sightings (D13 a).
**Done when:** unit tests with known correspondences.
**Decisions needed:** none.

---

## M3.5 slam_replay

**Goal:** read a bag, run corruptor → detector → filter in stamp order, write
the run folder (D4 a, `design.md` §9).
**Done when:** same config and seed twice give identical results; wall time of
one replay and one evaluation (Release, median of 3, `nproc`) and the bag
duration reported (the number D10 needs).
**Decisions needed:** none. ⚠ `<depend>` on rosbag2_cpp.

---

## M3.6 Evaluation

**Goal:** `asv_eval` (grown from `analyze_run.py`): position and heading error,
map error, NEES and NIS (`design.md` §10).
**Done when:** the matched check passes over M runs: ANEES inside its 95 % band
at about 95 % of the time steps (at least 90 %).
**Decisions needed:** D10 (runs per setting). ⚠ new package.
