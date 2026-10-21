# Critique of the 2026-10-20 review of the M1 code and the M2+ plan (Claude)

Short notes, not thesis text. Nothing in the code or the plan was changed.
Every "action" below is a **question for Marcell**; no decision is made here.

## How it was made

- Reviewed file: [`2026-10-20_m1_code_and_m2_plan_review.md`](2026-10-20_m1_code_and_m2_plan_review.md)
  (Gemini through `agy`, run by Marcell; the model name is not recorded in the file).
- State reviewed: commit 3a29d05 (after M1.8 and the docs update).
- Claude checked each point against the code and the docs: is the claim true,
  is it already covered, is the fix sound?
- Verdicts as in the 2026-10-18 critique: **valid** (true and new), **partly**
  (a real core, but overstated or with a wrong fix), **covered** (the docs
  already say it), **wrong**.
- "Checked" = read in the code or the docs, or computed here. Nothing was run in
  Gazebo; the gtests were not rerun.

## Verdict table

| # | Reviewer's point (short) | Reviewer | Verdict | Adjusted severity |
|---|---|---|---|---|
| A1 | On the tick of a waypoint switch, `Route` keeps the old segment's s and e, so LOS and d_end use them on the new segment | high | **valid** (real bug, small effect on Task 1) | medium |
| A2 | No test checks LOS or d_end on the switching tick | medium | **valid** | medium (goes with A1) |
| A3 | Speed law uses the course error, not the heading error | medium | **covered**: a documented choice (10.13 a); naming point is fair | low |
| A4 | dt can be 0, huge or negative | medium | **mostly wrong**: odom timeout and sim time bound it | low |
| A5 | Frame `world` hard-coded; twist assumed in the body frame | medium | **partly**: only matters for optional M7.3 | low |
| A6 | Three ways of loading parameters; add YAML config structs to `asv_core` | low | **partly**: real inconsistency, the fix adds a dependency to `asv_core` | low |
| A7 | `compare_runs.interpolate` divides by zero on duplicate stamps | low | **wrong** | none |
| B1 | `asv_core` is ROS-free but holds detector and queue, which need `asv_msgs` types | high | **partly**: no clash (M1 already does structs + adapter); worth one line in `design.md` §12 | low |
| B2 | Corruptor must make the GNSS velocity itself; `nav` must record ground truth; UM982 velocity noise unknown | medium | **covered**, except the UM982 number (still open, raised 2026-10-18) | medium |
| B3 | Close D11 now with "A, nominal = ideal"; move M2.4 after M2.1 | medium | **partly**: D11 does block M2.4/M2.6; the order is already so; "A" and "ideal" are two separate questions | medium |
| B4 | Cut scope: D6 a, D11 without processing, drop M5, Python replay | medium | **partly / repeat**: D6 and M5 were raised on 2026-10-18; the Python replay is a wrong fix | low |

## Points worth acting on

### A1 + A2. One-tick glitch at every waypoint switch — medium

- Checked, `src/guidance.cpp` `Route::update()` lines 39–54: s (`along_`)
  and e (`cross_`) are computed on segment i, then `segment_` becomes i+1;
  the comment says "computed on the next call".
- In the same tick `Follower::step()` (`src/follower.cpp` 105–122) uses them on
  the new segment:
  - `LosGuidance::compute()`: d = s_old + Δ walked along segment i+1, so the
    LOS target jumps ahead.
  - `remaining_distance()`: d_end = L_{i+1} − s_old + …, wrong.
  - `out.cross_track`: published e is the old segment's.
- Size on the Task 1 route (computed here from `task1_route.yaml`, Δ = 3.0 m,
  R_switch = 1.5 m, boat assumed exactly on the old path at the switch point):
  - LOS target off by 1.9–4.1 m for one 50 ms tick, at each of the 8 switches.
  - d_end too small by 2.0–16.8 m (largest at the first switch). With
    K_stop = 0.5 1/s the stop limit stays ≥ 7.7 m/s, far above the speeds
    (≤ 1.0 m/s), so u_cmd is **not** affected on Task 1.
  - So the visible effect is one tick of a wrong ψ_d → wrong steer and spin.
    The heading controller is stateless, so it lasts one tick. I expect it to
    be invisible in the M1.5 results, but that is unverified.
  - The published cross-track error: on that tick it is the old segment's e,
    small near the old path, so I expect no change to `cross_track_max_abs`
    in `analyze_run.py` (unverified).
- The reviewer's fix (recompute s and e on the new segment right after the
  switch) is sound. Keep "at most one switch per update" (test
  `AtMostOneSwitchPerUpdate`): segment 3→4 is only 3.04 m long, shorter than
  2 R_switch.
- A2 is true: `test_guidance.cpp` has no test that calls `los.compute()` or
  `remaining_distance()` on the tick of a switch; the LOS tests call
  `advance_to_segment()` and then update again.
- Cost of fixing: changes follower behaviour, so the M1.5 numbers (64.50 s,
  0.671 m, …) would need three new runs to stay valid. That is an "ask first"
  item (numbers that appear in the thesis).
- Question: fix it now (one small step: fix + test + 3 Task 1 runs), fix it
  without rerunning and mark the M1.5 numbers "before the fix", or only note it
  as a known limitation in `AUTONOMY.md` §8?

### B3. D11 blocks M2.4 and M2.6 — medium

- Checked: M2.4 and M2.6 are `[?]` on D11 (`master_plan.md` §4); D11's message
  content is open (`decision_notes.md` D11, questions 7–9).
- The reviewer's "Option A (nominal = ideal)" merges two questions:
  - 7a: message content A / B / C (A recommended in `decision_notes.md`).
  - 7b: nominal processing level ideal / tuned / Rover default. Your 2026-10-11
    reason was "Corrupted IMU messages should follow real world scenarios";
    "ideal" as nominal goes against that. With A, the processing is a set of
    switches, so "ideal" is still available as one level of a study.
- "Move M2.4 after M2.1": already the order (M2.1, M2.2, M2.4, M2.5, M2.6).
  M2.1, M2.2 and M2.5 do not need D11, so work can start without it.
- "D5 C is planar, so no roll/pitch message": correct, and `decision_notes.md`
  says the same (nothing in the core uses roll/pitch).
- Question: decide D11 (7a, 7b, 8, 9) when the M2 plan is written, so M2.4 can
  become `[ ]`?

### B2. UM982 velocity accuracy is still missing — medium

- Checked: `design.md` §4.2 already says the corruptor makes the GNSS velocity
  "from ground-truth position differences plus noise" (not from the smoothed
  odom twist, which the reviewer suggests, and which `design.md` says is unfit
  as true velocity). The `nav` profile (§8) already records ground-truth odom.
- Still open: "The UM982 velocity accuracy is still to be looked up
  [DS, to check]". Already listed in the 2026-10-18 critique ("What Gemini
  missed"); still unresolved.
- Question: look it up as part of the M2.6 brief (datasheet), or before the M2
  plan?

### A3. Name of the error used in the speed law — low

- Checked: `AUTONOMY.md` §5.4 defines ψ̃ = wrap(ψ_d − χ), the **course**
  error; `milestone1_plan.md` (implemented form after M1.2f) says "The same ψ̃
  feeds cos ψ̃ in u_cmd and the ψ_turn rule". So the behaviour is the agreed
  one, not a bug.
- The difference to the heading error is the sideslip angle atan2(v, u), small
  with yVV = 58 N/(m/s)².
- Fair part: the symbol ψ̃ and the code name `heading_error` both suggest the
  heading error. The guidance symbols are still "proposed".
- Question: rename the symbol to χ̃ (and `heading_error` → `course_error` in
  code) when App. C fixes the symbols, or keep ψ̃ with a note?

### A6 / B1. One pattern for M2/M3 configuration and types — low

- A6 checked: the follower gets ROS parameters (boat values from `asv.yaml`
  through the launch file); the teleop reads `asv.yaml` itself (T1 a, so
  `ros2 run` needs no parameter file); the player uses ROS parameters. These
  are three patterns.
- The reviewer's fix (yaml-cpp loaders in `asv_core`) adds a dependency to a
  package decided as "Eigen only" (D13 a). A lighter option: plain config
  structs in `asv_core`, YAML/ROS loading in the node or in slam_replay.
- B1 checked: `design.md` §4 and §6 say the nodes are "thin wrappers" around
  `asv_core`. M1 already works this way (`NavState` struct + `to_nav()` in the
  node). So there is no clash; only `design.md` §12 doesn't state it.
- Question: add one line to `design.md` §12 ("`asv_core` has its own structs;
  nodes convert from and to `asv_msgs`"), and settle the config pattern in the
  M2 plan?

## Where the reviewer was wrong

- **A7, division by zero**: `bisect_left` returns k with
  taus[k−1] < τ ≤ taus[k]; equality returns early, so b[0] − a[0] > 0
  always, even with duplicate stamps (`sorted(poses)` in `track()`, line 68,
  keeps duplicates, but bisect never picks a zero-width interval).
- **A4, dt**:
  - "Gazebo pauses → dt of seconds": with `use_sim_time` the ROS clock stops
    too.
  - An odom gap longer than `odom_timeout` = 0.5 s sets STOPPED
    (`Follower::step()` line 77), so dt ≤ 0.5 s in RUNNING.
  - "dt = 0 then a doubled step": after a skipped tick, the integral uses the
    real elapsed time since the last integration, which is correct.
  - Left over: `control_stamp_` is updated even when dt < 0. Ground truth comes
    from one Gazebo publisher, so this can't happen now (low).
- **A5, frames**: map = world with the same origin (`design.md` §2), so the
  frame id is only a label. M4.1 already plans `/asv/slam/odom` "same format
  as ground truth", and M7.3 already describes the body-frame twist from the
  filter inputs. Only relevant if optional M7.3 is done; a `child_frame_id`
  check then is cheap.
- **B4, Python replay**: the filter is C++ in `asv_core`; a Python replay
  would need bindings (pybind11, a new dependency) or a second filter. Reading
  a bag in C++ with `rosbag2_cpp` plus `rclcpp::Serialization` is the standard
  path, not a "bespoke engine".
- **B4, the rest**: D6 a/b and dropping M5 are repeats of the 2026-10-18 review
  (4.5, 3.2). M5 is already optional ("only if time"). D6 has a written
  fallback (a with one factor per mode). The reviewer said it would skip
  repeats.
- **Tone**: B3 and B4 again decide open decisions ("formally close D11",
  "lock in D6 a"). Those are yours.

## What the reviewer missed

- The 2026-10-18 points that are still open, besides D11 and UM982: map
  consistency metric (A), GNSS-only baseline (E), filter initialisation (F),
  divergence policy, removal of false landmarks, dates (K). None of them are
  needed before M3, but the M2 plan is the point to say where each one goes.

## Suggested next step

- Marcell picks what to adopt. The A1 fix would be its own short step (it
  changes M1.5 numbers). The `design.md` §12 line and the D11 questions go into
  the M2 plan.
- Nothing here blocks M2.1.
