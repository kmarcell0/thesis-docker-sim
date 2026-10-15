# M0 plan: Housekeeping

M0 needs no approved milestone plan (`CLAUDE.md`); this file only logs the
briefs and reports of its steps. Notes are short bullets.

Status: log (no approval needed)

---

## M0.1 Add the docs and correct the stale ones

**Goal:** new docs in the repository, stale statements corrected, `ws/data/`
ignored, `CONTEXT.md` settled, everything pushed.
**Done when:** `git fetch` + `git status` up to date with GitHub;
`git check-ignore -v ws/data/test.bag` names the new `.gitignore` line;
`grep -rn "already exists" ws/src/asv_sim/docs/milestone1_plan.md` finds nothing.
**Decisions needed:** none (D-numbers); two "ask first" items (below).

### Brief (agreed on 2026-10-15, short form)
- State at start: eeb6c81 already added `CLAUDE.md`, `docs/`, the corrected
  `milestone1_plan.md`; not pushed. Missing: `.gitignore` line, `CONTEXT.md`,
  this log, the `[x]` mark.
- Marcell's answers: remove `CONTEXT.md`; a second `M0.1:` commit (no amend);
  add `ws/data/` to `.gitignore` and push; nothing else unpushed or untracked
  on his machine.
- Files: ⚠ `.gitignore`, ⚠ `CONTEXT.md` (delete), `docs/plans/m0_plan.md` (new),
  `docs/master_plan.md` (mark), `CLAUDE.md` (CONTEXT.md paragraph, Known state,
  layout note).
- Check: the three "done when" commands; Python tests still 28 OK.

### Report (2026-10-15)
Commits: `git log --grep "^M0.1:"` (eeb6c81 docs, plus the second M0.1 commit).

1. What changed
   - `.gitignore`: new line `ws/data/`.
   - `CONTEXT.md`: removed (`git rm`).
   - `CLAUDE.md`: CONTEXT.md paragraph now says it was removed; Known-state
     bullet about `ws/data/` dropped; layout note says "git-ignored".
   - `docs/master_plan.md`: M0.1 marked `[x]`.
   - `docs/plans/m0_plan.md`: this log.
2. How it was checked
   - `git check-ignore -v ws/data/test.bag` → `.gitignore:8:ws/data/	ws/data/test.bag`
   - `grep -rn "already exists" ws/src/asv_sim/docs/milestone1_plan.md` → no output (rc 1)
   - `python3 -m unittest discover -s tests` (in container) → `Ran 28 tests ... OK`
   - `colcon test`: not run, `asv_sim` has no C++ tests yet (baseline unchanged).
   - Push and `git status` after `git fetch`: see the chat summary of this step
     (done after this file is committed).
3. What could not be checked: that nothing is left on Marcell's machine
   (he confirmed it, 2026-10-15).
4. How to read the code: no code in this step.
5. Numbers for the thesis: none.
6. Check your understanding
   - Why does `ws/data/` have to be ignored before the first bag is recorded?
   - Which files are now the source of truth instead of `CONTEXT.md`?

### Marcell's notes
Written by Marcell only; Claude never edits this part.

---

## M0.2 Simulation time everywhere

**Goal:** every node we start runs on Gazebo's simulation clock; measured to
hold also when Gazebo runs slower than real time.
**Done when:** `ros2 param get <node> use_sim_time` is true for each node; a
20 Hz test timer fires 20 (±1) times per simulated second at a real-time factor
clearly different from 1 (factor stated); the RTF of a normal headless run is
reported (for D10).
**Decisions needed:** none (D-numbers). Step option for `actuator_check.py`:
a) switch to the ROS clock and rerun, b) record as exception.

### Brief (agreed on 2026-10-15)
1. Goal: as above.
2. Decisions: `actuator_check.py` option **a** (Marcell, 2026-10-15): ROS
   clock, rerun once, compare with the Step 1 table; stop if a number changes.
3. Why: Gazebo stamps in simulation time; nodes on the wall clock fire timers
   too often per simulated second when RTF < 1, break TF lookups and shorten
   timed manoeuvres. Thesis: Integration and Time Synchronization; RTF feeds D10.
4. New ideas: simulation time (`/clock`, `use_sim_time`), real-time factor
   (glossary).
5. Files: `launch/sim.launch.py` (`SetParameter use_sim_time=True` for all
   nodes), `tools/sim_time_check.py` (new), `tools/actuator_check.py` (ROS
   clock), `docs/design.md` §13, this log, master plan mark, `CLAUDE.md` Known
   state.
6. How it works: ROS-clock timer at 0.05 s sim time → 20 ticks per sim second
   at any RTF; wall timer → 20 / RTF ticks per sim second;
   RTF = Δt_sim / Δt_wall. Gazebo slowed at runtime with
   `gz service .../set_physics` (unverified before the run).
7. Check: `ros2 param get` on bridge and a static TF node → True;
   `sim_time_check.py` at the normal RTF and at RTF ≈ 0.5 → 20 ± 1 ROS ticks per
   sim second, wall ticks ≈ 20 / RTF; actuator_check rerun vs. Step 1 table;
   28 Python tests OK.
8. Can't check: RTF with the GUI open (Marcell: `gz stats`).

### Report (2026-10-15)
Commit: `git log --grep "^M0.2:"`.

1. What changed
   - `ws/src/asv_sim/launch/sim.launch.py`: first action is
     `SetParameter(use_sim_time=True)`, so the bridge and all static TF nodes
     run on `/clock`.
   - `ws/src/asv_sim/tools/sim_time_check.py` (new): 20 Hz ROS-clock timer and
     20 Hz wall timer side by side; prints ticks per simulated second and the RTF.
   - `ws/src/asv_sim/tools/actuator_check.py`: `use_sim_time` on; phases timed
     with `get_clock().now()`; waits for `/clock`. The "is the sim running?"
     timeout stays wall time on purpose (sim time does not advance without
     Gazebo).
   - `docs/design.md` §13: measured facts below. `CLAUDE.md` Known state: RTF line.
2. How it was checked (headless sim, `colcon build` asv_sim OK)
   - `ros2 param get <node> use_sim_time` for all 10 nodes → `True` each
     (`/asv_bridge`, 9 `/static_tf_*`).
   - `sim_time_check.py --duration 30`, world RTF 1.0:
     `simulated 30.000 s, wall 35.395 s, RTF 0.848; ROS-clock timer 601 ticks =
     20.03 per sim s; wall timer 708 = 23.60 per sim s; PASS`
   - after `gz service .../set_physics ... real_time_factor: 0.5` (→ `data: true`):
     `wall 60.848 s, RTF 0.493; ROS-clock timer 600 = 20.00 per sim s; wall timer
     1217 = 40.57 per sim s; PASS`
   - `actuator_check.py` (RTF set back to 1.0): 10 N 1.00, 20 N 1.52, 40 N 2.27
     m/s (error −0.0 %); pods −0.4 rad → +0.1497 rad/s; differential +0.1755
     rad/s. Step 1 recorded 1.00 / 1.52 / 2.27, +0.15, +0.18: unchanged.
   - `python3 -m unittest discover -s tests` → `Ran 28 tests ... OK`; `colcon
     test`: no asv_sim tests yet.
3. What could not be checked: RTF with the GUI open (Marcell: `gz stats` while
   the window is open). The RTF depends on this PC and its load.
4. How to read the code: `sim.launch.py` `_launch_setup` (first line of
   `actions`); `sim_time_check.py` `main()`: wait for `/clock`, reset counts,
   loop until Δt_sim ≥ duration, RTF = Δt_sim / Δt_wall.
5. Numbers for the thesis
   - Headless RTF 0.848 [MEASURED, this PC] (input for D10: a recording of T
     simulated seconds takes about T / 0.85 wall seconds).
   - Physics step 0.004 s, requested RTF 1.0 [REPO `roboboat_course.sdf`].
6. Check your understanding
   - At RTF 0.5, how many times per simulated second would a
     `create_wall_timer` at 20 Hz fire, and why is that a problem for the
     follower?
   - Why does `actuator_check.py` keep a wall-clock timeout in `wait_for_odom`?
   - Where does a node with `use_sim_time` get its time before the first
     `/clock` message, and what does it read?

### Marcell's notes
Written by Marcell only; Claude never edits this part.

---

## M0.3 Installed packages

**Goal:** record which libraries the later steps need are installed in the
container.
**Done when:** result recorded in `docs/design.md` §12.
**Decisions needed:** none.

### Brief (agreed on 2026-10-15, short form)
- Read-only check; nothing gets installed. Missing packages are reported; the
  step that needs one asks for it.
- Files: `docs/design.md` §12 (table; stale "git-ignored from M0.1 on"), this
  log, master plan mark. (The progress-bar rule in `CLAUDE.md` was committed
  separately, a4af79c.)
- Check: `dpkg-query` for cv-bridge, OpenCV, rosbag2-cpp, PCL, yaml-cpp-vendor,
  numpy, scipy, matplotlib, Eigen, gtest; `python3 -c "import numpy, scipy,
  matplotlib"`.

### Report (2026-10-15)
Commit: `git log --grep "^M0.3:"`.

1. What changed: `docs/design.md` §12: table of packages, what needs them, and
   their versions; master plan mark; this log.
2. How it was checked (in the container, `dpkg-query -W -f='${Status}|${Version}'`)
   - `install ok installed`: ros-jazzy-cv-bridge 4.1.0, libopencv-dev 4.6.0,
     ros-jazzy-rosbag2-cpp 0.26.11, ros-jazzy-yaml-cpp-vendor 9.0.1,
     python3-numpy 1.26.4, python3-scipy 1.11.4, python3-matplotlib 3.6.3,
     libeigen3-dev 3.4.0, libgtest-dev 1.14.0, ros-jazzy-ament-cmake-gtest
     2.5.6 (extra, needed by M1.2).
   - `libpcl-dev`: `no packages found matching libpcl-dev`.
   - `python3 -c ...` → `numpy 1.26.4 scipy 1.11.4 matplotlib 3.6.3`.
   - Docs only: Python tests unchanged (28), not rerun.
3. What could not be checked: nothing.
4. How to read the code: no code.
5. Numbers for the thesis (Toolchain): the versions above [MEASURED].
6. Check your understanding
   - Which planned step would need PCL, and is it core or optional?
   - Why is a missing package added through the Dockerfile and not by
     installing it inside the running container?

### Marcell's notes
Written by Marcell only; Claude never edits this part.
