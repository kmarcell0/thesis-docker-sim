# Gemini review of `master_plan.md` v1.4, checked by Claude (2026-10-18)

Short notes, not thesis text. Nothing in the plan was changed. Every
"action" below is a **question for Marcell**; no decision is made here.

## How it was made

- Reviewer: Gemini (`gemini-3.8-flash-high`) through the `agy` CLI, read-only
  plan mode. Input: `master_plan.md`, `decision_notes.md` and `design.md`
  (v1.4 state, commit ca9730a), pasted into the prompt.
- Gemini's full answer: [`2026-10-18_gemini_master_plan_raw.md`](2026-10-18_gemini_master_plan_raw.md).
- Then Claude checked each point against the docs: is the claim true, is it
  already covered, is the proposed fix sound?
- Verdicts: **valid** (true and new), **partly** (a real core, but overstated
  or with a wrong fix), **covered** (the docs already say it), **wrong**.
- Letters as in D14 a: R = process noise covariance, Q = measurement noise
  covariance. (Gemini mixed the two conventions.)

## Verdict table

| # | Gemini's point (short) | Gemini | Verdict | Adjusted severity |
|---|---|---|---|---|
| 1.1a | D5 C holds the world-frame GNSS velocity in an outage, so the predicted path can't turn | high | **valid** (the rule is still open) | medium |
| 1.1b | GNSS velocity (in R) and GNSS position (in Q) are correlated noise, so P is overconfident | high | **partly**: absent in our simulation | low |
| 1.1 fix | Go back to D5 b | | **wrong fix**: b has the same outage problem; a better hold rule fixes C | |
| 1.2 | D6 b is unobservable (position and slow GNSS error have the same Jacobian) | high | **wrong** | none |
| 1.3 | Gyro bias becomes unobservable in outages; freeze it; test wrapping | medium | **covered / wrong fix** | low |
| 1.4a | "Only coloured detections start landmarks" means buoys seen only abeam or astern are never mapped | high | **partly valid** (the range part is wrong) | medium |
| 1.4b | Add a known-correspondence run to separate association errors from filter errors | | **valid** | medium |
| 1.5 | The replay queue may add latency or drop bag messages | medium | **wrong** (stamp-based, no wall clock) | none |
| 2.1 | Fake detector = inverse crime | high | **covered** (`design.md` §5) | low |
| 2.2 | Sweeps keep the filter noise fixed, so the NEES rise is trivial; add matched and tuning-sensitivity sweeps | high | **partly valid**: good addition, "identity" overstated | medium-high |
| 2.3a | Same-bag replays: shared deterministic errors bias the ANEES | high | **covered** (`decision_notes.md` D10 caveat) | medium |
| 2.3b | "In band at 95 % of time steps" treats correlated steps as independent | | **valid**, but Gemini's fix has the same flaw | low-medium |
| 2.4 | No consistency metric for the map, though the question asks for it | high | **valid** | high |
| 3.1 | Too much for one bachelor student | high | **partly**: real risk, but most extras are already optional or skipped | medium |
| 3.2 | Cut M1.7, M1.8, M5, M7, §14 | medium | **mostly covered**: M5/M7 optional, benchmarking `[-]`; M1.8 is a fair cut candidate | low |
| 3.3 | Critical path; no live work before replay works | high | **partly**: the order already does this; a real gap: the plan has no dates | medium |
| 4.1 | D5 C's reason ("outages are few") clashes with D9 a (outages are core) | high | **valid** | high |
| 4.2 | Outage prediction is not specified (world vs body frame) | high | **valid** (same as 1.1a) | medium |
| 4.3 | Benchmarking rows are stale | low | **wrong**: labelled "left out for now" on purpose | none |
| 4.4 | slam_replay could feed clean data to the filter; M4.1's guard does not cover it | high | **partly valid**: the risk is real, the proposed fix doesn't fit | medium |
| 4.5 | Close D6, D10, D11 now (D6 a, M = 30) | medium | **partly**: D11's accelerometer detail could be parked; the rest is Marcell's call and partly rests on 1.2 | low |
| 5.1 | No filter initialisation (x₀, P₀) | high | **valid** | medium-high |
| 5.2 | No GNSS-only baseline (landmark updates off) | high | **valid** | high |
| 5.3 | Add numerical guards, clamp P | medium | **partly**: a divergence policy is missing; clamping is wrong | low-medium |
| 5.4 | No removal of confirmed false landmarks | medium | **valid** | medium |

## Points worth acting on

Ordered by adjusted severity.

### A. Map consistency is not measured (2.4) — high
- Checked: `design.md` §10 defines NEES only for the pose, e = [x, y, ψ], n = 3.
  For the map it gives only RMSE, missed and spurious counts. The project goal
  (CLAUDE.md) asks for accuracy **and consistency** of both the pose and the map.
- `decision_notes.md` D6 already uses "map NEES" in its argument but never
  defines it.
- Possible form (proposed, not in the glossary yet):
  ε_j = (m_j − m̂_j)ᵀ P_jj⁻¹ (m_j − m̂_j), 2 degrees of freedom, for confirmed
  landmarks matched by the Hungarian assignment. Average it over the M runs, as
  for the pose.
- Question: add it to `design.md` §10 and to M3.6's "done when"?

### B. The D5 C reason clashes with D9 a (4.1) — high
- Checked: D5 status says "GNSS outages are few if any"; D9 a (2026-10-11) makes
  GNSS modes and outages core experiments. `decision_notes.md` itself says
  "outages are its weak spot, and outages are a core experiment".
- Gemini calls it "fatal". It isn't: per `decision_notes.md`, mapped buoys pin
  the position during an outage for every motion model. The motion model only
  matters where no mapped buoy is in view.
- What the thesis needs is a reason for D5 C that fits D9 a. Example: "outages
  are tested where buoys are in view", which also answers D5 question 2 (where
  the outages are placed).
- Question for Marcell: keep C and restate the reason, or narrow where outages
  are placed? (Changing D5 is his call; Gemini's "go back to b" doesn't fix the
  problem, see C.)

### C. Outage prediction rule (1.1a, 4.2) — medium
- Checked: `design.md` §6: "the last value is held with a growing uncertainty
  (rule set in the M3.3a brief)". Holding (v_E, v_N) in the **world** frame
  means the predicted path goes straight while the gyro says the boat is turning.
- Fix that keeps D5 C: at the start of an outage, rotate the last GNSS velocity
  into the body frame with ψ̂ and hold (u, v). Then predict
  x += (u cos ψ − v sin ψ)·dt, y += (u sin ψ + v cos ψ)·dt.
  Only during outages does the prediction then depend on ψ, so the Jacobian
  gets ∂/∂ψ terms there.
- This is about what b does in an outage, so going back to b (Gemini's fix)
  gains nothing for outages.
- Action: put this into the M3.3a brief as an option, with "hold world
  velocity" as the alternative.

### D. Sweep design: matched vs mismatched (2.2) — medium-high
- Checked: `design.md` §11: filter noise = inflation × nominal, fixed while the
  true noise changes. A matched check exists only at nominal (§10).
- Gemini overstates it: the NEES does not scale exactly 4× or 25×. Only the
  swept sensor's share of the error grows, and gating and association also react.
  But the direction of the result is predictable, and it mostly shows the chosen
  inflation.
- Gemini's addition is cheap with replay (D4 a): (1) each noise level with the
  filter matched to it (does the linearisation hold at high noise?), and (2)
  true noise at nominal, the filter's Q scaled 0.2× to 5× (tuning sensitivity).
  The current mismatched sweep stays: it answers "tuned once, then conditions
  change".
- Question: add (1) and/or (2) to `design.md` §11? It raises the run count (D10).

### E. GNSS-only baseline (5.2) — high
- Checked: the only baseline is optional M7.1 (robot_localization).
- A switch `landmark updates off` in the same filter, run on the same bags,
  shows what mapping the buoys adds. Expect little in RTK fixed (8 mm);
  the interesting cases are standalone, float and outages.
- Cheap: one config flag plus one extra setting per sweep row.

### F. Filter initialisation (5.1) — medium-high
- Checked: nothing in `design.md` §6 or M3.3a says how x₀ and P₀ are set.
- It matters for NEES: the initial error must fit P₀, or the first part of
  every run is inconsistent.
- Natural choice: first GNSS fix (lever arm removed) and first heading for x₀;
  P₀ from their reported σ; b_g = 0 with σ = the turn-on bias σ (0.0017 rad/s,
  [EST] `asv.yaml`). Gemini's "1 s stationary" adds little; the bag may start
  moving.
- Action: add to the M3.3a brief.

### G. Landmarks seen only outside the cameras (1.4a) — medium
- Checked: lidar 360°, colour only in the forward camera fields of view (ZED 72°,
  wide 85°). "Only coloured buoy detections may start a new landmark."
- Gemini is wrong that buoys beyond camera range stay unmapped: camera range is
  longer than lidar range for every class (A-0 8.9 vs 4.75 m, A-2 16.2 vs
  10.75 m, Sur-Mark 20.1 vs 18.75 m, `design.md` §5).
- Real part: a buoy seen only from the side or behind is never mapped. On this
  course most buoys are approached head-on, so the effect is probably small, but
  it is not measured.
- Action: count such buoys in the evaluation ("in lidar range, never in camera
  view") instead of changing the rule. The rule exists for a reason: it keeps
  false alarms and "other" vessels out of the map.

### H. Known-correspondence run (1.4b) — medium
- Checked: the true buoy ID is logged, and NIS is also computed with true IDs
  (`design.md` §10). But there is no filter **run** with true association.
- A switch "associate with the true ID" separates association failures from
  filter error in the detector-noise sweep.

### I. Landmark removal (5.4) — medium
- Checked: tentative landmarks are dropped; nothing removes a confirmed one.
- Open detail: do the fake detector's false alarms have a colour? If they are
  UNKNOWN, they can't start landmarks at all (rule in G) and the false-alarm
  sweep tests only association. The M3.1 brief should say which.

### J. Clean data leaking into slam_replay (4.4) — medium
- Checked: the M4.1 guard (`ros2 node info`) covers slam_node only. In
  slam_replay the filter gets its data in-process from the corruptor, not from
  topics, so Gemini's "throw on a forbidden topic name" doesn't fit.
- Better test for M3.5: run once with the noise profile set to zero and once at
  nominal. If the results don't differ, the filter is not reading the corrupted
  data. Or the filter's input type only accepts corruptor output.

### K. No dates (3.1, 3.3) — medium
- The plan orders the steps but gives no thesis deadline and no target week per
  milestone. Gemini's "out of time" warning can't be checked without them.
- Question: add a target date per milestone (Marcell's dates)?

### L. Smaller ones
- 2.3b: "ANEES in band at ≥ 90 % of steps" assumes independent steps. It is a
  common heuristic; say so in Filter Consistency. Gemini's fix (a time-averaged
  ANEES against χ²_{KMn}/(KM)) assumes the same independence, with an even
  narrower band, so it is not better.
- 5.3: add a divergence policy for sweeps (NaN or P not positive definite → mark
  the run "diverged", count it, keep going). Gemini's "clamp P_ii to 1e-8"
  would hide bugs and break the consistency results; don't.
- 4.5: with D5 C the accelerometer is not used by the filter, so the D11
  accelerometer-density question (70 vs 110 µg/√Hz) could be parked. The gyro
  processing part of D11 (low-pass, hold, stamps) still matters.
- 1.1b: real receivers do correlate Doppler and position errors. Our corruptor
  draws them independently, so the effect is absent in simulation. One line in
  the limitations.
- 3.2: M1.8 (teleop) feeds no thesis section; a fair candidate to skip.

## Where Gemini was wrong

- **1.2, D6 b "unobservable"**. The slow GNSS error is a Gauss-Markov state with
  a prior (σ_ss, T_c), and the position change between fixes is driven by the
  velocity input. So the filter can tell the two apart over time, and its
  covariance stays honest even where it can't. `decision_notes.md` D6 already
  has the evidence: b gave ANEES 0.95-1.07; a gave 9-29. Gemini ignored this
  table. True part: the map inherits the slow GNSS error. That limits absolute
  map accuracy (and should be stated), but the filter tracks it in the
  cross-covariances.
- **1.5, queue**: the queue works on `header.stamp` with no wall clock
  (`design.md` §3, §7, §9); replay and live use the same class on purpose.
- **1.3, "freeze b_g when σ_ψ > 15°"**: if nothing observes b_g, the EKF already
  leaves it alone and grows its variance. Bearings to mapped buoys still observe
  ψ in outages. The wrapping test is already in M3.3c's "done when".
- **4.3, benchmarking "stale"**: marked "left out for now" on purpose (v1.3
  change log); the M3.5 timing it quotes is for D10, not benchmarking.
- **Small facts**: "Hungarian data association in asv_slam" (Hungarian is only
  in the evaluation; the filter uses nearest neighbour); "D6 b saves 2-3 days"
  (the docs estimate half a day to a day); "M = 30" ignores the D10 rule.
- **Tone**: several fixes decide closed or open decisions (revert D5, force
  D6 a, fix M). Those are Marcell's (CLAUDE.md, Decisions).

## What Gemini missed

- The D5 C results depend on the GNSS velocity noise, and the UM982 velocity
  accuracy is still `[DS, to check]` (`design.md` §4.2). Needed before M2.6.
- `decision_notes.md` D6 question 5 (report total vs white covariance) and the
  D5 questions 1-2 (speed during outages, where outages go) are still
  unanswered. They decide the outage results more than the motion model does.

## Suggested next step

- Marcell reads A–K and picks which to adopt. Adopted items become doc edits
  (short-form step, own commit) or items for the M3.3a / M3.1 / M3.6 briefs.
- Nothing here blocks M1.2, the current step.
