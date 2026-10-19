Here is a critical, specific review of `docs/master_plan.md` and its supporting documents (`docs/decision_notes.md`, `docs/design.md`).

---

# Critical Review of Bachelor's Thesis Project Plan

---

## 1. Technical Soundness

### 1.1 Filter Design D5 Option C: Kinematic Model and Correlated Inputs
* **Citation:** `master_plan.md` §3 (Table row D5); `design.md` §6 ("Prediction (D5 C)") & §6.1; `decision_notes.md` §D5.
* **Quote:** 
  > *`master_plan.md` §3:* `decided: C (pose + gyro bias, GNSS velocity and gyro as inputs; option c plus a gyro-bias state), 2026-10-14: "The asv will allways be in view of the landmarks... so GNSS outages are few if any."`
  > *`design.md` §6:* `inputs are the GNSS velocity (v_E, v_N, world frame) and the gyro rate ω_m: x += v_E·dt, y += v_N·dt, ψ += (ω_m − b_g)·dt... Without a GNSS velocity (outage) the last value is held with a growing uncertainty...`
* **Why it matters:** 
  1. **Kinematic Decoupling & Outage Failure:** In D5 C, world-frame position propagation ($x, y$) is driven directly by world-frame GNSS Doppler velocity $(v_E, v_N)$, completely uncoupled from heading $\psi$ and body velocities $(u, v)$. In an outage, holding the last $(v_E, v_N)$ in the ENU world frame forces the filter to predict straight-line motion in geographic space. If the boat turns during an outage, the filter continues projecting position along the old world vector regardless of the gyro rate $\omega_m$. Holding world velocity during turns is physically wrong.
  2. **Correlated Process and Measurement Noise:** GNSS position fixes ($z_{pos}$) and GNSS Doppler velocity ($(v_E, v_N)$) originate from the same physical receiver tracking the exact same satellite carrier signals. In D5 C, GNSS velocity error enters as process noise $w_k$, while GNSS position enters as measurement noise $v_k$. This violates the fundamental Kalman filter assumption of zero cross-correlation between process and measurement noise ($E[w_k v_j^T] = 0$). This cross-correlation produces an overoptimistic and skewed covariance matrix $P$, directly corrupting NEES/NIS consistency evaluations.
* **Severity:** **High**
* **Concrete Fix:** Revert D5 from Option C to **Option B** (CTRV model with body velocities $u, v$, yaw rate $\omega$, and gyro bias $b_g$). Propagate position in the body frame ($\dot{x} = u \cos\psi - v \sin\psi$, $\dot{y} = u \sin\psi + v \cos\psi$). Treat GNSS Doppler velocity as a measurement update ($z_v = R(\psi)[u, v]^T$), not a process input. This preserves valid dead reckoning during outages and eliminates cross-correlated process/measurement noise.

---

### 1.2 Observability and Identifiability of Coloured GNSS Error (D6 Option b)
* **Citation:** `master_plan.md` §3 (Table row D6); `design.md` §6; `decision_notes.md` §D6.
* **Quote:** 
  > *`master_plan.md` §3:* `Add the slow GNSS error to the state (two extra states, reset at a mode switch; about half a day to a day more in M3.3b)... b in M3.3b, a kept as a switch...`
  > *`design.md` §6:* `z = ENU position of the antenna (2D)... with D6 b (recommended), only the white part, the slow part being two extra states...`
* **Why it matters:** 
  In D6 b, the GNSS position observation equation is:
  $$z_{pos} = \begin{bmatrix} x \\ y \end{bmatrix} + C(\psi)l_{xy} + \begin{bmatrix} \delta_E \\ \delta_N \end{bmatrix} + v_{white}$$
  The measurement Jacobians with respect to position $[x, y]^T$ and slow bias $[\delta_E, \delta_N]^T$ are identical ($I_2$). Without landmarks, boat position and GNSS bias are mathematically unidentifiable from GNSS alone. 
  Even when buoys are present: buoys are initialized into the state using the estimated boat pose ($m_{new} = p + h(z_{buoy})$). If the boat pose $p$ carries an initial GNSS bias $\delta_0$, the landmarks inherit that exact bias $\delta_0$. Subsequent buoy observations measure relative boat-to-landmark displacement, which is invariant under a common translation. The filter cannot decouple true position drift from GNSS bias drift.
* **Severity:** **High**
* **Concrete Fix:** Reject D6 Option b. Implement **D6 Option a** (treat GNSS error as white noise with covariance $Q_{GNSS}$ inflated to the total RMS specification). Frame the coloured noise not as a filter state, but as a deliberate violation of Kalman filter assumption A3 to test filter robustness in the thesis.

---

### 1.3 Heading Measurement and Observability of Gyro Bias ($b_g$)
* **Citation:** `master_plan.md` §4 (Step M3.3c); `design.md` §4.3 & §6 ("Heading update").
* **Quote:** 
  > *`master_plan.md` §4, M3.3c:* `heading update (with D5 C the gyro is an input, so there is no gyro update; the heading update is what makes b_g observable).`
  > *`design.md` §4.3:* `Availability is separate from the position mode... It drops during a sky-view outage.`
* **Why it matters:** 
  While GNSS heading is available, $b_g$ is observable through $\psi$ innovations over time. However, in a sky-view GNSS outage, GNSS heading drops simultaneously with position. If buoy observations are also degraded or sparse, $b_g$ becomes unobservable and heading errors accumulate. Additionally, the measurement model $h(\xi) = \text{wrap}(\pi/2 - \psi)$ relies on small-angle linearisation around nominal yaw. If heading uncertainty grows large during an outage, innovation wrapping introduces non-linearities and potential bimodality that can cause the EKF to diverge.
* **Severity:** **Medium**
* **Concrete Fix:** 
  1. Add a covariance threshold check on $\sigma_\psi$: if heading uncertainty exceeds a limit (e.g., $15^\circ$), freeze gyro bias estimation ($\dot{b}_g = 0$, inflate process noise) rather than allowing $b_g$ to drift on ambiguous landmark updates.
  2. Explicitly test innovation wrapping at the $\pm\pi$ branch cut in unit tests for step M3.3c.

---

### 1.4 Data Association: Premature Gating and Restriction on Landmark Initialization
* **Citation:** `master_plan.md` §4 (Step M3.4); `design.md` §5 & §6 ("Data association").
* **Quote:** 
  > *`design.md` §5:* `ZED 72° wide, up to 8.9 / 16.2 / 20.1 m... wide camera 85°... Elsewhere the colour is UNKNOWN.`
  > *`design.md` §6:* `A known colour must match the landmark's colour; an UNKNOWN colour uses the gate only. Only coloured buoy detections may start a new landmark.`
* **Why it matters:** 
  Lidar detects buoys across $360^\circ$ up to $18.75\text{ m}$. Cameras only cover forward azimuths ($72^\circ / 85^\circ$) with shorter ranges ($8.9\text{ m}$ for A-0 buoys). Under the rule *"Only coloured buoy detections may start a new landmark"*, any buoy detected by lidar abeam, astern, or beyond camera range will have colour `UNKNOWN` and can **never** be initialized as a landmark. The ASV will drive past landmarks without mapping them unless it steers directly toward them at close range.
  Furthermore, standard Nearest Neighbour (NN) data association is fragile under noise sweeps ($\sigma \times 2, 5$). Misassociating a measurement to an existing landmark corrupts the cross-covariance terms in $P$, precipitating immediate filter divergence.
* **Severity:** **High**
* **Concrete Fix:** 
  1. Modify the initialization rule: allow `UNKNOWN` detections to initialize tentative landmarks if they pass spatial gating and are confirmed across $n_{conf}$ consecutive lidar scans. Update the landmark's colour state later if camera sightings become available.
  2. In `asv_eval`, add an automated evaluation baseline using **known correspondences** (logged true buoy IDs) to distinguish pure filter estimation degradation from data association gating failures during noise sweeps.

---

### 1.5 Replay Design (D4 a) vs. Queue Latency Artifacts
* **Citation:** `master_plan.md` §1 ("How the numbers in the thesis are made"); `design.md` §3 (Rule 8) & §7.
* **Quote:** 
  > *`design.md` §7:* `Measurements wait in a buffer sorted by header.stamp for a fixed delay L (e.g. 0.2 s)... A measurement that arrives later than L is counted and reported, not applied to the wrong pose. slam_replay and slam_node use the same class...`
* **Why it matters:** 
  In live execution (`slam_node`), buffering for latency $L$ is required because ROS 2 transport induces jitter. But `slam_replay` reads directly from a recorded rosbag via `rosbag2_cpp`. If `slam_replay` enforces a live-style arrival delay buffer based on wall-clock time or artificial queue logic, it risks dropping bag messages or simulating artificial latency artifacts that do not exist in the recorded dataset.
* **Severity:** **Medium**
* **Concrete Fix:** Separate time-ordering from delay logic in `asv_core`. In `slam_replay`, read messages sequentially and sort them purely by `header.stamp` across all topics; do not drop messages based on arrival latency $L$. Reserve the drop-after-latency policy strictly for the live `slam_node`.

---

## 2. Scientific & Methodological Issues

### 2.1 Inverse Crimes in Synthetic Sensor Modeling
* **Citation:** `master_plan.md` §1 (Idea 4), §4 (Step M3.1); `design.md` §5.
* **Quote:** 
  > *`design.md` §5:* `The fake detector's errors have exactly the shape the filter assumes, so its results are optimistic (an inverse crime). M5.4 would calibrate it against the real pipeline; M5 is optional (D8), so without it the detector settings stay estimates...`
  > *`master_plan.md` §4, Step M2.7:* `buoys static (D7 a); mooring motion not simulated...`
* **Why it matters:** 
  An inverse crime occurs when synthetic data is generated by the exact mathematical model that the estimator assumes. Because M5 (real perception) is optional and static buoys are used (`<static>true</static>`), the fake detector synthesizes range and bearing by adding zero-mean Gaussian noise $(\sigma_r, \sigma_\beta)$ to ground truth, and the EKF assumes that exact Gaussian observation model. The evaluation tests whether the implementation matches its own equations, not whether EKF-SLAM is robust in a marine environment.
* **Severity:** **High**
* **Concrete Fix:** 
  Explicitly limit the scope of the thesis claims in the Introduction: state that the work investigates *filter numerical consistency under idealized parametric noise*, not real-world perception robustness. In M6.2, introduce structured synthetic stress tests: non-zero centroid bias, heavy-tailed (Student-t) noise, and synthetic mooring sway (M7.5 moved into M6.2).

---

### 2.2 Flawed Consistency Testing: Noise Sweeps with Fixed Filter Covariance
* **Citation:** `design.md` §11 ("Sweeps").
* **Quote:** 
  > *`design.md` §11:* `Filter tuning rule, stated per run: the filter's noise settings = inflation × nominal, kept fixed while the true noise changes ("mismatched"). The matched check (§10) is separate.`
* **Why it matters:** 
  In Kalman filtering, NEES is defined as $\epsilon_k = e_k^T P_k^{-1} e_k$. If true sensor noise variance is multiplied by $2\times$ or $5\times$, state errors $e_k$ scale with $\sigma$, causing $e_k e_k^T$ to scale by $4\times$ or $25\times$. If the filter's assumed covariance matrices ($R, Q$) are kept fixed at nominal, the estimated covariance $P_k$ remains unchanged. Consequently, NEES scaling by $4\times$ and $25\times$ is an elementary mathematical identity, not an empirical discovery about EKF-SLAM consistency.
* **Severity:** **High**
* **Concrete Fix:** Structure the sweep experiments into two distinct scientific investigations:
  1. **Noise Scaling under Matched Filter Covariance:** Scale true noise $\sigma$ by $1\times, 2\times, 5\times$ and set the filter's covariance matrices ($R, Q$) to match each true noise level. This genuinely tests whether non-linearities and Jacobian linearisations degrade EKF consistency at higher noise levels.
  2. **Covariance Mismatch (Tuning Sensitivity):** Fix true sensor noise at nominal ($1\times$), and scale the filter's assumed covariance $R$ by $0.2\times, 0.5\times, 1\times, 2\times, 5\times$ to evaluate sensitivity to under- and over-confidence.

---

### 2.3 Statistical Validity of ANEES on Identical Replay Trajectories (D10)
* **Citation:** `master_plan.md` §3 (Table row D10); `design.md` §10; `decision_notes.md` §D10.
* **Quote:** 
  > *`decision_notes.md` §D10:* `One caveat: all seeds replay the same recording, so errors that don't depend on the noise (the motion model's approximations, the planar lever arm) are shared by every run and don't average out. A larger M narrows the band around a possibly biased ANEES, so the matched check can get harder to pass as M grows.`
  > *`design.md` §10:* `The matched check in design.md §10 passes over M runs (D10): the ANEES is inside its 95 % band at about 95 % of the time steps (at least 90 %).`
* **Why it matters:** 
  1. **Deterministic Bias Breakdown:** The hypothesis test $\text{ANEES} \in [\chi^2(0.025)/M, \chi^2(0.975)/M]$ assumes that across Monte Carlo runs, sample errors $e_k^{(m)}$ are independent zero-mean random variables. When all $M$ runs replay the exact same trajectory, unmodeled deterministic effects (e.g., planar lever-arm tilt errors of $1.3\text{--}2.8\text{ cm}$, discretisation errors) are identical across all runs ($b_k \ne 0$). As $M \to \infty$, the confidence interval width shrinks as $1/\sqrt{M}$, while the deterministic bias $b_k^T P_k^{-1} b_k$ remains constant. ANEES is mathematically guaranteed to fail the $\chi^2$ acceptance test as $M$ increases.
  2. **Temporal Autocorrelation:** ANEES across time steps $k, k+1, \dots$ is strongly autocorrelated. Checking whether ANEES is inside the band for "at least 90% of time steps" treats correlated time-series samples as independent Bernoulli trials.
* **Severity:** **High**
* **Concrete Fix:** 
  1. In `design.md` §10, explicitly define the matched check to run on an idealized, disturbance-free trajectory (zero tilt, flat sea) where deterministic errors are negligible.
  2. For the main sweep runs, present ANEES time-history plots with bounding curves and report the time-averaged ANEES value $\bar{\epsilon} = \frac{1}{K}\sum_{k=1}^K \text{ANEES}_k$ tested against the wider two-sided bounds $\chi^2_{KMn} / (KM)$, rather than counting pointwise time-step crossings.

---

### 2.4 Lack of Map Consistency Metric
* **Citation:** `master_plan.md` §1; `design.md` §10 ("Evaluation").
* **Quote:** 
  > *`master_plan.md` §1:* `The thesis question: accuracy and consistency (NEES/NIS) of localisation and buoy mapping against ground truth under different sensor noise levels.`
  > *`design.md` §10:* `Map error: match estimated landmarks to true buoys one to one (Hungarian assignment)... Consistency (NEES): ε = eᵀP⁻¹e with e = [x, y, wrapped ψ] (n = 3).`
* **Why it matters:** 
  The core thesis question explicitly promises consistency evaluation of **both** localisation and buoy mapping. Yet Section 10 only formulates NEES for the vehicle pose ($n=3$). Map evaluation is limited to deterministic RMSE and Hungarian matching. Landmark covariance consistency is completely omitted, leaving half of the primary research question unaddressed.
* **Severity:** **High**
* **Concrete Fix:** Add a formal definition of **Landmark NEES** in `design.md` §10:
  $$\epsilon_{m, j} = (m_j - \hat{m}_j)^T P_{m_j m_j}^{-1} (m_j - \hat{m}_j) \sim \chi^2(2)$$
  computed for confirmed landmarks matched to ground truth buoys via Hungarian assignment, and define the Average Landmark NEES (ALNEES) metric across seeds.

---

## 3. Scope & Schedule Risk for a Bachelor's Thesis

### 3.1 Overall Scope vs. Single Bachelor Student Capacity
* **Citation:** `master_plan.md` §2, §4 (Milestones M0 through M7); `design.md` §12.
* **Quote:** 
  > *`master_plan.md` §2:* `The C++ code that the old docs describe was never written... No commit has ever contained a C++ source file of ours, and CMakeLists.txt says "No C++ targets yet".`
  > *`design.md` §12:* `asv_core (new), asv_msgs (new), asv_sim (exists), asv_perception (new), asv_slam (new), asv_eval (new).`
* **Why it matters:** 
  As of commit `6b8be2b` (v1.4, mid-October), zero custom C++ code exists in the repository. The plan proposes building **five new packages**, including a custom Eigen-based math library (`asv_core`), custom message generation (`asv_msgs`), a C++ guidance system (`asv_sim`), an entire C++ EKF-SLAM filter with sparse updates and Hungarian data association (`asv_slam`), a C++ `rosbag2` replay node, and a Python evaluation/sweep orchestrator (`asv_eval`). For a single bachelor student, this volume of low-level robotics C++ and ROS 2 middleware implementation presents an extreme risk of non-completion.
* **Severity:** **High**
* **Concrete Fix:** Ruthlessly cut all peripheral development. Focus exclusively on the minimum viable pipeline: M1.3 (follower) $\to$ M2.1/M2.6 (corruptor) $\to$ M3.1/M3.3/M3.4 (filter core) $\to$ M3.5/M3.6 (replay/eval) $\to$ M6 (sweeps).

---

### 3.2 Gold-Plating and Distractions to Eliminate
* **Citation:** `master_plan.md` §4 (Steps M1.7, M1.8, M5.1-M5.5, M7.1-M7.7); `design.md` §14.
* **Quote:** 
  > *`master_plan.md` §4:* `M1.7 Maneuver player... M1.8 Keyboard teleop (WASD) in C++... M5 Real perception... M7.1 robot_localization baseline... M7.6 Foxglove dashboard...`
* **Why it matters:** 
  - **M1.7 & M1.8:** Building custom open-loop maneuver players and C++ WASD terminal teleop nodes when a ground-truth waypoint follower (M1.3) already exists is unnecessary engineering overhead.
  - **M5 (Real Perception):** Lidar clustering, PCL integration (which is not even installed in Docker!), HSV colour thresholding, and camera-lidar fusion represent an entire computer vision thesis on their own. Keeping it in the document, even marked "optional", creates false expectations.
  - **`design.md` §14 (Benchmarking):** Retaining eight subsections detailing `/proc` parsing, Geekbench comparisons, and Jetson power profiles when benchmarking was formally deferred (D16) clutters the plan.
* **Severity:** **Medium**
* **Concrete Fix:** 
  1. Delete M1.7 and M1.8. Steer the boat exclusively using M1.3.
  2. Permanently remove M5 from the master plan and archive it as "Future Work".
  3. Strip M7.1, M7.4, M7.6, M7.7, and `design.md` §14 entirely from the active thesis documents.

---

### 3.3 Critical Path Bottleneck
* **Citation:** `master_plan.md` §4 (Milestones M1 through M3.6).
* **Why it matters:** 
  The critical path is strictly linear:
  $$\text{M1.3 (Path Follower)} \longrightarrow \text{M2.9 (Bag Recording)} \longrightarrow \text{M2.6 (Corruptor)} \longrightarrow \text{M3.3 (EKF Core)} \longrightarrow \text{M3.5 (Replay)} \longrightarrow \text{M6.2 (Sweeps)}$$
  Any delay in getting M1.3 and M2.9 operational directly delays the recording of bags, which blocks the filter, which blocks the sweeps. Writing and debugging the thesis text takes at least 4–6 weeks; if M3.6 is not complete by mid-semester, the student will have no data to write about.
* **Severity:** **High**
* **Concrete Fix:** Impose a hard deadline: bypass live RViz visualisation (M4.1) and live nodes entirely until offline `slam_replay` (M3.5) and evaluation (M3.6) produce final thesis figures.

---

## 4. Internal Inconsistencies, Contradictions & Ambiguities

### 4.1 Fatal Contradiction: D5 Motion Model vs. D9 Core Experiments
* **Citation:** `master_plan.md` §3 (Table rows D5 and D9); `decision_notes.md` §D5 & §D9.
* **Quote:** 
  > *`master_plan.md` §3, D5:* `decided: C... 2026-10-14: "The asv will allways be in view of the landmarks... so GNSS outages are few if any."`
  > *`master_plan.md` §3, D9:* `decided: a, 2026-10-11... GNSS modes and outages become core (cheap with replay)...`
* **Why it matters:** 
  Marcell explicitly justified choosing D5 Option C on 2026-10-14 by asserting that GNSS outages do not occur in this operational environment. Yet three days earlier (2026-10-11), Marcell decided D9 Option a, making GNSS outages a **core experiment** of the thesis. D5 C is the worst possible model during GNSS outages because GNSS velocity disappears, forcing an arbitrary hold-last-velocity heuristic. The foundational justification for D5 C directly contradicts the thesis scope established in D9.
* **Severity:** **High**
* **Concrete Fix:** Resolve the contradiction explicitly: either revoke D5 C in favor of D5 B (which models body velocity and handles outages natively), or amend D9 to remove GNSS outages from the core experimental matrix.

---

### 4.2 Ambiguity in Prediction Velocity Representation (World ENU vs. Body Frame)
* **Citation:** `master_plan.md` §7 (v1.4 Change log); `design.md` §1 & §6.
* **Quote:** 
  > *`master_plan.md` §7 (v1.4):* `the odom twist taken from the GNSS velocity rotated into the body frame.`
  > *`design.md` §6:* `inputs are the GNSS velocity (v_E, v_N, world frame)... x += v_E·dt, y += v_N·dt... Without a GNSS velocity (outage) the last value is held...`
* **Why it matters:** 
  `design.md` §6 specifies that the EKF integrates world-frame ENU velocities directly ($v_E, v_N$). But during an outage, if the filter holds $(v_E, v_N)$, the velocity vector fails to rotate as the ASV turns. Conversely, if the filter rotates the velocity into the body frame and holds forward speed $u$, the prediction equations require heading $\psi$ ($\dot{x} = u \cos\psi, \dot{y} = u \sin\psi$). But `decision_notes.md` §D5 C explicitly claims that in D5 C *"position prediction doesn't depend on heading"*. The equations in §6 and the text in `decision_notes.md` are mathematically contradictory.
* **Severity:** **High**
* **Concrete Fix:** Provide the exact mathematical equations for prediction during outages in `design.md` §6: convert GNSS velocity to body surge/sway ($u, v$) prior to integration, and use estimated heading $\psi$ to project into the world frame during outages. Update the error propagation Jacobians accordingly.

---

### 4.3 Stale Content: Benchmarking (D16) Still Embedded in Active Sections
* **Citation:** `master_plan.md` §5 ("Thesis map"); `design.md` §14.
* **Quote:** 
  > *`master_plan.md` §5 (Thesis map table):* `(left out for now, D16) | M3.7, M4.2, M5.5 | CPU time and memory of the onboard code`
  > *`master_plan.md` §4, Step M3.5:* `The report gives the wall time of one replay and of one evaluation... the number D10 needs.`
* **Why it matters:** 
  Section 3 records that D16 was decided on 2026-10-11 ("Leave out benchmarking for now"). However, the Thesis Map (§5) still includes a row assigning thesis sections to benchmarking steps, and `design.md` retains 200 lines of benchmarking protocol (§14). This creates ambiguity about whether benchmarking code must be written and tested.
* **Severity:** **Low**
* **Concrete Fix:** Remove the benchmarking row from the Thesis Map table in `master_plan.md` §5. Move `design.md` §14 to an archived document (`docs/benchmarking_archive.md`).

---

### 4.4 Topic Leakage Vulnerability in `slam_replay` under Decision D3 b
* **Citation:** `master_plan.md` §3 (Table row D3); `design.md` §1.
* **Quote:** 
  > *`master_plan.md` §3, D3:* `decided: b, 2026-10-11... Bridge keeps the normal names; noisy data goes to /asv/sensors/... Simpler, but a node left on a default name silently gets perfect data.`
  > *`design.md` §1:* `The price of D3 b: a node left on a normal name gets perfect data without any warning. Two guards: the filter's default topic names are the /asv/sensors/... ones, and the M4.1 check confirms with ros2 node info...`
* **Why it matters:** 
  Under D3 b, clean data occupies standard topics (`/asv/imu/data`, `/asv/gnss/fix`). The guard in M4.1 uses `ros2 node info` to check topic subscriptions in the live `slam_node`. However, `slam_replay` (M3.5) does not run as a live ROS graph; it reads directly from a bag file using C++ API calls (`rosbag2_cpp::SequentialReader`). A single hardcoded topic name or configuration typo in the C++ bag reader will cause `slam_replay` to feed clean Gazebo ground truth to the filter without triggering any warning, silently corrupting the thesis results.
* **Severity:** **High**
* **Concrete Fix:** In `asv_core` / `slam_replay`, insert a hardcoded programmatic assertion that checks the bag reader configuration: if any topic name requested by the filter matches `/asv/imu/data`, `/asv/gnss/fix`, or `/asv/ground_truth/*`, throw an immediate runtime exception (`std::runtime_error`) and terminate.

---

### 4.5 Unresolved Decision Deadlocks (D6, D10, D11)
* **Citation:** `master_plan.md` §3 ("Decisions"); `decision_notes.md`.
* **Quote:** 
  > *`master_plan.md` §3:* `Still open: D6, D10 and the exact content of the IMU message (D11)... D11 must be settled before M2.4 and M2.6; D6 before M3.3b; D10 before M3.6.`
* **Why it matters:** 
  M2.4 (`asv_msgs`) and M2.6 (`corruptor`) are blocked by D11. M3.3b is blocked by D6. Furthermore, D11 focuses heavily on ArduPilot accelerometer filtering, gravity removal, and attitude models. But under D5 C, the accelerometer is not used by the filter at all. Debating accelerometer noise specs (70 vs 110 µg/√Hz in `decision_notes.md` §D11) while D5 C is active wastes critical development time.
* **Severity:** **Medium**
* **Concrete Fix:** Force immediate closure of open decisions:
  - **D11:** Select **Option A (ideal/tuned)** immediately. Since D5 C does not use the accelerometer or orientation, strip roll/pitch and accelerometer complexity from the filter interface; publish standard raw gyro and accelerometer data.
  - **D6:** Select **Option a** (inflated white noise).
  - **D10:** Fix $M = 30$ runs as a standard across all sweep configurations.

---

## 5. Missing Elements (Risks, Steps, Checks)

### 5.1 Missing Filter Initialization and Alignment Protocol
* **Citation:** `master_plan.md` §4 (Step M3.3a); `design.md` §6.
* **Why it matters:** 
  The plan describes Jacobians and updates, but is completely silent on **filter initialization**:
  - How are the initial state vector $x_0$ and initial covariance matrix $P_0$ established?
  - Does the filter average the first $K$ GNSS fixes and heading azimuths?
  - What are the initial variances for position, heading, and gyro bias?
  If $P_0$ is set too small, the filter will reject early measurements and diverge; if $P_0$ is set too large, NEES will take hundreds of steps to converge, corrupting consistency metrics.
* **Severity:** **High**
* **Concrete Fix:** Add an explicit Initialization Specification to `design.md` §6: define an initialization state machine where the filter collects GNSS position and heading fixes for $1.0\text{ s}$ while stationary, sets $x_0 = [\bar{x}_{gnss}, \bar{y}_{gnss}, \bar{\psi}_{gnss}, 0]^T$, and initializes $P_0 = \text{diag}(\sigma_{pos}^2, \sigma_{pos}^2, \sigma_{heading}^2, \sigma_{b_g0}^2)$.

---

### 5.2 Missing Baseline Comparison (Ablation: SLAM vs. Dead-Reckoning)
* **Citation:** `master_plan.md` §4 (Step M7.1); `design.md` §11.
* **Why it matters:** 
  `robot_localization` was relegated to optional step M7.1. Without a baseline, the thesis cannot answer: *What is the quantifiable benefit of mapping buoys compared to standard GNSS/IMU dead-reckoning?* If GNSS is available, buoy updates may provide negligible position improvement. In outages, if buoy mapping fails, the filter degrades to dead reckoning anyway. Without a baseline comparator, the thesis cannot prove whether EKF-SLAM actually helped.
* **Severity:** **High**
* **Concrete Fix:** Move a simplified baseline mode into the core `asv_slam` package. Add a configuration parameter `enable_landmark_updates: false`. Running this mode produces a pure GNSS/IMU dead-reckoning trajectory on the exact same bag files, providing an immediate, rigorous ablation baseline for all M6 evaluation tables.

---

### 5.3 Numerical Covariance Conditioning and Divergence Guards
* **Citation:** `design.md` §6 ("Numerics").
* **Quote:** 
  > *`design.md` §6:* `Numerics: Joseph-form covariance update, covariance kept symmetric. H is sparse... written with full matrices it costs N³ instead of N²...`
* **Why it matters:** 
  While Joseph-form guarantees positive semi-definiteness in exact arithmetic, finite-precision floating-point operations in Eigen can lead to asymmetry or negative eigenvalues during large landmark updates or ill-conditioned matrix inversions ($(H P H^T + R)^{-1}$). If $P$ loses positive definiteness, Cholesky decomposition fails, producing NaNs that crash `slam_replay` in the middle of an overnight sweep.
* **Severity:** **Medium**
* **Concrete Fix:** In `asv_core`, mandate numerical conditioning guards after every measurement update:
  1. Enforce symmetry: $P = \frac{1}{2}(P + P^T)$.
  2. Perform an eigenvalue or Cholesky check: if any diagonal entry $P_{ii} \le 0$, clamp to a small numerical epsilon ($\epsilon = 10^{-8}$) and log a warning.

---

### 5.4 Data Association Failure Handling and Landmark Pruning
* **Citation:** `master_plan.md` §4 (Step M3.4); `design.md` §6.
* **Why it matters:** 
  The plan specifies how landmarks are initialized (tentative, confirmed after $n_{conf}$ sightings), but provides no mechanism for **landmark deletion or pruning** when false alarms are accepted into the map. In M6.2, detector noise and false-alarm rates $\lambda$ are swept. Without a pruning rule (e.g., decrementing a counter when a landmark is predicted within FOV but not observed), spurious landmarks will permanently pollute the state vector, corrupt the map RMSE, and artificially inflate the EKF computation time.
* **Severity:** **Medium**
* **Concrete Fix:** Add a landmark health/pruning policy to M3.4: maintain an observation counter and an expected-visibility counter. If a confirmed landmark is expected in FOV but missed $M_{miss}$ consecutive times, remove it from the state vector and covariance matrix.

---

## 6. Top-5 Most Important Changes

1. **Revert Motion Model D5 from C to B (CTRV with Body Velocities):**
   * *Action:* Eliminate world-frame GNSS velocity integration from the process model. Propagate pose using body-frame velocities $(u, v)$ and turn rate $\omega$. Treat GNSS Doppler velocity as a measurement update.
   * *Impact:* Eliminates fatal cross-correlation between process and measurement noise, prevents unphysical straight-line drift during turns in GNSS outages, and restores physical validity to dead reckoning.

2. **Reject D6 Option b; Formulate D6 Option a as an Intentional Assumption Test:**
   * *Action:* Drop the 2-state unobservable GNSS slow-bias estimator from the EKF state. Treat GNSS noise as white with inflated covariance in the filter.
   * *Impact:* Avoids an unidentifiable state formulation that cannot separate vehicle position from GNSS bias, saves 2–3 days of complex Jacobian implementation, and frames coloured noise cleanly as a thesis investigation of Assumption A3.

3. **Restructure M6 Sweeps into Matched vs. Mismatched Experiments:**
   * *Action:* Separate noise sweeps into (a) true noise swept with filter covariance correctly updated (matched tuning), and (b) true noise fixed at nominal while filter covariance is scaled (mismatched tuning).
   * *Impact:* Prevents a scientifically trivial result where NEES scaling simply reflects fixed filter variance, transforming the experiments into a valid investigation of non-linear filter consistency and tuning sensitivity.

4. **Define Landmark Consistency (ALNEES) and Add Dead-Reckoning Baseline:**
   * *Action:* Formulate the Landmark NEES metric for matched landmarks in `design.md` §10. Add a parameter switch in `asv_slam` to disable landmark updates for a baseline comparison.
   * *Impact:* Directly fulfills the primary thesis question (consistency of buoy mapping) and provides the essential ablation baseline needed to scientifically prove that SLAM improves localization over dead reckoning.

5. **Prune Scope and Eliminate Non-Essential Modules (M1.7, M1.8, M5, M7, Benchmarking):**
   * *Action:* Permanently cut the maneuver player (M1.7), teleop (M1.8), real perception pipeline (M5), `robot_localization` / Foxglove (M7), and benchmarking (§14). Drive the boat solely via the M1.3 waypoint follower.
   * *Impact:* Protects the bachelor thesis schedule from critical-path collapse, allowing the student to move immediately from M1.3 to the EKF-SLAM core and experimental sweeps.
