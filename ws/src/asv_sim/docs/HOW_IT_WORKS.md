# asvgen: how the parametric ASV model generator works

`asvgen` turns a short YAML description of the boat (`config/asv.yaml`) into a
complete Gazebo simulation model plus everything around it: the ROS 2 bridge
configuration, the static TF frames, and a design report with physical and
sensor-coverage metrics. This document explains every stage of the
pipeline, the equations behind it, its assumptions, and how its correctness
is checked.

Version: asvgen 2.0 (Gazebo Harmonic, VRX, ROS 2 Jazzy).

---

## 1. Why a generator

A simulated boat is described by many numbers that must stay mutually
consistent:

* **Geometry:** the visual shapes, collision shapes and buoyancy shapes of the hulls.
* **Mass properties:** total mass, centre of gravity (CoG) and inertia tensor,
  which depend on where every component sits.
* **Buoyancy model:** hull length, radius and the positions of the sampling points.
* **Sensors:** the pose of each sensor in the model, the identical pose in the TF
  tree used by ROS, and the topic names in the ROS-Gazebo bridge.

Editing these by hand is error-prone. Move the battery and the CoG and inertia
change; move a lidar and both its SDF pose and its TF frame must change. A
mismatch produces no error message, only wrong data: a boat that floats
crooked, or a point cloud drawn in the wrong place.

The generator makes the YAML file the **single source of truth**. Every
derived quantity is computed from it, so inconsistencies cannot occur. Because
generating a model takes a fraction of a second, design variants (sensor
positions, battery position) can be produced and compared systematically.
The report quantifies each variant, and a hash of the configuration in every
output makes each result traceable to the exact input that produced it.

## 2. Inputs, outputs and pipeline

```
config/asv.yaml  ─┐
--set overrides  ─┴─► 1 config ─► 2 sensors ─► 3 boat assembly ─► 6 SDF ──────► models/<name>/model.sdf
                       validate    expand        ├ 4 mass                         models/<name>/model.config
                       fingerprint               ├ 5 hydrostatics
                                                 └ actuation, drag   ─► outputs ─► config/<name>_bridge.yaml
                                                                                   config/<name>_frames.yaml
                                                       7 analysis ────────────► config/<name>_report.txt / .json
                                                                   └─ plot ────► config/<name>_layout.png
```

| Module | Stage | Responsibility |
|---|---|---|
| `config.py` | 1 | read YAML, apply `--set` overrides, validate against a schema, fingerprint |
| `sensors.py` | 2 | expand each sensor entry into SDF sensors, bridge topics, TF frames, visuals, masses |
| `boat.py` | 3 | hull, deck, components, sensor mounts, thruster pods, plugins |
| `mass.py` | 4 | combine parts into mass, CoG and inertia tensor |
| `hydrostatics.py` | 5 | draft, centre of buoyancy, metacentric heights, heel/trim, natural periods |
| `sdf_model.py`, `xml_utils.py` | 6 | serialise the assembled boat as SDF (an XML element tree) |
| `analysis.py` | 7 | RoboBoat rule checks, lidar/camera/IMU/GNSS coverage metrics |
| `outputs.py`, `plot.py` | | write files, draw the layout figure |
| `cli.py`, `make_asv.py` | | command line interface |

## 3. Conventions

* **Units:** SI throughout (m, kg, N, s, rad). Degrees appear only in the report.
* **Boat frame `base_link`:** x forward, y left, z up (ROS REP-103). The origin
  is midway between the hulls, at the height of the hull axes, at mid hull length.
* **World frame:** ENU (east, north, up). The water surface at rest is z = 0.
* **Orientations** are roll-pitch-yaw `rpy = (φ, θ, ψ)` with

  $$R(\varphi,\theta,\psi) = R_z(\psi)\,R_y(\theta)\,R_x(\varphi)$$

  i.e. rotations about the fixed parent axes in the order x, y, z (the SDF and
  ROS convention).
* **A pose** `(xyz, rpy)` of a child frame maps a point from child to parent:

  $$\mathbf p_\text{parent} = \mathbf t + R\,\mathbf p_\text{child}, \qquad
    \mathbf p_\text{child} = R^\top (\mathbf p_\text{parent} - \mathbf t)$$

  (`geometry.to_parent` / `geometry.to_child`).
* **Upside down:** a frame is upside down if its z axis points downwards, i.e.
  $R_{33} = \cos\varphi\cos\theta < 0$.

## 4. Stage 1: configuration (`config.py`)

### 4.1 Structure of the YAML

The file has two parts:

* **Fixed design**, which does not change between studies:
  * `hull`: length, radius, spacing, mass, and buoyancy sample points per hull
  * `deck`
  * `fixed_masses` (e.g. the electronics box)
  * `thrusters`: position, depth, mass, thrust limits, steering limit and rate
  * `hydrodynamics`: drag coefficients
* **Study parameters**, which do change:
  * `battery`: mass, size, position
  * `sensors`: one entry per physical sensor
  * `sensor_specs`: datasheet values, referenced by name

A sensor entry separates **where** a sensor is (`xyz`, `rpy`, optional
`mount`) from **what** it is (`spec`). Moving a sensor and swapping it for
another model are therefore independent one-line changes, and several sensors
can share one spec.

### 4.2 Validation

Before anything is built, the configuration is checked against a schema:

* required sections and fields
* expected types (`number`, `integer`, `bool`, `vec2`, `vec3`)
* known sensor types
* unique sensor names (they become topic and frame names)
* the referenced spec exists and is used by one sensor type only
* type-specific spec fields (e.g. a stereo camera needs `baseline`; with
  `depth: true` it also needs the depth fields)

All errors are collected and reported together with their location, for example
`sensors[1] (zed).xyz: expected vec3, got [0.4, 0]`. The program then exits
with code 2. Unknown keys produce **warnings** only: they are probably typos
but do no harm, and they appear at the top of the report.

### 4.3 Overrides

`--set key.path=value` changes one value before validation, without editing
the file. Sensors are addressed by name. The value is parsed as YAML, so
numbers, lists and booleans work:

```
--set battery.xyz=[0.1,0,0.225]
--set sensors.lidar.xyz=[0.72,0,0.45]
--set sensor_specs.zed2i_4mm.depth=false
```

`--name` sets the model name, so variants are written side by side
(`models/<name>/`, `config/<name>_*`) instead of overwriting each other.

### 4.4 Fingerprint

After the overrides, the effective configuration is serialised with sorted
keys and hashed (SHA-256, first 12 hex characters). The fingerprint is written
into the SDF, the bridge and frames files and the report. Any result can thus
be traced back to the exact configuration that produced it, and two identical
configurations always give the same fingerprint.

## 5. Stage 2: sensors (`sensors.py`)

Each sensor entry is expanded into a `SensorPart` holding:

* **`sdf`**: the `<sensor>` elements to place in `base_link`
* **`bridge`**: the topics the ROS-Gazebo bridge must translate
* **`frames`**: the static TF frames
* **`visuals`**: the body of the sensor, plus its height for the mount
* **`masses`**: entries for the mass budget

The same pose feeds the SDF sensor and its TF frame, so they cannot disagree.
Names are derived from the sensor name: topics `/<namespace>/<name>/...`,
frames `<name>_link` / `<name>_optical`.

| type | Gazebo sensor(s) | ROS topics | TF frames |
|---|---|---|---|
| `lidar` | `gpu_lidar` | `/asv/<n>/points` | `<n>_link` |
| `camera` | `camera` | `/asv/<n>/image_raw`, `/asv/<n>/camera_info` | `<n>_link`, `<n>_optical` |
| `stereo_camera` | 2 × `camera` (+ `depth_camera`) | `/asv/<n>/{left,right}/image_raw` + `camera_info` (+ `/asv/<n>/depth/image`, `/points`) | `<n>_link`, `<n>_{left,right}_optical` |
| `imu` | `imu` | `/asv/<n>/data` | `<n>_link` |
| `gnss` | `navsat` | `/asv/<n>/fix` | `<n>_link` |
| `gnss_dual` | 2 × `navsat` | `/asv/<n>/fix`, `/asv/<n>/secondary/fix` | `<n>_{primary,secondary}_link` |

### 5.1 Lidar

The lidar is modelled as a regular grid: `channels` beams evenly spread over
the vertical field of view `vfov = [lo, hi]`, each sampled `h_samples` times
over 360°.

* **No duplicated seam.** The horizontal range runs from −π to $\pi - 2\pi/N$,
  so the first and last columns do not coincide.
* **Water is invisible.** `visibility_mask = 7` hides the VRX water surface,
  whose visual has visibility flag 8. Real lidars get almost no returns from water.
* **Mounting angle is just the pose.** An upside-down mount is `rpy = (π, 0, 0)`,
  which maps the sensor's −7°…+52° field of view to +7°…−52° in the boat frame.

**Approximation.** The Livox Mid-360 uses a non-repetitive scan pattern that
covers its field of view progressively over time. A grid of 40 × 500 points
reproduces the point count per scan (20,000, i.e. 200,000 points/s at 10 Hz)
and the angular extent, but not the pattern. Per-scan coverage is therefore
more uniform in the simulation, and any beams-on-target figure is a grid
artefact at the level of single beams.

### 5.2 Cameras and stereo

A camera is a pinhole model with horizontal field of view `hfov` and
resolution w × h.

* **Vertical field of view** follows from square pixels:
  $\mathrm{vfov} = 2\arctan\!\big(\tfrac{h}{2}/f\big)$ with
  $f = \tfrac{w}{2}/\tan(\mathrm{hfov}/2)$ the focal length in pixels.
* **Camera info** is published next to the image (`.../image_raw` → `.../camera_info`).
* **Optical frame.** Images are labelled with the camera's optical frame
  (z forward, x right, y down), which is the camera body frame rotated by
  rpy = (−π/2, 0, −π/2). The generator writes this extra frame.

A **stereo camera** is two cameras `baseline` apart along the body's y axis:
the left one at +b/2, the right one at −b/2. The positions are computed with
`to_parent`, so they stay correct for any mounting angle.

With `depth: true` a `depth_camera` is added at the left camera's pose. Its
near and far clip planes equal the datasheet depth range. This depth is
**ideal** (exact geometry, no stereo matching error), a stand-in for the depth
the vendor SDK computes. Realistic stereo errors come from processing the
left/right images yourself; section 11.4 estimates their size.

### 5.3 IMU

Datasheets give **noise densities** $N_g$ (rad/s/√Hz) and $N_a$ (m/s²/√Hz).
Gazebo needs the standard deviation **per sample**, which is

$$\sigma = N\sqrt{f_s}$$

with $f_s$ the sample rate (`imu_white_noise`). For the ICM-45686 at 200 Hz this
gives 0.054 °/s and 0.99 mg.

Two bias terms are added:

* **Constant bias** (`gyro_bias`, `accel_bias`): drawn once per run from
  $\mathcal N(0, \sigma^2)$, modelling the residual after calibration.
* **Drifting bias** (`*_bias_drift`, `bias_drift_time` τ): a first-order
  Gauss-Markov process. Gazebo updates it every sample as
  $b_{k+1} = \phi\,b_k + w_k$, with $\phi = e^{-\Delta t/\tau}$ and
  $w_k \sim \mathcal N\!\big(0, \sigma_b^2 \tfrac{\tau}{2}(1-e^{-2\Delta t/\tau})\big)$.
  Its steady-state standard deviation is $\sigma_b\sqrt{\tau/2}$. The YAML gives
  that steady-state value, and the generator converts it:
  $\sigma_b = \sigma_\text{drift}/\sqrt{\tau/2}$.

The orientation output is referenced to ENU, so yaw 0 means east, like a real
AHRS. Without that setting Gazebo reports orientation relative to the spawn
orientation.

### 5.4 GNSS

A `navsat` sensor turns its world position into latitude, longitude and
altitude using the world's `<spherical_coordinates>`, adding Gaussian
horizontal and vertical noise.

The dual-antenna receiver (`gnss_dual`) is simulated as two antennas: the
**primary** gives the position, and the **primary → secondary** baseline
defines the heading. The heading is deliberately *not* derived from the two
simulated fixes. Gazebo's noise is independent per antenna, so the heading
error would be $\sqrt2\,\sigma_h/B$ (0.86° for B = 0.75 m), while the real
receiver solves the baseline jointly from carrier-phase measurements and
reaches 0.1° at B = 1 m. The receiver's heading is instead emulated from ground
truth plus datasheet noise by a separate ROS node (section 12).

## 6. Stage 3: assembly (`boat.py`)

The model consists of one rigid body and two moving pods:

```
base_link                 hulls, deck, electronics, battery, every sensor (+ mounts)
├─ left_steer_joint  (revolute, z)  → left_engine_link
│  └─ left_prop_joint (revolute, x) → left_prop_link
└─ right_steer_joint (revolute, z)  → right_engine_link
   └─ right_prop_joint (revolute, x) → right_prop_link
```

**Hulls** are drawn as cylinders with hemispherical ends. They collide and
float as plain cylinders of the full length L, because that is the shape VRX's
buoyancy model assumes.

**Sensor mounts.** Every sensor stands on a post of radius 15 mm, or hangs
from one if it is upside down. Where the post stands:

1. at the sensor's `mount: [x, y]`, if given (it must be on the deck or a hull);
2. otherwise straight below the sensor, if that point is on the deck or on a hull;
3. otherwise at the nearest point of the deck, with a horizontal arm out to the
   sensor, rotated to point at it.

A post on a hull starts at the hull's top surface. For hulls drawn as cylinders
with hemispherical ends (radius r, axis at $y_h$, half length $L/2$), that
height is

$$z_\text{top}(x,y) = \sqrt{r^2 - \Delta x^2 - \Delta y^2},\quad
\Delta x = \max\big(0, |x| - (L/2 - r)\big),\ \Delta y = |y - y_h|$$

This is how the GNSS antennas stand on the hull tips, outside the deck.

Posts and arms are visuals, and **simulated lidars and cameras see visuals**,
so the mounts occlude the sensors' view just as on the real boat. That is why
their position is a design parameter. The mounts are not counted in the mass
budget (a few hundred grams).

**Solids.** Every hull, the deck, every component box, sensor body, post and
arm is also registered as an axis-aligned box (`Solid`) with an owner and a
kind (`structure`, `component`, `sensor`, `mount`). The solids serve two
analyses: the overlap check (section 11.1) and the line-of-sight tests of the
lidar near field (section 11.3). Boxes enclose the real shapes, so both
analyses are slightly conservative.

Two point lists are also collected:

* **Occluders:** points on deck corners, hull ends, sensor bodies and posts,
  each tagged with its owner, used to name the parts of the boat a sensor sees.
* **Extents:** points bounding the boat, used for the size check.

## 7. Stage 4: mass properties (`mass.py`)

Every physical part is a `MassItem` with mass $m_i$, position $\mathbf r_i$ (its
own CoG) and its own inertia tensor $I_i$ about that point, aligned with the
boat axes. Hulls are solid cylinders about x, the deck and boxes are solid
cuboids, and sensors are point masses. The rigid body is

$$M = \sum_i m_i, \qquad \mathbf c = \frac{1}{M}\sum_i m_i\,\mathbf r_i$$

$$I = \sum_i \Big( I_i + m_i\big(\lVert\mathbf d_i\rVert^2 E - \mathbf d_i\mathbf d_i^\top\big) \Big),
\qquad \mathbf d_i = \mathbf r_i - \mathbf c$$

The second term is the parallel-axis (Huygens-Steiner) theorem in tensor form.
It also produces the products of inertia when parts sit off-centre. SDF's
`ixy`, `ixz`, `iyz` are the off-diagonal tensor elements, e.g.
$I_{xy} = -\sum m\,d_x d_y$.

Two bodies are computed:

* **`base`**, everything rigidly attached, written as `base_link`'s inertial.
* **`total`**, base plus pods, used for hydrostatics and the report.

Each pod's mass is split 85 % engine, 15 % propeller, and the pods keep their
own small inertials because they move relative to the hull.

## 8. Stage 5: hydrostatics (`hydrostatics.py`)

### 8.1 What VRX does

The buoyancy in the simulation comes from VRX's `Surface` plugin. For each of
N sample points $\mathbf p_i$ per hull, at every time step it:

1. finds the water surface elevation $\eta(x_i, y_i, t)$ from the wave field,
2. computes the immersion $\Delta z_i = \mathrm{clamp}(\eta - z_i,\ 0,\ r)$,
3. applies a vertical force at the point:

$$F_i = \rho\, g\, A_\text{seg}(r, \Delta z_i)\,\frac{L}{N}$$

$A_\text{seg}(r,h)$ is the area of a circular segment of height h. The point
must therefore be the **keel** (bottom) of its hull segment: the immersion is
measured from it. The cap at r means a hull can provide at most the buoyancy of
its lower half.

The generator places the N points of each hull at the keel ($z = -r$), at the
centres of N equal segments: $x_i = -L/2 + (i + \tfrac12) L/N$. (Earlier
hand-written versions put them at the hull axis, which made the boat float
about one hull radius too deep.)

### 8.2 Equilibrium (draft)

With the circular segment written via its angle,

$$A_\text{seg}(d) = \frac{r^2}{2}(\vartheta - \sin\vartheta), \qquad
\vartheta = 2\arccos\frac{r-d}{r}$$

Archimedes' principle for two hulls gives the draft d:

$$2\,\rho\,L\,A_\text{seg}(d) = M$$

$A_\text{seg}$ is monotonic in d, so the equation is solved by bisection on
[0, r] (60 iterations, far below any physical tolerance). If even
half-submerged hulls cannot carry the mass ($M \ge \rho L\pi r^2$), the
generator stops with an error, because VRX could not float the boat either.

Results:

* **Waterline:** $z_w = -r + d$ in the boat frame.
* **Spawn height** (puts the waterline at world z = 0): $-z_w$ plus 2 cm, so
  the boat is not spawned intersecting the water.
* **Buoyancy reserve:** $1 - A_\text{seg}(d)/(\pi r^2/2)$.

### 8.3 Initial stability

* **Centre of buoyancy** (centroid of the segments):
  $z_B = -\dfrac{4 r \sin^3(\vartheta/2)}{3(\vartheta - \sin\vartheta)}$
* **Waterline width** of one hull: $b = 2\sqrt{r^2 - (r-d)^2}$
* **Second moments of the waterplane** about the roll and pitch axes
  (s = half the hull spacing):

$$I_T = 2\Big(\frac{L b^3}{12} + L\,b\,s^2\Big), \qquad I_L = 2\,\frac{b L^3}{12}$$

* **Metacentric radius and height** ($V = M/\rho$):

$$BM = \frac{I}{V}, \qquad GM = z_B + BM - z_G$$

GM > 0 means a small inclination creates a righting moment. The $s^2$ term
dominates $I_T$, which is why catamarans are very stiff in roll. The current
design has GM ≈ 3.3 m in roll.

Small-angle estimates:

* **Heel:** $\arctan(y_G/GM_T)$; **trim:** $\arctan(-(x_G - x_B)/GM_L)$ with $x_B = 0$.
* **Natural periods:** $T = 2\pi\sqrt{I_G/(M g\,GM)}$, without added mass,
  so they are lower bounds.

### 8.4 Sampling error of the simulated pitch stiffness

The plugin replaces the continuous hull by N discrete points. Each represents a
segment of length L/N at its centre, so the pitch stiffness it produces is
proportional to

$$\sum_{i=0}^{N-1}\frac{L}{N}x_i^2 = \frac{L^3}{12}\Big(1 - \frac{1}{N^2}\Big)$$

instead of the exact $L^3/12$. With N = 4 the simulated boat has 93.75 % of
the analytical pitch stiffness; with N = 2 only 75 %. The report states this
ratio. Roll is unaffected, because roll stiffness comes from the hull spacing,
not the point distribution.

## 9. Actuation and drag

### 9.1 Steerable thrusters

Each pod is driven by two Gazebo systems:

* **`JointPositionController`** on the steering joint, in velocity-command mode.
  Each step it commands the joint velocity $-e/\Delta t$, where e is the angle
  error, clipped to ±`steer_rate`. The pod therefore turns like a rate-limited
  servo towards the commanded angle (topic `/asv/thrusters/<side>/pos`,
  radians), within ±`steer_limit`.
* **`Thruster`** on the propeller joint, in force mode. The commanded thrust F
  (topic `/asv/thrusters/<side>/thrust`, newtons, clamped to
  [`min_thrust`, `max_thrust`]) is applied as a force along the propeller joint
  axis, expressed in the world frame. Because that axis turns with the pod,
  steering redirects the thrust.

  With `velocity_control` the propeller is only spun kinematically, at
  $\omega = \operatorname{sign}(F)\sqrt{|F|/(\rho K_T D^4)}$ (Fossen's
  thrust-rpm relation), and no reaction torque is applied.

**Sign convention:** a positive steering angle turns the pod counter-clockwise
seen from above. The thrust then points forward-left at the stern, so the
**boat turns right**.

### 9.2 Water drag

VRX's `SimpleHydrodynamics` applies linear plus quadratic damping per degree of
freedom (Fossen's model), e.g. in surge

$$X = -(X_u + X_{uu}|u|)\,u$$

and likewise for sway (Y), heave (Z), roll (K), pitch (M) and yaw (N).
Added-mass terms are set to zero. The coefficients are taken from VRX's
RoboBoat-sized example boat and are **uncalibrated**. Speeds and turning
behaviour are only indicative until they are identified on the real boat.

### 9.3 Ground truth

An `OdometryPublisher` publishes the true pose and twist on
`/asv/ground_truth/odom`. It publishes a topic only, not TF, so it never
conflicts with the localisation estimate being evaluated.

## 10. Stage 6: SDF serialisation (`sdf_model.py`, `xml_utils.py`)

The model is built as an XML element tree (`E(tag, children, attributes)`,
`V(tag, value)`) rather than by pasting text. That guarantees well-formed XML
with correct escaping, and keeps the structure visible in the code. Numbers are
written with 6 significant digits. The file begins with comments stating the
generator version, the configuration fingerprint, the total mass, draft and
spawn height.

## 11. Stage 7: analysis (`analysis.py`)

All metrics are computed from the same data that produced the SDF, so they
describe exactly the generated model. They are written as text (`_report.txt`)
and as nested JSON (`_report.json`) for scripted comparison of variants.

### 11.1 Layout

All solids except the hull and deck structure are tested pairwise for
interpenetration: two boxes conflict if they overlap by more than 2 mm along all
three axes. A sensor's own body, post and arm are not tested against each
other, since they touch by construction.

Every component box must also sit on the deck: its bottom may not be below
the deck top, and its footprint must lie inside the deck. Conflicts are
reported with their overlap in cm. They do not stop generation (some
arrangements may be intentional), but the report flags them prominently.

### 11.2 RoboBoat 2026 rules

* **Size:** the bounding box of all extents must fit 6 × 3 × 3 ft.
* **Weight score:**

  | weight w (lbs) | points |
  |---|---|
  | w ≤ 70 | 80 + (70 − w) |
  | 70 < w ≤ 110 | 2(110 − w) |
  | 110 < w ≤ 140 | −250 − 5(w − 110) |
  | w > 140 | disqualified |

* **Thrust-to-weight score:** 100 · (total thrust in lbf) / (weight in lbs).

### 11.3 Lidar coverage

The beams of the column pointing straight ahead are expressed in the boat
frame. With $\mathbf f_s = R^\top \hat{\mathbf x}$ the boat's forward
direction in sensor coordinates and $\alpha = \operatorname{atan2}(f_{s,y}, f_{s,x})$,
each beam's elevation is

$$\varepsilon_k = \operatorname{elev}\big(R\,\mathbf u(\alpha, \beta_k)\big)$$

where $\beta_k$ are the beam angles in the sensor frame.

* **Beams on a target.** A target of height H above water at horizontal
  distance d is hit by beam k if $0 \le h + d\tan\varepsilon_k \le H$, where h
  is the lidar's height above the waterline. The report counts these beams for
  the three course buoys at 1–40 m.
* **Reach.** The largest distance at which a target still receives ≥ 2 beams
  (`MIN_BEAMS`), searched in 0.25 m steps.
* **Water ring.** The distance at which the lowest beam meets the water:
  $h/\tan(-\varepsilon_\text{min})$.
* **Near field (line of sight).** The water ring above ignores the boat.
  What the lidar really sees of the water around the boat is computed on a
  raster: water points every 0.1 m, out to 3 m beyond the hull outline
  (points under the boat are skipped). A water point P is **visible** if:
  * its distance lies within the lidar's range limits;
  * its direction, expressed in the lidar frame ($R^\top(P - o)$), lies inside
    the vertical field of view. The non-repetitive Mid-360 pattern fills its
    whole field over time, so the field of view rather than the discrete grid
    of beams is used;
  * the segment from the lidar to P is not blocked by any solid of the boat
    (slab test: the parameter intervals where the ray lies between each pair of
    box faces must overlap).

  From the raster, measured **from the hull outline** so that placements can be
  compared fairly:
  * **Blind gaps:** the distance from the bow, the stern and the sides to the
    first visible water, worst case across the boat's width or length.
  * **Front-lane coverage:** the share of visible water in the lane ahead of
    the boat (the boat's width + 0.5 m on each side, 3 m long).
  * **All-around coverage:** the share of visible water within 2 m of the hulls.

  Separately, beams are traced (all channels, every 5° of azimuth) against the
  solids; the share that ends on the boat is the **self-hit share**, the part
  of every scan wasted on self-returns. The figure shows the raster: light
  blue = visible water, grey = blind.

  Two effects dominate the result:
  * **The blind cone.** An upside-down Mid-360 sees nothing steeper than 52°
    below horizontal, so directly beneath it there is a blind cone of radius
    $h/\tan 52°$ (≈ 0.5 m at h = 0.68 m). Over the deck the cone is harmless;
    at the bow it becomes blind water ahead (tested in `test_blind_cone_at_the_bow`).
  * **Shadows.** Every object between the lidar and the water casts a shadow.
    Objects close to the lidar's height cast long ones, because they block the
    nearly horizontal beams that reach far: the GNSS antenna posts produce
    narrow stripes several metres long.
* **Self-view.** Each occluder point (not belonging to this sensor) is
  transformed into the sensor frame. It is reported if it lies inside the
  vertical field of view and the range limits. Points of the own boat in the
  point cloud must be cropped before mapping.

### 11.4 Camera coverage

With focal length $f = \tfrac{w}{2}/\tan(\mathrm{hfov}/2)$ pixels:

* **Target width** in pixels at distance d: $f\,W/d$.
* **Classification range:** the distance up to which a target is at least 20 px
  wide (`MIN_PIXELS`, an assumption): $f\,W/20$.
* **Near edge of visible water:** the bottom image edge ray has elevation
  $\varepsilon_b$; if negative, water is visible from $h_c/\tan(-\varepsilon_b)$.
* **Horizon row** (pinhole, rows counted downward): $v_h = h/2 + f\tan\varepsilon_0$,
  where $\varepsilon_0$ is the optical axis elevation. If it falls outside 0…h,
  the report says whether the horizon is above or below the image.
* **Self-view:** occluder points inside the image frustum.

For stereo cameras the depth uncertainty follows from the disparity
$\delta = f B / z$. A matching error $\sigma_\delta$ (assumed 0.25 px) gives

$$\sigma_z = \frac{z^2}{f B}\,\sigma_\delta$$

The error grows with the square of the distance. The report also gives the
distance up to which $\sigma_z/z < 10\,\%$: $z = 0.1\,fB/\sigma_\delta$.

### 11.5 IMU and GNSS

* **IMU:** per-sample noise (section 5.3). Also the lever arm from the CoG and
  the centripetal acceleration it adds in a 0.5 rad/s turn ($\omega^2 r$), a
  reason to mount the IMU near the CoG.
* **Dual GNSS:** baseline length B and its direction relative to the bow (the
  heading offset the receiver must be configured with). Heading accuracy is
  scaled from the datasheet value at 1 m as $\sigma_\psi \approx \sigma_{1\text{m}}/B$,
  compared with the independent-fix value $\sqrt2\,\sigma_h/B$. The primary
  antenna's lever arm is the offset the localisation filter needs.

## 12. Verification (`tests/test_asvgen.py`)

`python3 -m unittest discover -s tests -v` runs 28 tests:

| Area | What is checked |
|---|---|
| Geometry | rotation matrices orthonormal; yaw 90° maps x→y; `to_child` inverts `to_parent`; upside-down detection |
| Mass | CoG and inertia of point masses; sign of the product of inertia; box inertia formula |
| Hydrostatics | segment area at half/full immersion; segment centroid $4r/3\pi$; Archimedes balance at the solved draft; overloaded boats refused |
| Config | the shipped config is valid; missing fields, wrong types, unknown sensor type, duplicate names and missing spec fields are reported; unknown keys only warn; overrides by sensor name |
| Layout | shipped layout has no overlaps; an overlap and a box sunk into the deck are detected; antenna posts start on the hull surface; ray-box slab test; near-field results are consistent; the blind gap of a lidar at the bow equals its blind-cone radius |
| Pipeline | SDF contains every sensor, 4 joints, 2 buoyancy plugins; mass bookkeeping; unique topics and frames; IMU noise conversion; upside-down lidar looks down to −52°; camera horizon row formula |
| **Consistency with VRX** | VRX's `Surface` force law is re-implemented and applied to the *generated SDF*; at the predicted equilibrium height the total buoyancy equals the weight to 10⁻⁶ |

The last test closes the loop between the analytical hydrostatics and the
simulator's buoyancy model.

## 13. Using it for design studies

```bash
python3 tools/make_asv.py                       # config/asv.yaml -> models/asv, report
python3 tools/make_asv.py --check               # validate + report only, writes nothing
python3 tools/make_asv.py --plot                # + config/asv_layout.png (needs matplotlib)
python3 tools/make_asv.py --check --set battery.xyz=[0.1,0,0.225]
```

A parameter sweep, here the lidar height:

```bash
for z in 0.40 0.48 0.56 0.64; do
  python3 tools/make_asv.py --quiet --name asv_lidar_z$z --set sensors.lidar.xyz=[0.72,0,$z]
done
python3 - <<'EOF'
import glob, json
for f in sorted(glob.glob('config/asv_lidar_z*_report.json')):
    m = json.load(open(f))
    print(m['name'], m['sensors']['lidar']['max_distance_2_beams_m'], m['roboboat']['size_m'])
EOF
```

Each variant gets its own model folder, so it can also be run in Gazebo for
the same course and the SLAM error compared against ground truth.

### 13.1 Lidar placement study (`tools/study_lidar_placement.py`)

The study sweeps the lidar position (x = 0 … 0.85 m, z = 0.45 … 0.62 m), its
mount (post straight below, or beside the cameras with an arm) and the GNSS
antenna height: 540 variants, about a minute. It writes
`config/lidar_study.csv` and heat maps (`config/lidar_study.png`).

Hard constraints reject a placement if the boat no longer fits the RoboBoat
box, if anything overlaps, or if any part of the lidar appears in a camera
image. The remaining placements are ranked by

$$\text{score} = g_\text{ahead} + 3\,(1 - c_\text{lane}) + 1.5\,(1 - c_{2\text{m}}) + s_\text{self}$$

The weights are chosen so that losing 10 % of the lane ahead costs as much as a
0.3 m longer blind gap at the bow, putting the front first.

Result for the current boat:

| Lidar at | Gap ahead | Gap behind | Lane ahead seen | Within 2 m seen | Self-hits | Score |
|---|---|---|---|---|---|---|
| deck centre (x = 0) | 0.30 m | 0.30 m | 93 % | 95 % | 21 % | 0.80 |
| **x = 0.25, z = 0.62, side post** | **0.20 m** | **0.40 m** | **96 %** | **94 %** | **19 %** | **0.59** |
| x = 0.60 | 0.30 m | 0.90 m | 95 % | 92 % | 21 % | 0.79 |
| bow (x = 0.85) | 0.50 m | 1.50 m | 94 % | 89 % | 18 % | 1.04 |

Slightly forward of centre is optimal:
* **Front:** the blind cone still falls on the deck, and the lidar looks over
  the front cameras at a steeper angle, which shortens their shadows.
* **Bow:** at the bow, the blind cone lands on the water ahead.
* **Height:** higher is better throughout, up to the height limit.
* **Antenna height** changes the score by less than 2 %, so the antennas stay
  low and stiff, which is better for the heading accuracy.

## 14. Assumptions and limitations

* **Hulls** are cylinders (VRX `Surface` model); real hull shapes are only
  approximated, and hulls deeper than half-submerged cannot be modelled.
* **Hydrostatics** are small-angle (initial stability); periods exclude added
  mass and damping.
* **Drag coefficients** are uncalibrated; no added mass; no wind or current
  forces on the boat (VRX's wind plugin is not used).
* **Thrusters** apply ideal force along the propeller axis; no thrust
  dynamics, ventilation or interaction between propellers and hulls.
* **Lidar** scan pattern approximated by a regular grid; reflectivity and
  dropouts are not modelled; water is invisible to it.
* **Cameras:** pinhole without lens distortion, rolling shutter, motion blur,
  exposure or polariser effects. Stereo depth (if enabled) is ideal.
* **GNSS:** constant noise level (RTK fixed or not, chosen by the spec); no
  multipath, outages or RTK convergence; dual-antenna heading emulated outside
  the SDF.
* **Mass budget** excludes cables and mounts; some masses are estimates
  (marked `[EST]` in the YAML).
* **Line of sight** uses axis-aligned boxes around hulls, components, sensors
  and mounts (sensor bodies with their real, rotated dimensions). The water
  visibility assumes the lidar's pattern fills its field of view (true for the
  Mid-360 over time, not within one scan). Camera footprints in the figure
  ignore occlusion by the boat.
* **Coverage metrics** use the geometric thresholds `MIN_BEAMS` = 2,
  `MIN_PIXELS` = 20, and 0.25 px stereo matching accuracy; these are assumptions
  that should be stated with any result that uses them.

## 15. Extending

To add a sensor type, for example a radar:

1. In `config.py`, add its required spec fields to `SPEC_REQUIRED` (and
   optional ones to `SPEC_OPTIONAL`).
2. In `sensors.py`, write `build_<type>(s, spec, ns)` returning `sdf`,
   `bridge`, `frames`, `visuals` and `masses`, then register it in `BUILDERS`.
3. In `analysis.py`, optionally add an analysis function and register it in
   `ANALYSES`.
4. Add a test.

## 16. File map

```
asv_sim/
├── config/asv.yaml                 boat description (the only file you edit)
├── config/asv_*.yaml|txt|json|png  generated
├── models/asv/                     generated Gazebo model
├── tools/make_asv.py               entry point
├── tools/study_lidar_placement.py  lidar placement study (section 13.1)
├── tools/asvgen/                   the generator package (this document)
├── tests/test_asvgen.py            verification
└── docs/HOW_IT_WORKS.md            this file
```
