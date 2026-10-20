# robot_arm_description — model notes & provenance

Every value in `urdf/robot.urdf.xacro` is tagged **[STL]** (measured from the mesh),
**[PHOTO]** (inferred from the reference photos) or **[ASSUMED]** (placeholder — replace it).
Reproduce the STL numbers with `python3 tools/analyze_meshes.py`; re-check the whole model after any
edit with `python3 tools/verify_kinematics.py`.

## 1. Joint summary

| Joint | Parent → Child | Type | Axis (joint frame) | Origin in parent frame [m] | Limits [deg] | Confidence |
|---|---|---|---|---|---|---|
| `shoulder_pan_joint` | base_link → shoulder_link | revolute | Z | (0, 0, 0.080) | ±90 [ASSUMED] | Axis/origin **high**; limits **low** |
| `shoulder_lift_joint` | shoulder_link → upper_arm_link | revolute | Y | (0.0098, 0, 0.030) | −78 … +106 | Z & axis **high**; **X medium — verify** |
| `elbow_joint` | upper_arm_link → forearm_link | revolute | Y | (0, 0, 0.200) | −138 … +77 | Origin/axis **high**; zero pose is a convention |
| `wrist_pitch_joint` | forearm_link → wrist_link | revolute | Y | (0.140, 0, 0) | −102 … +102 | Origin/axis **high** (bolt hole pair) |
| `world_to_base` | world → base_link | fixed | – | 0 | – | – |

Zero pose (q = 0): upper arm vertical, forearm horizontal pointing forward (+X), wrist in line with the forearm.
This is a *model* convention; the physical servo-horn zero is unknown and must be calibrated (see §5).

## 2. Where each number came from

**[STL] — measured directly**
- Units are millimetres (bbox 43…241 mm, all five meshes watertight) → `scale="0.001"`.
- Base: ring ⌀109.8 centred at x=y=0, rim top z=70. Shoulder disc r=60, 10 mm thick (z −10…0) → sits flush on the rim → pan origin z = 70+10 = **80 mm**; pan axis = ring axis (Z).
- Shoulder ear window 40×20 mm centred z=30, tab holes 49.5 mm apart / 10 mm high (MG996R-class servo footprint) → shoulder axis height **30 mm**, axis direction Y (windows are in the ear planes).
- Upper arm: shoulder hole (0,−100), elbow hole (47.5, 94.28) in mesh coordinates, both along Y → link length **200.0 mm**; line tilted 13.74° from mesh +Z. The visual mesh is rotated by −13.74° so link +Z runs along the arm line.
- Forearm: elbow hole at mesh origin, wrist hole at mesh x=−140 → **140 mm**. The fork gap (30.4 mm) matches the 30 mm-thick upper arm head; hole diameters (10.2 / 9.7 mm) match. Mesh points toward −X, so it is flipped 180° about Z.
- Wrist: a U-clevis (gap 21 mm) that straddles the 20 mm-thick forearm tip; its pin hole (⌀6.8 along mesh X at y=7.5) lines up with the forearm's ⌀6.7 tip hole → wrist axis = forearm Y through the forearm tip. Mesh turned +90° about Z, pin on the link origin.
- Joint limits for lift / elbow / wrist: contiguous **mesh-vs-mesh collision-free window** around q=0 (adjacent links, 1° steps), minus 3° margin.

  | Joint | free window | limit used |
  |---|---|---|
  | lift | −81 … +109° | −78 … +106° |
  | elbow | −141 … +80° | −138 … +77° |
  | wrist | −105 … +105° | −102 … +102° |
  | pan | none (no geometric stop) | ±90° assumed |

- Collision geometry: boxes/cylinders fitted to slices of each mesh (link frames). Inertia tensors and centres of mass: exact polyhedral integrals of the STL volumes assuming uniform density, stored per kg and scaled by the mass placeholder.

**[PHOTO]**
- Base→shoulder is a turntable driven by a servo hidden in the base (photo 2) → revolute about Z.
- Two servos, one per shoulder ear (photo 2) → one lift joint with a doubled effort placeholder.
- Elbow has the servo on one fork tine and a bearing on the other (photo 1) → single revolute Y.
- Forearm tip carries a hinge pin with a small (SG90-class) servo at the wrist (photos 1, 3) → wrist pitch about Y.
- Photo 1 pose (arm leaning back, forearm horizontal) is reproduced by q ≈ (0, −37°, +37°, 0); see `docs/assembled_poses.png`.

**[ASSUMED] — replace with real data**
- Masses: base 1.00, shoulder 0.30, upper arm 0.25, forearm 0.10, wrist 0.03 kg. (Solid-PLA upper bounds from mesh volume: 0.185 / 0.196 / 0.384 / 0.133 / 0.024 kg. Real parts are hollow/infilled shells with servos added.)
- Effort 1.0 / 2.0 / 1.0 / 0.2 N·m and speed 3 rad/s (typical servo class, not measured); joint damping 0.05, friction 0.02.
- Pan limit ±90°; uniform-density COM/inertia (hollow parts, servos and wiring shift the real COM).
- Servo models are inferred from mounting patterns/photos, not from labels.

## 3. Highest-risk item — shoulder axis X

The servo window fixes the axis height (z=30) but not where the shaft sits along X. A standard servo's shaft is ~10 mm from one end of its body, so the candidates are **x = 9.8 mm** (used) or **x = 29.7 mm**. 9.8 was chosen because the ear's slanted rear edge is tangent (0.4 mm gap) to the 27.5 mm-radius upper-arm head only for that position. Measure on the real arm: horizontal distance from the turntable centre to the shoulder servo shaft. To change it, edit `shoulder_axis_x` in `robot.urdf.xacro`. Effect on reach: 20 mm in X.

## 4. Known simplifications
- Collision boxes are deliberately coarse and overlap between adjacent links (fork around arm head, clevis around forearm tip). Gazebo has self-collision off by default; for MoveIt later, disable adjacent pairs in the SRDF.
- Hollow base shell: only its outer envelope is modelled for collision.
- The verifier only checks *adjacent* link pairs; non-adjacent collisions at extreme combined poses (e.g. forearm vs base at very negative lift) are not covered.
- Joint zero offsets vs. the physical servos are not modelled.

## 5. Physical verification checklist
1. Shoulder shaft X offset from turntable centre (9.8 vs 29.7 mm) — §3.
2. Height of turntable disc above the ground (expect 80 mm).
3. Upper-arm shaft-to-shaft distance (expect 200 mm) and forearm hole-to-hole (expect 140 mm).
4. Servo horn positions at the "centre" command: how far the real arm sits from q=0 in each joint; note the servo direction sign per joint.
5. Real travel of each joint vs. the limits above (do any servos stop earlier than the geometric limit?).
6. Weigh each printed link with its servos → set the `*_mass` properties.
7. Which way the wrist bracket faces (mesh could be mirrored about the pin axis; both look near-identical).
