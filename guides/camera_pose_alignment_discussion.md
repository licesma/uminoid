# Camera Pose Alignment for Exo → Robot Data Collection

> Working doc to be discussed point by point in a later session. Each open
> point has a **Decision** slot — leave it empty until we resolve it together.

## Context / Problem

- The robot has arms + a camera. The exoskeleton matches the robot's arms, and
  the camera-to-arms distance also matches between exo and robot.
- For the current task the robot is basically **static**, only moving its arm.
  That could change later.
- **Domain gap:** it's hard to collect (human wearing the exo) at the same
  distance/height from the table that the robot will have at deployment.
- At deployment, the robot's height and distance from the table are **known**.
- What a vision policy ultimately needs to match between collection and
  deployment is the **camera pose relative to the workspace** — that's what
  determines the pixels the policy sees. Since the exo arms already match the
  robot arms kinematically, the camera viewpoint is the dominant remaining gap.

## Esteban's Plan

1. Run **spring/fiducial detection** on the **robot's camera**.
2. From the detection, measure the camera-to-table **horizontal distance** and
   **vertical height**.
3. While collecting (wearing the exo), **stream these values live**.
4. The operator physically adjusts — move closer to the table, adjust the seat
   height — to get as close as possible to the robot's measured values.

### Secondary idea (later step)

- Instead of only matching, **pass height + distance to the table into the
  policy** as inputs, so the robot learns across a range of heights/distances.
- Esteban's view: this is a later step, only after the simple matched-pose case
  is proven to work.

---

## Open Points to Discuss

### Point 1 — Two scalars underconstrain the camera pose

- **Concern:** camera pose is 6-DoF. Horizontal distance + vertical height
  leave **pitch, yaw, and lateral offset** free. The operator could match both
  numbers while the camera is pitched/yawed differently, producing a visibly
  different image. Pitch especially changes how the table projects.
- **Proposal:** drive alignment off the **full fiducial pose** (or at minimum
  forward / lateral / height / pitch / yaw), and show the operator a **per-axis
  delta** so they zero all error components, not just two. The USB recorder
  already derives pitch/roll from the IMU — fold that in.
- **Tradeoff:** more numbers for the operator to chase; may need the image
  overlay (Point 3) to be usable in practice.
- **Decision:**

### Point 2 — Tag placement should be the workspace

- **Concern:** "match camera to tag" only equals "match camera to workspace" if
  the tag sits where the manipulated objects will be — same fixed spot for both
  robot and exo sessions.
- **Proposal:** place the fiducial at the **workspace origin** (where objects
  are manipulated), flat on the table, identical placement for robot and exo.
- **Tradeoff:** the tag may occlude/clutter the actual task area; might need a
  setup phase where the tag is present, then removed for the real episode.
- **Decision:**

### Point 3 — Image overlay as the alignment UI (possibly the biggest win)

- **Concern:** numbers are an indirect proxy for "same image."
- **Proposal:** ghost the robot's **reference camera frame** semi-transparently
  over the **live exo feed** in the `PreviewServer`, and let the operator match
  the viewpoint holistically (framing of the real objects, not just readouts).
  Captures DoF the scalars miss; humans are good at visual registration.
- **Tradeoff:** more preview work; needs a stored robot reference frame and
  alignment of the two camera intrinsics/feeds.
- **Decision:**

### Point 4 — Log the full exo camera pose throughout collection

- **Concern:** humans drift mid-session; a live readout alone doesn't catch it.
- **Proposal:** log **per-frame exo camera pose** (cheap, CSV). Gives:
  - a **quality gate** — discard/reweight episodes where the operator wandered
    too far from target;
  - **future-proofing** — it's exactly the input the conditioning approach
    (secondary idea) would need later, so log it now even if unused today.
- **Tradeoff:** tag must stay visible during episodes for continuous pose, which
  conflicts somewhat with Point 2 (tag occluding the workspace).
- **Decision:**

---

## Tension to keep in mind: Matching vs. Conditioning

- **Matching** collapses all data to one camera pose.
- **Conditioning** only helps if the data deliberately **spans a range** of
  heights/distances.
- So they want **opposite data distributions**. They're not purely sequential —
  if conditioning is the real endgame, eventually you'd collect at varied poses
  on purpose. Worth deciding the endgame now so today's logging serves both.
- **Decision:**

## Smaller items

- **Depth cross-check:** use the robot's depth as an **independent sanity check**
  of the fiducial's table distance during setup (catches gross calibration
  errors). Don't rely on stereo depth for streamed precision — it's noisy on a
  bare table. **Decision:**
- **Mechanical repeatability aid:** floor marks for the chair + a seat-height
  detent set once from the reading removes per-session drift better than any
  live readout; the fiducial then just verifies. **Decision:**
- **Calibration:** for the robot↔exo comparison to be valid, both cameras need
  good intrinsics; same camera model is ideal so biases cancel. **Decision:**

## Implementation prerequisites (from earlier discussion)

- D435i color: single RGB sensor, max **1920×1080 @ 30 fps** (RGB8). Cannot
  stream two color resolutions at once.
- Plan: capture once at **1080p**, run fiducial detection on the full-res frame,
  **downscale to 640×480** (`cv::resize`, `INTER_AREA`, ~0.5–1.5 ms/frame) for
  the existing recording/preview path.
- Run detection **off the capture/write thread** (or every Nth frame) so the
  480 recording keeps 30 fps regardless of detector load.
