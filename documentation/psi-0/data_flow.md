# Ψ₀ Data Flow — Teleoperation to Fine-Tune

End-to-end mapping of every artifact and every field produced by the Psi0 pipeline
(`third_party/Psi0`), starting from a live teleop session and ending at the parquet/mp4
dataset that the fine-tune script consumes. All sizes are for the **G1** robot (see
`real/teleop/constants.py::G1_sizes`).

Cadence is **30 Hz** end-to-end (`FREQ = 30`, `DELAY = 1/30`).

---

# Stage 1 — Teleoperation (live capture)

Two cooperating processes managed by `TeleopManager` (`real/teleop/manager.py`) write
to a per-episode directory `<task>/episode_<N>/`.

## 1A — `RobotDataWorker` (state + frames)

`real/teleop/worker.py`. Runs at 30 Hz. For every camera frame it writes one image,
one depth file, and one JSONL row.

### `color/frame_NNNNNN.jpg`
- One file per frame.
- Source: ZMQ stream from the on-robot `realsense_server.py`.
- Encoded by `AsyncImageWriter` (`writers.py:17`) via `cv2.imdecode` → `cv2.imwrite`.
- Dimensions: **640 × 480**, RGB → JPEG.

### `depth/frame_NNNNNN.npy.lzma`
- One file per frame.
- Source: same ZMQ message, `np.frombuffer(depth_bytes, np.uint16).reshape((480, 640))`.
- Stored lzma-compressed (`worker.py::depth_writer_process`, line 328).

### `robot_data.jsonl`
One JSON object per line, one line per camera frame. Built by
`RobotDataWorker.get_robot_data` (`worker.py:429`) from the shared-memory
snapshot of the robot state.

| Field | Type | Size (G1) | Source / meaning |
|---|---|---|---|
| `time` | float | — | `time.time()` when the row is written |
| `robot_type` | str | — | `"g1"` or `"h1"` |
| `states.arm_state` | list[float] | **14** | Both-arm joint angles, encoder/SDK frame |
| `states.leg_state` | list[float] | **15** | Both-leg joint angles + waist |
| `states.hand_state` | list[float] | **14** | Dex3 hand joints (7 L + 7 R) |
| `states.hand_pressure_state` | list[dict] | up to 18 sensors × ≤4 readings | Tactile, formatted by `format_pressure_data` |
| `states.imu.quaternion` | list[float] | 4 | Body IMU |
| `states.imu.accelerometer` | list[float] | 3 | Body IMU |
| `states.imu.gyroscope` | list[float] | 3 | Body IMU |
| `states.imu.rpy` | list[float] | 3 | Body IMU |
| `states.odometry.position` | list[float] | 3 | Base odometry |
| `states.odometry.velocity` | list[float] | 3 | Base odometry |
| `states.odometry.rpy` | list[float] | 3 | Base odometry |
| `states.odometry.quat` | list[float] | 4 | Base odometry |
| `actions` | null | — | Placeholder — filled in by Stage 2 merger |
| `image` | str | — | `"color/frame_NNNNNN.jpg"` (relative path) |
| `depth` | str | — | `"depth/frame_NNNNNN.npy.lzma"` (relative path) |

Total raw float count in the shared-memory snapshot for G1:
`15 + 14 + 14 + 4 + 3 + 3 + 3 + 3 + 3 + 3 + 4 + 216 = 285` doubles.
(`HAND_PRESS_SIZE = 216` = 18 sensors × 12 raw readings; the writer down-samples to the
"usable readings" structure shown above.)

## 1B — `RobotTaskmaster` (actions / IK)

`real/teleop/master_whole_body.py`. Runs the whole-body IK loop. Every IK tick it
calls `IKDataWriter.write_data` (`writers.py:70`), appending one JSON line to
`ik_data.jsonl`.

### `ik_data.jsonl`

| Field | Type | Size | Meaning |
|---|---|---|---|
| `right_angles` | list[float] | 7 (or 12 for H1) | Right-hand joint command |
| `left_angles` | list[float] | 7 (or 12 for H1) | Left-hand joint command |
| `armtime` | float | — | Timestamp used to align with `robot_data` `time` |
| `iktime` | float | — | IK solver wall-clock |
| `sol_q` | list[float] | **29** | Full body solution: `[0:15]` legs+waist, `[15:29]` arms |
| `tau_ff` | list[float] | 29 | Feed-forward torque |
| `head_rmat` | list[list[float]] | 3 × 3 | Head rotation matrix from VR headset |
| `left_pose` | list[float] | 6 or 7 | Left wrist target pose |
| `right_pose` | list[float] | 6 or 7 | Right wrist target pose |
| `torso_height` | float | 1 | Commanded torso height |
| `torso_rpy` | list[float] | 3 | Commanded torso roll/pitch/yaw |
| `torso_vx` | float | 1 | Commanded base x-velocity |
| `torso_vy` | float | 1 | Commanded base y-velocity |
| `torso_vyaw` | float | 1 | Commanded base yaw rate |
| `torso_dyaw` | float | 1 | Yaw-delta term |
| `target_yaw` | float | 1 | Absolute yaw target |

## 1C — Episode dir layout at end of Stage 1

```
<task>/episode_<N>/
├── color/                frame_000000.jpg, frame_000001.jpg, …
├── depth/                frame_000000.npy.lzma, frame_000001.npy.lzma, …
├── robot_data.jsonl      (state, one row per camera frame, ~30 Hz)
├── ik_data.jsonl         (actions, one row per IK tick)
└── failed                (touch-file written iff user pressed `d`)
```

---

# Stage 2 — Per-episode merge (`DataMerger`)

`real/teleop/merger.py`, launched as a daemon subprocess at end-of-episode
(`master_whole_body.py::merge_data`).

### Algorithm
1. Load `robot_data.jsonl` and `ik_data.jsonl` into lists.
2. Keyed on each `robot_data` row's `time`, find the IK row whose `armtime` is closest.
3. If `|armtime − time| > DELAY/2 = ~16.7 ms`, **reuse the last good IK row** instead.
4. Attach the IK dict under the row's `actions` field.
5. Dump the modified list as a single pretty-printed JSON file.

### Output: `data.json`

One file per episode. Top-level is a **JSON list**, one entry per camera frame. Each
entry has the same schema as a `robot_data.jsonl` row, with `actions` populated
exactly as in `ik_data.jsonl` above.

```json
[
  {
    "time": 1737051422.401,
    "robot_type": "g1",
    "states": {
      "arm_state":  [...14...],
      "leg_state":  [...15...],
      "hand_state": [...14...],
      "hand_pressure_state": [ {"sensor_id":..., "sensor_type":"A","usable_readings":[...]} , ...],
      "imu":       {"quaternion":[...4], "accelerometer":[...3], "gyroscope":[...3], "rpy":[...3]},
      "odometry":  {"position":[...3], "velocity":[...3], "rpy":[...3], "quat":[...4]}
    },
    "actions": {
      "right_angles":[...7], "left_angles":[...7],
      "armtime":..., "iktime":...,
      "sol_q":[...29], "tau_ff":[...29],
      "head_rmat":[[...],[...],[...]],
      "left_pose":[...], "right_pose":[...],
      "torso_height":..., "torso_rpy":[...3],
      "torso_vx":..., "torso_vy":..., "torso_vyaw":...,
      "torso_dyaw":..., "target_yaw":...
    },
    "image": "color/frame_000000.jpg",
    "depth": "depth/frame_000000.npy.lzma"
  },
  ...
]
```

### Layout at end of Stage 2 (== what HuggingFace `g1_real_raw/<task>.zip` ships)

```
g1_real_raw/<task>/
├── episode_0/
│   ├── color/      frame_000000.jpg …
│   ├── depth/      frame_000000.npy.lzma …
│   ├── robot_data.jsonl
│   ├── ik_data.jsonl
│   └── data.json          ← canonical merged artifact
├── episode_1/ …
└── …
```

---

# Stage 3 — Raw → LeRobot (`scripts/data/raw_to_lerobot.py`)

Class `HE2LeRobotConverter` (`scripts/data/raw_to_lerobot.py:119`). Invoked as:

```bash
python scripts/data/raw_to_lerobot.py \
  --data-root  $PWD/data/real_teleop_g1/g1_real_raw \
  --work-dir   $PWD/data/real \
  --repo-id    psi0-real-g1 \
  --robot-type g1 \
  --task       $task
```

Per episode → one parquet + one MP4 + one row in `episodes_stats.jsonl`.

## 3A — `states` vector (per row)

Built by `build_obs` (line 186). Concatenation, in order:

| Slice | Field | Size | Source |
|---|---|---|---|
| `[0:14]` | `hand_state` | 14 | `frame.states.hand_state` |
| `[14:28]` | `arm_state` | 14 | `frame.states.arm_state` |
| `[28:31]` | `torso_rpy` | 3 | `prev_frame.actions.torso_rpy` (NB: **previous** frame) |
| `[31:32]` | `torso_height` | 1 | `prev_frame.actions.torso_height` |

→ **states ∈ ℝ³²** (G1).
For the very first frame `prev_rpy_height` is seeded to `[0,0,0] / 0.75`.
IMU, odometry, leg_state, depth, tactile are loaded by the converter but **dropped**
from the final `states` vector.

## 3B — `action` vector (per row)

Built by `build_act` (line 215). Concatenation, in order:

| Slice | Field | Size | Source |
|---|---|---|---|
| `[0:7]` | left hand cmd | 7 | `actions.left_angles` (verbatim for G1; for H1 the 12-D qpos is folded into 7 via `convert_h1_hand`) |
| `[7:14]` | right hand cmd | 7 | `actions.right_angles` (same H1 conversion) |
| `[14:28]` | arm cmd | 14 | `actions.sol_q[15:29]` (legs `sol_q[0:15]` are intentionally **dropped**) |
| `[28:31]` | torso_rpy | 3 | `actions.torso_rpy` |
| `[31:32]` | torso_height | 1 | `actions.torso_height` |
| `[32:33]` | torso_vx | 1 | `actions.torso_vx` |
| `[33:34]` | torso_vy | 1 | `actions.torso_vy` |
| `[34:35]` | torso_vyaw | 1 | `actions.torso_vyaw` |
| `[35:36]` | target_yaw | 1 | `actions.target_yaw` (note: `torso_dyaw` is read but **not** appended) |

→ **action ∈ ℝ³⁶**.

## 3C — Parquet row schema (`HE2LeRobotConverter.features`)

```
data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet
```

| Column | dtype | Meaning |
|---|---|---|
| `states` | Sequence(float32) | 32-D, as above |
| `action` | Sequence(float32) | 36-D, as above |
| `timestamp` | float32 | `frame_index / FPS` (so `0.0, 0.0333, …`) |
| `frame_index` | int64 | Index within the episode |
| `episode_index` | int64 | Global episode index |
| `index` | int64 | Same as `frame_index` (placeholder for future global index) |
| `task_index` | int64 | Foreign-key into `meta/tasks.jsonl` |
| `next.done` | bool | True only on the final frame of the episode |

## 3D — Video (`videos/chunk-XXX/egocentric/episode_NNNNNN.mp4`)

- Encoded by `imageio.imwrite(..., codec="libx264", fps=30)`.
- One file per episode.
- Frame source = the JPEG sequence in `color/`.
- Pixel format yuv420p, no audio (declared in `meta/info.json`).
- 480 × 640, channels-last `[H, W, C]`.
- **Depth, IMU, tactile, odometry from Stage 1 are NOT carried forward into the LeRobot dataset** — they exist only in `robot_data.jsonl`/`data.json`.

## 3E — Meta files

### `meta/info.json` — dataset manifest (`InfoDict`, dataclass at line 43)

| Field | Meaning |
|---|---|
| `codebase_version` | `"v2.1"` |
| `robot_type` | `"g1"`, `"h1"`, or `"mixed"` |
| `total_episodes` / `total_frames` / `total_tasks` / `total_videos` / `total_chunks` | scalar counts |
| `chunks_size` | episodes per chunk dir (default **1000**) |
| `fps` | `30` |
| `data_path` | `"data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"` |
| `video_path` | `"videos/chunk-{episode_chunk:03d}/egocentric/episode_{episode_index:06d}.mp4"` |
| `features.observation.images.egocentric` | `{dtype:"video", shape:[480,640,3], names:[height,width,channel], video_info:{fps:30, codec:"h264", pix_fmt:"yuv420p", is_depth_map:false, has_audio:false}}` |
| `features.states` | `{dtype:"float32", shape:[-1]}` |
| `features.action` | `{dtype:"float32", shape:[-1]}` |
| `features.{timestamp, frame_index, episode_index, index, next.done, task_index}` | scalars (see `write_meta`, line 549) |

### `meta/tasks.jsonl` — one row per task

| Field | Meaning |
|---|---|
| `task_index` | int |
| `task` | natural-language instruction, from `scripts/data/task_description_dict.json[<task_name>]` |
| `category` | parent dir name |
| `name` | leaf dir name (e.g. `Hug_box_and_move`) |

### `meta/episodes.jsonl` — one row per episode

| Field | Meaning |
|---|---|
| `episode_index` | int |
| `tasks` | `[task_index]` |
| `length` | frames in the episode |
| `dataset_from_index` / `dataset_to_index` | inclusive global-row span |
| `robot_type` | from `data.json` (`get_robot_type`, line 141) |
| `instruction` | duplicate of the task description |

### `meta/episodes_stats.jsonl` — appended atomically per episode by `make_one_episode`

| Field | Meaning |
|---|---|
| `episode_index` | int |
| `stats.action.{min,max,mean,std,count}` | per-dim arrays, length-36 (count is `[len(rows)]`) |
| `stats.timestamp.{min,max,mean,std,count}` | scalar arrays |

## 3F — Layout at end of Stage 3

```
data/real/<task>/
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet
│       ├── episode_000001.parquet
│       └── …
├── videos/
│   └── chunk-000/
│       └── egocentric/
│           ├── episode_000000.mp4
│           └── …
└── meta/
    ├── info.json
    ├── tasks.jsonl
    ├── episodes.jsonl
    └── episodes_stats.jsonl
```

---

# Stage 4 — Dataset-level statistics

`scripts/data/calc_modality_stats.py --work-dir … --task <task>`. Reads every
parquet, aggregates, writes `meta/stats.json`. Then a manual:

```bash
cp meta/stats.json meta/stats_psi0.json
```

(Ψ₀ training looks for `stats_psi0.json` by convention; today the two files are
identical.)

A patch step exists for a known LeRobot metadata bug:
```bash
python scripts/data/patch_lerobot_meta.py $PSI_HOME/data/real/<task>
```

---

# Stage 5 — Fine-tune

`scripts/train/psi0/finetune-real-psi0.sh <task>` consumes
`$PSI_HOME/data/real/<task>/` (the exact Stage-3+4 layout) and produces a
fine-tuned Ψ₀ checkpoint.

The training-side LeRobot loader keys off the schema declared in
`meta/info.json` (the `observation.images.egocentric` video feature plus the
flat `states` / `action` float32 sequences) — so anything we want consumed by
Ψ₀ must end up in **exactly** that shape: states 32-D, action 36-D, 30 fps,
one mp4 per episode, one parquet per episode under the chunked layout.

---

# Summary cheat-sheet

| Stage | Producer | Output | Key shapes (G1) |
|---|---|---|---|
| 1A | `RobotDataWorker` | `color/*.jpg`, `depth/*.npy.lzma`, `robot_data.jsonl` | 640×480; states 285-D snapshot |
| 1B | `RobotTaskmaster` | `ik_data.jsonl` | sol_q 29-D, hands 7-D each |
| 2  | `DataMerger` | `data.json` | per-frame list with merged `actions` |
| 3  | `HE2LeRobotConverter` | `data/*.parquet`, `videos/*.mp4`, `meta/*` | **states 32-D, action 36-D**, 30 fps |
| 4  | `calc_modality_stats.py` | `meta/stats.json` (+ `stats_psi0.json`) | per-feature aggregate stats |
| 5  | `finetune-real-psi0.sh` | Ψ₀ checkpoint | — |
