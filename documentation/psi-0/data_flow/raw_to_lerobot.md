# Stage 3 — Raw → LeRobot

`scripts/data/raw_to_lerobot.py`. Class `HE2LeRobotConverter`
(`scripts/data/raw_to_lerobot.py:119`). Consumes the merged `data.json` from
[[2. merge]] and produces LeRobot-format parquet + mp4 per episode.

Invoked as:

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

| Slice     | Field          | Size | Source                                                  |
| --------- | -------------- | ---- | ------------------------------------------------------- |
| `[0:14]`  | `hand_state`   | 14   | `frame.states.hand_state`                               |
| `[14:28]` | `arm_state`    | 14   | `frame.states.arm_state`                                |
| `[28:31]` | `torso_rpy`    | 3    | `prev_frame.actions.torso_rpy` (NB: **previous** frame) |
| `[31:32]` | `torso_height` | 1    | `prev_frame.actions.torso_height`                       |

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

Next: [[stats]].
