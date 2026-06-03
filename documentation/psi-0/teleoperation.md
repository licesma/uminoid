# Teleoperation on Psi0

The teleoperation code lives in `third_party/Psi0/real/teleop/`.

## Manager

`manager.py` defines `TeleopManager`, the top-level coordinator for a teleop session. It owns the shared state and the two worker processes that do the real work.

### Responsibilities

- **Shared state setup.** Creates a `multiprocessing.Manager` dict holding the lifecycle events (`kill_event`, `session_start_event`, `failure_event`, `end_event`) that the child processes synchronize on.
- **Shared memory buffers.** Allocates two `SharedMemory` blocks of `float64`:
  - `robot_data_shm` — sized per robot (`h1` or `g1`) to fit leg/arm/hand state, IMU (quaternion, accelerometer, gyro, RPY), and for `g1` also odometry and hand-press signals.
  - `teleop_shm` — a fixed 62-element buffer for teleop commands.
- **Process orchestration.** Spawns two processes that share those buffers:
  - `RobotTaskmaster` (from `master_whole_body.py`) — reads teleop input and produces robot commands.
  - `RobotDataWorker` (from `worker.py`) — reads robot state and writes recorded data.
- **Episode directories.** `ProgressTracker` picks the next episode directory; `update_directory` creates it along with `color/` and `depth/` subfolders for camera streams.
- **Session lifecycle.**
  - `start_session` — rolls a new directory, clears failure/kill flags, and sets `session_start_event` to unblock the workers.
  - `stop_session` — sets `kill_event` and clears the start flag.
  - `failure_event` — set via the `d` command to mark the episode as failed before stopping.
- **Cleanup.** `cleanup` signals end, joins the child processes (force-terminating if they overrun the timeout), shuts down the `Manager`, and closes/unlinks both shared memory blocks.
- **Operator CLI.** `run_command_loop` is a blocking stdin loop with the bindings:
  - `s` — start a session
  - `q` — stop and merge the current episode
  - `d` — mark failure and stop
  - `exit` — clean up and exit

### Configuration

Constructor parameters: `task_name`, `robot` (`h1` or `g1`), `debug`, and optional Pico VR streaming via `pico_streamer` / `pico_ip`. The loop runs at `FREQ = 30` Hz.

## Data collection

Data collection is owned by `RobotDataWorker` (`worker.py`), spawned by the manager. It runs at 30 Hz and writes one episode per session into the directory the manager prepared.

### Sources

- **Robot state** — pulled from `robot_shm_array`, which is kept up to date by the taskmaster. `get_robot_data` slices the buffer into:
  - `leg_state`, `arm_state`, `hand_state`
  - IMU (`quaternion`, `accelerometer`, `gyroscope`, `rpy`)
  - Odometry (`position`, `velocity`, `rpy`, `quat`) — `g1` only
  - Hand pressure — `g1` only, reshaped to 18×12 and filtered by `extract_usable` / `format_pressure_data`
- **Cameras** — fetched over ZMQ REQ from the on-robot frame server (`tcp://192.168.123.162:5556` for `h1`, `…164:5556` for `g1`). Each `get_frame` request returns a multipart message with `rgb_bytes`, `ir_bytes`, and a 480×640 `uint16` depth frame.
- **IR → operator view** — the IR frame is forwarded to the VR headset: either pushed to the `TeleoperatorProcess` via `ir_data_queue` (for Vuer), or H.264-encoded with GStreamer and TCP-streamed to a Pico over `PicoIRStreamer`.

### Per-frame loop

`process_data` does the following each tick:

1. Request a frame via ZMQ (`_recv_zmq_frame`).
2. Forward the IR image to the operator headset.
3. Sleep until the next 33 ms slot (`initial_capture_time + frame_idx * DELAY`). If the slot was missed, log a warning and reuse the previous robot state with a fresh timestamp.
4. Call `_write_robot_data`, which:
   - Writes the RGB frame as `color/frame_{idx:06d}.jpg` via `AsyncImageWriter`.
   - Enqueues the depth frame for the `depth_writer_process`, which LZMA-compresses it to `depth/frame_{idx:06d}.npy.lzma`.
   - Appends a JSON record (state + IMU + odometry + pressure + image/depth paths + timestamp) to `robot_data.jsonl` via `AsyncWriter`.
5. Increments `frame_idx`.

### Outputs per episode

Under `shared_data["dirname"]`:

- `robot_data.jsonl` — one JSON object per frame
- `color/frame_NNNNNN.jpg`
- `depth/frame_NNNNNN.npy.lzma`

Image and depth writes are offloaded to async writer threads/processes so the main 30 Hz loop is not blocked by disk I/O.
