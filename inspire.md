# Inspire Hand on G1 — Handoff Notes

## Goal
Run the Inspire right hand on the G1 via the DDS bridge (`inspire_g1` from the
`dfx_inspire_service` submodule), and drive it from the host with
`inspire_g1_test`.

## Preferred approach (use this one)
Clone the **whole `uminoid_exo_interface` repo with submodules** onto the G1 and
build from the parent repo's CMake setup. This is already known to work on a
different G1. Do **not** try to build `dfx_inspire_service` standalone — the
standalone path was attempted and hit hard-to-diagnose serial issues (see
"Attempted path" below).

## Access
- Host shell is connected to the G1 via SSH:
  - `ssh g1_robot` works (configured in `~/.ssh/config` with `ProxyJump yue`).
  - The G1 has **no general internet access**, but can be reached from the host
    on the 192.168.123.0/24 network.
  - Host interface facing G1: `enx00e04c6803fc` (host IP `192.168.123.222`).
  - G1 interface facing host: `eth0` (G1 IP `192.168.123.164`).
- File transfer to G1: use `scp -r ... g1_robot:~`.
- Parent repo on host: `~/repos/uminoid_exo_interface` (full checkout with
  submodules available).

## G1 machine facts
- Arch: `aarch64` (the ubuntu user is `unitree`, hostname `ubuntu`).
- Unitree SDK on the G1 lives at `/home/unitree/teleop/unitree_sdk2/` with a
  prebuilt static lib at `lib/aarch64/libunitree_sdk2.a`.
- Cyclone DDS is installed system-wide under `/usr/local/include/ddscxx` and
  `/usr/local/include/iceoryx/v2.0.2` with libs in `/usr/local/lib`.
- Hand is wired to `/dev/ttyUSB0` (single right hand, ID=1). `/dev/ttyUSB1`
  does not exist on this G1.
- Serial port is `crw-rw---- root:dialout` — run `inspire_g1` with `sudo`.

## What the bridge does
- `inspire_g1` subscribes to `rt/inspire/cmd` (MotorCmds_, 12 entries, first 6
  are right hand) and publishes state on `rt/inspire/state`.
- It relays commands to the Inspire hand over UART at 115200 8N1.
- `inspire_g1_test` (in `cpp/test/inspire_g1_test.cpp` of the parent repo)
  publishes to the cmd topic and reads state back, sweeping the index finger.

## How the other G1 was made to work (for reference)
The previous working flow from the host:
1. Edits went into the submodule (`third_party/dfx_inspire_service`) on its
   `master` branch (single hand on ttyUSB0, remove left hand).
2. Push submodule, then bump the parent repo's submodule pointer.
3. On the G1: `git pull && git submodule update --recursive` in the parent
   repo, then `cd cpp/build && make inspire_g1 -j4`.
4. Run `sudo ./dfx_inspire_service/inspire_g1` on the G1 (banner prints and
   it waits for DDS).
5. Run `./inspire_g1_test enx00e04c6803fc` on the host.

For *this* G1 we can't `git pull` (no internet). Instead, `scp -r` the whole
`~/repos/uminoid_exo_interface` to `g1_robot:~/` (or use `rsync`) after making
sure submodules are materialized locally on the host. Then build from the
parent repo on the G1.

## Current repo state (host)
- Repo: `~/repos/dfx_inspire_service` (the submodule, standalone clone).
- Branch layout:
  - `master` — original; the parent repo's submodule should still track this
    so nothing breaks for other G1s.
  - `g1-standalone` — my in-progress branch with:
    - `CMakeLists.txt` rewritten to build without the parent repo
      (hardcoded `/home/unitree/teleop/unitree_sdk2` paths, links the
      prebuilt aarch64 static lib).
    - `inspire_g1.cpp` has extra debug prints (`[dbg] ...`).
    - `include/SerialPort.h` patched: `recv()` now loops until `len` bytes
      received or timeout; termios sets `CREAD | CLOCAL`. These changes are
      probably harmless but are **not validated** and were attempts to fix
      the serial failure we saw. You may want to revert them before using
      master as the submodule source.
  - The parent repo's submodule pointer on master should remain unchanged by
    this work — we kept standalone edits off master intentionally.

## Two fixes required per-G1 (this is what actually blocked us)

Both fixes are edits to the `dfx_inspire_service` submodule, applied
**locally on the target G1** — do not commit them to master, since they are
per-unit. The previous "DDS handshake never completed / GetPosition returned
1" symptoms were entirely explained by these two issues; DDS was never
actually broken.

### Fix 1 — Hand ID is not always 1

The bridge hardcodes the Inspire hand ID to 1 in
`third_party/dfx_inspire_service/inspire_g1.cpp`:

```cpp
righthand = std::make_shared<inspire::InspireHand>(serial1, 1);
```

On the new G1 the hand responded only on **ID=2**. Probe the hand first and
change the literal to match. To probe, send a `GetPosition` frame to each
candidate ID over `/dev/ttyUSB0` and see which one replies 20 bytes:

```cpp
// frame: EB 90 <id> 04 11 0A 06 0C <checksum>
// valid reply is 20 bytes starting with 90 EB
```

(A minimal probe program used during debugging was `/tmp/probe_hand.cpp` on
the G1 — write one if you need to redo this.)

Symptom if wrong: `GetPosition` returns 1 forever (no 20-byte reply), and
`hand.SetPosition` has no physical effect because the hand isn't
addressed.

### Fix 2 — `SerialPort::recv()` must loop until `len` bytes

`third_party/dfx_inspire_service/include/SerialPort.h` does a single
`::read()` after `select()`. On this G1 the 20-byte `GetPosition` reply
arrives in multiple bursts, so a single read returns <20 bytes and
`GetPosition` fails the `len != 20` check.

Replace the `switch` block inside `recv()` with a loop that keeps
calling `select()` + `read()` until `recv_len == len` or the select
times out:

```cpp
ssize_t recv_len = 0;
while (recv_len < (ssize_t)len) {
  FD_ZERO(&rSet_);
  FD_SET(fd_, &rSet_);
  timeval tv = timeout_;
  int sret = select(fd_ + 1, &rSet_, NULL, NULL, &tv);
  if (sret <= 0) break;           // error or timeout
  ssize_t n = ::read(fd_, data + recv_len, len - recv_len);
  if (n <= 0) break;
  recv_len += n;
}
if (recv_len > 0) consecutive_timeouts_ = 0;
return recv_len;
```

Symptom if missing: even with the correct ID, `GetPosition` intermittently
returns 1 and `actual` in `inspire_g1_test` sticks at 0 even though
commands are received and the finger is moving.

### Things that looked broken but weren't

- **DDS / CycloneDDS config.** No `CYCLONEDDS_URI` needed. Default multicast
  discovery works between host (`enx…`) and G1 (`eth0`) on
  `192.168.123.0/24`. `isTimeout=0` on the bridge confirms commands are
  arriving; state goes back the same way.
- **Standalone vs full-repo build.** The prior handoff blamed the standalone
  build; it wasn't the cause. Either build path works once the two fixes
  above are applied. The full-repo path is still easier because CMake and
  unitree_sdk2 linking are already set up.
- **`libfmt` link error.** `dfx_inspire_service/CMakeLists.txt` has
  `link_libraries(unitree_sdk2 fmt rt pthread)` but nothing actually uses
  `fmt`. If `libfmt-dev` isn't installed on the G1 (and with no internet,
  it won't be), drop `fmt` from that line.
- **`librealsense` fetching `nlohmann/json` from GitHub.** Parent
  `cpp/CMakeLists.txt` adds librealsense as a subdirectory, and its CMake
  tries to clone nlohmann/json at configure time. Comment out the
  librealsense `add_subdirectory` (and the `usb_camera_recorder` /
  `collect` / `camera_test` targets that link `realsense2`) on the G1 —
  they aren't needed to build `inspire_g1`.

## What to do next (for a fresh G1)
1. Materialize submodules on the host:
   `git submodule update --init --recursive`.
2. Sync to the G1 (skip `data/` — it's huge):
   `rsync -a --exclude build --exclude .git --exclude __pycache__ --exclude data ~/repos/uminoid_exo_interface g1_robot:~/`.
3. On the G1, patch the three CMake annoyances (see "Things that looked
   broken but weren't" above):
   - drop `fmt` from `third_party/dfx_inspire_service/CMakeLists.txt`
   - comment out the `librealsense` `add_subdirectory` in `cpp/CMakeLists.txt`
     plus the `usb_camera_recorder`, `collect`, `camera_test` targets that
     depend on it.
4. Probe the hand ID on `/dev/ttyUSB0` and edit `inspire_g1.cpp:18` so the
   hardcoded ID matches (see Fix 1).
5. Apply the `SerialPort::recv()` loop patch (see Fix 2).
6. Build: `cd ~/uminoid_exo_interface/cpp && mkdir -p build && cd build &&
   cmake .. && make inspire_g1 inspire_g1_test -j4`.
7. Run `sudo ./dfx_inspire_service/inspire_g1` on the G1.
8. From the host: `./inspire_g1_test enx00e04c6803fc`.
9. Verify the right-hand index finger sweeps open/close.

## Useful paths to know
- Parent repo CMake: `~/repos/uminoid_exo_interface/cpp/CMakeLists.txt`
  - Adds `third_party/unitree_sdk2`, `third_party/dfx_inspire_service` as
    subdirectories.
- DDS test client source: `cpp/test/inspire_g1_test.cpp`.
- Bridge source: `third_party/dfx_inspire_service/inspire_g1.cpp`.
- Hand protocol: `third_party/dfx_inspire_service/include/inspire.h`
  (UART frame starts with `0xEB 0x90`, ID in byte 2, 20-byte reply for
  `GetPosition`).
