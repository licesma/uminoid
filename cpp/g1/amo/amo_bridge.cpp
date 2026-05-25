#include "amo_bridge.hpp"

#include "amo_zmq_helper.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

namespace {

// Wire format (must match py/collect/amo_sidecar/wire_protocol.py):
//   state  : uint64 seq, uint64 ts_ns,
//            float64 q[23], dq[23], quat[4], ang_vel[3], cmds[7]
//   action : uint64 seq, uint64 ts_ns, float64 q_target[15]
constexpr size_t STATE_FRAME_SIZE  = 8 + 8 + 8 * (23 + 23 + 4 + 3 + 7);  // 496
constexpr size_t ACTION_FRAME_SIZE = 8 + 8 + 8 * 15;                     // 136
static_assert(STATE_FRAME_SIZE == 496);
static_assert(ACTION_FRAME_SIZE == 136);

inline uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

template <typename T>
inline void put(uint8_t* buf, size_t& off, T value) {
  std::memcpy(buf + off, &value, sizeof(T));
  off += sizeof(T);
}

template <typename T>
inline T get(const uint8_t* buf, size_t& off) {
  T value;
  std::memcpy(&value, buf + off, sizeof(T));
  off += sizeof(T);
  return value;
}

// Layout-aware packer for the C++ -> Python state frame.
size_t pack_state_frame(uint8_t* buf,
                        uint64_t seq,
                        const MotorState& motor_state,
                        const ImuState& imu_state,
                        const std::array<float, 4>& quat_wxyz,
                        const AmoCommand& cmd) {
  size_t off = 0;
  put<uint64_t>(buf, off, seq);
  put<uint64_t>(buf, off, now_ns());

  for (int i = 0; i < 23; ++i)
    put<double>(buf, off, static_cast<double>(motor_state.q.at(AMO_OBSERVATION[i])));
  for (int i = 0; i < 23; ++i)
    put<double>(buf, off, static_cast<double>(motor_state.dq.at(AMO_OBSERVATION[i])));
  for (int i = 0; i < 4; ++i)
    put<double>(buf, off, static_cast<double>(quat_wxyz[i]));
  for (int i = 0; i < 3; ++i)
    put<double>(buf, off, static_cast<double>(imu_state.angular_velocity[i]));

  put<double>(buf, off, cmd.vx);
  put<double>(buf, off, cmd.yaw_target);
  put<double>(buf, off, cmd.vy);
  put<double>(buf, off, cmd.height);
  put<double>(buf, off, cmd.torso_yaw);
  put<double>(buf, off, cmd.torso_pitch);
  put<double>(buf, off, cmd.torso_roll);

  return off;
}

// Layout-aware unpacker for the Python -> C++ action frame.
// Caller must have already verified that the frame is ACTION_FRAME_SIZE bytes.
AmoAction unpack_action_frame(const uint8_t* buf) {
  size_t off = 0;
  AmoAction act;
  act.seq   = get<uint64_t>(buf, off);
  act.ts_ns = get<uint64_t>(buf, off);
  for (int i = 0; i < AMO_LOWER_DOF; ++i) {
    act.q_target[i] = get<double>(buf, off);
  }
  return act;
}

}  // namespace

AmoBridge::AmoBridge(ActionCallback on_action, ErrorCallback raise_error)
    : AmoBridge(std::move(on_action), Config{}, std::move(raise_error)) {}

AmoBridge::AmoBridge(ActionCallback on_action, Config cfg,
                     ErrorCallback raise_error)
    : cfg_(std::move(cfg)),
      on_action_(std::move(on_action)),
      raise_error_(std::move(raise_error)),
      ctx_(1),
      state_pub_(amo_zmq::make_pub_bound(ctx_, cfg_.state_endpoint)),
      action_sub_(amo_zmq::make_sub_conflate(ctx_, cfg_.action_endpoint)) {
  running_.store(true);
  receive_thread_ = std::thread(&AmoBridge::receive_loop_, this);
}

AmoBridge::~AmoBridge() {
  running_.store(false);
  // Closing sockets unblocks poll() with ETERM in the receive thread.
  try { action_sub_.close(); } catch (...) {}
  try { state_pub_.close();  } catch (...) {}
  if (receive_thread_.joinable()) receive_thread_.join();
}

void AmoBridge::publish_state(uint64_t seq,
                              const MotorState& motor_state,
                              const ImuState& imu_state,
                              const std::array<float, 4>& quat_wxyz,
                              const AmoCommand& cmd) {
  uint8_t buf[STATE_FRAME_SIZE];
  const size_t written =
      pack_state_frame(buf, seq, motor_state, imu_state, quat_wxyz, cmd);
  if (written != STATE_FRAME_SIZE) {
    raise_error_("[amo_bridge] BUG: packed " + std::to_string(written) +
                 " bytes, expected " + std::to_string(STATE_FRAME_SIZE));
    return;
  }
  
  try {
    amo_zmq::send_dontwait(state_pub_, buf, STATE_FRAME_SIZE);
  } catch (const zmq::error_t& e) {
    raise_error_(std::string("[amo_bridge] state send failed: ") + e.what());
  }
}

void AmoBridge::receive_loop_() {
  amo_zmq::run_receive_loop(action_sub_, running_,
      // on_msg: per-frame handler.
      [this](const uint8_t* buf, size_t size) {
        if (size != ACTION_FRAME_SIZE) {
          debug_bad_frame_counts_.fetch_add(1);  // TODO(amo-bridge): remove
          // Wire-format mismatch is not transient -- two sides disagree on the
          // protocol. Escalate so collect can shut down rather than silently
          // running with a dead AMO.
          raise_error_("[amo_bridge] action frame size " +
                       std::to_string(size) + " != expected " +
                       std::to_string(ACTION_FRAME_SIZE) +
                       " (wire format mismatch with sidecar)");
          return;
        }
        const AmoAction act = unpack_action_frame(buf);
        debug_successful_action_.fetch_add(1);  // TODO(amo-bridge): remove
        on_action_(act);
      },
      // on_fatal: ZMQ-level error that isn't EINTR or shutdown. Escalate.
      raise_error_);
}
