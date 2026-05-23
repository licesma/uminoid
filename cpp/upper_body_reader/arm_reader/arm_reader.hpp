#pragma once

#include "arm_angle_converter.hpp"
#include "skeleton_arm.hpp"
#include "utils/csv_saver.hpp"

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct ArmSnapshot {
  ArmLine line{};
  uint64_t sample_counter = 0;
};

/**
 * Wraps a SkeletonArm with a read thread, mutex-protected latest value,
 * and blocking/non-blocking snapshot access.
 */
class ArmReader {
 public:
  /** Pass nullptr for an inactive arm. */
  explicit ArmReader(std::unique_ptr<SkeletonArm> arm = nullptr,
                     const std::string& csv_path = "",
                     const std::function<void(const std::string&)>& raise_error = nullptr,
                     const ArmAngleConverter* converter = nullptr,
                     bool from_left = true,
                     const std::string& command_csv_path = "");
  ~ArmReader();

  ArmReader(const ArmReader&) = delete;
  ArmReader& operator=(const ArmReader&) = delete;

  /** Non-blocking: returns the latest reading. */
  ArmLine snapshot() const {
    return snapshot_with_counter().line;
  }

  /** Non-blocking: returns the latest reading plus its sample counter. */
  ArmSnapshot snapshot_with_counter() const;

  /** Blocking: waits until a new reading arrives since the last call. */
  std::optional<ArmLine> wait_for_next();

  /** Blocks on wait_for_next() in a loop, writing each reading to csv_path.
   *  Returns when the arm stops or stop_requested() returns true. */
  void collect_loop(const std::function<int()>&  collection_id,
                    const std::function<bool()>& stop,
                    const std::function<bool()>& pause = [] { return false; });

  void stop();

 private:
  void read_loop();

  std::unique_ptr<SkeletonArm> arm_;
  std::function<void(const std::string&)> raise_error_;
  CsvSaver csv_;
  CsvSaver command_csv_;
  const ArmAngleConverter* converter_ = nullptr;
  bool from_left_ = true;
  std::thread thread_;

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  ArmLine latest_{};
  uint64_t frame_seq_{0};
  uint64_t consumed_seq_{0};
  bool stopped_{false};
};
