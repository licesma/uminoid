#pragma once

#include "../constants.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

/** Data returned by a single-arm SkeletonArm. */
struct ArmLine {
  uint64_t timestamp;
  uint64_t host_timestamp;
  std::array<uint16_t, ARM_JOINT_COUNT> data;
};

/**
 * Abstract interface for reading joint data from one arm.
 * Implementation: DynamixelArm (USB/U2D2).
 */
class SkeletonArm {
 public:
  virtual ~SkeletonArm() = default;

  SkeletonArm(const SkeletonArm&) = delete;
  SkeletonArm& operator=(const SkeletonArm&) = delete;

  virtual void Stop() = 0;

  /** Blocks until new data is available for this arm. */
  virtual std::optional<ArmLine> GetNextLine(
      const std::function<void(const std::string&)>& raise_error) = 0;

 protected:
  SkeletonArm() = default;
};
