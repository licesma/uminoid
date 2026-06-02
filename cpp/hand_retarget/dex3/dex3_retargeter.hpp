#pragma once

#include "hand_retarget/dex3/single_dex3_retargeter.hpp"
#include "hand_retarget/hand_retargeter.hpp"
#include "utils/csv_saver.hpp"

#include <functional>
#include <optional>
#include <string>

/**
 * Two-handed Dex3-1 retargeter. Initializes the unitree DDS ChannelFactory
 * once, then constructs a SingleDex3Retargeter for each enabled side.
 *
 * Owns the yaml load and the recording csv. Per-tick work is delegated to
 * the per-side retargeters; csv stays here so the row schema is stable
 * regardless of which sides are enabled (disabled sides log "null").
 */
class Dex3Retargeter : public HandRetargeter {
public:
    Dex3Retargeter(
        bool left_enabled, bool right_enabled,
        const std::string& network_interface,
        const std::string& recording_label = "",
        const std::function<void(const std::string&)>& raise_error = nullptr
    );

protected:
    void retarget_step(
        const opt<ManusHand>& left,
        const opt<ManusHand>& right,
        int  collection_id,
        bool paused
    ) override;

    void finish() override;

public:
    // Right arrow (+1) / left arrow (-1) nudges the right hand's
    // thumb_rotation target by a fixed step. No-op when the right side is
    // disabled.
    void handle_arrow(int dir) override;

private:
    std::optional<SingleDex3Retargeter> left_;
    std::optional<SingleDex3Retargeter> right_;
    CsvSaver                            hand_csv_;
};
