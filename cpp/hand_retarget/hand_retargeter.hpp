#pragma once

#include "manus/manus_hand.hpp"
#include "manus/manus_reader.hpp"
#include "utils/type.hpp"

#include <functional>
#include <string>

/**
 * Template-method base for any Manus-driven hand retargeter.
 *
 * Owns the ManusReader, the per-side enable flags, the recording label,
 * and the pump loop. Subclasses implement process() to do the per-tick
 * retarget + send + (optional) feedback + csv work, and may override
 * finish() to flush any subclass-owned resources at loop end.
 *
 * Pose types and bounds shapes vary across hands (Inspire is 6-DoF /
 * 5 fingers; Dex3 is 7 motors / 3 active fingers with side-specific
 * joint orderings), so they stay in the subclass.
 */
class HandRetargeter {
public:
    HandRetargeter(
        bool left_enabled, bool right_enabled,
        const std::string& recording_label,
        const std::function<void(const std::string&)>& raise_error
    );
    virtual ~HandRetargeter() = default;

    void retarget_loop(
        const std::function<bool()>& stop,
        const std::function<int()>&  collection_id = [] { return 0; },
        const std::function<bool()>& pause        = [] { return false; }
    );

protected:
    // Per-tick hook: called once per Manus pose. left/right are nullopt when
    // that side has no fresh data. Subclasses retarget, send to hardware,
    // optionally read feedback, and (if !paused) append to their own csv.
    virtual void retarget_step(
        const opt<ManusHand>& left,
        const opt<ManusHand>& right,
        int  collection_id,
        bool paused
    ) = 0;

    // Called once after the pump loop exits. Default no-op; override to
    // flush subclass-owned resources (e.g. close a CsvSaver).
    virtual void finish() {}

    bool        left_enabled_;
    bool        right_enabled_;
    std::string recording_label_;
    ManusReader manus_;
};
