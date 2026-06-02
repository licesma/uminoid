#pragma once

#include "hand_retarget/dex3/dex3_values.hpp"
#include "hand_retarget/dex3/single_dex3_controller.hpp"
#include "manus/manus_hand.hpp"
#include "utils/type.hpp"

#include <atomic>

/**
 * Retargets manus glove curls onto ONE Dex3-1 hand.
 *
 * Pure mapping layer. Active-joint open/closed q come from the URDF bounds
 * via dex3_retarget_config (no yaml: curl=0 is one bound endpoint, curl=1
 * is the other; direction lives in Dex3ActiveCurlEnd*). Static-joint hold
 * values come from Dex3StaticHoldQ*. The only per-deployment yaml input is
 * the manus normalization bounds, passed in here at construction.
 */
class SingleDex3Retargeter {
public:
    using JointPose = Dex3PerJoint<float>;

    struct FingerBounds { double low, high; bool invert; };
    struct ManusBounds  { FingerBounds thumb, index, middle; };

    SingleDex3Retargeter(Dex3Side side, const ManusBounds& manus_bounds);

    void step(const opt<ManusHand>& hand);

    // Adjusts the thumb_rotation target by delta rad, clamped to URDF bounds.
    // thumb_rotation is a static-hold joint (not driven from the glove), so
    // this is the operator's manual override knob for it. Thread-safe.
    void nudge_thumb_rotation(float delta);

    // Csv accessors (delegate to the controller).
    bool                        is_ready  () const { return controller_.is_ready();   }
    JointPose                   last_cmd_q() const { return controller_.last_cmd_q(); }
    SingleDex3Controller::State state     () const { return controller_.state();      }

private:
    JointPose compute_target(const opt<ManusHand>& hand) const;

    Dex3Side             side_;
    ManusBounds          manus_bounds_;
    SingleDex3Controller controller_;
    std::atomic<float>   thumb_rotation_q_;
};
