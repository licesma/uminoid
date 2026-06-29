#pragma once

#include "hand_retarget/dex3/dex3_values.hpp"
#include "hand_retarget/dex3/single_dex3_controller.hpp"
#include "manus/manus_hand.hpp"
#include "utils/type.hpp"

#include <atomic>

/**
 * Retargets manus glove curls onto ONE Dex3-1 hand.

 */
class SingleDex3Retargeter {
public:
    using JointPose = Dex3PerJoint<float>;

    struct FingerBounds { double low, high; bool invert; };
    struct ManusBounds  { FingerBounds thumb, index, middle; };

    SingleDex3Retargeter(Dex3Side side, const ManusBounds& manus_bounds);

    void step(const opt<ManusHand>& hand);

    
    void nudge_thumb_rotation(float delta);

    
    void set_thumb_rotation(float q);

    float thumb_rotation() const {
        return thumb_rotation_q_.load(std::memory_order_relaxed);
    }

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
