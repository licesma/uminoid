#include "hand_retarget/dex3/single_dex3_retargeter.hpp"

#include "hand_retarget/dex3/dex3_helper.hpp"
#include "hand_retarget/dex3/dex3_retarget_config.hpp"

#include <algorithm>

namespace {

float scale_curl(float measured, float low, float high) {
    if (high <= low) return 0.0f;
    return std::clamp((measured - low) / (high - low), 0.0f, 1.0f);
}

float apply_curl(const SingleDex3Retargeter::FingerBounds& b, float measured) {
    const float c = scale_curl(measured, float(b.low), float(b.high));
    return b.invert ? (1.0f - c) : c;
}

// q at curl=0 / q at curl=1 for an active joint, looked up from the URDF
// bound and the per-side curl-end direction.
struct OpenClosed { float open, closed; };
OpenClosed open_closed(Dex3Joint j, Dex3Side side) {
    const Dex3Bounds  b   = (side == Dex3Side::Left ? Dex3BoundsLeft : Dex3BoundsRight)[j];
    const Dex3CurlEnd end = (side == Dex3Side::Left ? Dex3ActiveCurlEndLeft
                                                   : Dex3ActiveCurlEndRight)[j];
    return (end == Dex3CurlEnd::low) ? OpenClosed{b.high, b.low}
                                     : OpenClosed{b.low,  b.high};
}

float static_hold(Dex3Joint j, Dex3Side side) {
    return (side == Dex3Side::Left ? Dex3StaticHoldQLeft : Dex3StaticHoldQRight)[j];
}

}  // namespace

SingleDex3Retargeter::SingleDex3Retargeter(Dex3Side side, const ManusBounds& manus_bounds)
    : side_        (side),
      manus_bounds_(manus_bounds),
      controller_  (side)
{}

SingleDex3Retargeter::JointPose
SingleDex3Retargeter::compute_target(const opt<ManusHand>& hand) const {
    JointPose out{};
    for (int i = 0; i < Dex3NumJoints; ++i) {
        const auto j = static_cast<Dex3Joint>(i);
        if (!dex3_helper::is_active(j)) {
            out[j] = static_hold(j, side_);
            continue;
        }
        
        float curl = 0.0f;
        if (hand) {
            switch (j) {
                case Dex3Joint::thumb_palm_bend:
                    curl = apply_curl(manus_bounds_.thumb,  hand->thumb.pinky_to_thumb);  break;
                case Dex3Joint::index_palm_bend:
                    curl = apply_curl(manus_bounds_.index,  hand->index.wrist_to_tip);    break;
                case Dex3Joint::middle_palm_bend:
                    curl = apply_curl(manus_bounds_.middle, hand->middle.wrist_to_tip);   break;
                default: break;
            }
        }
        const auto oc = open_closed(j, side_);
        out[j] = oc.open + curl * (oc.closed - oc.open);
    }
    return out;
}

void SingleDex3Retargeter::step(const opt<ManusHand>& hand) {
    controller_.send_slewed(compute_target(hand));
}
