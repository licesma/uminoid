#include "hand_retarget/hand_retargeter.hpp"

HandRetargeter::HandRetargeter(
    bool left_enabled, bool right_enabled,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : left_enabled_(left_enabled),
      right_enabled_(right_enabled),
      recording_label_(recording_label),
      manus_(manus_defaults::LEFT_ADDRESS, manus_defaults::RIGHT_ADDRESS, raise_error)
{}

void HandRetargeter::retarget_loop(
    const std::function<bool()>& stop,
    const std::function<int()>&  collection_id,
    const std::function<bool()>& pause
) {
    while (auto pose = manus_.wait_for_next(stop)) {
        auto& [left, right] = *pose;
        retarget_step(left, right, collection_id(), pause());
    }
    finish();
}
