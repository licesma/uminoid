#pragma once

#include "hand_retarget/hand_retargeter.hpp"

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <optional>
#include <string>
#include <yaml-cpp/yaml.h>

#include "utils/csv_saver.hpp"
#include "utils/type.hpp"

using InspirePose = Eigen::Matrix<double, 6, 1>;

struct InspireFeedback {
    opt<InspirePose> actual;
    opt<InspirePose> force;
};


class InspireRetargeter : public HandRetargeter {
public:
    InspireRetargeter(
        bool left_enabled, bool right_enabled,
        const std::string& recording_label,
        const std::function<void(const std::string&)>& raise_error
    );

protected:
    void retarget_step(
        const opt<ManusHand>& left,
        const opt<ManusHand>& right,
        int  collection_id,
        bool paused
    ) override;

    void finish() override;

    virtual void send(
        const opt<InspirePose>& left_target,
        const opt<InspirePose>& right_target
    ) = 0;


    virtual std::pair<InspireFeedback, InspireFeedback> read_feedback() {
        return {{}, {}};
    }

private:
    struct FingerBounds { double low, high; };
    struct HandBounds {
        FingerBounds index, middle, ring, pinky, thumb;
    };

    opt<InspirePose> retarget(const opt<ManusHand>& hand, HandSide side) const;

    static double scale(float value, double low, double high);
    static HandBounds load_bounds(const YAML::Node& node);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    CsvSaver   hand_csv_;
};
