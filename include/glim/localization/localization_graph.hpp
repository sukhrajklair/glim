#pragma once

#include <optional>

#include <glim/localization/keyframe_selector.hpp>
#include <glim/localization/localization_callbacks.hpp>

namespace glim {

struct LocalizationResult {
  PoseWithCov pose;
  LocalizationState state = LocalizationState::kUninitialized;
};

class LocalizationGraph {
public:
  LocalizationGraph();

  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_frame(const EstimationFrame::ConstPtr& frame);

  std::optional<LocalizationResult> take_result();

private:
  KeyframeSelector keyframe_selector;
  std::optional<LocalizationResult> pending_result;
};

}  // namespace glim
