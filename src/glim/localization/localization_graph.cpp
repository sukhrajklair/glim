#include <glim/localization/localization_graph.hpp>

namespace glim {

LocalizationGraph::LocalizationGraph() = default;

void LocalizationGraph::insert_imu(const double, const Eigen::Vector3d&, const Eigen::Vector3d&) {
  // Placeholder for IMU integration.
}

void LocalizationGraph::insert_frame(const EstimationFrame::ConstPtr& frame) {
  if (!frame) {
    return;
  }

  if (!keyframe_selector.is_keyframe(frame)) {
    return;
  }

  LocalizationResult result;
  result.pose.stamp = frame->stamp;
  result.pose.pose = frame->T_world_lidar;
  result.pose.covariance.setIdentity();
  result.state = LocalizationState::kTracking;
  pending_result = result;
}

std::optional<LocalizationResult> LocalizationGraph::take_result() {
  if (!pending_result) {
    return std::nullopt;
  }

  auto result = pending_result;
  pending_result.reset();
  return result;
}

}  // namespace glim
