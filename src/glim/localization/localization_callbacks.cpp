#include <glim/localization/localization_callbacks.hpp>

namespace glim {

CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> LocalizationCallbacks::on_insert_imu;
CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> LocalizationCallbacks::on_insert_frame;
CallbackSlot<void(const PoseWithCov& pose, LocalizationState state)> LocalizationCallbacks::on_localization_pose;

}  // namespace glim
