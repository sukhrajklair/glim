#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/util/callback_slot.hpp>

namespace glim {

struct PoseWithCov {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
};

enum class LocalizationState {
  kUninitialized,
  kTracking,
  kLost,
};

/**
 * @brief Localization-related callbacks
 */
struct LocalizationCallbacks {
  /**
   * @brief IMU input callback
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  static CallbackSlot<void(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel)> on_insert_imu;

  /**
   * @brief Odometry estimation frame input callback
   * @param frame  Odometry estimation frame
   */
  static CallbackSlot<void(const EstimationFrame::ConstPtr& frame)> on_insert_frame;

  /**
   * @brief Localization pose output callback
   * @param pose   Estimated pose with covariance
   * @param state  Localization state
   */
  static CallbackSlot<void(const PoseWithCov& pose, LocalizationState state)> on_localization_pose;
};

}  // namespace glim
