#include <glim/localization/localization_module.hpp>

#include <chrono>

#include <glim/localization/localization_callbacks.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <spdlog/spdlog.h>

namespace glim {

LocalizationModule::LocalizationModule()
: running(false),
  kill_switch(false),
  localization_graph(std::make_shared<LocalizationGraph>()),
  logger(create_module_logger("localization")),
  imu_callback_id(-1),
  frame_callback_id(-1) {}

LocalizationModule::~LocalizationModule() {
  stop();
}

void LocalizationModule::start() {
  if (running) {
    return;
  }

  kill_switch = false;
  running = true;

  imu_callback_id = OdometryEstimationCallbacks::on_insert_imu.add(
    [this](const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
      insert_imu(stamp, linear_acc, angular_vel);
    });
  frame_callback_id = OdometryEstimationCallbacks::on_update_new_frame.add(
    [this](const EstimationFrame::ConstPtr& frame) { insert_frame(frame); });

  worker_thread = std::thread([this] { run(); });
}

void LocalizationModule::stop() {
  if (!running) {
    return;
  }

  kill_switch = true;
  running = false;

  if (worker_thread.joinable()) {
    worker_thread.join();
  }

  if (imu_callback_id >= 0) {
    OdometryEstimationCallbacks::on_insert_imu.remove(imu_callback_id);
    imu_callback_id = -1;
  }

  if (frame_callback_id >= 0) {
    OdometryEstimationCallbacks::on_update_new_frame.remove(frame_callback_id);
    frame_callback_id = -1;
  }
}

void LocalizationModule::insert_frame(const EstimationFrame::ConstPtr& frame) {
  input_frame_queue.push_back(frame);
}

void LocalizationModule::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Eigen::Matrix<double, 7, 1> imu_data;
  imu_data << stamp, linear_acc, angular_vel;
  input_imu_queue.push_back(imu_data);
}

void LocalizationModule::spin() {
  auto imu_frames = input_imu_queue.get_all_and_clear();
  for (const auto& imu : imu_frames) {
    const double stamp = imu[0];
    const Eigen::Vector3d linear_acc = imu.block<3, 1>(1, 0);
    const Eigen::Vector3d angular_vel = imu.block<3, 1>(4, 0);
    localization_graph->insert_imu(stamp, linear_acc, angular_vel);
  }

  auto frames = input_frame_queue.get_all_and_clear();
  for (const auto& frame : frames) {
    localization_graph->insert_frame(frame);
  }

  auto result = localization_graph->take_result();
  if (result) {
    LocalizationCallbacks::on_localization_pose(result->pose, result->state);
  }
}

void LocalizationModule::run() {
  while (!kill_switch) {
    if (input_imu_queue.empty() && input_frame_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    spin();
  }
}

}  // namespace glim
