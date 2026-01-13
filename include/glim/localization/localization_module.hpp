#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <glim/localization/localization_graph.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class LocalizationModule {
public:
  LocalizationModule();
  ~LocalizationModule();

  void start();
  void stop();

  void insert_frame(const EstimationFrame::ConstPtr& frame);
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  void spin();

private:
  void run();

  std::atomic_bool running;
  std::atomic_bool kill_switch;
  std::thread worker_thread;

  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;

  std::shared_ptr<LocalizationGraph> localization_graph;
  std::shared_ptr<spdlog::logger> logger;

  int imu_callback_id;
  int frame_callback_id;
};

}  // namespace glim
