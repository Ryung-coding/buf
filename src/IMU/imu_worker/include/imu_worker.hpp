#ifndef IMU_WORKER_HPP
#define IMU_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "imu_interfaces/msg/imu_measured.hpp"
#include "mujoco_interfaces/msg/mu_jo_co_meas.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include <chrono>
#include <functional>

class IMUnode : public rclcpp::Node {
public:
  IMUnode();

private:
  void PublishMeasurement();
  void heartbeat_timer_callback();
  void mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg);

  rclcpp::Publisher<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
    
  // Publisher&Timer for Heartbeat signal
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // MuJoCo Subscriber
  rclcpp::Subscription<mujoco_interfaces::msg::MuJoCoMeas>::SharedPtr mujoco_subscription_;

  double alpha_;
  double beta_;

  std::array<double, 3> q_filtered_;
  std::array<double, 3> qdot_filtered_;

 uint8_t heartbeat_state_;  // previous node state
};

#endif // IMU_WORKER_HPP