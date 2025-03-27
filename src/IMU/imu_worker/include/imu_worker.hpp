#ifndef IMU_WORKER_HPP
#define IMU_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "imu_interfaces/msg/imu_measured.hpp"
#include "mujoco_interfaces/msg/mu_jo_co_meas.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include <chrono>
#include <deque>
#include <functional>

constexpr double two_PI = 2.0 * M_PI;

struct DelayedData
{
  rclcpp::Time stamp;
  std::array<double, 3> q;
  std::array<double, 3> qdot;
};

class IMUnode : public rclcpp::Node {
public:
  IMUnode();

private:
  void PublishMuJoCoMeasurement();
  void heartbeat_timer_callback();
  void mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Subscriber
  rclcpp::Subscription<mujoco_interfaces::msg::MuJoCoMeas>::SharedPtr mujoco_subscription_;

  // Buffer (FIFO) to store data for delayed output
  std::deque<DelayedData> data_buffer_;

  // Duration representing 3ms delay (3,000,000ns)
  rclcpp::Duration delay_{0, 3000000};

  // Heartbeat
  uint8_t heartbeat_state_;
};

#endif // IMU_WORKER_HPP