#ifndef ALLOCATOR_WORKER_HPP
#define ALLOCATOR_WORKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "controller_interfaces/msg/controller_output.hpp"
#include "allocator_interfaces/msg/pwm_val.hpp"
#include "allocator_interfaces/msg/allocator_debug_val.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

using Eigen::Matrix4d;
using Eigen::Vector4d;

class AllocatorWorker : public rclcpp::Node {
public:
  AllocatorWorker();
  ~AllocatorWorker() = default;

private:
  void controllerCallback(const controller_interfaces::msg::ControllerOutput::SharedPtr msg);
  void jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg);
  void heartbeat_timer_callback();
  void debugging_timer_callback();
  void publishJointVal();
  void publishPwmVal();

  // Subscribers
  rclcpp::Subscription<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_subscriber_;
  rclcpp::Subscription<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_subscriber_;
  
  // Publishers
  rclcpp::Publisher<allocator_interfaces::msg::PwmVal>::SharedPtr pwm_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<allocator_interfaces::msg::AllocatorDebugVal>::SharedPtr debug_val_publisher_;

  // Timers for publishing
  rclcpp::TimerBase::SharedPtr pwm_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  // heartbeat state  
  uint8_t  hb_state_;     // current heartbeat value
  bool     hb_enabled_;   // gate flag

  // Time tracking
  size_t buffer_size_ = 1000;           // Size of the moving average window
  std::vector<double> dt_buffer_;      // Circular buffer for dt values
  size_t buffer_index_  = 0;           // Current index in the circular buffer
  double dt_sum_ = 0.0;                // Sum of dt values in the buffer
  double filtered_frequency_ = 1200.0; // [Hz] calculated from average dt
  rclcpp::Time last_callback_time_;    // Timestamp of the last callback

  Vector4d f = Vector4d::Zero(); // PID-control result [N.m N.m N.m N]
  Vector4d u = Vector4d::Zero(); // Allocated result [N N N N]
  Vector4d pwm = Vector4d::Zero(); // Allocated result [pwm pwm pwm pwm]
  Matrix4d A;
  Matrix4d A_inv;
  double pwm_alpha_;
  double pwm_beta_;
  
  double arm_des[4][5] = { // [rad]
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
  };

  // mujoco or dynamixel read
  double arm_mea[4][5] = { // [rad]
    {0., -0.84522, 1.50944, 0.90812, 0.},
    {0., -0.84522, 1.50944, 0.90812, 0.},
    {0., -0.84522, 1.50944, 0.90812, 0.},
    {0., -0.84522, 1.50944, 0.90812, 0.}
  };

  uint8_t heartbeat_state_; // previous node state
};

#endif // ALLOCATOR_WORKER_HPP
