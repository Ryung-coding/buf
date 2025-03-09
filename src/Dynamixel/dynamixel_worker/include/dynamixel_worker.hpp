#ifndef DYNAMIXEL_WORKER_HPP
#define DYNAMIXEL_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "allocator_interfaces/msg/joint_val.hpp"

constexpr double PI = 3.1415926535897932384626433832706;
constexpr double rad2ppr = 2048.0 / PI;
constexpr double ppr2rad = PI / 2048.0;

class DynamixelNode : public rclcpp::Node {
public:
  DynamixelNode();

private:
  // Callback to handle received JointVal messages
  void allocatorCallback(const allocator_interfaces::msg::JointVal::SharedPtr msg);
  void heartbeat_timer_callback();

  // ROS2 subscriber for joint_val topic
  rclcpp::Subscription<allocator_interfaces::msg::JointVal>::SharedPtr joint_val_subscriber_;
  rclcpp::Publisher<allocator_interfaces::msg::JointVal>::SharedPtr joint_val_publisher_;

  // Publisher & Timer for Heartbeat signal
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  double arm_1_rad[5] = {0., 0., 0., 0., 0.};
  double arm_2_rad[5] = {0., 0., 0., 0., 0.};
  double arm_3_rad[5] = {0., 0., 0., 0., 0.};
  double arm_4_rad[5] = {0., 0., 0., 0., 0.};

  int16_t arm_1_ppr[5] = {0, 0, 0, 0, 0};
  int16_t arm_2_ppr[5] = {0, 0, 0, 0, 0};
  int16_t arm_3_ppr[5] = {0, 0, 0, 0, 0};
  int16_t arm_4_ppr[5] = {0, 0, 0, 0, 0};

  double arm_1_mea[5] = {0., 0., 0., 0., 0.};
  double arm_2_mea[5] = {0., 0., 0., 0., 0.};
  double arm_3_mea[5] = {0., 0., 0., 0., 0.};
  double arm_4_mea[5] = {0., 0., 0., 0., 0.};

  uint8_t heartbeat_state_; // previous node state
};

#endif // DYNAMIXEL_WORKER_HPP
