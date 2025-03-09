#ifndef TEENSY_WORKER_HPP
#define TEENSY_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include "allocator_interfaces/msg/pwm_val.hpp"
#include "sbus_interfaces/msg/kill_cmd.hpp"
#include "mujoco_interfaces/msg/motor_thrust.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"

#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>


class TeensyNode : public rclcpp::Node {
public:
  TeensyNode();

  ~TeensyNode();

private:
  // Callback to handle received PwmVal messages
  void allocatorCallback(const allocator_interfaces::msg::PwmVal::SharedPtr msg);
  void KillCmdCallback(const sbus_interfaces::msg::KillCmd::SharedPtr msg);
  void watchdogCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);

  // Publisher
  void MJC_publish(const double pwm1, const double pwm2, const double pwm3, const double pwm4);

  // ROS2 subscriber for pwm_val topic
  rclcpp::Subscription<allocator_interfaces::msg::PwmVal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::KillCmd>::SharedPtr killcmd_subscription_;
  rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr watchdog_subscription_;

  // ROS2 publisher for mujoco
  rclcpp::Publisher<mujoco_interfaces::msg::MotorThrust>::SharedPtr mujoco_publisher_;

  int sock_;                  // SocketCAN socket file descriptor.
  struct sockaddr_can addr_;  // CAN interface address structure.

  // Coefficients for conversions
  const double LPF_alpha_ = 0.5; // Low-pass filter coefficient (0 < alpha_ < 1)
  const double LPF_beta_ = 1.0 - LPF_alpha_; // Low-pass filter coefficient
  const double K1_ = 8568.19;
  const double b_ = 331.7;
  const double n_pwm_ = 0.67;
  const double C_T_ = 2.6242 / 1000000.0;
  const double n_thrust_ = 1.8555;
  const double C1_tau_ = 1.5958 / 100000000.0;
  const double C2_tau_ = 0.0;
  const double C3_tau_ = 0.0;

  // Latest PWM values
  double pwm1_ = 0.0; // [0, 1]
  double pwm2_ = 0.0; // [0, 1]
  double pwm3_ = 0.0; // [0, 1]
  double pwm4_ = 0.0; // [0, 1]

  // Latest RPM values
  double rpm1_ = 0.0; // [rpm]
  double rpm2_ = 0.0; // [rpm]
  double rpm3_ = 0.0; // [rpm]
  double rpm4_ = 0.0; // [rpm]
  
  // Latest Force values
  double f1_ = 0.0; // [N]
  double f2_ = 0.0; // [N]
  double f3_ = 0.0; // [N]
  double f4_ = 0.0; // [N]

  // Latest Moment values
  double m1_ = 0.0; // [Nm]
  double m2_ = 0.0; // [Nm]
  double m3_ = 0.0; // [Nm]
  double m4_ = 0.0; // [Nm]

  // Watchdog state
  uint8_t watchdog_state_ = 1; // default(normal) is 1.
  bool kill_activated_ = false;
};

#endif // TEENSY_WORKER_HPP
