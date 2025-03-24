#ifndef ARM_CHANGER_WORKER_HPP
#define ARM_CHANGER_WORKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "sbus_interfaces/msg/kill_cmd.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include <array>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

constexpr double PI = 3.1415926535897932384626433832706;

class ArmChangerWorker : public rclcpp::Node {
public:
  ArmChangerWorker();
  ~ArmChangerWorker() = default;

private:
  // Callback to handle received PwmVal messages
  void sbus_callback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void killCmd_callback(const sbus_interfaces::msg::KillCmd::SharedPtr msg);
  void watchdog_callback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
  void compute_ik(const double x, const double y, const double z, const Eigen::Vector3d &heading);
  void publishJointVal();

  // Publisher
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_publisher_;
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;

  // ROS2 subscriber for pwm_val topic
  rclcpp::Subscription<sbus_interfaces::msg::KillCmd>::SharedPtr killcmd_subscription_;
  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;

  // DH params
  const double a1_ = 134.;
  const double a2_ = 115.;
  const double a3_ = 110.;
  const double a4_ = 24.;
  const double a5_ = 104.;

  // workspace constrain
  double x_min_; 
  double x_max_;
  double y_fixed_;
  double z_min_;
  double z_max_;

  const Eigen::Vector3d heading_fixed_ = Eigen::Vector3d(0.0, 0.0, 1.0); // only z-up

  // Latest Joint values
  double th1_ = 0.0;       // [rad]
  double th2_ = PI/4.0;    // [rad]
  double th3_ = -PI/2.0;   // [rad]
  double th4_ = 3*PI /4.0; // [rad]
  double th5_ = 0.0;       // [rad]

  Eigen::VectorXd a1_q, a2_q, a3_q, a4_q;

  // Watchdog state
  uint8_t watchdog_state_ = 1; // default(normal) is 1.
  bool kill_activated_ = true;
};

#endif // ARM_CHANGER_WORKER_HPP
