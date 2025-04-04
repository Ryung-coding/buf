#ifndef CONTROLLER_WORKER_HPP
#define CONTROLLER_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <algorithm>
#include <cstddef>

#include "sbus_interfaces/msg/sbus_signal.hpp"
#include "controller_interfaces/msg/controller_output.hpp"
#include "controller_interfaces/msg/controller_debug_val.hpp"
#include "mocap_interfaces/msg/mocap_measured.hpp"
#include "imu_interfaces/msg/imu_measured.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"

#include "fdcl/control.hpp"
#include "controller_param.h"

#define Loop_us 500 // controller thread loop dt [us]

constexpr double two_PI = 2.0 * M_PI;
constexpr double minus_PI = -M_PI;
inline constexpr double dt = Loop_us / 1000000.0; // [sec]

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  ~ControllerNode();

private:
  fdcl::state_t * state_;
  fdcl::command_t * command_;

  std::atomic<bool> thread_running_;
  std::thread controller_thread_;

  fdcl::control fdcl_controller_;

  double f_out;
  Vector3 M_out;

  void sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg);
  void imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg);
  void controller_timer_callback();
  void heartbeat_timer_callback();
  void debugging_timer_callback();
  void controller_loop();

  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<mocap_interfaces::msg::MocapMeasured>::SharedPtr optitrack_mea_subscription_;
  rclcpp::Subscription<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_mea_subscription_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_publisher_;
  
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerDebugVal>::SharedPtr debug_val_publisher_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  // sbus state
  int   sbus_chnl_[9] = {1024, 1024, 352, 1024, 352, 352, 352, 352, 352};
  double ref_[4] = {0.0, 0.0, 0.0, 0.0}; // [m, m, m, rad]

  double x_[3] = {0.0, 0.0, 0.0}; // [m]
  double v_[3] = {0.0, 0.0, 0.0}; // [m/s]
  double a_[3] = {0.0, 0.0, 0.0}; // [m/s^2]

  Matrix3 R_;
  double w_[3] = {0.0, 0.0, 0.0}; // [rad/s]

  uint8_t heartbeat_state_ = 0;
};

#endif // CONTROLLER_WORKER_HPP
