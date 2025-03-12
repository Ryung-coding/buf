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

#include "controller_params.hpp"

constexpr double two_PI = 2.0 * M_PI;
constexpr double minus_PI = -M_PI;

template <std::size_t N>
class cascade_PID {
public:
    cascade_PID(const std::array<double, N>& Kp,
                const std::array<double, N>& Ki,
                const std::array<double, N>& Kd,
                const std::array<double, N>& Sat_gain,
                const std::array<double, N>& lpf_gain,
                double dt);

    double update(double ref,
                  const std::array<double, N>& msr,
                  std::array<double, N>& output);

private:
  double dt;
  std::array<double,N> Kp, Ki, Kd;
  std::array<double,N> Sat_gain;
  std::array<double,N> lpfAlpha;
  std::array<double,N> lpfBeta;

  std::array<double,N> integral;
  std::array<double,N> prev_err;
  std::array<double,N> prev_derivative;
};

// Extern template declarations to avoid link duplication
extern template class cascade_PID<2>;
extern template class cascade_PID<3>;
extern template class cascade_PID<4>;

class heading_PID {
public:
  heading_PID(const std::array<double, 2>& Kp,
              const std::array<double, 2>& Ki,
              const std::array<double, 2>& Kd,
              const std::array<double, 2>& Sat_gain,
              const std::array<double, 2>& lpf_gain,
              double dt);

  double update(double ref,
                const std::array<double, 2>& msr,
                std::array<double, 2>& output);

private:double dt;
  std::array<double,2> Kp, Ki, Kd;
  std::array<double,2> Sat_gain;
  std::array<double,2> lpfAlpha;
  std::array<double,2> lpfBeta;

  std::array<double,2> integral;
  std::array<double,2> prev_err;
  std::array<double,2> prev_derivative;
};

template <ControlMode M>
class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();

private:

  static constexpr std::size_t ROLL_DIM = 
    (M == ControlMode::POS) ? 4 :
    (M == ControlMode::VEL) ? 3 : 
                              2; // (ATT -> 2)

  static constexpr std::size_t PITCH_DIM = 
    (M == ControlMode::POS) ? 4 :
    (M == ControlMode::VEL) ? 3 :
                              2; // (ATT -> 2)

  static constexpr std::size_t YAW_DIM = 2;
  static constexpr std::size_t Z_DIM = 2;

  // (2) Cascade PID members with those dims
  cascade_PID<ROLL_DIM>  pid_roll_;
  cascade_PID<PITCH_DIM> pid_pitch_;
  heading_PID            pid_yaw_;
  cascade_PID<Z_DIM>     pid_z_;

  void sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg);
  void optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg);
  void imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg);
  void controller_timer_callback();
  void heartbeat_timer_callback();
  void debugging_timer_callback();

  rclcpp::Subscription<sbus_interfaces::msg::SbusSignal>::SharedPtr sbus_subscription_;
  rclcpp::Subscription<mocap_interfaces::msg::MocapMeasured>::SharedPtr optitrack_mea_subscription_;
  rclcpp::Subscription<imu_interfaces::msg::ImuMeasured>::SharedPtr imu_mea_subscription_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerOutput>::SharedPtr controller_publisher_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Publisher<controller_interfaces::msg::ControllerDebugVal>::SharedPtr debug_val_publisher_;
  rclcpp::TimerBase::SharedPtr debugging_timer_;

  double weight = 0.0; // drone mg [N]

  // Desired states
  int   sbus_chnl_[9] = {1024, 1024, 352, 1024, 352, 352, 352, 352, 352};
  double sbus_ref_[4] = {0.0, 0.0, 0.0, 0.0};

  // Sensor states
  double imu_roll_[2]  = {0.0, 0.0}; // [rad, rad/s]
  double imu_pitch_[2] = {0.0, 0.0}; // [rad, rad/s]
  double imu_yaw_[2]   = {0.0, 0.0}; // [rad, rad/s]
  double opti_x_[2]    = {0.0, 0.0}; // [m, m/s]
  double opti_y_[2]    = {0.0, 0.0}; // [m, m/s]
  double opti_z_[2]    = {0.0, 0.0}; // [m, m/s]

  // For debugging mid-values
  double pid_midval_roll_[4]  = {0.0, 0.0, 0.0, 0.0}; // [m, m/s, rad, rad/s]
  double pid_midval_pitch_[4] = {0.0, 0.0, 0.0, 0.0}; // [m, m/s, rad, rad/s]
  double pid_midval_yaw_[2]   = {0.0, 0.0}; // [rad, rad/s]
  double pid_midval_z_[2]     = {0.0, 0.0}; // [m, m/s]

  uint8_t heartbeat_state_ = 0;
};

#endif // CONTROLLER_WORKER_HPP
