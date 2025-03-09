#include "controller_worker.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

template <std::size_t N>
cascade_PID<N>::cascade_PID(const std::array<double, N>& Kp,
                            const std::array<double, N>& Ki,
                            const std::array<double, N>& Kd,
                            const std::array<double, N>& Sat_gain,
                            const std::array<double, N>& lpf_gain,
                            double dt)
  : dt(dt), Kp(Kp), Ki(Ki), Kd(Kd), Sat_gain(Sat_gain),
    integral{}, prev_err{}, prev_derivative{}
{
  for (std::size_t i = 0; i < N; i++) {
    lpfAlpha[i] = lpf_gain[i];
    lpfBeta[i]  = 1.0 - lpf_gain[i];
  }
}

template <std::size_t N>
double cascade_PID<N>::update(double ref, const std::array<double, N>& msr, std::array<double, N>& output) {

  double error = ref - msr[0]; // First axis error: ref - msr[0]

  for (std::size_t i = 0; i < N; i++) {
    if (i > 0) {error = output[i-1] - msr[i];}

    // Integral
    integral[i] += error * dt;
    integral[i] = std::clamp(integral[i], -Sat_gain[i], Sat_gain[i]);

    // Derivative
    double dRaw = (error - prev_err[i]) / dt;
    double derivative = (lpfAlpha[i] * dRaw) + (lpfBeta[i] * prev_derivative[i]);

    // Sum
    output[i] = (Kp[i] * error) + (Ki[i] * integral[i]) + (Kd[i] * derivative);

    // Update states
    prev_err[i] = error;
    prev_derivative[i] = derivative;
  }

  return output[N-1]; // returns final output
}

// Explicit template instantiations
template class cascade_PID<2>;
template class cascade_PID<3>;
template class cascade_PID<4>;


//--------------------------------------
// ControllerNode Implementation
//--------------------------------------
template <ControlMode M>
ControllerNode<M>::ControllerNode() : Node("controller_node"),
   pid_roll_(  Kp_r, Ki_r, Kd_r, Sat_gain_r, lpf_gain_r, dt ),
   pid_pitch_( Kp_p, Ki_p, Kd_p, Sat_gain_p, lpf_gain_p, dt ),
   pid_yaw_(   Kp_y, Ki_y, Kd_y, Sat_gain_y, lpf_gain_y, dt ),
   pid_z_(     Kp_z, Ki_z, Kd_z, Sat_gain_z, lpf_gain_z, dt )
{
  // Drone total mass * gravity example
  weight = 9.80665 * 4.5; // ~49.03325 N

  // Subscriptions
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("sbus_signal", 1, std::bind(&ControllerNode::sbusCallback, this, std::placeholders::_1));
  optitrack_mea_subscription_ = this->create_subscription<mocap_interfaces::msg::MocapMeasured>("optitrack_mea", 1, std::bind(&ControllerNode::optitrackCallback, this, std::placeholders::_1));
  imu_mea_subscription_ = this->create_subscription<imu_interfaces::msg::ImuMeasured>("imu_mea", 1, std::bind(&ControllerNode::imuCallback, this, std::placeholders::_1));

  // Publishers
  controller_publisher_ = this->create_publisher<controller_interfaces::msg::ControllerOutput>("controller_output", 1);
  heartbeat_publisher_  = this->create_publisher<watchdog_interfaces::msg::NodeState>("controller_state", 1);
  debug_val_publisher_  = this->create_publisher<controller_interfaces::msg::ControllerDebugVal>("controller_info", 1);

  // Timers

  controller_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ControllerNode::controller_timer_callback, this));
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::debugging_timer_callback, this));

  // Initialize times
  current_callback_time_ = this->now();
  last_callback_time_    = this->now();
}

template <ControlMode M>
void ControllerNode<M>::controller_timer_callback() {
  //-------- Loop Time Calculate --------
  current_callback_time_ = this->now();
  current_dt = (current_callback_time_ - last_callback_time_).seconds();
  if (current_dt > 0.0) {filtered_frequency_ = 0.05 * (1.0 / current_dt) + 0.95 * filtered_frequency_;}
  last_callback_time_ = current_callback_time_;

  //--------------- ROLL ---------------
  std::array<double,ROLL_DIM> msrRoll{};  
  msrRoll[0] = imu_roll_[1]; // [rad/s]
  if constexpr (ROLL_DIM > 1) msrRoll[1] = imu_roll_[0]; // [rad]
  if constexpr (ROLL_DIM > 2) msrRoll[2] = opti_x_[1];   // [m/s]
  if constexpr (ROLL_DIM > 3) msrRoll[3] = opti_x_[0];   // [m]

  std::array<double,ROLL_DIM> outRoll{};
  double Mx = pid_roll_.update(sbus_ref_[0], msrRoll, outRoll);

  for (std::size_t i=0; i<ROLL_DIM; i++) {pid_midval_roll_[i] = outRoll[i];}

  //-------------- PITCH --------------
  std::array<double,PITCH_DIM> msrPitch{};
  msrPitch[0] = imu_pitch_[1]; // [rad/s]
  if constexpr (PITCH_DIM > 1) msrPitch[1] = imu_pitch_[0]; // [rad]
  if constexpr (PITCH_DIM > 2) msrPitch[2] = opti_y_[1];    // [m/s]
  if constexpr (PITCH_DIM > 3) msrPitch[3] = opti_y_[0];    // [m]

  std::array<double,PITCH_DIM> outPitch{};
  double My = pid_pitch_.update(sbus_ref_[1], msrPitch, outPitch);

  for (std::size_t i=0; i<PITCH_DIM; i++){pid_midval_pitch_[i] = outPitch[i];}

  //-------------- YAW --------------
  std::array<double,YAW_DIM> msrYaw{};
  msrYaw[0] = imu_yaw_[0]; // [rad]
  msrYaw[1] = imu_yaw_[1]; // [rad/s]
  std::array<double,YAW_DIM> outYaw{};
  double Mz = pid_yaw_.update(sbus_ref_[2], msrYaw, outYaw);

  for (int i=0; i<2; i++) {pid_midval_yaw_[i] = outYaw[i];}

  //-------------- Z --------------
  std::array<double,Z_DIM> msrZ{};
  msrZ[0] = opti_z_[0]; // [m]
  msrZ[1] = opti_z_[1]; // [m/s]
  std::array<double,Z_DIM> outZ{};
  double F = pid_z_.update(sbus_ref_[3], msrZ, outZ) + weight;

  for (int i=0; i<2; i++) {pid_midval_z_[i] = outZ[i];}

  //----------- Publsih -----------
  controller_interfaces::msg::ControllerOutput msg;
  msg.force = F;
  msg.moment = {Mx, My, Mz};
  controller_publisher_->publish(msg);
}

template <ControlMode M>
void ControllerNode<M>::sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {
  // save sbus_channel
  sbus_chnl_[0] = msg->ch[1];  // x command
  sbus_chnl_[1] = msg->ch[0];  // y command
  sbus_chnl_[2] = msg->ch[3];  // heading command
  sbus_chnl_[3] = msg->ch[2];  // z command
  sbus_chnl_[4] = msg->ch[9];  // kill command
  sbus_chnl_[5] = msg->ch[8];  // toggle E
  sbus_chnl_[6] = msg->ch[7];  // toggle G
  sbus_chnl_[7] = msg->ch[10]; // left-dial
  sbus_chnl_[8] = msg->ch[11]; // right-dial
  
  // remap SBUS data to double
  double ref_r = static_cast<double>(sbus_chnl_[0] - 1024) / 672.;  // [-1, 1]
  double ref_p = static_cast<double>(sbus_chnl_[1] - 1024) / 672.;  // [-1, 1]
  double ref_y = static_cast<double>(sbus_chnl_[2] - 1024) / 672.;  // [-1, 1]
  double ref_z = static_cast<double>(sbus_chnl_[3] -  352) / 1344.; // [ 0, 1]
  
  if constexpr (M == ControlMode::POS) {
    sbus_ref_[0] += ref_r * 0.01; // [m]
    sbus_ref_[1] += ref_p * 0.01; // [m]
    sbus_ref_[2] += ref_y * 0.01; // [rad]
    sbus_ref_[3] =  ref_z * 1.0;    // [m]
  }
  else if constexpr (M == ControlMode::VEL) {
    sbus_ref_[0] =  ref_r * 0.1;    // [m/s]
    sbus_ref_[1] =  ref_p * 0.1;    // [m/s]
    sbus_ref_[2] += ref_y * 0.0001; // [rad]
    sbus_ref_[3] =  ref_z * 1.0;    // [m]
  }
  else if constexpr (M == ControlMode::ATTITUDE) {
    sbus_ref_[0] =  ref_r * 1.0;   // [rad]
    sbus_ref_[1] =  ref_p * 1.0;   // [rad]
    sbus_ref_[2] += ref_y * 0.01; // [rad]
    sbus_ref_[3] =  ref_z * 1.5;    // [m]
  }
  else {sbus_ref_[0] = 0.0; sbus_ref_[1] = 0.0; sbus_ref_[2] = 0.0; sbus_ref_[3] = 0.0;}
}

template <ControlMode M>
void ControllerNode<M>::optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg) {
  opti_x_[0] = msg->pos[0]; opti_x_[1] = msg->vel[0];
  opti_y_[0] = msg->pos[1]; opti_y_[1] = msg->vel[1];
  opti_z_[0] = msg->pos[2]; opti_z_[1] = msg->vel[2];
}

template <ControlMode M>
void ControllerNode<M>::imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg) {
  imu_roll_[0]  = msg->q[0];    imu_roll_[1]  = msg->qdot[0];
  imu_pitch_[0] = msg->q[1];    imu_pitch_[1] = msg->qdot[1];
  imu_yaw_[0]   = msg->q[2];    imu_yaw_[1]   = msg->qdot[2];
}

template <ControlMode M>
void ControllerNode<M>::heartbeat_timer_callback() {
  heartbeat_state_++;
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;
  heartbeat_publisher_->publish(state_msg);
}

template <ControlMode M>
void ControllerNode<M>::debugging_timer_callback() {
  controller_interfaces::msg::ControllerDebugVal info_msg;

  for (int i = 0; i < 9; i++) {info_msg.sbus_chnl[i] = sbus_chnl_[i];}
  for (int i = 0; i < 4; i++) {info_msg.des_pos[i] = sbus_ref_[i];}

  info_msg.pid_mx[0] = pid_midval_roll_[0];
  info_msg.pid_mx[1] = pid_midval_roll_[1];
  info_msg.pid_mx[2] = pid_midval_roll_[2];
  info_msg.pid_mx[3] = pid_midval_roll_[3];

  info_msg.pid_my[0] = pid_midval_pitch_[0];
  info_msg.pid_my[1] = pid_midval_pitch_[1];
  info_msg.pid_my[2] = pid_midval_pitch_[2];
  info_msg.pid_my[3] = pid_midval_pitch_[3];

  info_msg.pid_mz[0] = pid_midval_yaw_[0];
  info_msg.pid_mz[1] = pid_midval_yaw_[1];

  info_msg.pid_f[0] = pid_midval_z_[0];
  info_msg.pid_f[1] = pid_midval_z_[1];

  for (int i = 0; i < 2; i++) {
    info_msg.imu_roll[i]  = imu_roll_[i];
    info_msg.imu_pitch[i] = imu_pitch_[i];
    info_msg.imu_yaw[i]   = imu_yaw_[i];
    info_msg.opti_x[i]    = opti_x_[i];
    info_msg.opti_y[i]    = opti_y_[i];
    info_msg.opti_z[i]    = opti_z_[i];
  }

  debug_val_publisher_->publish(info_msg);
}

template class ControllerNode<ControlMode::POS>;
template class ControllerNode<ControlMode::VEL>;
template class ControllerNode<ControlMode::ATTITUDE>;

// Main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode<CONTROL_MODE>>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
