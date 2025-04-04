#include "controller_worker.hpp"

#include <iostream>
#include <vector>

ControllerNode::ControllerNode()
 : Node("controller_node"),
   state_(new fdcl::state_t()),
   command_(new fdcl::command_t()),
   thread_running_(true),
   fdcl_controller_(state_, command_)
{
  // Subscriptions
  sbus_subscription_ = this->create_subscription<sbus_interfaces::msg::SbusSignal>("sbus_signal", 1, std::bind(&ControllerNode::sbusCallback, this, std::placeholders::_1));
  optitrack_mea_subscription_ = this->create_subscription<mocap_interfaces::msg::MocapMeasured>("optitrack_mea", 1, std::bind(&ControllerNode::optitrackCallback, this, std::placeholders::_1));
  imu_mea_subscription_ = this->create_subscription<imu_interfaces::msg::ImuMeasured>("imu_mea", 1, std::bind(&ControllerNode::imuCallback, this, std::placeholders::_1));

  // Publishers
  controller_publisher_ = this->create_publisher<controller_interfaces::msg::ControllerOutput>("controller_output", 1);
  heartbeat_publisher_  = this->create_publisher<watchdog_interfaces::msg::NodeState>("controller_state", 1);
  debug_val_publisher_  = this->create_publisher<controller_interfaces::msg::ControllerDebugVal>("controller_info", 1);

  // Timers
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::debugging_timer_callback, this));

  // main-tasking thread
  controller_thread_ = std::thread(&ControllerNode::controller_loop, this);
}

void ControllerNode::controller_timer_callback() {
  state_->x << x_[0], x_[1], x_[2];
  state_->v << v_[0], v_[1], v_[2];
  state_->a << a_[0], a_[1], a_[2];
  state_->R = R_;
  state_->W << w_[0], w_[1], w_[2];

  // command_->xd << ref_[0], ref_[1], ref_[2];
  command_->xd << 0.0, 0.0, -1.0;
  command_->xd_dot.setZero();
  command_->xd_2dot.setZero();
  command_->xd_3dot.setZero();
  command_->xd_4dot.setZero();

  // command_->b1d << std::cos(ref_[3]), std::sin(ref_[3]), 0.0;
  command_->b1d << 1.0, 0.0, 0.0;
  command_->b1d_dot.setZero();
  command_->b1d_ddot.setZero();

  fdcl_controller_.position_control();
  fdcl_controller_.output_fM(f_out, M_out);

  //----------- Publsih -----------
  controller_interfaces::msg::ControllerOutput msg;
  msg.force = f_out;
  // msg.force = 30.0;
  msg.moment = {M_out[0], M_out[1], M_out[2]};
  controller_publisher_->publish(msg);
}

void ControllerNode::sbusCallback(const sbus_interfaces::msg::SbusSignal::SharedPtr msg) {
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
  ref_[0] = static_cast<double>(sbus_chnl_[0] - 1024) / 672.;  // [-1, 1]
  ref_[1] = static_cast<double>(sbus_chnl_[1] - 1024) / 672.;  // [-1, 1]
  double ref_yaw = static_cast<double>(sbus_chnl_[2] - 1024) / 672.;  // [-1, 1]
  ref_[2] = static_cast<double>(sbus_chnl_[3] -  352) / 1344.; // [ 0, 1]

  ref_[3] += ref_yaw * 0.005; // [rad], this clamps to (2-PI, PI)
  ref_[3] = fmod(ref_[3] + M_PI, two_PI);
  if (ref_[3] < 0) {ref_[3] += two_PI;}
  ref_[3] -= M_PI;

  ref_[2] = ref_[2] * 1.5; // scale
}

void ControllerNode::optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg) {
  x_[0] = -msg->pos[0]; x_[1] = -msg->pos[1]; x_[2] = -msg->pos[2];
  v_[0] = -msg->vel[0]; v_[1] = -msg->vel[1]; v_[2] = -msg->vel[2];
  a_[0] = msg->acc[0]; a_[1] = msg->acc[1]; a_[2] = -msg->acc[2];
}

void ControllerNode::imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg) {
  // quat -> rot matrix
  const double w = msg->q[0];
  const double x = msg->q[1];
  const double y = msg->q[2];
  const double z = msg->q[3];
  
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;
  
  R_(0,0) = 1.0 - 2.0 * (yy + zz);
  R_(0,1) = 2.0 * (xy - wz);
  R_(0,2) = 2.0 * (xz + wy);
  R_(1,0) = 2.0 * (xy + wz);
  R_(1,1) = 1.0 - 2.0 * (xx + zz);
  R_(1,2) = 2.0 * (yz - wx);
  R_(2,0) = 2.0 * (xz - wy);
  R_(2,1) = 2.0 * (yz + wx);
  R_(2,2) = 1.0 - 2.0 * (xx + yy);

  // std::ostringstream oss;
  // oss << "R:\n" 
  //     << state_->R(0,0) << " " << state_->R(0,1) << " " << state_->R(0,2) << "\n"
  //     << state_->R(1,0) << " " << state_->R(1,1) << " " << state_->R(1,2) << "\n"
  //     << state_->R(2,0) << " " << state_->R(2,1) << " " << state_->R(2,2);
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "%s", oss.str().c_str());

  // gyro
  w_[0] = msg->w[0]; w_[1] = -msg->w[1]; w_[2] = msg->w[2];
}

void ControllerNode::heartbeat_timer_callback() {
  heartbeat_state_++;
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;
  heartbeat_publisher_->publish(state_msg);
}

void ControllerNode::debugging_timer_callback() {
  controller_interfaces::msg::ControllerDebugVal info_msg;

  // for (int i = 0; i < 9; i++) {info_msg.sbus_chnl[i] = sbus_chnl_[i];}
  // for (int i = 0; i < 4; i++) {info_msg.des_pos[i] = sbus_ref_[i];}

  // if constexpr (M == ControlMode::POS) {
  // info_msg.pid_mx[0] = pid_midval_roll_[0];
  // info_msg.pid_mx[1] = pid_midval_roll_[1];
  // info_msg.pid_mx[2] = pid_midval_roll_[2];
  // info_msg.pid_mx[3] = pid_midval_roll_[3];

  // info_msg.pid_my[0] = pid_midval_pitch_[0];
  // info_msg.pid_my[1] = pid_midval_pitch_[1];
  // info_msg.pid_my[2] = pid_midval_pitch_[2];
  // info_msg.pid_my[3] = pid_midval_pitch_[3];
  // }
  // else if constexpr (M == ControlMode::VEL) {
  //   info_msg.pid_mx[1] = pid_midval_roll_[0];
  //   info_msg.pid_mx[2] = pid_midval_roll_[1];
  //   info_msg.pid_mx[3] = pid_midval_roll_[2];

  //   info_msg.pid_my[1] = pid_midval_pitch_[0];
  //   info_msg.pid_my[2] = pid_midval_pitch_[1];
  //   info_msg.pid_my[3] = pid_midval_pitch_[2];
  // }
  // else if constexpr (M == ControlMode::ATTITUDE) {
  //   info_msg.pid_mx[2] = pid_midval_roll_[0];
  //   info_msg.pid_mx[3] = pid_midval_roll_[1];

  //   info_msg.pid_my[2] = pid_midval_pitch_[0];
  //   info_msg.pid_my[3] = pid_midval_pitch_[1];
  // }

  // info_msg.pid_mz[0] = pid_midval_yaw_[0];
  // info_msg.pid_mz[1] = pid_midval_yaw_[1];

  // info_msg.pid_f[0] = pid_midval_z_[0];
  // info_msg.pid_f[1] = pid_midval_z_[1];

  // for (int i = 0; i < 2; i++) {
  //   info_msg.imu_roll[i]  = imu_roll_[i];
  //   info_msg.imu_pitch[i] = imu_pitch_[i];
  //   info_msg.imu_yaw[i]   = imu_yaw_[i];
  //   info_msg.opti_x[i]    = opti_x_[i];
  //   info_msg.opti_y[i]    = opti_y_[i];
  //   info_msg.opti_z[i]    = opti_z_[i];
  // }

  debug_val_publisher_->publish(info_msg);
}

void ControllerNode::controller_loop() {
  constexpr auto period = std::chrono::microseconds(Loop_us);
  auto next_time = std::chrono::steady_clock::now() + period;

  while (rclcpp::ok() && thread_running_) {
    controller_timer_callback();
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_until(next_time);
    next_time += period;
  }
}

ControllerNode::~ControllerNode() {
  thread_running_ = false;
  if (controller_thread_.joinable()) {
    controller_thread_.join();
  }

  delete state_;
  delete command_;
}

// Main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();

  // wait forever until signal (Ctrl+C)
  rclcpp::on_shutdown([&]() {
    RCLCPP_INFO(node->get_logger(), "Shutdown signal received");
  });

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
