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

  state_->x << 0.0, 0.0, 0.0;
  state_->v.setZero();
  state_->a.setZero();
  state_->R.setIdentity();
  state_->W.setZero();

  command_->xd.setZero();
  command_->xd_dot.setZero();
  command_->xd_2dot.setZero();
  command_->xd_3dot.setZero();
  command_->xd_4dot.setZero();
  command_->b1d << 1.0, 0.0, 0.0;
  command_->b1d_dot.setZero();
  command_->b1d_ddot.setZero();

  fdcl_controller_.position_control();
  fdcl_controller_.output_fM(f_out, M_out);

  //----------- Publsih -----------
  controller_interfaces::msg::ControllerOutput msg;
  msg.force = f_out;
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
  double ref_r = static_cast<double>(sbus_chnl_[0] - 1024) / 672.;  // [-1, 1]
  double ref_p = static_cast<double>(sbus_chnl_[1] - 1024) / 672.;  // [-1, 1]
  double ref_y = static_cast<double>(sbus_chnl_[2] - 1024) / 672.;  // [-1, 1]
  double ref_z = static_cast<double>(sbus_chnl_[3] -  352) / 1344.; // [ 0, 1]
  
  sbus_ref_[0] += ref_r * 1.0; // [m]
  sbus_ref_[1] += ref_p * 1.0; // [m]

  sbus_ref_[2] += ref_y * 0.005; // [rad], this clamps to (2-PI, PI)
  sbus_ref_[2] = fmod(sbus_ref_[2] + M_PI, two_PI);
  if (sbus_ref_[2] < 0) {sbus_ref_[2] += two_PI;}
  sbus_ref_[2] -= M_PI;

  sbus_ref_[3] =  ref_z * 1.5;  // [m]
}

void ControllerNode::optitrackCallback(const mocap_interfaces::msg::MocapMeasured::SharedPtr msg) {
  opti_x_[0] = msg->pos[0]; opti_x_[1] = msg->vel[0];
  opti_y_[0] = msg->pos[1]; opti_y_[1] = msg->vel[1];
  opti_z_[0] = msg->pos[2]; opti_z_[1] = msg->vel[2];
}

void ControllerNode::imuCallback(const imu_interfaces::msg::ImuMeasured::SharedPtr msg) {
  imu_roll_[0]  = msg->q[0];    imu_roll_[1]  = msg->qdot[0];
  imu_pitch_[0] = msg->q[1];    imu_pitch_[1] = msg->qdot[1];
  imu_yaw_[0]   = msg->q[2];    imu_yaw_[1]   = msg->qdot[2];
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
