#include "teensy_worker.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <cstring>
#include <unistd.h>
#include <algorithm>

using namespace std::chrono_literals;

TeensyNode::TeensyNode() : Node("teensy_node") {
  // Subscription
  killcmd_subscription_ = this->create_subscription<sbus_interfaces::msg::KillCmd>("sbus_kill", 1, std::bind(&TeensyNode::KillCmdCallback, this, std::placeholders::_1));
  watchdog_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>("watchdog_state", 1, std::bind(&TeensyNode::watchdogCallback, this, std::placeholders::_1));

  this->declare_parameter<std::string>("mode", "None");
  this->get_parameter("mode", mode_);  // store into member

  if (mode_ == "real"){
    allocator_subscription_ = this->create_subscription<allocator_interfaces::msg::PwmVal>("motor_cmd", 1, std::bind(&TeensyNode::allocatorCallback_CAN_send, this, std::placeholders::_1));

    // Create RAW socket for SocketCAN.
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
      return;
    }
    // Retrieve interface index for "can0".
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Interface index retrieval failed");
      return;
    }
    // Configure CAN address structure.
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr.ifr_ifindex;
    // Bind the socket to the CAN interface.
    if (bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket binding failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "teensy CAN : GOOD!");
  }
  else if (mode_ == "sim"){
    allocator_subscription_ = this->create_subscription<allocator_interfaces::msg::PwmVal>("motor_cmd", 1, std::bind(&TeensyNode::allocatorCallback_MUJ_send, this, std::placeholders::_1));
    mujoco_publisher_ = this->create_publisher<mujoco_interfaces::msg::MotorThrust>("motor_write", 1);

    /* THIS SHOULD BE REMOVED LATER */

    // Create RAW socket for SocketCAN.
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
      return;
    }
    // Retrieve interface index for "can0".
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Interface index retrieval failed");
      return;
    }
    // Configure CAN address structure.
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr.ifr_ifindex;
    // Bind the socket to the CAN interface.
    if (bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket binding failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "teensy CAN : GOOD!");

    /* THIS SHOULD BE REMOVED LATER */

  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode_.c_str());
  }
}

/* for real */
void TeensyNode::allocatorCallback_CAN_send(const allocator_interfaces::msg::PwmVal::SharedPtr msg) {
  // Create CAN frame structure and set CAN ID and DLC.
  struct can_frame frame;
  frame.can_id = 0x123; // Set CAN ID.
  frame.can_dlc = 8;    // 4 channels of 16-bit data → total 8 bytes.

  // Mapping: input 0 -> 16383, input 1 -> 32767.
  uint16_t mapped1 = static_cast<uint16_t>(msg->pwm1 * 16384.) + 16383;
  uint16_t mapped2 = static_cast<uint16_t>(msg->pwm2 * 16384.) + 16383;
  uint16_t mapped3 = static_cast<uint16_t>(msg->pwm3 * 16384.) + 16383;
  uint16_t mapped4 = static_cast<uint16_t>(msg->pwm4 * 16384.) + 16383;

  // Split 16-bit values into 2 bytes each and pack them into the CAN frame.
  frame.data[0] = (mapped1 >> 8) & 0xFF;
  frame.data[1] = mapped1 & 0xFF;
  frame.data[2] = (mapped2 >> 8) & 0xFF;
  frame.data[3] = mapped2 & 0xFF;
  frame.data[4] = (mapped3 >> 8) & 0xFF;
  frame.data[5] = mapped3 & 0xFF;
  frame.data[6] = (mapped4 >> 8) & 0xFF;
  frame.data[7] = mapped4 & 0xFF;
  
  write(sock_, &frame, sizeof(frame));


  // int nbytes = write(sock_, &frame, sizeof(frame));
  // if (nbytes != sizeof(frame)) {
  //   RCLCPP_ERROR(this->get_logger(), "CAN 프레임 전송 에러");
  // } else {
  //   RCLCPP_INFO(this->get_logger(), "CAN 메시지 전송: [%.3f, %.3f, %.3f, %.3f]",
  //     msg->pwm1, msg->pwm2, msg->pwm3, msg->pwm4);
  // }
}

/* for sim */
void TeensyNode::allocatorCallback_MUJ_send(const allocator_interfaces::msg::PwmVal::SharedPtr msg) {
  // first, apply 3ms time-delay
  rclcpp::Time now_time = this->now();

  // Push the new data into the buffer
  DelayedData new_data;
  new_data.stamp = now_time;
  new_data.pwm_val[0] = msg->pwm1; new_data.pwm_val[1] = msg->pwm2; new_data.pwm_val[2] = msg->pwm3; new_data.pwm_val[3] = msg->pwm4;
  data_buffer_.push_back(new_data);

  // Remove older data that is no longer needed to reduce memory usage (older than 10ms)
  while (!data_buffer_.empty()) {
    if ((now_time - data_buffer_.front().stamp).nanoseconds() > 10000000LL) {
      data_buffer_.pop_front();
    }
    else { break; }
  }

  // Determine the target time: current time minus the desired delay
  rclcpp::Time target_time = this->now() - delay_;

  // Search backwards (from newest to oldest) to find the first sample with stamp <= target_time
  DelayedData delayed_data;
  bool found = false;

  for (auto it = data_buffer_.rbegin(); it != data_buffer_.rend(); ++it) {
    if (it->stamp <= target_time) {
      delayed_data = *it;
      found = true;
      break;
    }
  }

  if (!found) { return; }

  // pwm LPF
  pwm1_ = LPF_alpha_ * delayed_data.pwm_val[0] + LPF_beta_ * pwm1_;
  pwm2_ = LPF_alpha_ * delayed_data.pwm_val[1] + LPF_beta_ * pwm2_;
  pwm3_ = LPF_alpha_ * delayed_data.pwm_val[2] + LPF_beta_ * pwm3_;
  pwm4_ = LPF_alpha_ * delayed_data.pwm_val[3] + LPF_beta_ * pwm4_;

  // 1. PWM to RPM
  auto compute_rpm = [&](double pwm) -> double {
    return K1_ * std::pow(pwm, n_pwm_) + b_;
  };

  rpm1_ = compute_rpm(pwm1_);
  rpm2_ = compute_rpm(pwm2_);
  rpm3_ = compute_rpm(pwm3_);
  rpm4_ = compute_rpm(pwm4_);

  // 2. RPM to Thrust
  auto compute_thrust = [&](double omega) -> double {
    return C_T_ * std::pow(omega, n_thrust_);
  };

  f1_ = compute_thrust(rpm1_);
  f2_ = compute_thrust(rpm2_);
  f3_ = compute_thrust(rpm3_);
  f4_ = compute_thrust(rpm4_);

  // f1_ = msg->pwm1;
  // f2_ = msg->pwm2;
  // f3_ = msg->pwm3;
  // f4_ = msg->pwm4;


  // 3. RPM to Torque
  auto compute_torque = [&](double omega) -> double {
    return C1_tau_ * omega * omega + C2_tau_ * omega + C3_tau_;
  };

  m1_ = compute_torque(rpm1_);
  m2_ = -compute_torque(rpm2_);
  m3_ = compute_torque(rpm3_);
  m4_ = -compute_torque(rpm4_);
  // m1_ = 0.21496*f1_;
  // m2_ = -0.21496*f2_;
  // m3_ = 0.21496*f3_;
  // m4_ = -0.21496*f4_;

  // RCLCPP_INFO(this->get_logger(), "[f1: %.2f, f2: %.2f, f3: %.2f, f4: %.2f]", f1_, f2_, f3_, f4_);
  
  // Populate the MotorThrust message
  mujoco_interfaces::msg::MotorThrust wrench;
  wrench.force[0] = f1_; wrench.force[1] = f2_; wrench.force[2] = f3_; wrench.force[3] = f4_;
  wrench.moment[0] = m1_; wrench.moment[1] = m2_; wrench.moment[2] = m3_; wrench.moment[3] = m4_;
  
  mujoco_publisher_->publish(wrench);

  /* THIS SHOULD BE REMOVED LATER */
  allocatorCallback_CAN_send(msg);
  /* THIS SHOULD BE REMOVED LATER */
}

/* for both */
void TeensyNode::KillCmdCallback(const sbus_interfaces::msg::KillCmd::SharedPtr msg) {
  // SBUS kill (act just once)
  if (msg->kill_activated && !pwm_overriding_){
    allocator_subscription_.reset();
    
    // bind appropriate dummy-zero timer
    if (mode_ == "real") {
      CAN_overriding();
      publish_dummy_zeros_timer_ = this->create_wall_timer(10ms,std::bind(&TeensyNode::CAN_overriding, this));
    } else if (mode_ == "sim") {
      MUJOCO_overriding();
      publish_dummy_zeros_timer_ = this->create_wall_timer(10ms, std::bind(&TeensyNode::MUJOCO_overriding, this));
    }

    RCLCPP_INFO(this->get_logger(), "!! KILL - ACTIVATED !!");
    pwm_overriding_ = true;
  }
}

void TeensyNode::watchdogCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
  // Watchdog update
  watchdog_state_ = msg->state;
}

void TeensyNode::CAN_overriding(){
  struct can_frame frame_zeros = frame_zeros_;
  write(sock_, &frame_zeros, sizeof(frame_zeros));
}

void TeensyNode::MUJOCO_overriding(){
  // Populate the MotorThrust message
  mujoco_interfaces::msg::MotorThrust wrench;
  wrench.force[0] = 0; wrench.force[1] = 0; wrench.force[2] = 0; wrench.force[3] = 0;
  wrench.moment[0] = 0; wrench.moment[1] = 0; wrench.moment[2] = 0; wrench.moment[3] = 0;

  mujoco_publisher_->publish(wrench);
}

TeensyNode::~TeensyNode() {
  if (sock_ >= 0) { // if mode==sim >>> sock = -1; (do nothing)
    close(sock_);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyNode>());
  rclcpp::shutdown();
  return 0;
}