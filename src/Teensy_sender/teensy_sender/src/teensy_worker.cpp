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
  std::string mode;
  this->get_parameter("mode", mode);

  if (mode == "real"){
    allocator_subscription_ = this->create_subscription<allocator_interfaces::msg::PwmVal>("motor_cmd", 1, std::bind(&TeensyNode::allocatorCallback_teensy, this, std::placeholders::_1));

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
  }
  else if (mode == "sim"){
    allocator_subscription_ = this->create_subscription<allocator_interfaces::msg::PwmVal>("motor_cmd", 1, std::bind(&TeensyNode::allocatorCallback_mujoco, this, std::placeholders::_1));
    mujoco_publisher_ = this->create_publisher<mujoco_interfaces::msg::MotorThrust>("motor_write", 1);

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


  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode.c_str());
  }
}

/* for real */
void TeensyNode::allocatorCallback_teensy(const allocator_interfaces::msg::PwmVal::SharedPtr msg) {
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
  
  int nbytes = write(sock_, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {can_err_cnt++;}
}

/* for sim */
void TeensyNode::allocatorCallback_mujoco(const allocator_interfaces::msg::PwmVal::SharedPtr msg) {
  // pwm LPF
  pwm1_ = LPF_alpha_ * msg->pwm1 + LPF_beta_ * pwm1_;
  pwm2_ = LPF_alpha_ * msg->pwm2 + LPF_beta_ * pwm2_;
  pwm3_ = LPF_alpha_ * msg->pwm3 + LPF_beta_ * pwm3_;
  pwm4_ = LPF_alpha_ * msg->pwm4 + LPF_beta_ * pwm4_;

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

  // 3. RPM to Torque
  auto compute_torque = [&](double omega) -> double {
    return C1_tau_ * omega * omega + C2_tau_ * omega + C3_tau_;
  };

  m1_ = compute_torque(rpm1_);
  m2_ = -compute_torque(rpm2_);
  m3_ = compute_torque(rpm3_);
  m4_ = -compute_torque(rpm4_);

  // RCLCPP_INFO(this->get_logger(), "[f1: %.2f, f2: %.2f, f3: %.2f, f4: %.2f]", f1_, f2_, f3_, f4_);
  
  // Populate the MotorThrust message
  mujoco_interfaces::msg::MotorThrust wrench;
  wrench.force[0] = f1_; wrench.force[1] = f2_; wrench.force[2] = f3_; wrench.force[3] = f4_;
  wrench.moment[0] = m1_; wrench.moment[1] = m2_; wrench.moment[2] = m3_; wrench.moment[3] = m4_;
  
  mujoco_publisher_->publish(wrench);



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
  
  int nbytes = write(sock_, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {can_err_cnt++;}



  
}

/* for both */
void TeensyNode::KillCmdCallback(const sbus_interfaces::msg::KillCmd::SharedPtr msg) {
  // SBUS kill update
  kill_activated_ = msg->kill_activated;
  // RCLCPP_INFO(this->get_logger(), "kill_activated_: %s", kill_activated_ ? "true" : "false");
}

void TeensyNode::watchdogCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
  // Watchdog update
  watchdog_state_ = msg->state;
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