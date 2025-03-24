#ifndef DYNAMIXEL_WORKER_HPP
#define DYNAMIXEL_WORKER_HPP

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "watchdog_interfaces/msg/node_state.hpp"
#include "dynamixel_interfaces/msg/joint_val.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Dynamixel Setting
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Position PID Gain Control Table Address (XM-430)
#define ADDR_POSITION_P_GAIN 84
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_D_GAIN 80

// Velocity PID Gain Control Table Address
#define ADDR_VELOCITY_P_GAIN 78
#define ADDR_VELOCITY_I_GAIN 76

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 4000000
#define DEVICE_NAME "/dev/ttyUSB1"

constexpr double PI = 3.1415926535897932384626433832706;
constexpr double rad2ppr = 2048.0 / PI;
constexpr double ppr2rad = PI / 2048.0;
// constexpr int NUM_ARMS = 4;
// constexpr int JOINTS_PER_ARM = 5;
// constexpr int TOTAL_MOTORS = NUM_ARMS * JOINTS_PER_ARM; // 20 motors

// only use arm1
constexpr int NUM_ARMS = 1;                  
constexpr int JOINTS_PER_ARM = 5;
constexpr int TOTAL_MOTORS = NUM_ARMS * JOINTS_PER_ARM; // 5 motors (only arm_1)
// only use arm1

typedef dynamixel_interfaces::msg::JointVal JointValMsg;
typedef dynamixel_interfaces::msg::JointVal MotorPositionMsg;

class DynamixelNode : public rclcpp::Node {
public:
  DynamixelNode();
  bool init_Dynamixel();

private:
  bool Dynamixel_Write();
  bool Dynamixel_Read();

  void save_des_pos(double* arm_1, double* arm_2, double* arm_3, double* arm_4);
  void armchanger_callback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg);
  void set_position_pid(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);
  void set_velocity_pid(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain);
  void heartbeat_timer_callback();
  void read_timer_callback();


  // ROS2 subscriber for joint_val topic
  rclcpp::Subscription<dynamixel_interfaces::msg::JointVal>::SharedPtr joint_val_subscriber_;
  
  // Publisher & Timer for Heartbeat signal
  rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<dynamixel_interfaces::msg::JointVal>::SharedPtr motor_position_publisher_;
  
  rclcpp::TimerBase::SharedPtr read_timer_;
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

  int present_position;
  uint8_t heartbeat_state_; // previous node state
};

extern std::array<int, TOTAL_MOTORS> DXL_DES_POS;
extern std::array<int, TOTAL_MOTORS> DXL_CUR_POS;
extern std::array<uint8_t, TOTAL_MOTORS> DXL_IDS;

#endif // DYNAMIXEL_WORKER_HPP