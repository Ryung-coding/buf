#include "dynamixel_worker.hpp"

using namespace std::chrono_literals;
using namespace dynamixel;

PortHandler *portHandler;
PacketHandler *packetHandler;
GroupSyncWrite *groupSyncWrite;
GroupSyncRead *groupSyncRead;

DynamixelNode::DynamixelNode() : Node("dynamixel_node") {
  // Subscription
  joint_val_subscriber_ = this->create_subscription<allocator_interfaces::msg::JointVal>("joint_val", 1, std::bind(&DynamixelNode::allocatorCallback, this, std::placeholders::_1));

  // Publish
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("dynamixel_state", 1);
  joint_val_publisher_ = this->create_publisher<allocator_interfaces::msg::JointVal>("joint_mea", 1);

  // Timer
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamixelNode::heartbeat_timer_callback, this));
}

void DynamixelNode::allocatorCallback(const allocator_interfaces::msg::JointVal::SharedPtr msg) {
  // save msg
  for (int i = 0; i < 5; ++i) {
    arm_1_rad[i] = msg->a1_q[i];
    arm_2_rad[i] = msg->a2_q[i];
    arm_3_rad[i] = msg->a3_q[i];
    arm_4_rad[i] = msg->a4_q[i];
  }

  // Dynamixel Write [Example]
  for (int i = 0; i < 5; ++i) {
    arm_1_ppr[i] = static_cast<int16_t>(arm_1_rad[i] * rad2ppr);
    arm_2_ppr[i] = static_cast<int16_t>(arm_2_rad[i] * rad2ppr);
    arm_3_ppr[i] = static_cast<int16_t>(arm_3_rad[i] * rad2ppr);
    arm_4_ppr[i] = static_cast<int16_t>(arm_4_rad[i] * rad2ppr);
  }

  // Dynamixel Read [Example]
  for (int i = 0; i < 5; ++i) {
    arm_1_mea[i] = static_cast<double>(arm_1_ppr[i] * ppr2rad);
    arm_2_mea[i] = static_cast<double>(arm_2_ppr[i] * ppr2rad);
    arm_3_mea[i] = static_cast<double>(arm_3_ppr[i] * ppr2rad);
    arm_4_mea[i] = static_cast<double>(arm_4_ppr[i] * ppr2rad);
  }
  
  // Publish msg
  allocator_interfaces::msg::JointVal joint_val_msg;

  for (int i = 0; i < 5; ++i) {
    joint_val_msg.a1_q[i] = arm_1_mea[i];
    joint_val_msg.a2_q[i] = arm_2_mea[i];
    joint_val_msg.a3_q[i] = arm_3_mea[i];
    joint_val_msg.a4_q[i] = arm_4_mea[i];
  }

  joint_val_publisher_->publish(joint_val_msg);
}

void DynamixelNode::heartbeat_timer_callback() {
  heartbeat_state_++;

  // Populate the NodeState message
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;

  // Publish the sbus_state message
  heartbeat_publisher_->publish(state_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelNode>());
  rclcpp::shutdown();
  return 0;
}
