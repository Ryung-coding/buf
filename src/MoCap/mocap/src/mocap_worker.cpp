#include "mocap_worker.hpp"

using namespace std::chrono_literals;

OptiTrackNode::OptiTrackNode() : Node("optitrack_node") {
  mocap_publisher_ = this->create_publisher<mocap_interfaces::msg::MocapMeasured>(
    "optitrack_mea", 10
  );

  // Create a publisher for Heartbeat signal
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("optitrack_state", 10);

  timer_ = this->create_wall_timer(
    1ms, std::bind(&OptiTrackNode::PublishMeasurement, this)
  );

  // Create a timer to publish Node state messages at 10Hz
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OptiTrackNode::heartbeat_timer_callback, this));

  // Subscription True Measuring value from MuJoCo
  mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("mujoco_meas", 1, std::bind(&OptiTrackNode::mujoco_callback, this, std::placeholders::_1));

  alpha_ = 0.01;
  beta_ = 1.0 - alpha_;

  pos_filtered_.fill(0.0);
  vel_filtered_.fill(0.0);
}

void OptiTrackNode::PublishMeasurement() {
  auto output_msg = mocap_interfaces::msg::MocapMeasured();
  output_msg.pos = { pos_filtered_[0], pos_filtered_[1], pos_filtered_[2] };
  output_msg.vel = { vel_filtered_[0], vel_filtered_[1], vel_filtered_[2] };

  mocap_publisher_->publish(output_msg);
}

void OptiTrackNode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  // MuJoCo Measuring update
  for (size_t i = 0; i < 3; ++i) {
    pos_filtered_[i] = alpha_ * msg->pos[i] + (1.0 - alpha_) * pos_filtered_[i];
    vel_filtered_[i] = alpha_ * msg->vel[i] + (1.0 - alpha_) * vel_filtered_[i];
  }
}

void OptiTrackNode::heartbeat_timer_callback() {
  heartbeat_state_++;

  // Populate the NodeState message
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;

  // Publish the sbus_state message
  heartbeat_publisher_->publish(state_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptiTrackNode>());
  rclcpp::shutdown();
  return 0;
}