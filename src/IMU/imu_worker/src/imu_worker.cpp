#include "imu_worker.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

IMUnode::IMUnode() : Node("imu_node") {
  imu_publisher_ = this->create_publisher<imu_interfaces::msg::ImuMeasured>("imu_mea", 1);

  // Create a publisher for Heartbeat signal
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("imu_state", 1);

  timer_ = this->create_wall_timer(1ms, std::bind(&IMUnode::PublishMeasurement, this));

  // Create a timer to publish Node state messages at 10Hz
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IMUnode::heartbeat_timer_callback, this));

  // Subscription True Measuring value from MuJoCo
  mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("mujoco_meas", 1, std::bind(&IMUnode::mujoco_callback, this, std::placeholders::_1));

  alpha_ = 0.01;
  beta_ = 1.0 - alpha_;

  q_filtered_.fill(0.0);
  qdot_filtered_.fill(0.0);
}

void IMUnode::PublishMeasurement() {
  auto output_msg = imu_interfaces::msg::ImuMeasured();
  output_msg.q = { q_filtered_[0], q_filtered_[1], q_filtered_[2] };
  output_msg.qdot = { qdot_filtered_[0], qdot_filtered_[1], qdot_filtered_[2] };

  imu_publisher_->publish(output_msg);
}

void IMUnode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  // MuJoCo Measuring update
  for (size_t i = 0; i < 3; ++i) {
    q_filtered_[i] = alpha_ * msg->q[i] + (1.0 - alpha_) * q_filtered_[i];
    qdot_filtered_[i] = alpha_ * msg->qdot[i] + (1.0 - alpha_) * qdot_filtered_[i];
  }
}

void IMUnode::heartbeat_timer_callback() {
  heartbeat_state_++;

  // Populate the NodeState message
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;

  // Publish the sbus_state message
  heartbeat_publisher_->publish(state_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUnode>());
  rclcpp::shutdown();
  return 0;
}
