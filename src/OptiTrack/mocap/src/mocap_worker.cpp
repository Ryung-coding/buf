#include "mocap_worker.hpp"

using namespace std::chrono_literals;

OptiTrackNode::OptiTrackNode()
 : Node("optitrack_node"),
  gen_(std::random_device{}()),
  pos_dist_(0.0, noise_pos_std_dev),
  vel_dist_(0.0, noise_vel_std_dev),
  acc_dist_(0.0, noise_acc_std_dev)
{
  mocap_publisher_ = this->create_publisher<mocap_interfaces::msg::MocapMeasured>("optitrack_mea", 1);

  // Create a publisher for Heartbeat signal
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("optitrack_state", 10);

  // Create a timer to publish Node state messages at 10Hz
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OptiTrackNode::heartbeat_timer_callback, this));


  // Mode = sim -> mujoco Sub
  // Mode = real -> Reading OptiTrack
  this->declare_parameter<std::string>("mode", "None");
  std::string mode;
  this->get_parameter("mode", mode);

  if (mode == "real"){
    RCLCPP_WARN(this->get_logger(), "Opti Node : I cannot do anything :(");
  }
  else if (mode == "sim"){
    // Subscription True Measuring value from MuJoCo
    mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("mujoco_meas", 1, std::bind(&OptiTrackNode::mujoco_callback, this, std::placeholders::_1));
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&OptiTrackNode::PublishMuJoCoMeasurement, this));
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode.c_str());
  }
}

/* for real */

/* for sim */
void OptiTrackNode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  
  // Capture the current ROS time when the data is received
  rclcpp::Time now_time = this->now();

  // Push the new data into the buffer
  DelayedData new_data;
  new_data.stamp = now_time;
  for (size_t i = 0; i < 3; ++i) {
    new_data.pos[i] = msg->pos[i];
    new_data.vel[i] = msg->vel[i];
    new_data.acc[i] = msg->acc[i];
  }
  data_buffer_.push_back(new_data);

  // Remove older data that is no longer needed to reduce memory usage (older than 10ms)
  while (!data_buffer_.empty()) {
    if ((now_time - data_buffer_.front().stamp).nanoseconds() > 10000000LL) {
      data_buffer_.pop_front();
    }
    else { break; }
  }
}

void OptiTrackNode::PublishMuJoCoMeasurement() {
  if (data_buffer_.empty()) { return; }

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

  std::array<double, 3> noisy_pos;
  std::array<double, 3> noisy_vel;
  std::array<double, 3> noisy_acc;
  for (size_t i = 0; i < 3; ++i) {
    noisy_pos[i] = delayed_data.pos[i] + pos_dist_(gen_)*0;
    noisy_vel[i] = delayed_data.vel[i] + vel_dist_(gen_)*0;
    noisy_acc[i] = delayed_data.acc[i] + acc_dist_(gen_)*0;
  }

  // Publish data
  auto output_msg = mocap_interfaces::msg::MocapMeasured();
  output_msg.pos = { noisy_pos[0], noisy_pos[1], noisy_pos[2] };
  output_msg.vel = { noisy_vel[0], noisy_vel[1], noisy_vel[2] };
  output_msg.acc = { noisy_acc[0], noisy_acc[1], noisy_acc[2] };

  mocap_publisher_->publish(output_msg);
}

/* for Both */
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