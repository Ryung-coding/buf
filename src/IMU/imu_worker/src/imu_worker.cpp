#include "imu_worker.hpp"

using namespace std::chrono_literals;

constexpr inline std::array<double, 4> quaternion_multiply(const std::array<double, 4>& q1, const std::array<double, 4>& q2) noexcept {
  // Cache components to minimize repeated array accesses
  const double a = q1[0], b = q1[1], c = q1[2], d = q1[3];
  const double e = q2[0], f = q2[1], g = q2[2], h = q2[3];

return { a * e - b * f - c * g - d * h,    // Real part
         a * f + b * e + c * h - d * g,    // i component
         a * g - b * h + c * e + d * f,    // j component
         a * h + b * g - c * f + d * e };  // k component
}

IMUnode::IMUnode()
: Node("imu_node"),
  gen_(std::random_device{}()),
  angle_dist_(0.0, noise_quat_std_dev),
  axis_dist_(0.0, 1.0),
  noise_dist_(0.0, noise_gyro_std_dev)
{
  imu_publisher_ = this->create_publisher<imu_interfaces::msg::ImuMeasured>("imu_mea", 1);

  // Create a timer to publish Node state messages at 10Hz
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IMUnode::heartbeat_timer_callback, this));

  // Create a publisher for Heartbeat signal
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("imu_state", 1);

  // Mode = sim -> mujoco Sub
  // Mode = real -> Reading IMU
  this->declare_parameter<std::string>("mode", "None");
  std::string mode;
  this->get_parameter("mode", mode);

  if (mode == "real"){
    RCLCPP_WARN(this->get_logger(), "IMU Node : I cannot do anything :(");
  }
  else if (mode == "sim"){
    // Subscription True Measuring value from MuJoCo
    mujoco_subscription_ = this->create_subscription<mujoco_interfaces::msg::MuJoCoMeas>("mujoco_meas", 1, std::bind(&IMUnode::mujoco_callback, this, std::placeholders::_1));
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&IMUnode::PublishMuJoCoMeasurement, this));
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s. No initialization performed.", mode.c_str());
  }
}
/* for real */

/* for sim */
void IMUnode::mujoco_callback(const mujoco_interfaces::msg::MuJoCoMeas::SharedPtr msg) {
  // Capture the current ROS time when the data is received
  rclcpp::Time now_time = this->now();

  // Push the new data into the buffer
  DelayedData new_data;
  new_data.stamp = now_time;
  new_data.q[0] = msg->q[0]; new_data.q[1] = msg->q[1]; new_data.q[2] = msg->q[2]; new_data.q[3] = msg->q[3];
  new_data.w[0] = msg->w[0]; new_data.w[1] = msg->w[1]; new_data.w[2] = msg->w[2];

  data_buffer_.push_back(new_data);

  // Remove older data that is no longer needed to reduce memory usage (older than 10ms)
  while (!data_buffer_.empty()) {
    if ((now_time - data_buffer_.front().stamp).nanoseconds() > 10000000LL) {
      data_buffer_.pop_front();
    }
    else { break; }
  }
}

void IMUnode::PublishMuJoCoMeasurement() {
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

  // Generate small rotation noise as a quaternion using member distributions
  double delta_angle = angle_dist_(gen_)*0;
  double ax = axis_dist_(gen_)*0;
  double ay = axis_dist_(gen_)*0;
  double az = axis_dist_(gen_)*0;
  double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm < 1e-6) { norm = 1.0; }
  ax /= norm; ay /= norm; az /= norm;

  double half_angle = delta_angle / 2.0;
  double cos_half = std::cos(half_angle);
  double sin_half = std::sin(half_angle);
  std::array<double, 4> noise_q = { cos_half, sin_half * ax, sin_half * ay, sin_half * az };

  // Retrieve simulation quaternion (assumed [w, x, y, z] order)
  std::array<double, 4> sim_q = delayed_data.q;

  // Combine noise quaternion with simulation quaternion (noise applied first)
  std::array<double, 4> noisy_q = quaternion_multiply(noise_q, sim_q);

  // Normalize resulting quaternion
  double norm_noisy = std::sqrt(noisy_q[0]*noisy_q[0] + noisy_q[1]*noisy_q[1] +
                                noisy_q[2]*noisy_q[2] + noisy_q[3]*noisy_q[3]);
  for (int i = 0; i < 4; ++i) {
    noisy_q[i] /= norm_noisy;
  }

  // Add white noise to angular velocity measurements using pre-initialized distribution
  double noisy_w[3] = {
    delayed_data.w[0] + noise_dist_(gen_)*0,
    delayed_data.w[1] + noise_dist_(gen_)*0,
    delayed_data.w[2] + noise_dist_(gen_)*0
  };

  // Construct and publish the IMU measurement message
  auto output_msg = imu_interfaces::msg::ImuMeasured();
  output_msg.q = { noisy_q[0], noisy_q[1], noisy_q[2], noisy_q[3] };
  output_msg.w = { noisy_w[0], noisy_w[1], noisy_w[2] };

  imu_publisher_->publish(output_msg);
}

/* for Both */
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
