#include "allocator_worker.hpp"

AllocatorWorker::AllocatorWorker() : Node("allocator_node") {

  // u = A * f
  const double l_1 = 0.18;  // Arm length
  const double l_2 = 0.14;
  const double C_m = 0.21496;
  A = (Matrix4d() << -l_1,  l_1,  l_1, -l_1,
                     -l_2, -l_2,  l_2,  l_2,
                      C_m, -C_m,  C_m, -C_m,
                      1.0,  1.0,  1.0,  1.0).finished();
  A_inv = A.inverse();

  // F = a * pwm^2 + b
  pwm_alpha_ = 46.5435;
  pwm_beta_ = 8.6111;

  // Subscriber
  controller_subscriber_ = this->create_subscription<controller_interfaces::msg::ControllerOutput>("controller_output", 1, std::bind(&AllocatorWorker::controllerCallback, this, std::placeholders::_1));
  joint_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("joint_mea", 1, std::bind(&AllocatorWorker::jointValCallback, this, std::placeholders::_1));

  // Publishers
  pwm_publisher_ = this->create_publisher<allocator_interfaces::msg::PwmVal>("pwm_val", 1);
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("allocator_state", 1);
  debug_val_publisher_ = this->create_publisher<allocator_interfaces::msg::AllocatorDebugVal>("allocator_info", 1);

  // Timers for periodic publishing
  pwm_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&AllocatorWorker::publishPwmVal, this));
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AllocatorWorker::heartbeat_timer_callback, this));
  debugging_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AllocatorWorker::debugging_timer_callback, this));

  // Initialize times
  current_callback_time_ = this->now();
  last_callback_time_    = this->now();
}

void AllocatorWorker::controllerCallback(const controller_interfaces::msg::ControllerOutput::SharedPtr msg) {

  //-------- Loop Time Calculate --------
  current_callback_time_ = this->now();
  current_dt = (current_callback_time_ - last_callback_time_).seconds();
  if (current_dt > 0.0) {filtered_frequency_ = 0.05 * (1.0 / current_dt) + 0.95 * filtered_frequency_;}
  last_callback_time_ = current_callback_time_;

  // get [Mx My Mz F]
  f << msg->moment[0], msg->moment[1], msg->moment[2], msg->force;

  // compute f [f1 f2 f3 f4] in [N]
  u = A_inv * f;
  u = u.cwiseMax(8.7).cwiseMin(145.1); // this clamps to [8.7, 145.1]

  // resolve f[N] to pwm[0.0~1.0]
  pwm = ((u.array() - pwm_beta_).array() / pwm_alpha_).cwiseSqrt().matrix();
  pwm = pwm.cwiseMax(0.0).cwiseMin(1.0); // this clamps to [0, 1]

  // RCLCPP_INFO(this->get_logger(), "Clamped PWM: [f1: %.2f, f2: %.2f, f3: %.2f, f4: %.2f]", pwm[0], pwm[1], pwm[2], pwm[3]);
}

void AllocatorWorker::jointValCallback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) {
  latest_joint_val_ = *msg;
}

void AllocatorWorker::publishPwmVal() {
  auto pwm_msg = allocator_interfaces::msg::PwmVal();
  // this range should be [0, 1]
  pwm_msg.pwm1 = pwm[0];
  pwm_msg.pwm2 = pwm[1];
  pwm_msg.pwm3 = pwm[2];
  pwm_msg.pwm4 = pwm[3];
  pwm_publisher_->publish(pwm_msg);
}

void AllocatorWorker::heartbeat_timer_callback() {
  heartbeat_state_++;

  // Populate the NodeState message
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;

  // Publish the sbus_state message
  heartbeat_publisher_->publish(state_msg);
}

void AllocatorWorker::debugging_timer_callback() {
  // Populate the debugging message
  allocator_interfaces::msg::AllocatorDebugVal info_msg;
  info_msg.pwm[0] = pwm[0];
  info_msg.pwm[1] = pwm[1];
  info_msg.pwm[2] = pwm[2];
  info_msg.pwm[3] = pwm[3];

  info_msg.loop_rate = filtered_frequency_;

  // Publish
  debug_val_publisher_->publish(info_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AllocatorWorker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
