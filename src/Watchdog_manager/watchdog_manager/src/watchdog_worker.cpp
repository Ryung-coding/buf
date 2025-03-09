#include "watchdog_worker.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

WatchDogNode::WatchDogNode() : Node("watchdog_node") {

    // SBUS subscription
    sbus_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>(
        "sbus_state", 10,
        std::bind(&WatchDogNode::sbusCallback, this, std::placeholders::_1)
    );

    // CONTROLLER subscription
    controller_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>(
        "controller_state", 10,
        std::bind(&WatchDogNode::controllerCallback, this, std::placeholders::_1)
    );

    // ALLOCATOR subscription
    allocator_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>(
        "allocator_state", 10,
        std::bind(&WatchDogNode::allocatorCallback, this, std::placeholders::_1)
    );

    // DYNAMIXEL subscription
    dynamixel_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>(
        "dynamixel_state", 10,
        std::bind(&WatchDogNode::dynamixelCallback, this, std::placeholders::_1)
    );

    // IMU subscription
    imu_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>(
        "imu_state", 10,
        std::bind(&WatchDogNode::imuCallback, this, std::placeholders::_1)
    );

    // OPTITRACK subscription
    optitrack_subscription_ = this->create_subscription<watchdog_interfaces::msg::NodeState>(
        "optitrack_state", 10,
        std::bind(&WatchDogNode::optitrackCallback, this, std::placeholders::_1)
    );

    // publish
    watchdog_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>(
        "watchdog_state", 10
    );

    timer_ = this->create_wall_timer(
        100ms, std::bind(&WatchDogNode::publishKill, this)
    );
}

// Heartbeat Callback
void WatchDogNode::sbusCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
    node_heartbeat_["sbus_node"] = msg->state;
    last_heartbeat_time_["sbus_node"] = this->now();
}

void WatchDogNode::controllerCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
    node_heartbeat_["controller_node"] = msg->state;
    last_heartbeat_time_["controller_node"] = this->now();
}

void WatchDogNode::allocatorCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
    node_heartbeat_["allocator_node"] = msg->state;
    last_heartbeat_time_["allocator_node"] = this->now();
}

void WatchDogNode::dynamixelCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
    node_heartbeat_["dynamixel_node"] = msg->state;
    last_heartbeat_time_["dynamixel_node"] = this->now();
}

void WatchDogNode::imuCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
    node_heartbeat_["imu_node"] = msg->state;
    last_heartbeat_time_["imu_node"] = this->now();
}

void WatchDogNode::optitrackCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg) {
    node_heartbeat_["optitrack_node"] = msg->state;
    last_heartbeat_time_["optitrack_node"] = this->now();
}

// 10Hz timer state manage
void WatchDogNode::publishKill() {
    auto now = this->now();
    bool node_failure_detected = false;
    watchdog_interfaces::msg::NodeState msg;
    msg.state = 0;

    for (const auto& node : node_heartbeat_) {
        std::string node_name = node.first;
        
        if ((now - last_heartbeat_time_[node_name]).seconds() > 0.2) {
            RCLCPP_WARN(this->get_logger(), "--[%s] DIED!--", node_name.c_str());
            node_failure_detected = true;
            break;
        }
    }

    if (node_failure_detected) {
        msg.state = 255;
    }
    else {
        msg.state = 1;
    }

    watchdog_publisher_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WatchDogNode>());
    rclcpp::shutdown();
    return 0;
}