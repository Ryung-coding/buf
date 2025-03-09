#ifndef WATCHDOG_WORKER_HPP
#define WATCHDOG_WORKER_HPP

#include <rclcpp/rclcpp.hpp>
#include "watchdog_interfaces/msg/node_state.hpp"
#include <unordered_map>

class WatchDogNode : public rclcpp::Node {
public:
    WatchDogNode();

private:
    void sbusCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
    void controllerCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
    void allocatorCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
    void dynamixelCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
    void imuCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
    void optitrackCallback(const watchdog_interfaces::msg::NodeState::SharedPtr msg);
    void publishKill();

    rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr sbus_subscription_;
    rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr controller_subscription_;
    rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr allocator_subscription_;
    rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr dynamixel_subscription_;
    rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr imu_subscription_;
    rclcpp::Subscription<watchdog_interfaces::msg::NodeState>::SharedPtr optitrack_subscription_;

    rclcpp::Publisher<watchdog_interfaces::msg::NodeState>::SharedPtr watchdog_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Heartbeat state
    std::unordered_map<std::string, uint8_t> node_heartbeat_;
    std::unordered_map<std::string, rclcpp::Time> last_heartbeat_time_;
};

#endif // WATCHDOG_WORKER_HPP