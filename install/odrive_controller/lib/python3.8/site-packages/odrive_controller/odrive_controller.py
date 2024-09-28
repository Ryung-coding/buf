#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import odrive
from odrive.enums import *
import time

ODRIVE_SERIAL_NUMBER = "3682387E3333"
LOOP_FREQUENCY = 1000  # Loop frequency in Hz
LOOP_PERIOD = 1.0 / LOOP_FREQUENCY  # Loop period in seconds
FILTER_WEIGHT = 0.95  # Weight for the low-pass filter (0 < FILTER_WEIGHT < 1)

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')

        self.subscription_joint0 = self.create_subscription(Float64MultiArray, '/joint0_torque_controller/commands', self.listener_callback_joint0, 10)
        self.subscription_joint1 = self.create_subscription(Float64MultiArray, '/joint1_torque_controller/commands', self.listener_callback_joint1, 10)

        self.publisher_dubal_data = self.create_publisher(JointState, 'dubal_data', 10)

        self.odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
        self.check_and_clear_errors()

        # Set control mode and disable watchdog
        self.odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.config.enable_watchdog = False
        self.odrv0.axis1.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.config.enable_watchdog = False

        # Reset encoder position to 0
        self.odrv0.axis0.encoder.set_linear_count(0)
        self.odrv0.axis1.encoder.set_linear_count(0)

        # Initialize previous position values for velocity calculation
        self.previous_position_axis0 = 0.0
        self.previous_position_axis1 = 0.0

        # Initialize filtered velocity values
        self.filtered_velocity_axis0 = 0.0
        self.filtered_velocity_axis1 = 0.0

        # Set a timer to publish data at LOOP_FREQUENCY Hz
        self.timer = self.create_timer(LOOP_PERIOD, self.publish_joint_states)

    def check_and_clear_errors(self):
        for axis in [self.odrv0.axis0, self.odrv0.axis1]:
            axis_error = axis.error
            motor_error = axis.motor.error

            if axis_error & AxisError.MOTOR_FAILED:
                axis.clear_errors()

            if motor_error & MotorError.CURRENT_SENSE_SATURATION:
                axis.clear_errors()

    def listener_callback_joint0(self, msg):
        self.check_and_clear_errors()
        torque = msg.data[0]
        self.odrv0.axis0.controller.input_torque = torque

    def listener_callback_joint1(self, msg):
        self.check_and_clear_errors()
        torque = msg.data[0]
        self.odrv0.axis1.controller.input_torque = torque

    def lowpassfilter(self, filter_value, data, past_weight):
        return data * (1 - past_weight) + filter_value * past_weight

    def publish_joint_states(self):
        # Calculate current position for both axes
        current_position_axis0 = self.odrv0.axis0.encoder.pos_estimate
        current_position_axis1 = self.odrv0.axis1.encoder.pos_estimate

        # Calculate velocity using numerical differentiation
        velocity_axis0 = (current_position_axis0 - self.previous_position_axis0) / LOOP_PERIOD
        velocity_axis1 = (current_position_axis1 - self.previous_position_axis1) / LOOP_PERIOD

        # Apply low-pass filter to the calculated velocities
        self.filtered_velocity_axis0 = self.lowpassfilter(self.filtered_velocity_axis0, velocity_axis0, FILTER_WEIGHT)
        self.filtered_velocity_axis1 = self.lowpassfilter(self.filtered_velocity_axis1, velocity_axis1, FILTER_WEIGHT)

        # Update previous position
        self.previous_position_axis0 = current_position_axis0
        self.previous_position_axis1 = current_position_axis1

        # Gather data and publish
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["axis0", "axis1"]
        joint_state_msg.effort = [
            self.odrv0.axis0.controller.input_torque,
            self.odrv0.axis1.controller.input_torque
        ]
        joint_state_msg.velocity = [self.filtered_velocity_axis0, self.filtered_velocity_axis1]
        joint_state_msg.position = [current_position_axis0, current_position_axis1]

        self.publisher_dubal_data.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)

    odrive_controller = ODriveController()

    try:
        rclpy.spin(odrive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        odrive_controller.odrv0.axis0.controller.input_torque = 0
        odrive_controller.odrv0.axis1.controller.input_torque = 0

        odrive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
