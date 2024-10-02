#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import odrive
from odrive.enums import *
import time

LEG_ODRIVE_SERIAL_NUMBER = "305D36533037"
WHEEL_ODRIVE_SERIAL_NUMBER = "3682387E3333"

LOOP_FREQUENCY = 1000
LOOP_PERIOD = 1.0 / LOOP_FREQUENCY

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')

        self.odrv_leg = odrive.find_any(serial_number=LEG_ODRIVE_SERIAL_NUMBER)
        self.odrv_wheel = odrive.find_any(serial_number=WHEEL_ODRIVE_SERIAL_NUMBER)

        self.wheel_initial_pos_axis0 = 0.0
        self.wheel_initial_pos_axis1 = 0.0

        self.leg_pos_axis0_past=0.0
        self.leg_pos_axis1_past=0.0
        self.wheel_pos_axis0_past=0.0
        self.wheel_pos_axis1_past=0.0

        self.initialize_odrive()

        self.target_position_axis0 = 0
        self.target_position_axis1 = 0

        self.timer = self.create_timer(LOOP_PERIOD, self.dubal_data_loop)

        self.subscription_wheel = self.create_subscription(Float64MultiArray, '/odrive_wheel/commands', self.wheel_callback, 10)
        self.subscription_leg = self.create_subscription(Float64MultiArray, '/odrive_leg/commands', self.leg_callback, 10)
        
        self.publisher_dubal_data = self.create_publisher(JointState, 'dubal_data', 10)



    def initialize_odrive(self):
        self.odrv_leg.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.odrv_leg.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        while self.odrv_leg.axis0.current_state != AXIS_STATE_IDLE or self.odrv_leg.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        self.wheel_initial_pos_axis0 = self.odrv_wheel.axis0.encoder.pos_estimate        
        self.wheel_initial_pos_axis1 = self.odrv_wheel.axis1.encoder.pos_estimate

        self.odrv_leg.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_leg.axis0.config.enable_watchdog = False
        self.odrv_leg.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_leg.axis1.config.enable_watchdog = False

        self.odrv_wheel.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_wheel.axis0.config.enable_watchdog = False
        self.odrv_wheel.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_wheel.axis1.config.enable_watchdog = False

    def wheel_callback(self, msg):
        torque0 = msg.data[0]
        torque1 = msg.data[1]
        self.odrv_wheel.axis0.controller.input_torque = torque0
        self.odrv_wheel.axis1.controller.input_torque = torque1

    def leg_callback(self, msg):
        torque0 = msg.data[0]
        torque1 = msg.data[1]
        self.odrv_leg.axis0.controller.input_torque = torque0
        self.odrv_leg.axis1.controller.input_torque = torque1

    def dubal_data_loop(self):
        self.leg_pos_axis0 = self.odrv_leg.axis0.encoder.pos_estimate
        self.leg_pos_axis1 = self.odrv_leg.axis1.encoder.pos_estimate
        self.leg_vel_axis0 = (self.leg_pos_axis0 - self.leg_pos_axis0_past) / LOOP_PERIOD
        self.leg_vel_axis1 = (self.leg_pos_axis1 - self.leg_pos_axis1_past) / LOOP_PERIOD

        self.wheel_pos_axis0 = self.odrv_wheel.axis0.encoder.pos_estimate - self.wheel_initial_pos_axis0
        self.wheel_pos_axis1 = self.odrv_wheel.axis1.encoder.pos_estimate - self.wheel_initial_pos_axis1
        self.wheel_vel_axis0 = (self.wheel_pos_axis0 - self.wheel_pos_axis0_past) / LOOP_PERIOD
        self.wheel_vel_axis1 = (self.wheel_pos_axis1 - self.wheel_pos_axis1_past) / LOOP_PERIOD

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["wheel_axis0", "wheel_axis1", "leg_axis0", "leg_axis1"]
        joint_state_msg.velocity = [self.wheel_vel_axis0, self.wheel_vel_axis1, self.leg_vel_axis0, self.leg_vel_axis1]
        joint_state_msg.position = [self.wheel_pos_axis0, self.wheel_pos_axis1, self.leg_pos_axis0, self.leg_pos_axis1]

        self.publisher_dubal_data.publish(joint_state_msg)

        self.leg_pos_axis0_past=self.leg_pos_axis0
        self.leg_pos_axis1_past=self.leg_pos_axis1
        self.wheel_pos_axis0_past=self.wheel_pos_axis0
        self.wheel_pos_axis1_past=self.wheel_pos_axis1

def main(args=None):
    rclpy.init(args=args)

    odrive_controller = ODriveController()

    try:
        rclpy.spin(odrive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        odrive_controller.odrv_leg.axis0.controller.input_torque = 0
        odrive_controller.odrv_leg.axis1.controller.input_torque = 0
        odrive_controller.odrv_wheel.axis0.controller.input_torque = 0
        odrive_controller.odrv_wheel.axis1.controller.input_torque = 0

        odrive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
