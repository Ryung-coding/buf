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
FILTER_WEIGHT = 0.95
LOOP_FREQUENCY = 1000
LOOP_PERIOD = 1.0 / LOOP_FREQUENCY

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        self.odrv_leg = odrive.find_any(serial_number=LEG_ODRIVE_SERIAL_NUMBER)
        self.odrv_wheel = odrive.find_any(serial_number=WHEEL_ODRIVE_SERIAL_NUMBER)

        self.wheel_pos_joint0_initial=0.0
        self.wheel_pos_joint1_initial=0.0
        
        self.wheel_pos_joint0_past = 0.0
        self.wheel_pos_joint1_past = 0.0
        self.wheel_vel_joint0_estimate = 0.0
        self.wheel_vel_joint1_estimate = 0.0
        self.leg_pos_joint0_past = 0.0
        self.leg_pos_joint1_past = 0.0
        self.leg_vel_joint0_estimate = 0.0
        self.leg_vel_joint1_estimate = 0.0
        self.torque0 = 0.0
        self.torque1 = 0.0
        self.torque2 = 0.0
        self.torque3 = 0.0

        self.initialize_odrive()

        self.timer = self.create_timer(LOOP_PERIOD, self.contol_loop)

        self.subscription_wheel_joint0 = self.create_subscription(Float64MultiArray, '/joint0_torque_controller/commands', self.wheel_callback_joint0, 10)
        self.subscription_wheel_joint1 = self.create_subscription(Float64MultiArray, '/joint1_torque_controller/commands', self.wheel_callback_joint1, 10)
        self.subscription_leg_joint2 = self.create_subscription(Float64MultiArray, '/joint2_torque_controller/commands', self.leg_callback_joint2, 10)
        self.subscription_leg_joint3 = self.create_subscription(Float64MultiArray, '/joint3_torque_controller/commands', self.leg_callback_joint3, 10)

        self.publisher_dubal_data = self.create_publisher(JointState, 'dubal_data', 10)

    def initialize_odrive(self):
        self.odrv_leg.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.odrv_leg.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        while self.odrv_leg.axis0.current_state != AXIS_STATE_IDLE or self.odrv_leg.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        self.odrv_leg.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.wheel_pos_joint0_initial=self.odrv_wheel.axis0.encoder.pos_estimate 
        self.wheel_pos_joint1_initial=self.odrv_wheel.axis1.encoder.pos_estimate 
        self.odrv_leg.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_leg.axis0.config.enable_watchdog = False
        self.odrv_leg.axis1.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv_leg.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_leg.axis1.config.enable_watchdog = False
        self.odrv_wheel.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv_wheel.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_wheel.axis0.config.enable_watchdog = False
        self.odrv_wheel.axis1.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv_wheel.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_wheel.axis1.config.enable_watchdog = False

    def lowpassfilter(self, filter_value, data, past_weight):
        return data * (1 - past_weight) + filter_value * past_weight

    def wheel_callback_joint0(self, msg):
        self.torque0 = msg.data[0]
        self.odrv_wheel.axis0.controller.input_torque = self.torque0
        
    def wheel_callback_joint1(self, msg):
        self.torque1 = msg.data[0]
        self.odrv_wheel.axis1.controller.input_torque = self.torque1

    def leg_callback_joint2(self, msg):
        self.torque2 = msg.data[0]
        self.odrv_leg.axis0.controller.input_torque = self.torque2

    def leg_callback_joint3(self, msg):
        self.torque3 = msg.data[0]
        self.odrv_leg.axis1.controller.input_torque = self.torque3
        
    def contol_loop(self):
        wheel_pos_joint0_now = self.odrv_wheel.axis0.encoder.pos_estimate - self.wheel_pos_joint0_initial
        wheel_pos_joint1_now = self.odrv_wheel.axis1.encoder.pos_estimate - self.wheel_pos_joint1_initial
        wheel_vel_joint0 = (wheel_pos_joint0_now - self.wheel_pos_joint0_past) / LOOP_PERIOD
        wheel_vel_joint1 = (wheel_pos_joint1_now - self.wheel_pos_joint1_past) / LOOP_PERIOD
        self.wheel_vel_joint0_estimate = self.lowpassfilter(self.wheel_vel_joint0_estimate, wheel_vel_joint0, FILTER_WEIGHT)
        self.wheel_vel_joint1_estimate = self.lowpassfilter(self.wheel_vel_joint1_estimate, wheel_vel_joint1, FILTER_WEIGHT)
        self.wheel_pos_joint0_past = wheel_pos_joint0_now
        self.wheel_pos_joint1_past = wheel_pos_joint1_now

        leg_pos_joint0_now = self.odrv_leg.axis0.encoder.pos_estimate
        leg_pos_joint1_now = self.odrv_leg.axis1.encoder.pos_estimate
        leg_vel_joint0 = (leg_pos_joint0_now - self.leg_pos_joint0_past) / LOOP_PERIOD
        leg_vel_joint1 = (leg_pos_joint1_now - self.leg_pos_joint1_past) / LOOP_PERIOD
        self.leg_vel_joint0_estimate = self.lowpassfilter(self.leg_vel_joint0_estimate, leg_vel_joint0, FILTER_WEIGHT)
        self.leg_vel_joint1_estimate = self.lowpassfilter(self.leg_vel_joint1_estimate, leg_vel_joint1, FILTER_WEIGHT)
        self.leg_pos_joint0_past = leg_pos_joint0_now
        self.leg_pos_joint1_past = leg_pos_joint1_now

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.velocity = [self.wheel_vel_joint0_estimate, self.wheel_vel_joint1_estimate, self.leg_vel_joint0_estimate, self.leg_vel_joint1_estimate]
        joint_state_msg.position = [wheel_pos_joint0_now, wheel_pos_joint1_now, leg_pos_joint0_now, leg_pos_joint1_now]

        self.publisher_dubal_data.publish(joint_state_msg)


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


