#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import odrive
from odrive.enums import *
import time

# ODrive 시리얼 넘버
LEG_ODRIVE_SERIAL_NUMBER = "305D36533037"
WHEEL_ODRIVE_SERIAL_NUMBER = "3682387E3333"

# PID 파라미터 설정 (다리)
Kp = [0.1, 0.1]
Ki = [1.5, 1.5]
Kd = [0.005, 0.005]
I = [0.0, 0.0]

# 다리 제어 경계값
LEG_UP_BOUND = 700
LEG_DOWN_BOUND = 1400

# 필터 가중치 (휠)
FILTER_WEIGHT = 0.95
LOOP_FREQUENCY = 1000
LOOP_PERIOD = 1.0 / LOOP_FREQUENCY

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')

        # ODrive 연결
        # 기존 코드
        self.odrv_leg = odrive.find_any(serial_number=LEG_ODRIVE_SERIAL_NUMBER)
        self.odrv_wheel = odrive.find_any(serial_number=WHEEL_ODRIVE_SERIAL_NUMBER)

        # 수정된 코드
        #self.odrv_leg = odrive.find_any(path='/dev/odrive_leg')
        #self.odrv_wheel = odrive.find_any(path='/dev/odrive_wheel')


        self.initial_pos_axis0 = 0.0
        self.initial_pos_axis1 = 0.0

        self.initialize_odrive()

        # 초기 목표 위치 및 속도 설정 (다리)
        self.target_position_axis0 = 1.3
        self.target_position_axis1 = -self.target_position_axis0

        # 휠 속도 관련 초기화
        self.previous_position_axis0 = 0.0
        self.previous_position_axis1 = 0.0
        self.filtered_velocity_axis0 = 0.0
        self.filtered_velocity_axis1 = 0.0


        # 제어 주기 설정
        self.timer = self.create_timer(LOOP_PERIOD, self.control_loop)

        # 토픽 구독 및 퍼블리싱 설정 (다리, 휠)
        self.subscription_sbus = self.create_subscription(JointState, '/sbus_data', self.sbus_callback, 10)
        self.subscription_wheel_joint0 = self.create_subscription(Float64MultiArray, '/joint0_torque_controller/commands', self.wheel_callback_joint0, 10)
        self.subscription_wheel_joint1 = self.create_subscription(Float64MultiArray, '/joint1_torque_controller/commands', self.wheel_callback_joint1, 10)

        # JointState 데이터 퍼블리셔 설정
        self.publisher_dubal_data = self.create_publisher(JointState, 'dubal_data', 10)

    def initialize_odrive(self):
        # 다리 ODrive 캘리브레이션 및 closed loop 제어 설정
        self.odrv_leg.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.odrv_leg.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        while self.odrv_leg.axis0.current_state != AXIS_STATE_IDLE or self.odrv_leg.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        #초기값 설정용 ㅋㅋ ㄹㅇㅋㅋ 이새끼 때문이네
        self.initial_pos_axis0 = self.odrv_wheel.axis0.encoder.pos_estimate        
        self.initial_pos_axis1 = self.odrv_wheel.axis1.encoder.pos_estimate

        self.odrv_leg.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_leg.axis0.config.enable_watchdog = False
        self.odrv_leg.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_leg.axis1.config.enable_watchdog = False

        # 휠 ODrive 설정
        self.odrv_wheel.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_wheel.axis0.config.enable_watchdog = False
        self.odrv_wheel.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv_wheel.axis1.config.enable_watchdog = False

    def sbus_callback(self, msg):
        if len(msg.position) >= 3:
            sbus_value = msg.position[2]
            if sbus_value > LEG_DOWN_BOUND:
                self.target_position_axis0 = 0.5
            elif sbus_value > LEG_UP_BOUND:
                self.target_position_axis0 = 1.3
            else:
                self.target_position_axis0 = 2.3
            self.target_position_axis1 = -self.target_position_axis0

    def wheel_callback_joint0(self, msg):
        torque = msg.data[0]
        self.odrv_wheel.axis0.controller.input_torque = torque

    def wheel_callback_joint1(self, msg):
        torque = msg.data[0]
        self.odrv_wheel.axis1.controller.input_torque = torque

    def lowpassfilter(self, filter_value, data, past_weight):
        return data * (1 - past_weight) + filter_value * past_weight

    def compute_pid(self, r, y, y_dot, dt, pid_case):
        error = r - y
        P = Kp[pid_case] * error
        I[pid_case] += Ki[pid_case] * error * dt
        if abs(I[pid_case]) > 100:
            I[pid_case] = 100 if I[pid_case] > 0 else -100
        D = Kd[pid_case] * (0 - y_dot)
        u = P + I[pid_case] + D
        return u

    def control_loop(self):
        # 다리 제어 루프
        for axis_index, axis in enumerate([self.odrv_leg.axis0, self.odrv_leg.axis1]):
            current_position = axis.encoder.pos_estimate
            current_velocity = axis.encoder.vel_estimate
            target_position = self.target_position_axis0 if axis_index == 0 else self.target_position_axis1
            torque = self.compute_pid(target_position, current_position, current_velocity, LOOP_PERIOD, axis_index)
            axis.controller.input_torque = torque

        # 휠 속도 계산
        current_position_axis0 = self.odrv_wheel.axis0.encoder.pos_estimate - self.initial_pos_axis0
        current_position_axis1 = self.odrv_wheel.axis1.encoder.pos_estimate - self.initial_pos_axis1
        
        velocity_axis0 = (current_position_axis0 - self.previous_position_axis0) / LOOP_PERIOD
        velocity_axis1 = (current_position_axis1 - self.previous_position_axis1) / LOOP_PERIOD
        self.filtered_velocity_axis0 = self.lowpassfilter(self.filtered_velocity_axis0, velocity_axis0, FILTER_WEIGHT)
        self.filtered_velocity_axis1 = self.lowpassfilter(self.filtered_velocity_axis1, velocity_axis1, FILTER_WEIGHT)
        
        #update 
        self.previous_position_axis0 = current_position_axis0
        self.previous_position_axis1 = current_position_axis1

        # wheel 데이터 전송 To controller(cpp)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["axis0", "axis1"]
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
        odrive_controller.odrv_leg.axis0.controller.input_torque = 0
        odrive_controller.odrv_leg.axis1.controller.input_torque = 0
        odrive_controller.odrv_wheel.axis0.controller.input_torque = 0
        odrive_controller.odrv_wheel.axis1.controller.input_torque = 0

        odrive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
