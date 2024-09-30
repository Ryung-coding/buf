#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import odrive
from odrive.enums import *
import time

ODRIVE_SERIAL_NUMBER = "305D36533037"

# Control Constants
LOOP_HZ = 1000  # 제어 루프 주파수
SBUS_CENTER = 1024
SBUS_RANGE = 676
KILL_BOUND = 700
CONNECT_BOUND = 700
LEG_UP_BOUND = 700
LEG_DOWN_BOUND = 1400
DEADZONE_SBUS = 2
DEADZONE_INPUT = 0
LIM_INPUT = 4
ANTI_WINDUP_GAIN = 100

# PID 파라미터 설정
Kp = [0.06, 0.06]  # P 게인
Ki = [1, 1]  # I 게인
Kd = [0.005, 0.005]  # D 게인
I = [0.0, 0.0]  # 적분 항 초기화

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')

        # sbus_data 토픽 구독
        self.subscription_sbus = self.create_subscription(JointState, '/sbus_data', self.sbus_callback, 10)
        
        # ODrive 초기화
        self.odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
        self.check_and_clear_errors()

        # Axis0과 Axis1에 대한 캘리브레이션 수행
        for axis in [self.odrv0.axis0, self.odrv0.axis1]:
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        # 캘리브레이션 완료 대기
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE or self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        # 엔코더 에러 확인
        for i, axis in enumerate([self.odrv0.axis0, self.odrv0.axis1]):
            if axis.encoder.error != 0:
                print(f"Axis {i} encoder error: {axis.encoder.error}")
                return  # 에러가 있으면 종료

        # 캘리브레이션 성공 후 모터 제어 시작
        for axis in [self.odrv0.axis0, self.odrv0.axis1]:
            axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # 초기 목표 위치 설정
        self.target_position_axis0 = 5.0  # 초기 M0의 목표 위치
        self.target_position_axis1 = -self.target_position_axis0  # M1은 M0의 반대 위치로 설정

        self.loop_frequency = LOOP_HZ  # 1000Hz 제어 루프
        self.dt = 1.0 / self.loop_frequency

        # 제어 루프 시작
        self.timer = self.create_timer(self.dt, self.control_loop)

    def check_and_clear_errors(self):
        # ODrive 에러 확인 및 초기화
        for axis in [self.odrv0.axis0, self.odrv0.axis1]:
            axis_error = axis.error
            motor_error = axis.motor.error

            if axis_error != 0:
                axis.clear_errors()

            if motor_error != 0:
                axis.clear_errors()

    def sbus_callback(self, msg):
        # sbus_data의 position[2] 값을 목표 위치로 변환
        if len(msg.position) >= 3:
            sbus_value = msg.position[2]

            # 조건을 바탕으로 -1, 0, 1로 변환
            if sbus_value > LEG_DOWN_BOUND:
                converted_value = -1
            elif sbus_value > LEG_UP_BOUND:
                converted_value = 0
            else:
                converted_value = 1

            # 목표 위치 설정
            if converted_value == -1:
                self.target_position_axis0 = 0.5
            elif converted_value == 0:
                self.target_position_axis0 = 1.3
            elif converted_value == 1:
                self.target_position_axis0 = 2.3

            # M1의 목표 위치는 M0의 반대
            self.target_position_axis1 = -self.target_position_axis0

			#self.get_logger().info(f"Updated target position Axis0: {self.target_position_axis0}, Axis1: {self.target_position_axis1}")

    def compute_pid(self, r, y, y_dot, dt, pid_case):
        # PID 계산
        error = r - y

        # P gain
        P = Kp[pid_case] * error

        # I gain
        I[pid_case] += Ki[pid_case] * error * dt
        if abs(I[pid_case]) > ANTI_WINDUP_GAIN:
            I[pid_case] = ANTI_WINDUP_GAIN if I[pid_case] > 0 else -ANTI_WINDUP_GAIN

        # D gain
        D = Kd[pid_case] * (0 - y_dot)

        u = P + I[pid_case] + D
        return u

    def control_loop(self):
        # Axis0에 대해 목표 위치를 유지하도록 제어
        for axis_index, axis in enumerate([self.odrv0.axis0, self.odrv0.axis1]):
            # 현재 위치 및 속도 가져오기
            current_position = axis.encoder.pos_estimate
            current_velocity = axis.encoder.vel_estimate

            # 목표 위치 설정 (M0과 M1은 서로 반대 위치로 이동)
            if axis_index == 0:
                target_position = self.target_position_axis0
            else:
                target_position = self.target_position_axis1

            # 현재 위치와 목표 위치 출력
			#self.get_logger().info(f"Axis {axis_index} | Current Position: {current_position:.2f}, Target Position: {target_position:.2f}")

            # PID 컨트롤러를 이용해 토크 계산
            torque = self.compute_pid(target_position, current_position, current_velocity, self.dt, axis_index)

            # 토크 적용
            axis.controller.input_torque = torque

def main(args=None):
    rclpy.init(args=args)

    odrive_controller = ODriveController()

    try:
        rclpy.spin(odrive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 모터 정지
        odrive_controller.odrv0.axis0.controller.input_torque = 0
        odrive_controller.odrv0.axis1.controller.input_torque = 0

        odrive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
