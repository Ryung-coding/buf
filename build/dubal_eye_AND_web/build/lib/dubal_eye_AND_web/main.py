import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import cv2
import socket
import socketio
import numpy as np
import time
import threading

# data 전역변수
data = {'Long': 127.0772, 'Lat': 37.6315, 'Alt': 50,
        'r': -1.0, 'p': -1.0, 'y': -1.0,
        'PL_NUM': '0000', 'PL_STATE': -1,
        'u1': -1.0, 'u2': -1.0, 'u3': -1.0,
        'V1': 0.0, 'V2': 0.0,
        'v1': 0.0, 'v2': 0.0, 'v3': 0.0,
        'SIV': 0, 'FIX': 0}

class WebSocket:
    def __init__(self, server_url, room_id):
        self.server_url = server_url
        self.room_id = room_id
        self.sio = socketio.Client()
        self.isConnected = False
        self.stop_event = threading.Event()  # 스레드 종료 플래그
        self.timer = None  # 타이머 관리 변수

        # Socket.IO 이벤트 핸들러 등록
        self.sio.on('connect', self.connect)
        self.sio.on('roomJoined', self.on_room_joined)
        self.sio.on('disconnect', self.disconnect)
    
    def begin(self):
        EventThread = threading.Thread(target=self.wait_for_events)
        EventThread.start()
        self.sio.connect(self.server_url)

    def wait_for_events(self):
        self.sio.wait()

    def connect(self):
        self.sio.emit('joinOrCreateRoom', [self.room_id, 0])

    def on_room_joined(self, room):
        if str(room) == self.room_id:
            self.isConnected = True

    def disconnect(self):
        self.isConnected = False

    def start_sending_SensorData(self, data):
        self.timer = threading.Timer(0.18, self.send_SensorData, args=(data,))
        self.timer.start()

    def send_SensorData(self, data):
        if not self.stop_event.is_set():  # stop_event가 설정되지 않았을 때만 실행
            message = {
                'room': self.room_id,
                'datas': {
                    'type': 3,
                    'datas': list(data.values())
                }
            }
            self.sio.emit('message', message)
            self.start_sending_SensorData(data)

    def stop(self):
        # 타이머와 스레드를 안전하게 종료
        self.stop_event.set()
        self.timer.cancel()
        self.sio.disconnect()

class UDPsocket:
    def __init__(self, target_address, scale_factor):
        # UDP 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = target_address

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # 웹캠
        self.cap.set(cv2.CAP_PROP_FPS, 13)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        self.new_width = int(1280 * scale_factor)
        self.new_height = int(720 * scale_factor)
    
    def start_sending_frame(self):
        Frame_sending_Thread = threading.Thread(target=self.CAM_process)
        Frame_sending_Thread.start()

    def CAM_process(self):
        last_send_time = time.time() - 0.16 # start immediately
        while True:
            current_time = time.time()
                
            ret, frame = self.cap.read()

            if current_time - last_send_time >= 0.16:
                last_send_time = current_time

                resized_frame = cv2.resize(frame, (self.new_width, self.new_height), cv2.INTER_AREA)

                # 프레임을 JPEG로 인코딩
                result, encoded_frame = cv2.imencode('.jpg', resized_frame, self.encode_param)

                # 데이터를 네트워크로 전송
                try:
                    self.sock.sendto(encoded_frame.tobytes(), self.server_address)
                except Exception as e:
                        print(f"Error - {e}")

# ------------------------------------------------------------------------------------#

# GPS 데이터 콜백 함수
def gps_callback(msg):
    global data
    data['Long'] = msg.position[0]
    data['Long'] = 127.0772
    data['Lat'] = msg.position[1]
    data['Lat'] = 37.6315
    data['Alt'] = msg.position[2]
    data['SIV'] = msg.position[3]
    data['FIX'] = msg.position[4]
    print(f"Updated GPS Data: {data}")

# IMU 데이터 콜백 함수
def imu_callback(msg):
    global data
    data['r'] = msg.position[2]*180.0/np.pi
    data['p'] = msg.position[1]*180.0/np.pi
    data['u3'] = msg.velocity[1]*180.0/np.pi
    data['u2'] = msg.velocity[2]*180.0/np.pi
    print(f"Updated IMU Data: {data}")

# Controller Input 데이터 콜백 함수
def controller_callback0(msg):
    global data
    data['u1'] = msg.position[0]  # pos0 값을 u1에 저장
    data['u3'] = 0  # vel0 값을 u3에 저장
    print(f"Updated Controller Input Data: {data}")

# Controller Input 데이터 콜백 함수
def controller_callback1(msg):
    global data
    data['u2'] = msg.position[0]  # pos1 값을 u2에 저장
    data['u3'] = 0  # vel0 값을 u3에 저장
    print(f"Updated Controller Input Data: {data}")

# Controller Input 데이터 콜백 함수
def dubal_callback(msg):
    global data
    data['v1'] = msg.velocity[0]  # pos1 값을 u2에 저장
    data['v2'] = msg.velocity[1]  # vel0 값을 u3에 저장
    print(f"Updated dubal Input Data: {data}")

# ------------------------------------------------------------------------------------#

def main(args=None):
    rclpy.init()
    node = Node('sensor_data_subscriber')
    node.create_subscription(JointState, 'gps_data', gps_callback, 10)
    node.create_subscription(JointState, 'imu_data', imu_callback, 10)
    node.create_subscription(JointState, '/joint0_torque_controller/commands', controller_callback0, 10)
    node.create_subscription(JointState, '/joint1_torque_controller/commands', controller_callback1, 10)
    node.create_subscription(JointState, 'dubal_data', dubal_callback, 10)

    # 웹소켓 송수신 관련 시작 #
    Socketclient = WebSocket('http://13.125.65.10:5024', 'AA:11:BB:22:CC:33')
    Socketclient.begin()

    while not Socketclient.isConnected:
        time.sleep(0.05)

    Socketclient.start_sending_SensorData(data)

    # 카메라 UDP 송신 관련 시작 #
    # server_address = ('192.168.0.87', 5000)  # 햄트북
    server_address = ('192.168.0.86', 5000)  # 주트북

    UDPclient = UDPsocket(server_address, 0.5) # scale factor
    UDPclient.start_sending_frame()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 리소스 해제
        UDPclient.cap.release()
        UDPclient.sock.close()
        Socketclient.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
