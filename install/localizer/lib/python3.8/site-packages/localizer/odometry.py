import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pyproj import Transformer
import numpy as np
from numpy.linalg import inv, cholesky

# data 전역변수 (모든 값은 float64로 저장)
data = {'Long': np.float64(127.07636855358194), 'Lat': np.float64(37.63220434752274), 'hAcc': np.float64(1e10),
        'y': np.float64(-1.0),
        'Ax0_pos': np.float64(0.0), 'Ax1_pos': np.float64(0.0)}

odom = None  # 초기 오도메트리 객체
publisher = None  # 초기 퍼블리셔 객체
Axis_offset  = None # Axis0,1 pos offset 저장.

class UKF:
    def __init__(self,
                 r=np.float64(0.0525), d=np.float64(0.0957),
                 Q=np.diag([np.float64(0.5), np.float64(0.5), np.float64(0.01)]), 
                 R=np.diag([np.float64(1e8), np.float64(1e8), np.float64(0.01)]), 
                 initial_state=np.array([0.0, 0.0, 0.0], dtype=np.float64), 
                 initial_P=np.diag([np.float64(10.0), np.float64(10.0), np.float64(0.1)]), 
                 kappa=np.float64(0)):
        self.r = r
        self.d = d
        self.Q = Q
        self.R = R
        self.x = initial_state.copy()
        self.P = initial_P.copy()
        self.kappa = kappa

        self.r_div_2 = self.r / np.float64(2.0)
        self.round2rad = np.float64(2.0) / np.pi

        self.prev_pos0 = np.float64(0.0)
        self.prev_pos1 = np.float64(0.0)

    @staticmethod
    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def normalize_angles(angles):
        return (angles + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def circular_mean(angles, weights):
        sum_sin = np.sum(weights * np.sin(angles))
        sum_cos = np.sum(weights * np.cos(angles))
        return np.arctan2(sum_sin, sum_cos)

    def convert(self, current_pos0, current_pos1):
        delta_pos0 = (current_pos0 - self.prev_pos0) * self.round2rad
        delta_pos1 = (current_pos1 - self.prev_pos1) * self.round2rad
        delta_pos0 = self.normalize_angle(delta_pos0)
        delta_pos1 = self.normalize_angle(delta_pos1)

        self.prev_pos0 = current_pos0
        self.prev_pos1 = current_pos1
        return delta_pos0, delta_pos1
    
    def sigma_points(self, xm, P):
        Xi = np.zeros((3, 7), dtype=np.float64)
        W = np.zeros(7, dtype=np.float64)

        Xi[:, 0] = xm
        W[0] = self.kappa / (3 + self.kappa)

        try:
            U = cholesky((3 + self.kappa) * P)
        except np.linalg.LinAlgError:
            U = cholesky((3 + self.kappa) * (P + 1e-6 * np.eye(3, dtype=np.float64)))

        Xi[:, 1:4] = xm[:, np.newaxis] + U
        Xi[:, 4:7] = xm[:, np.newaxis] - U

        W[1:7] = np.float64(1) / (2 * (3 + self.kappa))

        return Xi, W

    def unscented_transform(self, Xi, W, noiseCov):
        xm = np.sum(W[np.newaxis, :] * Xi, axis=1)
        xm[2] = self.circular_mean(Xi[2, :], W)

        delta = Xi - xm[:, np.newaxis]
        delta[2, :] = self.normalize_angles(delta[2, :])

        xcov = np.einsum('k,ik,jk->ij', W, delta, delta) + noiseCov
        return xm, xcov

    def fx(self, Xi, delta_pos0, delta_pos1):
        cos_yaw = np.cos(Xi[2, :])
        sin_yaw = np.sin(Xi[2, :])
        delta_sum = delta_pos0 + delta_pos1
        delta_diff = -delta_pos0 + delta_pos1

        x_new = Xi[0, :] + (self.r_div_2 * cos_yaw) * delta_sum
        y_new = Xi[1, :] + (self.r_div_2 * sin_yaw) * delta_sum
        yaw_new = self.normalize_angles(Xi[2, :] + (self.r_div_2 / self.d) * delta_diff)

        return np.vstack((x_new, y_new, yaw_new))

    def update(self, current_pos0, current_pos1, z, R_override=None):
        delta_pos0, delta_pos1 = self.convert(current_pos0, current_pos1)
        z[2] = self.normalize_angle(z[2])

        Xi, W = self.sigma_points(self.x, self.P)
        fXi = self.fx(Xi, delta_pos0, delta_pos1)
        xp, Pp = self.unscented_transform(fXi, W, self.Q)

        current_R = R_override if R_override is not None else self.R
        zp, Pz = self.unscented_transform(fXi, W, current_R)

        delta_f = Xi - xp[:, np.newaxis]
        delta_f[2, :] = self.normalize_angles(delta_f[2, :])
        delta_h = fXi - zp[:, np.newaxis]
        delta_h[2, :] = self.normalize_angles(delta_h[2, :])
        Pxz = (delta_f * W) @ delta_h.T

        K = Pxz @ inv(Pz.astype(np.float64))
        y = z - zp
        y[2] = self.normalize_angle(y[2])

        self.x = xp + K @ y
        self.x[2] = self.normalize_angle(self.x[2])
        self.P = Pp - K @ Pz @ K.T

        return self.x.copy()

class Odometry:
    def __init__(self, ukf_object, Lon, Lat, yaw_offset, node):
        self.transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652") 
        self.transformer_to_wgs84 = Transformer.from_crs("EPSG:32652", "EPSG:4326")
        self.ukf = ukf_object
        self.base_lon = np.float64(Lon)
        self.base_lat = np.float64(Lat)
        self.yaw_offset = np.float64(yaw_offset)
        self.base_x, self.base_y = self.transformer_to_utm.transform(self.base_lat, self.base_lon)

        self.prev_lon = np.float64(Lon)
        self.prev_lat = np.float64(Lat)
        self.node = node  # Node 객체 저장s
        
        self.estim_x = np.zeros(3, dtype=np.float64)
        self.z = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.xy_cov_min = np.float64(700000.0)
        self.xy_cov_max = np.float64(1e9)
        self.R_override_min = np.diag([self.xy_cov_min, self.xy_cov_min, self.ukf.R[2, 2]])
        self.R_override_max = np.diag([self.xy_cov_max, self.xy_cov_max, self.ukf.R[2, 2]])

    def update(self, data):
        if np.float64(data['Long']) != self.prev_lon or np.float64(data['Lat']) != self.prev_lat:
            R = self.calcul_R(np.float64(data['hAcc']))
            x, y = self.convert2meter(np.float64(data['Long']), np.float64(data['Lat']))
            self.z[0] = x
            self.z[1] = y
            self.z[2] = np.float64(data['y']) - self.yaw_offset
            self.estim_x = self.ukf.update(np.float64(data['Ax0_pos']), np.float64(data['Ax1_pos']), self.z, R)
            self.prev_lon = data['Long']
            self.prev_lat = data['Lat']
            # self.node.get_logger().info(f"new -> lon: {data['Long']:.5f} | lat: {data['Lat']:.5f}")
        else:
            self.z[2] = np.float64(data['y']) - self.yaw_offset
            self.estim_x = self.ukf.update(np.float64(data['Ax0_pos']), np.float64(data['Ax1_pos']), self.z, self.R_override_max)
            # self.node.get_logger().info(f"old -> lon: {data['Long']:.5f} | lat: {data['Lat']:.5f}")
            
        estm_lon, estm_lat = self.convert2deg(self.estim_x[0], self.estim_x[1])
        estm_yaw = self.estim_x[2]
        return np.array([estm_lon, estm_lat, estm_yaw], dtype=np.float64)

    def convert2meter(self, lon, lat):
        x, y = self.transformer_to_utm.transform(float_lat, float_long)
        return x - self.base_x, y - self.base_y

    def convert2deg(self, x, y):
        abs_x = self.base_x + x
        abs_y = self.base_y + y
        lat, long = self.transformer_to_wgs84.transform(abs_x, abs_y)
        return long, lat
    
    def calcul_R(self, hAcc):
        if hAcc <= 3400:
            R = self.R_override_min
        elif hAcc >= 8000:
            R = self.R_override_max
        else:
            xy_cov = self.xy_cov_min + (self.xy_cov_max - self.xy_cov_min) * (hAcc - 3400) / 4600.0
            R = np.diag([xy_cov, xy_cov, self.ukf.R[2, 2]], dtype=np.float64)
        return R

# ------------------------------------------------------------------------------------#

# GPS 데이터 콜백 함수
def gps_callback(msg):
    global data
    # fuckin do nothing!
    # data['Long'] = np.float64(msg.position[0])
    # data['Lat'] = np.float64(msg.position[1])
    # data['hAcc'] = np.float64(msg.position[2])

# IMU 데이터 콜백 함수
def imu_callback(msg):
    global data
    data['y'] = np.float64(msg.position[2])
# ODrive 데이터 콜백 함수
def odrive_callback(msg, node):
    global data, odom, Axis_offset, publisher
    if Axis_offset is not None:
        data['Ax0_pos'] = -np.float64(msg.position[1]) + Axis_offset[1]
        data['Ax1_pos'] = np.float64(msg.position[0]) - Axis_offset[0]
    else:
        Axis_offset = (np.float64(msg.position[0]), np.float64(msg.position[1]))

    if odom is not None:
        estimated_state = odom.update(data)
        # node.get_logger().info(f"estim lon: {estimated_state[0]:.8f} | lat: {estimated_state[1]:.8f} | yaw: {estimated_state[2]:.3f} | x: {odom.estim_x[0]:.3f} | y: {odom.estim_x[1]:.3f}")
        # node.get_logger().info(f"ax0 pos: {data['Ax0_pos']:.3f} ax1 pos: {data['Ax1_pos']:.3f} yaw: {data['y']:.3f}")
        # 퍼블리시
        ukf_msg = JointState()
        ukf_msg.header.stamp = node.get_clock().now().to_msg()
        ukf_msg.position = [estimated_state[0], estimated_state[1], estimated_state[2]]
        publisher.publish(ukf_msg)

# ------------------------------------------------------------------------------------#

def main():
    global odom, publisher

    rclpy.init()
    node = Node('odometry_node')
    node.create_subscription(JointState, 'gps_data', gps_callback, 10)
    node.create_subscription(JointState, 'imu_data', imu_callback, 10)
    node.create_subscription(JointState, 'dubal_data', lambda msg: odrive_callback(msg, node), 10)
    publisher = node.create_publisher(JointState, 'ukf_state', 10)


    # UKF 파라미터
    r = np.float64(0.0525)
    d = np.float64(0.0957)
    Q = np.diag([np.float64(0.5), np.float64(0.5), np.float64(0.01)])
    R = np.diag([np.float64(700000.0), np.float64(700000.0), np.float64(0.01)])
    init_state = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    init_P = np.diag([np.float64(10.0), np.float64(10.0), np.float64(0.1)])
    
    ukf = UKF(r=r, d=d, Q=Q, R=R, initial_state=init_state, initial_P=init_P, kappa=np.float64(0))
    odom = Odometry(ukf, data['Long'], data['Lat'], np.float64(0.662), node)

    for i in range(10):
        estimated_state = odom.update(data)
        # print(estimated_state)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
