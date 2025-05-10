import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from mujoco_interfaces.msg import MotorThrust, MuJoCoMeas, MujocoState
from dynamixel_interfaces.msg import JointVal

import mujoco
import mujoco.viewer
from rclpy.executors import SingleThreadedExecutor
import threading
import time
from collections import deque

import signal

class MuJoCoSimulatorNode(Node):
    def __init__(self, executor):
        super().__init__('mujoco_node')
        self.executor = executor

        # Get the package directory and initialize MuJoCo model
        package_share_dir = get_package_share_directory('mujoco_sim')
        scene_file_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
        self.model = mujoco.MjModel.from_xml_path(scene_file_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.001  # 1kHz simulation frequency

        # Initialize motor thrust and moments
        self.motor_thrusts = [0.0, 0.0, 0.0, 0.0]
        self.motor_moments = [0.0, 0.0, 0.0, 0.0]
        self.a1_des = [0.0, -0.84522, 1.50944, 0.90812, 0.0]
        self.a2_des = [0.0, -0.84522, 1.50944, 0.90812, 0.0]
        self.a3_des = [0.0, -0.84522, 1.50944, 0.90812, 0.0]
        self.a4_des = [0.0, -0.84522, 1.50944, 0.90812, 0.0]

        # Timer to run the simulation at 1ms intervals
        self._sim_times = deque(maxlen=1200)
        now = time.monotonic()
        self._sim_times.append(now)
        self.timer = self.create_timer(0.001, self.run_simulation)  # 1ms interval (1kHz)

        # Start MuJoCo viewer in a separate thread
        self.viewer_thread = threading.Thread(target=self.run_viewer, daemon=True)
        self.data_lock = threading.Lock()
        self.viewer_thread.start()

        # publishers
        self.mujoco_meas_publisher = self.create_publisher(MuJoCoMeas, '/mujoco_meas', 1)
        self.mujoco_state_publisher = self.create_publisher(MujocoState, '/mujoco_state', 1)

        # Timer to publish /mujoco_state
        self.create_timer(0.1, self.publish_mujoco_state)

        # Start Subscriber after some delay
        self._delayed_timer = self.create_timer(3.0, self._start_sub)

    def _start_sub(self):
        self._delayed_timer.cancel()
        
        self.create_subscription(MotorThrust, '/motor_write', self.motor_thrust_callback, 1)
        self.create_subscription(JointVal, '/joint_write', self.joint_arm_callback, 1)

    def motor_thrust_callback(self, msg: MotorThrust):
        self.motor_thrusts = msg.force
        self.motor_moments = msg.moment
    
    def joint_arm_callback(self, msg: JointVal):
        self.a1_des = msg.a1_des
        self.a2_des = msg.a2_des
        self.a3_des = msg.a3_des
        self.a4_des = msg.a4_des

    def run_simulation(self):
        now = time.monotonic()
        self._sim_times.append(now)
        
        cutoff = now - 1.0
        while self._sim_times and self._sim_times[0] < cutoff: self._sim_times.popleft()

        # Set control inputs for MuJoCo
        with self.data_lock:
            self.data.ctrl[0:4] = self.motor_thrusts
            self.data.ctrl[4:8] = self.motor_moments
            self.data.ctrl[8:13] = self.a1_des
            self.data.ctrl[13:18] = self.a2_des
            self.data.ctrl[18:23] = self.a3_des
            self.data.ctrl[23:28] = self.a4_des

            # Perform one simulation step (1kHz)
            mujoco.mj_step(self.model, self.data)

            # Publish the drone's current state
            self.publish_mujoco_meas()

    def run_viewer(self):
        # Launch MuJoCo viewer in passive mode (20Hz update rate)
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                with self.data_lock:
                    viewer.sync()  # Sync the viewer to the current simulation state

        if rclpy.ok(): rclpy.shutdown()

    def publish_mujoco_meas(self):
        # Extract state information
        pos = self.data.qpos[:3]
        q = self.data.qpos[3:7]
        vel = self.data.qvel[:3]
        w = self.data.qvel[3:6]
        acc = self.data.qacc[:3]

        # --- Joint slices based on XML ordering ---
        base_offset    = 7
        joints_per_arm = 5

        # Arm3 joints are qpos[7:12]
        a3_q = list(self.data.qpos[ base_offset                    : base_offset + joints_per_arm ])
        # Arm2 joints are qpos[12:17]
        a2_q = list(self.data.qpos[ base_offset + 1*joints_per_arm : base_offset + 2*joints_per_arm ])
        # Arm1 joints are qpos[17:22]
        a1_q = list(self.data.qpos[ base_offset + 2*joints_per_arm : base_offset + 3*joints_per_arm ])
        # Arm4 joints are qpos[22:27]
        a4_q = list(self.data.qpos[ base_offset + 3*joints_per_arm : base_offset + 4*joints_per_arm ])
        # ------------------------------------------------

        # Create and publish the message
        msg = MuJoCoMeas(
            q=[q[0], q[1], q[2], q[3]],
            w=[w[0], w[1], w[2]],
            pos=[pos[0], pos[1], pos[2]],
            vel=[vel[0], vel[1], vel[2]],
            acc=[acc[0], acc[1], acc[2]],
            a1_q = a1_q,
            a2_q = a2_q,
            a3_q = a3_q,
            a4_q = a4_q
        )
        self.mujoco_meas_publisher.publish(msg)

    def publish_mujoco_state(self):
        # measured_hz = len(self._sim_times) / (self._sim_times[-1] - self._sim_times[0])
        measured_hz = 50.0
        msg = MujocoState()
        msg.hz = measured_hz
        self.mujoco_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = MuJoCoSimulatorNode(executor)

    signal.signal(signal.SIGINT, signal.default_int_handler)

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if 'Unable to convert call argument to Python object' in str(e): pass
        else: raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()