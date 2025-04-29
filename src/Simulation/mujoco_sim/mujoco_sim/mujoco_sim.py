import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from mujoco_interfaces.msg import MotorThrust, MuJoCoMeas
from dynamixel_interfaces.msg import JointVal

import mujoco
import mujoco.viewer
from rclpy.executors import SingleThreadedExecutor
import threading

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
        self.a1_des = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.a2_des = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.a3_des = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.a4_des = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Publisher & Subscriber setup
        self.create_subscription(MotorThrust, '/motor_write', self.motor_thrust_callback, 1)
        self.create_subscription(JointVal, '/joint_write', self.joint_arm_callback, 1)
        self.mujoco_meas_publisher = self.create_publisher(MuJoCoMeas, '/mujoco_meas', 1)

        # Timer to run the simulation at 1ms intervals
        self.timer = self.create_timer(0.001, self.run_simulation)  # 1ms interval (1kHz)

        # Start MuJoCo viewer in a separate thread
        self.viewer_thread = threading.Thread(target=self.run_viewer, daemon=True)
        self.data_lock = threading.Lock()
        self.viewer_thread.start()

    def motor_thrust_callback(self, msg: MotorThrust):
        self.motor_thrusts = msg.force
        self.motor_moments = msg.moment
    
    def joint_arm_callback(self, msg: JointVal):
        self.a1_des = msg.a1_des
        self.a2_des = msg.a2_des
        self.a3_des = msg.a3_des
        self.a4_des = msg.a4_des

    def run_simulation(self):
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

        # Viewer has been closed, trigger shutdown
        self.executor.shutdown()

    def publish_mujoco_meas(self):
        # Extract state information
        pos = self.data.qpos[:3]
        q = self.data.qpos[3:7]
        vel = self.data.qvel[:3]
        w = self.data.qvel[3:6]
        acc = self.data.qacc[:3]

        # --- Joint slices based on XML ordering ---
        #   free joint (7 DOF) → Arm3_joint1..5 → Arm2_joint1..5 → Arm1_joint1..5 → Arm4_joint1..5
        base_offset    = 7           # free joint 다음 qpos 인덱스
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
            pos=[-pos[0], -pos[1], -pos[2]],
            vel=[-vel[0], -vel[1], -vel[2]],
            acc=[-acc[0], -acc[1], -acc[2]],
            a1_q = a1_q,
            a2_q = a2_q,
            a3_q = a3_q,
            a4_q = a4_q
        )
        self.mujoco_meas_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = MuJoCoSimulatorNode(executor)

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()