import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}]: {message}'

import rclpy
from rclpy.node import Node
from sbus_interfaces.msg import KillCmd, SbusSignal
from watchdog_interfaces.msg import NodeState

import asyncio
import threading

from .sbus_receiver import SBUSReceiver

class SbusNode(Node):
  def __init__(self):
    super().__init__('sbus_node')

    # Publisher for SBUS node
    self.channel_publisher_ = self.create_publisher(SbusSignal, 'sbus_signal', 1)
    self.killcmd_publisher_ = self.create_publisher(KillCmd, 'sbus_kill', 1)
    self.heartbeat_publisher_ = self.create_publisher(NodeState, 'sbus_state', 1)
    
    # Create a new event loop for asyncio and run it on a separate thread
    self.loop = asyncio.new_event_loop()
    self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
    self.thread.start()

    # Run the SBUS receiver in the asyncio event loop
    asyncio.run_coroutine_threadsafe(self.sbus_loop(), self.loop)

  async def sbus_loop(self):
    """
    1. Open the serial port.
    2. Wait for SBUS frames (await sbus.get_frame()).
    3. Convert each frame to a ROS2 message and publish it.
    """

    port_name = "/dev/ttyUSB0"
    
    try:
      sbus = await SBUSReceiver.create(port_name)
    except Exception as e:
      self.get_logger().error(f"!! SBUS port Failed : >> {port_name} << !!")
      return
    self.get_logger().info("SBUS Receiver connected.")

    while rclpy.ok():
      # Wait for the next SBUS frame from the queue
      frame = await sbus.get_frame()
      channels = frame.get_rx_channels()
      failsafe_status = frame.get_failsafe_status()

      # Log the received channels and failsafe status
      # self.get_logger().info(f"Received channels: {channels}, Failsafe status: {failsafe_status}")

      # Create and populate the SbusSignal message
      msg = SbusSignal()
      msg.ch = channels
      msg.sbus_signal = failsafe_status

      # Publish
      self.channel_publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = SbusNode()
  try:
    rclpy.spin(node)  # Keep the node alive to process callbacks and publish messages
  except KeyboardInterrupt:
    pass
  node.destroy_node()
  rclpy.shutdown()