#!/usr/bin/env python3
import os
import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rosbag2_py import (
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata
)


class RosbagRecorderNode(Node):
    """
    Controlled rosbag2 recorder.
    Master node decides when recording is allowed.
    """

    def __init__(self):
        super().__init__('rosbag_recorder')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('base_directory', '~/rosbags')
        self.declare_parameter('record_all', True)

        self.base_dir = os.path.expanduser(
            self.get_parameter('base_directory').value
        )
        self.record_all = self.get_parameter('record_all').value

        os.makedirs(self.base_dir, exist_ok=True)

        # ----------------------------
        # Internal state
        # ----------------------------
        self.writer = None
        self.recording = False
        self.robot_state = 'DISABLED'

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.cmd_sub = self.create_subscription(
            String,
            '/robot/recording/command',
            self.command_cb,
            10
        )

        self.state_sub = self.create_subscription(
            String,
            '/robot/state',
            self.state_cb,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/robot/recording/status',
            10
        )

        self.get_logger().info("Rosbag recorder ready (idle)")

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def state_cb(self, msg: String):
        self.robot_state = msg.data

        # Hard safety stop
        if self.robot_state == 'FAULT' and self.recording:
            self.get_logger().warn("FAULT detected → stopping recording")
            self.stop_recording()

    def command_cb(self, msg: String):
        cmd = msg.data.upper()

        if cmd == 'START':
            self.start_recording()

        elif cmd == 'STOP':
            self.stop_recording()

        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # --------------------------------------------------
    # Recording control
    # --------------------------------------------------

    def start_recording(self):
        if self.recording:
            self.get_logger().warn("Already recording")
            return

        if self.robot_state not in ('READY', 'TEST'):
            self.get_logger().warn(
                f"Recording blocked (robot_state={self.robot_state})"
            )
            return

        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(self.base_dir, f'terralift_{timestamp}')

        self.get_logger().info(f"Starting rosbag: {bag_path}")

        try:
            self.writer = SequentialWriter()

            storage_options = StorageOptions(
                uri=bag_path,
                storage_id='sqlite3'
            )

            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            self.writer.open(storage_options, converter_options)

            # Record everything (master can filter later)
            if self.record_all:
                self.writer.set_filter(None)

            self.recording = True
            self.publish_status("RECORDING")

        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag: {e}")
            self.publish_status("ERROR")
            self.recording = False

    def stop_recording(self):
        if not self.recording:
            return

        self.get_logger().info("Stopping rosbag recording")

        try:
            self.writer = None
        except Exception as e:
            self.get_logger().error(f"Error stopping rosbag: {e}")

        self.recording = False
        self.publish_status("IDLE")

    # --------------------------------------------------

    def publish_status(self, state: str):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)

    # --------------------------------------------------

    def destroy_node(self):
        if self.recording:
            self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
