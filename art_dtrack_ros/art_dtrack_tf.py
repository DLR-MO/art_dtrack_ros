from math import floor
import socket
import threading
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from transforms3d.quaternions import mat2quat
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from .art_dtrack.art_dtrack_receiver import (
    ArtDtrackReceiver,
    DtrackBody,
    DtrackMeasurementTool,
    DtrackMeasurementToolReference,
)


class ArtDtrackTfPublisher(Node):
    """Receive data from the markerbased tracking."""

    def __init__(self):
        """Initialize the MarkerbasedTracking node."""
        super().__init__("markerbased_tracking")
        self.ip = (
            self.declare_parameter("ip", "192.168.0.53")
            .get_parameter_value()
            .string_value
        )
        self.port = (
            self.declare_parameter("port", 4100).get_parameter_value().integer_value
        )
        self.buffer_size = (
            self.declare_parameter("buffer_size", 1024)
            .get_parameter_value()
            .integer_value
        )
        self.con_timeout = (
            self.declare_parameter("timeout", 3.0).get_parameter_value().double_value
        )
        self.frame = (
            self.declare_parameter("frame", "base_track")
            .get_parameter_value()
            .string_value
        )
        self.bodies = (
            self.declare_parameter("bodies", ["Tree Target 2", "pen", "antlers"])
            .get_parameter_value()
            .string_array_value
        )
        self.measurement_tools = (
            self.declare_parameter("measurement_tools", ["M1", "M2"])
            .get_parameter_value()
            .string_array_value
        )
        self.measurement_tool_reference_bodies = (
            self.declare_parameter("measurement_tool_reference_bodies", ["MR1", "MR2"])
            .get_parameter_value()
            .string_array_value
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.receiver = ArtDtrackReceiver(
            ip=self.ip, port=self.port, con_timeout=self.con_timeout
        )

    def start(self):
        while rclpy.ok():
            try:
                msg = self.receiver.receive(self.buffer_size)
                # print(msg)
                for body in msg.bodies:
                    if body.id >= 0 and body.id < len(self.bodies):
                        self.body2tf(body, msg.timestamp_full)
                for body in msg.measurement_tools:
                    if body.id >= 0 and body.id < len(self.measurement_tools):
                        self.mt2tf(body, msg.timestamp_full)
                for body in msg.measurement_tool_reference_bodies:
                    if body.id >= 0 and body.id < len(
                        self.measurement_tool_reference_bodies
                    ):
                        self.mtr2tf(body, msg.timestamp_full)
            except socket.error:
                self.get_logger().warn("Currently no connection to the UDP receiver.")
                continue

    def body2tf(self, body: DtrackBody, ts: float):
        t = TransformStamped()
        t.header.frame_id = self.frame
        t.child_frame_id = self.bodies[body.id]  # ex.: "Tree Target 2"
        t.transform.translation.x = body.x / 1000
        t.transform.translation.y = body.y / 1000
        t.transform.translation.z = body.z / 1000
        # swap y and z
        # body.rot[1], body.rot[2] = [body.rot[2], body.rot[1]]
        quaternion_wxyz = mat2quat(body.rot)
        t.transform.rotation.w = quaternion_wxyz[0]
        t.transform.rotation.x = quaternion_wxyz[1]
        t.transform.rotation.y = quaternion_wxyz[2]
        t.transform.rotation.z = quaternion_wxyz[3]
        t.header.stamp.sec = floor(ts)
        t.header.stamp.nanosec = floor((ts % 1) * 10e8)
        self.tf_broadcaster.sendTransform(t)

    def mt2tf(self, body: DtrackMeasurementTool, ts: float):
        t = TransformStamped()
        # t.header.frame_id = self.frame
        t.header.frame_id = self.measurement_tool_reference_bodies[
            body.id
        ]  # use MT reference body with same ID, make sure to align M1 with MR1 etc.
        t.child_frame_id = self.measurement_tools[body.id]
        t.transform.translation.x = body.x / 1000
        t.transform.translation.y = body.y / 1000
        t.transform.translation.z = body.z / 1000
        # swap y and z
        # body.rot[1], body.rot[2] = [body.rot[2], body.rot[1]]
        quaternion_wxyz = mat2quat(body.rot)
        t.transform.rotation.w = quaternion_wxyz[0]
        t.transform.rotation.x = quaternion_wxyz[1]
        t.transform.rotation.y = quaternion_wxyz[2]
        t.transform.rotation.z = quaternion_wxyz[3]
        t.header.stamp.sec = floor(ts)
        t.header.stamp.nanosec = floor((ts % 1) * 10e8)
        self.tf_broadcaster.sendTransform(t)

    def mtr2tf(self, body: DtrackMeasurementToolReference, ts: float):
        t = TransformStamped()
        t.header.frame_id = self.frame
        t.child_frame_id = self.measurement_tool_reference_bodies[body.id]  # ex.: "MR1"
        t.transform.translation.x = body.x / 1000
        t.transform.translation.y = body.y / 1000
        t.transform.translation.z = body.z / 1000
        # swap y and z
        # body.rot[1], body.rot[2] = [body.rot[2], body.rot[1]]
        quaternion_wxyz = mat2quat(body.rot)
        t.transform.rotation.w = quaternion_wxyz[0]
        t.transform.rotation.x = quaternion_wxyz[1]
        t.transform.rotation.y = quaternion_wxyz[2]
        t.transform.rotation.z = quaternion_wxyz[3]
        t.header.stamp.sec = floor(ts)
        t.header.stamp.nanosec = floor((ts % 1) * 10e8)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArtDtrackTfPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, args=())
    spin_thread.daemon = True
    spin_thread.start()
    node.start()


if __name__ == "__main__":
    main()
