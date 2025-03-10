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

from .art_dtrack.art_dtrack_receiver import ArtDtrackReceiver, DtrackBody


class ArtDtrackTfPublisher(Node):
    """Recieve data from the markerbased tracking."""

    def __init__(self):
        """Initialize the MarkerbasedTracking node."""
        super().__init__('markerbased_tracking')
        self.ip = self.declare_parameter(
            'ip', '192.168.0.53').get_parameter_value().string_value
        self.port = self.declare_parameter(
            'port', 4100).get_parameter_value().integer_value
        self.buffer_size = self.declare_parameter(
            'buffer_size', 1024).get_parameter_value().integer_value
        self.con_timeout = self.declare_parameter(
            'timeout', 3.0).get_parameter_value().double_value
        self.frame = self.declare_parameter(
            'frame', 'base_track').get_parameter_value().string_value
        self.bodies = self.declare_parameter(
            'bodies', ['body_1']).get_parameter_value().string_array_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.receiver = ArtDtrackReceiver(ip=self.ip, port=self.port, con_timeout=self.con_timeout)
    
    def start(self):
        while rclpy.ok():
            try:
                msg = self.receiver.receive(self.buffer_size)
                for body in msg.bodies:
                    if body.id >= 0 and body.id < len(self.bodies):
                        self.body2tf(body, msg.timestamp_full)
            except socket.error:
                self.get_logger().warn('Currently no connection to the UDP reciever.')
                continue
    
    def body2tf(self, body: DtrackBody, ts: float):
        t = TransformStamped()
        t.header.frame_id = self.frame
        t.child_frame_id = self.bodies[body.id]
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
    spin_thread.deamon = True
    spin_thread.start()
    node.start()

if __name__ == '__main__':
    main()