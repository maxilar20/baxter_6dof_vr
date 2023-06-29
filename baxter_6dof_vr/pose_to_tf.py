import contextlib
from geometry_msgs.msg import TransformStamped, PoseArray, Pose

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster



class FramePublisher(Node):

    def __init__(self):
        super().__init__('pose_tf2_frame_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.handle_aruco_pose,
            1)
        
        self.subscription  # prevent unused variable warning

    def handle_aruco_pose(self, msg):

        for pose in msg.poses:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_link_rotated'
            t.child_frame_id = "aruco"

            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    with contextlib.suppress(KeyboardInterrupt):
        rclpy.spin(node)
    rclpy.shutdown()