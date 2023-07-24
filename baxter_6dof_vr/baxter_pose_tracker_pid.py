#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2Servo
from pymoveit2.robots import baxter

import numpy as np
import time

from geometry_msgs.msg import Pose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import do_transform_pose


class PoseFollower(Node):
    def __init__(self):
        super().__init__("servo_pose_follower")

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=baxter.joint_names(),
            base_link_name=baxter.base_link_name(),
            end_effector_name=baxter.end_effector_name(),
            group_name=baxter.MOVE_GROUP_ARM,
            execute_via_moveit=True,
        )

        self.moveit2_servo = MoveIt2Servo(
            node=self,
            frame_id=baxter.base_link_name(),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frame = "base"
        self.pose_frame = "unity_world"  # TODO: Change to param
        self.ee_frame = "right_hand"  # TODO: Change to param

        self.goal_pos = np.array([0.8, -0.2, 0.25])
        self.current_pos = self.goal_pos
        self.goal_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        self.subscription = self.create_subscription(
            Pose, "target_pose", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.frame,
                self.ee_frame,
                rclpy.time.Time(),
            )

            self.current_pos = np.array(
                [
                    base_tf.transform.translation.x,
                    base_tf.transform.translation.y,
                    base_tf.transform.translation.z,
                ]
            )
            self.current_quaternion = np.array(
                [
                    base_tf.transform.rotation.x,
                    base_tf.transform.rotation.y,
                    base_tf.transform.rotation.z,
                    base_tf.transform.rotation.w,
                ]
            )

        except Exception as e:
            self.get_logger().info(
                f"Could not transform {self.frame} to {self.pose_frame}: {e}"
            )

        delta_pos = self.goal_pos - self.current_pos
        print(
            f"current pos: {self.current_pos}, goal pos: {self.goal_pos}, delta pos: {delta_pos}"
        )

        distance = np.linalg.norm(delta_pos)

        if distance < 0.1:
            print("Servoing", tuple(np.clip(10 * delta_pos, -1, 1)))
            self.moveit2_servo.servo(
                linear=tuple(np.clip(10 * delta_pos, -1, 1)), angular=(0, 0, 0)
            )

    def listener_callback(self, msg):
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.frame,
                self.pose_frame,
                rclpy.time.Time(),
            )

            pose_base = do_transform_pose(msg, base_tf)

            self.goal_pos = np.array(
                [
                    pose_base.position.x,
                    pose_base.position.y,
                    pose_base.position.z,
                ]
            )
            self.goal_quaternion = np.array(
                [
                    pose_base.orientation.x,
                    pose_base.orientation.y,
                    pose_base.orientation.z,
                    pose_base.orientation.w,
                ]
            )

            delta_pos = self.goal_pos - self.current_pos
            distance = np.linalg.norm(delta_pos)
            print(
                f"current pos: {self.current_pos}, goal pos: {self.goal_pos}, delta pos: {delta_pos}, distance: {distance}"
            )

            if distance < 0.1:
                print("Servoing", np.clip(10 * delta_pos, -1, 1))
                self.moveit2_servo.servo(
                    linear=tuple(np.clip(10 * delta_pos, -1, 1)), angular=(0, 0, 0)
                )

            else:
                print("jumping")
                self.moveit2.move_to_pose(
                    position=self.goal_pos,
                    quat_xyzw=self.goal_quaternion,
                    cartesian=True,
                )

        except Exception as e:
            self.get_logger().info(
                f"Could not transform {self.frame} to {self.pose_frame}: {e}"
            )


def main(args=None):
    rclpy.init(args=args)

    pose_follower = PoseFollower()

    # Declare parameters for position and orientation
    pose_follower.declare_parameter("position", [0.8, -0.2, 0.25])
    pose_follower.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    pose_follower.declare_parameter("cartesian", False)

    rclpy.spin(pose_follower)

    pose_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
