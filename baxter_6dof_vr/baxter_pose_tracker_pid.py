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
import transforms3d


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

        self.base_tf = None

        self.goal_pos = np.array([0.8, -0.2, 0.25])
        self.goal_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.current_pos = self.goal_pos
        self.current_quaternion = self.goal_quaternion

        self.subscription = self.create_subscription(
            Pose, "target_pose", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        self.get_ee_pos()

        delta_pos = self.goal_pos - self.current_pos
        distance = np.linalg.norm(delta_pos)
        mov = tuple(np.clip(10 * delta_pos, -1, 1))
        if distance < 0.02:
            mov = (0, 0, 0)

        current_quat = [
            self.current_quaternion[3],
            self.current_quaternion[0],
            self.current_quaternion[1],
            self.current_quaternion[2],
        ]
        goal_quat = [
            self.goal_quaternion[3],
            self.goal_quaternion[0],
            self.goal_quaternion[1],
            self.goal_quaternion[2],
        ]
        mov_quat = transforms3d.quaternions.qmult(
            goal_quat, transforms3d.quaternions.qinverse(current_quat)
        )
        mov_euler = np.clip(
            2 * np.array(transforms3d.euler.quat2euler(mov_quat)), -0.9, 0.9
        )
        # mov_rot = tuple(mov_euler)
        print(mov_euler)

        if self.jumping():
            print("Still jumping")

        else:
            # if distance < 0.01:
            #     print("Arrived")
            # elif 0.01 < distance < 0.2:
            print("Servoing", mov)
            self.moveit2_servo.servo(linear=mov, angular=mov_euler)

        print(" ")

    def jump(self, goal_pos, goal_quat):
        self.moveit2.move_to_pose(
            position=goal_pos,
            quat_xyzw=goal_quat,
            cartesian=True,
        )

    def jumping(self):
        return (
            self.moveit2._MoveIt2__is_motion_requested
            or self.moveit2._MoveIt2__is_executing
        )

    def listener_callback(self, msg):
        if self.base_tf is None:
            self.get_base_tf()

        pose_base = do_transform_pose(msg, self.base_tf)

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

        self.get_ee_pos()

        delta_pos = self.goal_pos - self.current_pos
        distance = np.linalg.norm(delta_pos)

        if distance > 0.2 and not self.jumping():
            print("Starting jump")
            self.jump(self.goal_pos, self.goal_quaternion)

    def get_ee_pos(self):
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.frame,
                self.ee_frame,
                rclpy.time.Time(),
            )

            self.current_pos = np.array(
                [
                    ee_tf.transform.translation.x,
                    ee_tf.transform.translation.y,
                    ee_tf.transform.translation.z,
                ]
            )
            self.current_quaternion = np.array(
                [
                    ee_tf.transform.rotation.x,
                    ee_tf.transform.rotation.y,
                    ee_tf.transform.rotation.z,
                    ee_tf.transform.rotation.w,
                ]
            )

        except Exception as e:
            self.get_logger().info(
                f"Could not transform {self.frame} to {self.pose_frame}: {e}"
            )

    def get_base_tf(self):
        try:
            self.base_tf = self.tf_buffer.lookup_transform(
                self.frame,
                self.pose_frame,
                rclpy.time.Time(),
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
