#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
import json
import os
import numpy as np

class BottleDetectorArmController(Node):
    JOINT_NAMES_FULL = [
        "panda_finger_joint1", "panda_joint1", "panda_joint2", "panda_finger_joint2",
        "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    ]

    JOINT_NAMES_GRIPPER = [
        "panda_finger_joint1", "panda_finger_joint2"
    ]

    def __init__(self):
        super().__init__('bottle_detector_arm_controller')
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_command', 10)
        self.subscription = self.create_subscription(String, '/object_label', self.listener_callback, 10)
        self.motion_triggered = set()
        self.poses = self.load_poses('poses.json')
        self.active_bottle = None

    def load_poses(self, file_path):
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load poses from {file_path}: {e}")
            return {}

    def publish_joint_state(self, name, position, velocity=None, effort=None):
        msg = JointState()
        msg.name = name
        msg.position = position
        msg.velocity = velocity if velocity else [0.0] * len(name)
        msg.effort = effort if effort else [0.0] * len(name)
        self.publisher_.publish(msg)

    def interpolate_and_publish(self, start, end, name, steps=100, delay=0.01):
        for i in range(steps + 1):
            t = i / steps
            interpolated = []
            for j in range(len(start)):
                if j == 0 or j == 3:
                    interpolated.append(start[j])
                else:
                    interpolated.append((1 - t) * start[j] + t * end[j])
            self.publish_joint_state(name, interpolated)
            time.sleep(delay)

    def move_between_poses(self, pose1, pose2, include_gripper=False):
        name = self.JOINT_NAMES_FULL[:len(pose1['position'])]
        if include_gripper:
            name = self.JOINT_NAMES_FULL
        self.interpolate_and_publish(pose1['position'], pose2['position'], name)

    def move_to_scan_pose(self):
        if self.active_bottle is None:
            return
        self.get_logger().info(f'Moving to scan pose for {self.active_bottle}...')
        pose = self.poses[self.active_bottle].get("scan_pose", {})
        self.publish_joint_state(
            name=self.JOINT_NAMES_FULL,
            position=pose.get("position", []),
            velocity=pose.get("velocity", []),
            effort=pose.get("effort", [])
        )
        time.sleep(3)

    def hold_gripper_state(self):
        self.publish_joint_state(
            name=self.JOINT_NAMES_GRIPPER,
            position=self.poses[self.active_bottle]["gripper_close"]["position"],
            velocity=self.poses[self.active_bottle]["gripper_close"].get("velocity", []),
            effort=self.poses[self.active_bottle]["gripper_close"].get("effort", [])
        )

    def execute_motion(self):
        p = self.poses[self.active_bottle]
        time.sleep(2)
        self.move_to_scan_pose()
        time.sleep(3)
        self.move_between_poses(p["scan_pose"], p["pick_pose"])

        time.sleep(3)
        self.publish_joint_state(
            name=self.JOINT_NAMES_GRIPPER,
            position=p["gripper_close"]["position"],
            velocity=p["gripper_close"].get("velocity", []),
            effort=p["gripper_close"].get("effort", [])
        )
        time.sleep(3)

        self.hold_gripper_state()
        time.sleep(5)
        self.move_between_poses(p["pick_pose1"], p["carry_pose"])
        self.move_between_poses(p["carry_pose"], p["drop_pose"])

        time.sleep(3)
        self.publish_joint_state(
            name=self.JOINT_NAMES_GRIPPER,
            position=p["gripper_open"].get("position", []),
            velocity=p["gripper_open"].get("velocity", []),
            effort=p["gripper_open"].get("effort", [])
        )
        time.sleep(3)
    def listener_callback(self, msg):
        label = msg.data.lower()
        self.get_logger().info(f"Received label: {label}")
        if label == "bottle":
            for key in self.poses:
                if key not in self.motion_triggered:
                    self.active_bottle = key
                    self.motion_triggered.add(key)
                    self.get_logger().info(f"{key} detected via label '{label}'. Executing motion...")
                    self.execute_motion()
                    break

def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectorArmController()
    node.active_bottle = "bottle1"
    node.move_to_scan_pose()
    node.get_logger().info("Waiting for YOLO to detect 'bottle1' or 'bottle2'...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

