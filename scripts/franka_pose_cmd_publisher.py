#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class PoseCommandPublisher(Node):
    def __init__(self):
        super().__init__('pose_command_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/franka/end_effector_pose_cmd', 10)

        # Define the center position and orientation
        self.center_position = Point(x=0.5, y=0.0, z=0.3)
        quat = R.from_euler('xyz', [np.pi, 0, 0]).as_quat()
        self.center_orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    def run(self):
        # Motion sequence: list of (motion_name, position_offset)
        motions = [
            ('forward', [0.2, 0.0, 0.0], 0.5),
            ('backward', [-0.2, 0.0, 0.0], 1),
            ('left', [0.0, 0.2, 0.0], 2),
            ('right', [0.0, -0.2, 0.0], 10),
            ('up', [0.0, 0.0, 0.2], 20),
            ('down', [0.0, 0.0, -0.2], 50),
        ]

        # Perform translational motions
        for motion_name, offset, freq in motions:
            self.get_logger().info(f"Performing motion: {motion_name}")
            total_time = 5.0  # Total time for the motion
            N = int(total_time*freq)  # Number of steps
            # Move to the offset position
            for i in range(N): # divid the target into N steps
                offset_target = np.array(offset) / N * (i+1)
                self.publish_pose(offset_target, self.center_orientation)
                time.sleep(1/freq) 

            # Move back to the center position
            self.publish_pose([0.0, 0.0, 0.0], self.center_orientation)
            time.sleep(2.0)

        # Perform slow rotation of the end-effector
        self.get_logger().info("Performing slow rotation")
        total_rotation = np.pi  # Rotate up to 360 degrees
        rotation_steps = 50
        rotation_increment = total_rotation / rotation_steps

        for step in range(rotation_steps + 1):
            angle = rotation_increment * step
            quat = R.from_euler('xyz', [np.pi+0.2, 0, 0 + angle]).as_quat()
            orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.publish_pose([0.0, 0.0, 0.0], orientation)
            time.sleep(0.2)

        quat = R.from_euler('xyz', [np.pi, 0, 0]).as_quat()
        orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.publish_pose([0.0, 0.0, 0.0], orientation)
        self.get_logger().info("Motion sequence completed.")

    def publish_pose(self, position_offset, orientation):
        # Update the message header
        msg = PoseStamped()
        msg.header.frame_id = "0"  # Adjust the frame ID as needed

        msg.header.stamp = self.get_clock().now().to_msg()

        # Set the new position
        msg.pose.position.x = self.center_position.x + position_offset[0]
        msg.pose.position.y = self.center_position.y + position_offset[1]
        msg.pose.position.z = self.center_position.z + position_offset[2]

        # Set the orientation
        msg.pose.orientation = orientation

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseCommandPublisher()
    node.run()

if __name__ == '__main__':
    main()
