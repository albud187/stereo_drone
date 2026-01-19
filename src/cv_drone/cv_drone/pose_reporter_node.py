#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np

TS_POSE_GT = "gt_pose"
TP_POSE_RPY = "pose_RPY"


class PoseReporterNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_reporter_node")

        self.rpy_pose = None

        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

        # subscribers
        self.create_subscription(Pose, TS_POSE_GT, self.pose_cb, 10)

        # publishers
        self.rpy_pose_pub = self.create_publisher(Pose, TP_POSE_RPY, 10)
        
    def quat_to_rpy(self, q_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float]:
        """
        description: 
            - convert quaternion (qx,qy,qz,qw) to roll, pitch, yaw (radians)
            - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#
        inputs:
            - `q_xyzw` is quaternion that MUST be in form (qx, qy, qz, qw)
        outputs:
            - `result` (roll, pitch, yaw)
        """
        qx, qy, qz, qw = q_xyzw

        # roll 
        roll_comp_1 = 2.0 * (qw*qx + qy*qz)
        roll_comp_2 = 1.0 - 2.0 * (qx**2 + qy**2)
        roll = np.arctan2(roll_comp_1, roll_comp_2)

        # pitch 
        quat_pitch_calc = 2.0 * (qw*qy - qx*qz)

        pitch1 = 1.0 + quat_pitch_calc 
        pitch1 = np.clip(pitch1, 0.0, None)

        pitch2 = 1.0 - quat_pitch_calc 
        pitch2 = np.clip(pitch2, 0.0, None)

        pitch_comp_1 = np.sqrt(pitch1)
        pitch_comp_2 = np.sqrt(pitch2)
        pitch = -np.pi/2.0 + 2.0 * np.arctan2(pitch_comp_1, pitch_comp_2)

        # yaw 
        yaw_comp1 = 2.0*(qw*qz + qx*qy)
        yaw_comp2 = 1.0 - 2.0*(qy**2+qz**2)
        yaw = np.arctan2(yaw_comp1, yaw_comp2)

        result = (float(roll), float(pitch), float(yaw))
        return result 

    def pose_cb(self, msg: Pose) -> None:
        """
        description:
            - converts Pose orientation quaternion into RPY (radians), store in self.rpy_pose
        inputs:
            - `msg`: Pose with quaternion orientation
        outputS:
            - None
        """
        if self.rpy_pose is None:
            self.rpy_pose = Pose()

        self.rpy_pose.position = msg.position

        q = msg.orientation
        q_tuple = (q.x, q.y, q.z, q.w)
        
        roll, pitch, yaw = self.quat_to_rpy(q_tuple)

        self.rpy_pose.orientation.x = roll
        self.rpy_pose.orientation.y = pitch
        self.rpy_pose.orientation.z = yaw
        self.rpy_pose.orientation.w = 0.0  

    def timer_cb(self) -> None:
        if self.rpy_pose is not None:
            self.rpy_pose_pub.publish(self.rpy_pose)

def main(args=None):
    rclpy.init(args=args)
    node = PoseReporterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()