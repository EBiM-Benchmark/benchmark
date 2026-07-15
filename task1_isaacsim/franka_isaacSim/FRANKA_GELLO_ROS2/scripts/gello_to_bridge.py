#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


LEFT_ISAAC_JOINT_NAMES = [
    "left_fr3v2_joint1",
    "left_fr3v2_joint2",
    "left_fr3v2_joint3",
    "left_fr3v2_joint4",
    "left_fr3v2_joint5",
    "left_fr3v2_joint6",
    "left_fr3v2_joint7",
]

RIGHT_ISAAC_JOINT_NAMES = [
    "right_fr3v2_joint1",
    "right_fr3v2_joint2",
    "right_fr3v2_joint3",
    "right_fr3v2_joint4",
    "right_fr3v2_joint5",
    "right_fr3v2_joint6",
    "right_fr3v2_joint7",
]

LEFT_GRIPPER_JOINT_NAME = "left_robotiq_opening"
RIGHT_GRIPPER_JOINT_NAME = "right_robotiq_opening"


class GelloToBridge(Node):
    def __init__(self):
        super().__init__("gello_to_bridge")

        self.left_pub = self.create_publisher(
            JointState,
            "/bridge/left_joint_commands",
            10,
        )

        self.right_pub = self.create_publisher(
            JointState,
            "/bridge/right_joint_commands",
            10,
        )

        self.left_gripper_pub = self.create_publisher(
            JointState,
            "/bridge/left_robotiq_joint_commands",
            10,
        )

        self.right_gripper_pub = self.create_publisher(
            JointState,
            "/bridge/right_robotiq_joint_commands",
            10,
        )


        self.left_sub = self.create_subscription(
            JointState,
            "/left/gello/joint_states",
            self.left_callback,
            10,
        )

        self.right_sub = self.create_subscription(
            JointState,
            "/right/gello/joint_states",
            self.right_callback,
            10,
        )

        self.left_gripper_sub = self.create_subscription(
            Float32,
            "/left/gripper/gripper_client/target_gripper_width_percent",
            self.left_gripper_callback,
            10,
        )

        self.right_gripper_sub = self.create_subscription(
            Float32,
            "/right/gripper/gripper_client/target_gripper_width_percent",
            self.right_gripper_callback,
            10,
        )

        self.get_logger().info("GELLO -> Isaac bridge started with arm and gripper remapping")

    def make_command(self, msg: JointState, isaac_joint_names) -> JointState:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id

        out.name = isaac_joint_names
        out.position = list(msg.position[:7])
        out.velocity = []
        out.effort = []

        return out

    def make_gripper_command(self, msg: Float32, joint_name: str) -> JointState:
        open_fraction = max(0.0, min(1.0, float(msg.data)))

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [joint_name]
        out.position = [open_fraction]
        out.velocity = []
        out.effort = []

        return out

    def left_callback(self, msg: JointState):
        self.left_pub.publish(self.make_command(msg, LEFT_ISAAC_JOINT_NAMES))

    def right_callback(self, msg: JointState):
        self.right_pub.publish(self.make_command(msg, RIGHT_ISAAC_JOINT_NAMES))

    def left_gripper_callback(self, msg: Float32):
        command = self.make_gripper_command(msg, LEFT_GRIPPER_JOINT_NAME)
        self.left_gripper_pub.publish(command)

    def right_gripper_callback(self, msg: Float32):
        command = self.make_gripper_command(msg, RIGHT_GRIPPER_JOINT_NAME)
        self.right_gripper_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = GelloToBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
