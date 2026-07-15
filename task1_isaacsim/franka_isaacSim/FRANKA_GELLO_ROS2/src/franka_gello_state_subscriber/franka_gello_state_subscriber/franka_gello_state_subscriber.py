import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class DuoSub(Node):
    def __init__(self):
        super().__init__("test_gello_duo_sub")
        self.create_subscription(JointState, "/left/gello/joint_states", self.left_arm_cb, 10)
        self.create_subscription(JointState, "/right/gello/joint_states", self.right_arm_cb, 10)
        self.create_subscription(Float32, "/left/gripper/gripper_client/target_gripper_width_percent", self.left_gripper_cb, 10)
        self.create_subscription(Float32, "/right/gripper/gripper_client/target_gripper_width_percent", self.right_gripper_cb, 10)

    def left_arm_cb(self, msg):
        print("LEFT ARM :", ["%.3f" % x for x in msg.position], flush=True)

    def right_arm_cb(self, msg):
        print("RIGHT ARM:", ["%.3f" % x for x in msg.position], flush=True)

    def left_gripper_cb(self, msg):
        print("LEFT GRIPPER :", "%.3f" % msg.data, flush=True)

    def right_gripper_cb(self, msg):
        print("RIGHT GRIPPER:", "%.3f" % msg.data, flush=True)

rclpy.init()
node = DuoSub()
print("Listening to left/right arm + gripper...")
rclpy.spin(node)
