import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardStateSubscriber(Node):
    def __init__(self):
        super().__init__("keyboard_state_subscriber")
        self.create_subscription(
            String,
            "/keyboard/state",
            self.keyboard_cb,
            10,
        )

    def keyboard_cb(self, msg: String):
        print(msg.data, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
