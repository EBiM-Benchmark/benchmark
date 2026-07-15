import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_KEYS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "q": "turn_left",
    "e": "turn_right",
}


class KeyboardStatePublisher(Node):
    def __init__(self):
        super().__init__("keyboard_state_publisher")
        self.publisher = self.create_publisher(String, "/keyboard/state", 10)
        self.get_logger().info("Press w/a/s/d/q/e. Press Ctrl+C to exit.")

    def publish_key(self, key: str):
        msg = String()
        msg.data = key
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {key}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardStatePublisher()

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            key = sys.stdin.read(1).lower()

            if key in VALID_KEYS:
                node.publish_key(key)

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
