import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PedalStatePublisher(Node):
    def __init__(self):
        super().__init__("pedal_state_publisher")
        self.pub = self.create_publisher(String, "/pedal/state", 10)
        self.pressed = set()
        self.get_logger().info("Pedal publisher started. a=A, b=B, c=C")

    def publish_state(self):
        order = ["A", "B", "C"]
        state = "+".join([x for x in order if x in self.pressed])
        if state == "":
            state = "NONE"

        msg = String()
        msg.data = state
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {state}")


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.02)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = PedalStatePublisher()
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == "\x03":
                break

            if key == "a":
                node.pressed = {"A"}
                node.publish_state()

            elif key == "b":
                node.pressed = {"B"}
                node.publish_state()

            elif key == "c":
                if "A" in node.pressed:
                    node.pressed = {"A", "C"}
                elif "B" in node.pressed:
                    node.pressed = {"B", "C"}
                else:
                    node.pressed = {"C"}
                node.publish_state()

            elif key == " ":
                node.pressed = set()
                node.publish_state()

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.pressed = set()
        node.publish_state()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()