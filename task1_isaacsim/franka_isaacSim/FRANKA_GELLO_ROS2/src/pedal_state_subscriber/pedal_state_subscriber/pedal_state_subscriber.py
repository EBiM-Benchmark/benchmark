import rclpy
from rclpy.node import Node
from std_msgs.msg import String


STATE_TO_ACTION = {
    "A": "forward",
    "B": "backward",
    "A+C": "left",
    "B+C": "right",
}


class PedalStateSubscriber(Node):
    def __init__(self):
        super().__init__("pedal_state_subscriber")
        self.create_subscription(String, "/pedal/state", self.cb, 10)
        print("pedal_state_subscriber started", flush=True)

    def cb(self, msg):
        action = STATE_TO_ACTION.get(msg.data)
        if action:
            print(action, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = PedalStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
