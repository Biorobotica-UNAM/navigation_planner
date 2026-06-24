#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class EnablePublisher(Node):
    def __init__(self):
        super().__init__("enable_publisher")

        # Declare and read parameter (default = False)
        self.declare_parameter("use_namespace", False)
        use_namespace = self.get_parameter("use_namespace").get_parameter_value().bool_value

        suffix = "/potential_fields/enable"

        if use_namespace:
            prefix = self.get_namespace()
            # ensure single slash handling
            if prefix == "/":
                topic = suffix
            else:
                topic = prefix + suffix
        else:
            topic = suffix

        self.pub = self.create_publisher(Bool, topic, 10)

        # Publish immediately, then retry every second.
        # The first message may be lost if DDS discovery has not yet matched
        # the subscriber in potential_fields_node; subsequent ticks guarantee
        # delivery once the connection is established.
        self.create_timer(1.0, self.publish_enable)
        self.publish_enable()

    def publish_enable(self):
        msg = Bool(data=True)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnablePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
