import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DataSubscriber(Node):

    def __init__(self):
        super().__init__("data_subscriber")
        self.subscription = self.create_subscription(
            String, "topic", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)

    data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
