#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.sub =self.create_subscription(Image,'/image',self.cb,10)

    def cb(self, msg: Image):
        self.get_logger().info(
            f"Image received: {msg.width}x{msg.height}, encoding={msg.encoding}, data={len(msg.data)} bytes"
                )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImageSubscriber())
    rclpy.shutdown()

if __name__ =='__main__':
    main()
