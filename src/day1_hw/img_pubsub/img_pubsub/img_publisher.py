#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.pub =self.create_publisher(Image,'/image',10)
        self.timer =self.create_timer(0.5,self.publish_image)# 2Hz

    def publish_image(self):
        w, h =320,240
        msg = Image()
        msg.header.stamp =self.get_clock().now().to_msg()
        msg.header.frame_id ='camera'
        msg.height = h
        msg.width = w
        msg.encoding ='rgb8'
        msg.is_bigendian =0
        msg.step = w *3
        msg.data = bytes([0, 0, 255] * (h * w))

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImagePublisher())
    rclpy.shutdown()

if __name__ =='__main__':
    main()

