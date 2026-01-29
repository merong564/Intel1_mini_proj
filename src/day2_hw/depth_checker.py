import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point # 중심점 전달용
from detect_msg import Rcinfo


# ================================
# 설정 상수
# ================================
DEPTH_TOPIC = '/robot1/oakd/stereo/image_raw'  # Depth 이미지 토픽
CAMERA_INFO_TOPIC = '/robot1/oakd/stereo/camera_info'  # CameraInfo 토픽
MAX_DEPTH_METERS = 5.0                 # 시각화 시 최대 깊이 값 (m)
NORMALIZE_DEPTH_RANGE = 3.0            # 시각화 정규화 범위 (m)
# ================================

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.bridge = CvBridge()
        self.K = None
        self.should_exit = False
        self.target_u = None
        self.target_v = None

        self.subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            10)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10)

        # YOLO로부터 좌표 구독 (추가된 부분)
        self.point_sub = self.create_subscription(Point, '/center_point', self.point_callback, 10)

        self.publisher = self.create_publisher(Rcinfo, 'detected_msg', 10)

    def point_callback(self, msg):
        # YOLO 노드에서 받은 좌표 업데이트
        self.target_u = int(msg.x)
        self.target_v = int(msg.y)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            # self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.get_logger().info(f'center point: x: {self.target_u}, y: {self.target_v}')

    def depth_callback(self, msg):
        if self.should_exit:
            return

        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo...')
            return

        # depth_image: uint16 or float32 in mm
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_mm.shape

        # origin
        # cx = self.K[0, 2]
        # cy = self.K[1, 2]
        # u, v = int(cx), int(cy)
        # distance_mm = depth_mm[v, u]
        # distance_m = distance_mm / 1000.0  # mm → m

        #####
        # corrected_u = int(self.target_u * (width / 320))      # preview 토픽 가져올 경우
        # corrected_v = int(self.target_v * (height / 320))
        # corrected_u = int(self.target_u * (width))
        # corrected_v = int(self.target_v * (height))
        # corrected_u = np.clip(corrected_u, 0, width - 1)
        # corrected_v = np.clip(corrected_v, 0, height - 1)
        # distance_mm = depth_mm[corrected_v, corrected_u]
        distance_mm = depth_mm[int(self.target_u), int(self.target_v)]
        distance_m = distance_mm / 1000.0

        # self.get_logger().info(f"Image size: {width}x{height}, Distance at (u={u}, v={v}) = {distance_m:.2f} meters")
        self.get_logger().info(f'center point: x: {self.target_u}, y: {self.target_v}')
        self.get_logger().info(f'distance_m : {distance_m}')
        # 시각화용 정규화 (mm → m 고려)
        depth_vis = np.nan_to_num(depth_mm, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)  # mm
        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)

        # 컬러맵 적용
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # 중심점 시각화
        # cv2.circle(depth_colored, (u, v), 5, (0, 0, 0), -1)
        # cv2.line(depth_colored, (0, v), (width, v), (0, 0, 0), 1)
        # cv2.line(depth_colored, (u, 0), (u, height), (0, 0, 0), 1)
        # orgin
        # cv2.circle(depth_colored, (self.target_u, self.target_v), 5, (0, 0, 0), -1)
        # cv2.line(depth_colored, (0, self.target_v), (width, self.target_v), (0, 0, 0), 1)
        # cv2.line(depth_colored, (self.target_u, 0), (self.target_u, height), (0, 0, 0), 1)

        ####
        cv2.circle(depth_colored, (self.target_u, self.target_v), 5, (0, 0, 0), -1)
        cv2.line(depth_colored, (0, self.target_v), (width, self.target_v), (0, 0, 0), 1)
        cv2.line(depth_colored, (self.target_u, 0), (self.target_u, height), (0, 0, 0), 1)

        cv2.imshow('Depth Image with Center Mark', depth_colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True

def main():
    rclpy.init()
    node = DepthChecker()

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
