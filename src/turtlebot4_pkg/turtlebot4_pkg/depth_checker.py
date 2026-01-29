import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point # 중심점 전달용
from detect_msg.msg import Rcinfo
from geometry_msgs.msg import PointStamped
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

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
        self.target_u = 1
        self.target_v = 1
        self.distance_m=1
        self.detected=False

        self.pt_map = None
        self.camera_frame = None 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # RC car 정보 퍼블리시
        self.publisher = self.create_publisher(Rcinfo, 'detected_msg', 10)
        self.timer = self.create_timer(1.0, self.detect_callback)     ###### 1.0 수정


    def point_callback(self, msg):
        if msg.x == -1 and msg.y == -1:
            self.detected = False
            self.get_logger().info(f'self.detected : {self.detected}')
        else:
            # YOLO 노드에서 받은 좌표 업데이트
            self.target_u = int(msg.x)
            self.target_v = int(msg.y)
            self.detected = True
            self.get_logger().info(f'####### target pos: {self.target_u}, {self.target_v} #######')

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'center point: x: {self.target_u}, y: {self.target_v}')

    # depth camera 서브스크라이브 콜백함수
    def depth_callback(self, msg):
        if self.should_exit:
            return
        
        if self.detected == False:
            return

        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo...')
            return

        # depth_image: uint16 or float32 in mm
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_mm.shape

        distance_mm = depth_mm[int(self.target_u), int(self.target_v)]
        self.distance_m = distance_mm / 1000.0

        self.get_logger().info(f'center point: x: {self.target_u}, y: {self.target_v}')
        self.get_logger().info(f'distance_m : {self.distance_m}')
        # 시각화용 정규화 (mm → m 고려)
        depth_vis = np.nan_to_num(depth_mm, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)  # mm
        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)

        # 컬러맵 적용
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        cv2.circle(depth_colored, (self.target_u, self.target_v), 5, (0, 0, 0), -1)
        cv2.line(depth_colored, (0, self.target_v), (width, self.target_v), (0, 0, 0), 1)
        cv2.line(depth_colored, (self.target_u, 0), (self.target_u, height), (0, 0, 0), 1)

        # 3D 좌표로 변환 실행
        self.camera_frame = msg.header.frame_id
        self.transform_3d()

        cv2.imshow('Depth Image with Center Mark', depth_colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True
    
    # RC car 정보 퍼블리시 콜백함수
    def detect_callback(self):
        if self.detected and self.pt_map is not None:
            detect_msg = Rcinfo()
            # detect_msg.x = float(self.target_u)
            # detect_msg.y = float(self.target_v)
            detect_msg.dist = float(self.distance_m)
            detect_msg.detected = bool(self.detected)
            detect_msg.map = self.pt_map            # 타겟의 3D 좌표
            self.get_logger().info(f'pt_map: {self.pt_map}')
            
            self.publisher.publish(detect_msg)
    
    def transform_3d(self):
        #frame_id = getattr(self, 'camera_frame', None)
        if self.camera_frame is None:
            self.get_logger().warn("camera_frame not set yet")
            return
    
        if 0.2 < self.distance_m < 5.0:
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]

            X = (self.target_u - cx) * self.distance_m / fx
            Y = (self.target_v - cy) * self.distance_m / fy
            Z = self.distance_m

            pt_camera = PointStamped()
            # pt_camera.header.stamp = self.get_clock().now().to_msg()

            pt_camera.header.stamp = Time().to_msg()

            #pt_camera.header.frame_id = frame_id
            pt_camera.header.frame_id = self.camera_frame
            pt_camera.point.x = X
            pt_camera.point.y = Y
            pt_camera.point.z = Z

            try:
                self.pt_map = self.tf_buffer.transform(
                    pt_camera,
                    'map',
                    timeout=Duration(seconds=1.0)
                )
                # self.pt_map = do_transform_point(pt_camera, transform)
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")

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
