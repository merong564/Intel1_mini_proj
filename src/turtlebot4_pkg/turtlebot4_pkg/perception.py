import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from sensor_msgs.msg import Image as ROSImage, CameraInfo, CompressedImage
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion,Point
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # <--- [추가 1] QoS 관련 임포트
from detect_msg.msg import Rcinfo

from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from std_msgs.msg import Bool

import numpy as np
import cv2
import threading
import math

from rclpy.time import Time

from message_filters import Subscriber, ApproximateTimeSynchronizer
import time
from queue import Queue
from ultralytics import YOLO


class DepthToMap(Node):
    def __init__(self,model):
        super().__init__('depth_to_map_node')
        self.model = model
        self.bridge = CvBridge()
        self.K = None  # Camera intrinsics
        self.lock = threading.Lock()
        self.processed_rgb = None  # 가공된 이미지를 저장할 변수 추가
        self.is_navigating = False  # goal을 보냈는지 확인하는 플래그
        self.arrived = False # AMR이 default goal에 도착했는지 확인하는 플래그
        self.detected=False  # AMR이 RC car를 인지했는지 확인하는 플래그

        #### QoS Add
        self.sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Build topic names using namespace
        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'
        # self.depth_topic = f'/stereo_bag'
        # self.rgb_topic = f'compressed_bag'
        # self.info_topic = f'cam_info_bag'

        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None
        self.distance= None  # 인지되는 객체의 거리값 [m]
        self.classNames = model.names if hasattr(model, 'names') else ['Object']



        # TF2 buffer for transforms (camera_frame -> base_link -> map)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Dock, set initial pose, undock
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        # Publishers for the processed images (for rqt_image_view)
        self.rgb_sub = None
        self.depth_sub = None
        self.ts = None

        # RC car 정보 퍼블리시
        self.publisher = self.create_publisher(Rcinfo, 'detected_msg', 10)
        self.timer = self.create_timer(1.0, self.detect_publish)


        # Subscribe to camera intrinsics (needed once)
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)

        # Subscribe whether robot is arrived at default goal
        self.create_subscription(Bool, '/arrived_default_goal', self.arrived_info_callback, 1)


        # Thread for mouse click input
        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        # Delay to allow TF tree to stabilize
        self.get_logger().info("TF Tree stabilization starting. Will begin transforms in 5 sec.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info("TF Tree stabilized. Starting image processing & publishing.")
        # Loop timer to process images and publish overlays
        self.timer = self.create_timer(0.2, self.process_and_publish)
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        """ Store camera intrinsics (fx, fy, cx, cy) from CameraInfo """
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)
            if not self.logged_intrinsics:
                self.get_logger().info(
                    f"Camera intrinsics: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                    f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
                )
                self.logged_intrinsics = True
    
    def arrived_info_callback(self, arrived_msg):
        if arrived_msg.data and self.rgb_sub is None:
            self.get_logger().info('Robot arrived at default goal')
            self.arrived = True

            # Time-synchronized subscribers for RGB + Depth images
            ######## QoS Add for deducing camera delay
            self.rgb_sub = Subscriber(self, CompressedImage, self.rgb_topic, qos_profile=self.sensor_qos_profile)
            self.depth_sub = Subscriber(self, ROSImage, self.depth_topic, qos_profile=self.sensor_qos_profile)

            # ApproximateTimeSynchronizer: allows slight timestamp mismatch
            self.ts = ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub],
                queue_size=10,
                slop=3  # default: 0.1
            )
            self.ts.registerCallback(self.synced_callback)
            self.get_logger().info('Perception started successfully!')
        elif not arrived_msg.data:
            self.get_logger().info('Wait for arriving...')
            

    def synced_callback(self, rgb_msg, depth_msg):
        """ Called when RGB and Depth frames arrive together (synchronized) """
        try:
            with self.lock:
                # Decode RGB image from compressed format
                np_arr = np.frombuffer(rgb_msg.data, np.uint8)
                rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                # self.get_logger().error(f"########rgb: {rgb}, size: {rgb.size}")
                if rgb is not None and rgb.size > 0:
                    if not self.logged_rgb_shape:
                        self.get_logger().info(f"!!sync success!!")
                        self.logged_rgb_shape = True
                    self.rgb_image = rgb

                # Convert Depth image from ROS format to OpenCV
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                if depth is not None and depth.size > 0:
                    if not self.logged_depth_shape:
                        self.get_logger().info(f"Depth image shape: {depth.shape}")
                        self.logged_depth_shape = True
                    self.depth_image = depth
                    self.camera_frame = depth_msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Synced callback failed: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        """ Store pixel coordinates when user clicks on RGB image """
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked pixel: ({x}, {y})")

    def gui_loop(self):
        """ Basic OpenCV window to accept mouse clicks for pixel selection """
        cv2.namedWindow('Click to select pixel', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Click to select pixel', 640, 480)
        cv2.setMouseCallback('Click to select pixel', self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            img = None
            with self.lock:
                if self.processed_rgb is not None:
                    img = self.processed_rgb.copy()
                elif self.rgb_image is not None:
                    img = self.rgb_image.copy()
            if img is not None:
                cv2.imshow('Click to select pixel', img)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    self.gui_thread_stop.set()
            else:
                cv2.waitKey(10)
    
    # RC car 정보 퍼블리시 콜백함수
    def detect_publish(self):
        if self.detected and self.goal is not None:
            detect_msg = Rcinfo()
            # detect_msg.x = float(self.target_u)
            # detect_msg.y = float(self.target_v)
            detect_msg.dist = float(self.distance)
            detect_msg.detected = bool(self.detected)
            detect_msg.goal = self.goal
            self.get_logger().info(f'goal point: {self.goal}')
            
            self.publisher.publish(detect_msg)
            self.detected = False

    def process_and_publish(self):
        """ Process RGB & Depth, overlay info, transform clicked point, and publish """
        # debugging add
        known_frames = self.tf_buffer.all_frames_as_yaml()
        center_x=-1
        center_y=-1

        if 'map' not in known_frames:
             self.get_logger().error("ALERT: TF Buffer does NOT contain 'map'! (Listening to wrong topic?)")
        # all_frames = self.tf_buffer.all_frames_as_yaml()
        # self.get_logger().info(f"▶▶▶ CURRENT FRAMES: \n{all_frames}")
        # debugging add
        
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            click = self.clicked_point
            frame_id = getattr(self, 'camera_frame', None)

        if rgb is not None and depth is not None and frame_id:
            self.get_logger().info(f'Convert 3D coordinate Start')
            try:
                rgb_display = rgb.copy()
                depth_display = depth.copy()

                # Normalize and color-map the raw depth image for better visualization
                depth_normalized = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

                results = self.model.predict(
                    ##########33
                    rgb, 
                    stream=True, 
                    imgsz=416,      # 704 → 416으로 변경 (추론만 작은 크기로)
                    conf=0.5,       # confidence 임계값 추가
                    verbose=False   # 로그 끄기
                )

                target_box = None

                for r in results:
                    if not hasattr(r, 'boxes') or r.boxes is None:
                        continue
                    for box in r.boxes:
                        cls = int(box.cls[0]) if box.cls is not None else 0
                        if cls != 1: 
                            continue
                        conf = float(box.conf[0]) if box.conf is not None else 0.0

                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        target_box = (x1, y1, x2, y2)
                        orig_cx = (x1 + x2) / 2
                        orig_cy = (y1 + y2) / 2

                        label = f"{self.classNames[cls]} {conf:.2f}"
                        ##########
                        cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(rgb, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)



                h, w = self.rgb_image.shape[:2]
                img_to_show = cv2.resize(self.rgb_image, (704, 704))

                if target_box is not None:
                    self.detected = True    # RC car 인지 여부 True
                    x1, y1, x2, y2 = target_box
                    center_x = int(orig_cx * (704 / w))
                    center_y = int(orig_cy * (704 / h))
                    
                    # 3. 리사이즈된 이미지(704) 위에 점 찍기
                    cv2.circle(img_to_show, (center_x, center_y), 5, (0, 255, 0), -1)
                else:
                    # 인지값이 없을 경우, 모든 좌표를 -1로 출력
                    self.detected = True    # RC car 인지 여부 False
                    self.get_logger().info(f'########## No Detect ##########')
                    # p = Point()
                    # p.x = float(-1)
                    # p.y = float(-1)
                    # self.point_publisher.publish(p)
                ########
                with self.lock:
                    # GUI 스레드에서 보여줄 이미지를 img_to_show(리사이즈+점)로 업데이트
                    self.processed_rgb = img_to_show.copy()

                    # Calculate depth and transform to map frame for clicked pixel
                    #x, y = click

                    # 이미 goal로 이동 중이라면 다시 gotoPose 명령을 보내지 않음
                    if self.is_navigating:
                        self.get_logger().info('Robot is already moving to the car...', once=True)
                        return
                    
                    x, y = center_x, center_y
                    if x < rgb_display.shape[1] and y < rgb_display.shape[0] \
                            and y < depth_display.shape[0] and x < depth_display.shape[1]:

                        z = float(depth_display[y, x]) / 1000.0  # mm -> meters
                        # self.get_logger().info(f'First if, z: {z}')
                        self.distance = z

                        if 0.2 < z < 5.0:  # Valid range filter
                            # self.get_logger().info(f'Sec if, z: {z}')
                            fx, fy = self.K[0, 0], self.K[1, 1]
                            cx, cy = self.K[0, 2], self.K[1, 2]

                            # Back-project to camera coordinate system
                            X = (x - cx) * z / fx
                            Y = (y - cy) * z / fy
                            Z = z

                            # Create PointStamped for TF transform
                            pt_camera = PointStamped()
                            pt_camera.header.stamp = Time().to_msg()
                            pt_camera.header.frame_id = frame_id
                            pt_camera.point.x = X
                            pt_camera.point.y = Y
                            pt_camera.point.z = Z

                            # self.get_logger().info(f'pt_camera.point {pt_camera.point}')

                            # Transform to map frame
                            try:
                                pt_map = self.tf_buffer.transform(
                                    pt_camera,
                                    'map',
                                    timeout=Duration(seconds=1.0)
                                )
                                self.get_logger().info(
                                    f"Map coordinate: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})"
                                )

                                ## move robot: go to pt_map
                                goal_pose = PoseStamped()
                                goal_pose.header.frame_id = 'map'
                                goal_pose.header.stamp = self.get_clock().now().to_msg()
                                goal_pose.pose.position.x = pt_map.point.x
                                goal_pose.pose.position.y = pt_map.point.y
                                goal_pose.pose.position.z = 0.0
                                yaw = 0.0
                                qz = math.sin(yaw / 2.0)
                                qw = math.cos(yaw / 2.0)
                                goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

                                self.goal = goal_pose

                                # # 클릭 좌표 초기화
                                # with self.lock:
                                # self.clicked_point = None
                                self.is_navigating = True
                                center_x = -1
                                center_y = -1

                            except Exception as e:
                                self.get_logger().warn(f"TF transform failed: {e}")

                        # Draw overlay markers
                        text = f"{z:.2f} m" if 0.2 < z < 5.0 else "Invalid"
                        cv2.putText(rgb_display, '+', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.circle(rgb_display, (x, y), 4, (0, 255, 0), -1)
                        cv2.putText(depth_colored, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.circle(depth_colored, (x, y), 4, (255, 255, 255), -1)

                # Publish processed RGB image
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_display, encoding="bgr8")
                rgb_msg.header.stamp = self.get_clock().now().to_msg()
                self.rgb_pub.publish(rgb_msg)

                # Publish processed Depth image (color-mapped)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding="bgr8")
                depth_msg.header.stamp = self.get_clock().now().to_msg()
                self.depth_pub.publish(depth_msg)

            except Exception as e:
                self.get_logger().warn(f"Image process/publish error: {e}")

    def destroy_node(self):
        """ Ensure GUI thread stops cleanly """
        self.gui_thread_stop.set()
        super().destroy_node()


def main():
    model_path = '/home/rokey/Desktop/Intel1_mini_proj/src/turtlebot4_pkg/turtlebot4_pkg/yolov11n_amr.pt'
    model = YOLO(model_path, task='detect')

    rclpy.init()
    node = DepthToMap(model)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.gui_thread_stop.set()
    node.gui_thread.join()
    node.destroy_node()


if __name__ == '__main__':
    main()

# this code performs depth to 3D point transformation and navigation goal setting
# it uses ROS2, OpenCV, and TurtleBot4 navigation stack
# it listens to depth images and camera info, computes 3D points from depth data,
# transforms them to the map frame, and sends navigation goals based on user clicks in a GUI
# it also displays RGB and depth images side by side with overlays for clicked points
# it uses a multi-threaded executor to handle GUI and image processing concurrently
# the node handles TF transformations to convert points from the camera frame to the map frame
# the user can click on the depth image to set a navigation goal, which the robot will then attempt to reach
# it also publishes processed RGB and depth images for visualization in rqt_image_view 
