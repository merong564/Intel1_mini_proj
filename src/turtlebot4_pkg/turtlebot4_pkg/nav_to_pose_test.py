import rclpy
from rclpy.node import Node
from detect_msg.msg import Rcinfo
from std_msgs.msg import Bool,String

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class Move(Node):
    def __init__(self, navigator):
        super().__init__('move_node')
        self.get_logger().info(f'Move node init start')
        self.navigator = navigator
        # self.detected = False
        # self.pt_map = None
        # self.distance = 0.0
        # self.should_exit = False
        # self.navigation_initialized = False
        # self.previous_detected = None  # 이전 상태 추적
        self.arrive_pub = self.create_publisher(Bool, '/arrived_default_goal',10)
        
        # 초기 목표 위치 설정
        # self.default_goal = [-1.998, 0.818]
        self.is_moving = True #is it move?
        # self.is_spinning = False # 회전 중인지 확인
        # self.yolo_detected = False

        # self.yolo_sub = self.create_subscription(
        #     String,
        #     'yolo_result',
        #     self.web_sub_callback,
        #     10)

        # self.subscription = self.create_subscription(
        #     Rcinfo,
        #     '/detected_msg',
        #     self.detect_sub_callback,
        #     10)
        
        # 타이머로 navigation 초기화 및 실행
        self.timer = self.create_timer(0.5, self.navigation_callback)

        # self.get_logger().info(f'self.navigation_initialized:{self.navigation_initialized}')
            
        # 인지값 받아오는 서브스크라이버 콜백함수
    # def detect_sub_callback(self, msg):
    #     self.detected = msg.detected
    #     self.pt_map = msg.map  # geometry_msgs/PointStamped
    #     self.distance = msg.dist
        
    #     if self.pt_map is not None:
    #         self.get_logger().info(
    #             f'Received: detected={self.detected}, '
    #             f'dist={self.distance}, '
    #             f'map=({self.pt_map.point.x:.2f}, {self.pt_map.point.y:.2f}, {self.pt_map.point.z:.2f})'
    #         )
    #     else:
    #         self.get_logger().info(f'Received: detected={self.detected}, dist={self.distance}')

    # def web_sub_callback(self, msg):
    #     # YOLO 노드에서 String(data="True") 또는 "False"를 보냄
    #     if msg.data == "True":
    #         self.yolo_detected = True
    #     else:
    #         self.yolo_detected = False

    # spin 도는 콜백함수, 목표 위치로 이동
    def navigation_callback(self):
        # self.get_logger().info(f'nav callback, {self.navigation_initialized}')
        # 네비게이터가 이미 다른 명령을 수행 중이면 건너뜀
        # if not self.navigator.isTaskComplete():
        #     return
        
        #when webcam detect, go to initial goal pose
        # if self.yolo_detected:
        #     goal_pose = self.navigator.getPoseStamped(
        #     self.default_goal, 
        #     TurtleBot4Directions.EAST
        # )
        
        #     self.navigator.startToPose(goal_pose)
        #     self.is_moving = True

            # when arrive default goal pose, publish 'arrive_pub' and spin in holding position
        if self.is_moving:
            # result = self.navigator.getResult()
            # Task가 성공적으로 끝났는지 확인 (SUCCEEDED = 1)
            # if result == 1: 
            msg = Bool()
            msg.data = True
            self.arrive_pub.publish(msg)
            self.get_logger().info('Goal Reached! Arrived message published.')

            # self.is_moving = False
            # self.navigator.spin(spin_dist=6.28) # 약 360도(6.28 rad) 제자리 회전
            # self.is_spinning = True
            return
                
        # if self.is_spinning:
        #     self.get_logger().info('Spin completed.')
        #     self.is_spinning = False
        #     return
        
        # detected 상태가 변경되었을 때만 새로운 목표로 이동
        # if self.detected and self.pt_map is not None:
        #     final_goal_x = self.pt_map.point.x
        #     final_goal_y = self.pt_map.point.y
            
        #     self.get_logger().info(f'Target DETECTED! Moving to map coordinates: ({final_goal_x:.2f}, {final_goal_y:.2f})')
            
        #     final_goal_pose = self.navigator.getPoseStamped(
        #         [final_goal_x, final_goal_y], 
        #         TurtleBot4Directions.EAST
        # )
        
        # self.navigator.startToPose(final_goal_pose)
        


def main():
    rclpy.init()
    
    navigator = TurtleBot4Navigator()

    # # 최종에서는 주석 해제 필요
    # # # Start on dock
    # if not navigator.getDockedStatus():
    #     navigator.info('Docking before initialising pose')
    #     navigator.dock()

    # # Set initial pose
    # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    # navigator.setInitialPose(initial_pose)

    # # Wait for Nav2
    # navigator.waitUntilNav2Active()

    # # # Undock
    # navigator.undock()
    
    move_node = Move(navigator)

    

    try:
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        move_node.get_logger().info('Shutdown requested via Ctrl+C.')
    finally:
        move_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
