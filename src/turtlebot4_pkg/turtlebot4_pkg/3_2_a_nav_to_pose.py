import rclpy
from rclpy.node import Node
from detect_msg.msg import Rcinfo
from std_msgs.msg import Bool,String

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult

class Move(Node):
    def __init__(self, navigator):
        super().__init__('move_node')
        self.get_logger().info(f'Move node init start')
        self.navigator = navigator
        self.detected = False
        self.goal = None
        self.final_goal_pose = None
        # self.distance = 0.0
        # self.should_exit = False
        # self.navigation_initialized = False
        # self.previous_detected = None  # 이전 상태 추적
        ns = self.get_namespace()
        self.arrive_pub = self.create_publisher(Bool, f'{ns}/arrived_default_goal',10)

        self.arrived_success = False
        self.moving_to_default = False
        self.webcam_detected = True        ####### 추후 False 변경 필요

        
        # 초기 목표 위치 설정
        self.default_goal = [-1.998, 0.818]
        self.is_moving = False #is it move?
        self.is_spinning = False # 회전 중인지 확인

        self.yolo_sub = self.create_subscription(
            String,
            f'{ns}/yolo_result',
            self.web_sub_callback,
            10)

        self.subscription = self.create_subscription(
            Rcinfo,
            f'{ns}/detected_msg',
            self.detect_sub_callback,
            10)
        
        # 타이머로 navigation 초기화 및 실행
        self.timer = self.create_timer(0.5, self.navigation_callback)

        # self.get_logger().info(f'self.navigation_initialized:{self.navigation_initialized}')
            
        # 인지값 받아오는 서브스크라이버 콜백함수
    def detect_sub_callback(self, msg):
        self.detected = msg.detected
        self.goal = msg.goal  # geometry_msgs/PointStamped
        self.distance = msg.dist
        
        if self.goal is not None:
            self.goal_x = self.goal.pose.position.x
            self.goal_y = self.goal.pose.position.y
            self.get_logger().info(
                f'Received: detected={self.detected}, '
                f'dist={self.distance}, '
                f'goal=({self.goal_x:.2f}, {self.goal_y:.2f})'
            )
        else:
            self.get_logger().info(f'Received: detected={self.detected}, dist={self.distance}')

    def web_sub_callback(self, msg):
    #     # YOLO 노드에서 String(data="True") 또는 "False"를 보냄
        if msg.data == "True":
            self.webcam_detected = True
        else:
            self.webcam_detected = False

    # spin 도는 콜백함수, 목표 위치로 이동
    def navigation_callback(self):

        # 1. [도착 후 계속 실행] 도착 성공 플래그가 True면 계속 퍼블리시
        if self.arrived_success:
            msg = Bool()
            msg.data = True
            self.arrive_pub.publish(msg)
            self.get_logger().info(f'Is it arrived? {self.arrived_success}')

        # 2. [이동 중 모니터링] Default Goal로 이동 중일 때만 결과 확인 
        if self.moving_to_default:      # AMR이 gotoPose 중이면 다른 명령 중지
            self.get_logger().info(f'moving_to_default:{self.moving_to_default}')
            
            if self.navigator.isTaskComplete():   
                self.get_logger().info(f'navigator task : {self.navigator.isTaskComplete()}')     

                # gotoPose 작업 성공 여부 확인     
                result = self.navigator.getResult() 

                if result == TaskResult.SUCCEEDED:      # default goal 도착 시
                    self.get_logger().info('Goal Reached! Setting arrived flag to True.')
                    
                    # 성공 상태 저장 -> 이제 위의 1번 if문이 매번 실행됨
                    self.arrived_success = True 
                    
                    # 도착 직후 1회만 스핀 동작 수행
                    # self.navigator.spin(spin_dist=6.28)
                    
                else:
                    # 실패하거나 취소된 경우 (재시도 로직이 필요하면 여기에 추가)
                    self.get_logger().warn('Goal Failed or Canceled!')
                
                # 결과가 뭐든 간에 이동 태스크는 끝났으므로 플래그 해제
                self.moving_to_default = False
            
                # 이동 중(moving_to_default=True)일 때는 다른 명령 안 받게 리턴
                return
        
        # 3. [이동 시작] 아직 도착 안했고, 이동 중도 아니고, 욜로 감지되면 이동 시작
        if self.webcam_detected and not self.arrived_success and not self.moving_to_default:
            self.get_logger().info('YOLO Detected -> Start moving to Default Goal')
            
            default_goal = self.navigator.getPoseStamped(
                self.default_goal, 
                TurtleBot4Directions.EAST
            )
            self.navigator.startToPose(default_goal)       # default goal로 이동
            
            # 이동 시작했음을 표시 (이걸 True로 해야 2번 로직이 돔)
            self.moving_to_default = True
            return
        

        
        
        # 4. AMR에서 객체 인식한 경우, 새로운 목표(객체 위치)로 이동
        if self.detected and self.goal is not None:
            self.get_logger().info(f'Target DETECTED! Moving to map coordinates: ({self.goal_x:.2f}, {self.goal_y:.2f})')
            
            self.final_goal_pose = self.navigator.getPoseStamped(
                [self.goal_x, self.goal_y], 
                TurtleBot4Directions.EAST)
        
            # 아래 함수 사용하지 않고, 일정 거리 이내까지 직진하도록 수정 필요
            self.navigator.startToPose(self.final_goal_pose)
        


def main():
    rclpy.init()
    
    navigator = TurtleBot4Navigator()

    # 최종에서는 주석 해제 필요
    # # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # # Undock
    navigator.undock()
    
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
