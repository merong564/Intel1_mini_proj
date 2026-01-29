# import rclpy
# from rclpy.node import Node
# from detect_msg.msg import Rcinfo

# from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# class Move(Node):
#     def __init__(self):
#         super().__init__('move_node')

#         self.subscription = self.create_subscription(
#             Rcinfo,
#             'detected_msg',
#             self.detect_sub_callback,
#             10)
        
#     def detect_sub_callback(self, msg):
#         self.detected = msg.detected
#         self.pt_map = msg.map
#         self.distance = msg.dist 



# def main():
#     rclpy.init()

#     navigator = TurtleBot4Navigator()

#     # Start on dock
#     if not navigator.getDockedStatus():
#         navigator.info('Docking before intialising pose')
#         navigator.dock()

#     # Set initial pose
#     initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
#     navigator.setInitialPose(initial_pose)

#     # Wait for Nav2
#     navigator.waitUntilNav2Active()

#     # Set goal poses
#     # goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
#     ### code change #####
#     goal_pose = navigator.getPoseStamped([-1.998, 0.818], TurtleBot4Directions.EAST)

# #   Position(-1.55069, 0.0668084, 0), Orientation(0, 0, -0.962154, 0.272507) = Angle: -2.5896


#     # Undock
#     navigator.undock()

#     # Go to each goal pose
#     navigator.startToPose(goal_pose)

#     rclpy.shutdown()

#     # rclpy.init()
#     # move_node = Move()

#     # try:
#     #     while rclpy.ok() and not move_node.should_exit:
#     #         rclpy.spin(move_node, timeout_sec=0.1)
#     # except KeyboardInterrupt:
#     #     pass
#     # finally:
#     #     move_node.destroy_node()
#     #     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from detect_msg.msg import Rcinfo

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class Move(Node):
    def __init__(self, navigator):
        super().__init__('move_node')
        self.get_logger().info(f'Move node init start')
        self.navigator = navigator
        self.detected = False
        self.pt_map = None
        self.distance = 0.0
        self.should_exit = False
        self.navigation_initialized = False
        self.previous_detected = None  # 이전 상태 추적
        
        # 초기 목표 위치 설정
        self.default_goal = [-1.998, 0.818]

        self.subscription = self.create_subscription(
            Rcinfo,
            'detected_msg',
            self.detect_sub_callback,
            10)
        
        # 타이머로 navigation 초기화 및 실행
        self.timer = self.create_timer(0.5, self.navigation_callback)

        self.get_logger().info(f'self.navigation_initialized:{self.navigation_initialized}')
            
        
    def detect_sub_callback(self, msg):
        self.detected = msg.detected
        self.pt_map = msg.map  # geometry_msgs/PointStamped
        self.distance = msg.dist
        
        if self.pt_map is not None:
            self.get_logger().info(
                f'Received: detected={self.detected}, '
                f'dist={self.distance}, '
                f'map=({self.pt_map.point.x:.2f}, {self.pt_map.point.y:.2f}, {self.pt_map.point.z:.2f})'
            )
        else:
            self.get_logger().info(f'Received: detected={self.detected}, dist={self.distance}')
    
    def navigation_callback(self):
        self.get_logger().info(f'nav callback, {self.navigation_initialized}')
        # 한 번만 초기화
        if not self.navigation_initialized:
            
            self.get_logger().info(f'self.navigation_initialized:{self.navigation_initialized}')
            
            # Start on dock
            # if not self.navigator.getDockedStatus():
            #     self.navigator.info('Docking before initialising pose')
            #     self.navigator.dock()
            self.navigator.dock()

            # Set initial pose
            initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
            self.navigator.setInitialPose(initial_pose)

            # Wait for Nav2
            self.navigator.waitUntilNav2Active()

            # Undock
            self.navigator.undock()
            
            self.get_logger().info('Navigation initialized. Monitoring detection...')
            self.navigation_initialized = True
            return
        
        # detected 상태가 변경되었을 때만 새로운 목표로 이동
        if self.detected and self.pt_map is not None:
            goal_x = self.pt_map.point.x
            goal_y = self.pt_map.point.y
            
            self.get_logger().info(f'Target DETECTED! Moving to map coordinates: ({goal_x:.2f}, {goal_y:.2f})')
            
            goal_pose = self.navigator.getPoseStamped(
                [goal_x, goal_y], 
                TurtleBot4Directions.EAST
            )
            
            self.navigator.startToPose(goal_pose)
            
        else:
            # detected=False: 초기 목표값으로 이동
            self.get_logger().info(f'No target. Moving to default goal: {self.default_goal}')
            
            goal_pose = self.navigator.getPoseStamped(
                self.default_goal, 
                TurtleBot4Directions.EAST
            )
            
            self.navigator.startToPose(goal_pose)


def main():
    rclpy.init()
    
    navigator = TurtleBot4Navigator()
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
