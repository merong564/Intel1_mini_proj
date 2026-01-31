import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TestArrivedPublisher(Node):
    def __init__(self):
        super().__init__('test_arrived_publisher')
        
        # 1. 토픽 이름과 타입을 메인 코드와 동일하게 맞춤
        self.publisher_ = self.create_publisher(Bool, '/arrived_default_goal', 1)
        
        # 1초마다 메시지를 발행하는 타이머 생성
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.count = 0
        self.switch_time = 5  # 5초 후에 도착 신호(True)를 보냄

    def timer_callback(self):
        msg = Bool()
        
        # 2. 처음엔 False를 보내다가 일정 시간 후 True로 변경
        if self.count < self.switch_time:
            msg.data = False
            self.get_logger().info(f'[{self.count}s] Robot is moving... (Publishing False)')
        else:
            msg.data = True
            # 한 번 True가 되면 계속 True를 유지 (실제 도착 상태 유지)
            self.get_logger().info(f'[{self.count}s] Robot ARRIVED! Triggering perception... (Publishing True)')

        self.publisher_.publish(msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestArrivedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()