import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNote, AudioNoteVector

class BeepBoopPublisher(Node):
    def __init__(self):
        super().__init__('beep_boop_publisher')
        self.publisher_ = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)
        self.timer = self.create_timer(1, self.publish_beep_boop) 
        self.published = False

    def publish_beep_boop(self):
        if self.published:
            return  

        msg = AudioNoteVector()
        msg.append = False

        note1 = AudioNote()
        note1.frequency = 880
        note1.max_runtime.sec = 0
        note1.max_runtime.nanosec = 300_000_000

        note2 = AudioNote()
        note2.frequency = 440
        note2.max_runtime.sec = 0
        note2.max_runtime.nanosec = 300_000_000

        note3 = AudioNote()
        note3.frequency = 880
        note3.max_runtime.sec = 0
        note3.max_runtime.nanosec = 300_000_000

        note4 = AudioNote()
        note4.frequency = 440
        note4.max_runtime.sec = 0
        note4.max_runtime.nanosec = 300_000_000

        msg.notes = [note1, note2, note3, note4]

        self.publisher_.publish(msg)
        self.get_logger().info('삐뽀삐뽀 소리 명령 전송 완료!')
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = BeepBoopPublisher()
    rclpy.spin_once(node, timeout_sec=2) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()