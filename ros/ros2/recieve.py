import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class VideoCommandReceiver(Node):
    def __init__(self):
        super().__init__('video_command_receiver')
        self.subscription = self.create_subscription(
            String,
            'video_trigger',
            self.listener_callback,
            10)
        self.get_logger().info('Subscriber node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        if msg.data == 'start_video':
            self.get_logger().info('Starting webcam.py...')
            subprocess.Popen(['python3', '../../webcam.py'])

def main(args=None):
    rclpy.init(args=args)
    node = VideoCommandReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

