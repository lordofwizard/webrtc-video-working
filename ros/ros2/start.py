import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VideoTriggerPublisher(Node):
    def __init__(self):
        super().__init__('video_trigger_publisher')
        self.publisher_ = self.create_publisher(String, 'video_trigger', 10)
        self.timer = self.create_timer(2.0, self.publish_trigger)
        self.get_logger().info('Publisher node has been started.')

    def publish_trigger(self):
        msg = String()
        msg.data = 'start_video'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = VideoTriggerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

