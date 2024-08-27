import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2topic.api import get_topic_types

class TopicListNode(Node):
    def __init__(self):
        super().__init__('topic_list_node')
        self.publisher_ = self.create_publisher(String, 'topic_list', 10)
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        topics = get_topic_types()
        message = 'Available topics:\n'
        for topic, type in topics:
            message += f'Topic: {topic}, Type: {type}\n'
        self.publisher_.publish(String(data=message))
        self.get_logger().info('Publishing topic list')

def main(args=None):
    rclpy.init(args=args)
    node = TopicListNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
