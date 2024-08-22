from flask import Flask, render_template, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

app = Flask(__name__)

class TopicListClient(Node):
    def __init__(self):
        super().__init__('topic_list_client')
        self.subscription = self.create_subscription(
            String,
            'topic_list',
            self.listener_callback,
            10
        )
        self.topic_list = []

    def listener_callback(self, msg):
        self.topic_list = msg.data.split('\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/topics')
def topics():
    rclpy.init()
    client = TopicListClient()
    rclpy.spin_once(client)
    rclpy.shutdown()
    return jsonify(client.topic_list)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
