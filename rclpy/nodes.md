# rclpy: Node Basics

Most nodes have publishers and subscribers, both of which are creating by
calling functions of the _Node_ instance:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        self.publisher = self.create_publisher(String, 'output_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info("Recieved: %s" % msg.data)
        self.publisher.publish(msg)

if __name___ == "__main__":
    rclpy.init()
    my_node = MyNode()
    rclpy.spin(my_node)
```

