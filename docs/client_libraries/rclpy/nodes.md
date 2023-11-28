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
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()
```

## Shutdown Hooks

ROS1 had rospy.on_shutdown() - but there is
[not an equivalent in ROS2](https://github.com/ros2/rclpy/issues/244). It really is not needed though, since we manually shut things down rather than
was the case in rospy which used many global variables:

```python
try:
    rclpy.spin(my_node)
except KeyboardInterrupt:
    pass
finally:
    my_node.on_shutdown()  # do any custom cleanup
    my_node.destroy_node()
    rclpy.shutdown()
```
