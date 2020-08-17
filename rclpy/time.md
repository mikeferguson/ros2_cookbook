# rclpy: Time

To get the equivalent of rospy.Time.now(), you now need a ROS2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def some_func(self):
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
```

Converting from Duration to messages is common:

```python
import rclpy
from rclpy.duration import Duration

msg.duration = Duration(seconds=1).to_msg()
```

Timers are created from the Node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("my_node")

        # Create a timer that fires every quarter second
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        self.get_logger().info("timer has fired")
```
