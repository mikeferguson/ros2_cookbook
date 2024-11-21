# rclpy: Time

There are four separate ways that you may see time represented in ROS 2:

 * Plain ol' `int`: representing a number of nanoseconds.
 * `float`: representing the fractional number of seconds
 * [`rclpy.time.Time`](https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/time.py) - the preferred Pythonic interface
 * [`builtin_interfaces/msg/Time.msg`](https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg): the message representation.

Here's how you can convert them.

| Converting this →<br>to this ↓ | int                            | float                                                                                              | rclpy                 | msg                       |
| ------------------------------ | ------------------------------ | -------------------------------------------------------------------------------------------------- | --------------------- | ------------------------- |
| int                            | -                              | `int(t * 1e9)`                                                                                     | `t.nanoseconds`       | `t.sec * 1e9 + t.nanosec` |
| float                          | `t / 1e9`                      | -                                                                                                  | `t.nanoseconds / 1e9` | `t.sec + t.nanosec / 1e9` |
| rclpy                          | `Time(nanoseconds=t)`          | `nano, sec = math.modf(t)`<br>`Time(int(sec), int(nano * 1e9))`                                    | -                     | `Time.from_msg(t)`        |
| msg                            | `Time(nanoseconds=t).to_msg()` | `nano, sec = math.modf(t)`<br>`builtin_interfaces.msg.Time(sec=int(sec), nanosec=int(nano * 1e9))` | `t.to_msg()`          | -                         |

Important notes:

 * You cannot do comparisons/math between mixed types or messages
 * It is only mildly infuriating that `rclpy.time.Time` has the full word `nanoseconds` accessor and the message has `nanosec`.

## Now

To get the equivalent of rospy.Time.now(), you now need a ROS 2 node:

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

## Timers

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

## Rates

Using Rate objects in ROS 2 is a bit more complex than in ROS 1. Due to implementation
details, we need to spin() or the sleep() function will block. This is most easily
accomplished using a thread:

```python
import threading

# Run spin in a thread, make thread daemon so we don't have to join it to exit
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

rate = node.create_rate(10)
while rclpy.ok():
    print('This prints at 10hz')
    rate.sleep()
```
