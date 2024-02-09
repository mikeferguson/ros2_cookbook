# rclpy: TF2

The TF2 library provides easy access to transformations. All of the examples below
require a dependency on the _tf2_ros_ package.

## Listening for Transforms

```python
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
```

## Applying Transforms

TF2 can be extended by packages that provide implementations of _transform_.
The _tf2_geometry_msgs package provides these for _geometry_msgs_. The example
below uses _geometry_msgs.msg.PointStamped_ - but this should work for any
data type in _geometry_msgs_:

```python
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# Setup buffer/listener as above

p1 = PointStamped()
p1.header.frame_id = 'source_frame'
# fill in p1

p2 = buffer.transform(p1, 'target_frame')
```

### Using the Latest Transform

```python
from rclpy.time import Time

# Setting the stamp to a blank Time() instance will use the latest available data
p1 = PointStamped()
p1.header.frame_id = 'source_frame'
p1.header.stamp = Time().to_msg()

p2 = buffer.transform(p1, 'target_frame')
```

## Transformations

In ROS 1, tf included the _transformations_ module. tf2 has no similar module.
It is recommended to use transforms3d Python package, which is available through
apt on Ubuntu 22.04:

```
sudo apt-get install python3-transforms3d
```

Or via pip on other operating systems:

```
sudo pip3 install transforms3d
```

For instance, to rotate a point:

```python
import numpy as np
from transforms3d.quaternion import quat2mat

# Create rotation matrix from quaternion
R = quat2mat([w, x, y, z])
# Create a vector to rotate
V = np.array([x, y, z]).reshape((3, 1))
# Rotate the vector
M = np.dot(R, V)

p = PointStamped()
p.point.x = M[0, 0]
p.point.y = M[1, 0]
p.point.z = M[2, 0]
```

Additionally, in ROS 2 Humble there is the [tf_transformations package](https://github.com/DLu/tf_transformations)
which should make it as easy as changing your imports from tf.transformations to tf_transformations.
