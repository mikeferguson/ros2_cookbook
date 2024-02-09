# rclpy: Parameters

## Declaring and Accessing Parameters

```python
# node is rclpy.node.Node instance
# 42 is a great default for a parameter
node.declare_parameter('my_param_name', 42)

# To get the value:
param = node.get_parameter('my_param_name').value
```

## Declaring Multiple Parameters at Once

There seems to be a fairly undocumentated part of the rclpy API:

```python
node.declare_parameters(
    namespace='',
    parameters=[
        ('my_param_name', 'default value'),
        ('my_other_param', 42)
    ]
)
```

## Dynamic Parameters

In ROS 2, all parameters can be dynamically updated through a ROS 2 service
(there is no need to define duplicate stuff as with dynamic reconfigure).

```python
from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        # Declare a parameter
        self.declare_parameter('my_param_name', 42)

        # Then create callback
        self.add_on_set_parameters_callback(self.callback)
    
    def callback(self, parameters):
        result = SetParametersResult(successful=True)

        for p in parameters:
            if p.name == 'my_param_name':
                if p.type_ != p.Type.INTEGER:
                    result.successful = False
                    result.reason = 'my_param_name must be an Integer'
                    return result
                if p.value < 20:
                    result.successful = False
                    result.reason = 'my_param_name must be >= 20;
                    return result

        # Return success, so updates are seen via get_parameter()
        return result
```

For an example of calling the set_parameters service, see
[ROS Answers](https://answers.ros.org/question/308541/ros2-rclpy-set-parameter-example/)
