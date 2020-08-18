# rclcpp: Parameters

Parameters need to be declared. At the same time, you can get the value if you
are not planning to update the value again later:

```cpp
// node is an instance of rclcpp::Node
// 42 is a great default for a parameter
int param = node.declare_parameter<int>("my_param_name", 42);
```

To get the value:

```cpp
int param;
node.get_parameter("my_param_name", param);
```

## Dynamic Parameters

In ROS2, all parameters can be dynamically updated through a ROS2 service
(there is no need to define duplicate stuff as with dynamic reconfigure).

The example below works in Eloquent or later (earlier ROS2 releases supported
only a single callback and had a slightly different API).
See the documentation for
[rclcpp::ParameterType](http://docs.ros2.org/latest/api/rclcpp/namespacerclcpp.html#enum-members)
for valid types.

```cpp
#include <vector>
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode()
  {
    // Declare parameters first

    // Then create callback
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::updateCallback, this, std::placeholders::_1));
  }

private:
  // This will get called whenever a parameter gets updated
  rcl_interfaces::msg::SetParametersResult updateCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const rclcpp::Parameter & param : parameters)
    {
      if (param.get_name() == "my_param_name")
      {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
          result.successful = false;
          result.reason = "my_param_name must be a string";
          break;
        }

        // Optionally do something with parameter
      }
    }

    return result;
  }

  // Need to hold a pointer to the callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

```