# rclcpp: Nodes and Components

## Creating a Component

ROS 2 introduces components, which are nodes that can be run together in a single
process. The benefits of using node compositions has been documented in a recent paper,
[Impact of ROS 2 Node Composition in Robotic Systems](https://arxiv.org/abs/2305.09933).
Given the ease of use and the provided tooling, it really makes sense to make just about
every node a component, and then let the rclcpp_components tooling create a node
executable for you:

```cpp
#include <rclcpp/rclcpp.hpp>

namespace my_pkg
{

class MyComponent : public rclcpp::Node
{
public:
  MyComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("node_name", options)
  {
    // Do all your setup here - subscribers/publishers/timers
    // After you return, an executor will be started

    // Note: you cannot use shared_from_this()
    //       here because the node is not fully
    //       initialized.
  }
};

}  // namespace my_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MyComponent)
```

The in the CMakeLists.txt:

```cmake
add_library(my_component SHARED
  src/my_component.cpp
)
ament_target_dependencies(my_component
  rclcpp
  rclcpp_components
)

# Also add a node executable which simply loads the component
rclcpp_components_register_node(my_component
  PLUGIN "my_pkg::MyComponent"
  EXECUTABLE my_node
)
```

## Executors

To run an executor in a thread:

```cpp
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

rclcpp::executors::SingleThreadedExecutor executor;

// Node is rclcpp::Node::SharedPtr instance
executor.add_node(node);
std::thread executor_thread(
  std::bind(&rclcpp::executors::SingleThreadedExecutor::spin,
            &executor));
```
