# rclcpp: Nodes and Components

## Executors

To run an executor in a thread:

```
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
