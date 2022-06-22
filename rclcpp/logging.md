# rclcpp: Logging

## Changing the logging level

```cpp
#include <rclcpp/loger.hpp>

rclcpp::get_logger("nav2_costmap_2d").set_level(rclcpp::Logger::Level::Debug);
```
