# ROS2 Cookbook

 * Client Libraries
   * rclcpp [API](http://docs.ros2.org/latest/api/rclcpp/)
     * [Nodes and Components](rclcpp/nodes.md)
     * [Parameters](rclcpp/parameters.md)
     * Logging
     * [Time](rclcpp/time.md)
     * [TF2](rclcpp/tf2.md)
     * [Workarounds](rclcpp/workarounds.md)
   * rclpy [API](http://docs.ros2.org/latest/api/rclpy/)
     * [Nodes](rclpy/nodes.md)
     * [Parameters](rclpy/parameters.md)
     * [Time](rclpy/time.md)
     * [TF2](rclpy/tf2.md)
 * [ros2launch](pages/launch.md)
 * [Networking (DDS & CycloneDDS)](pages/networking.md)
 * Command Line Tools
   * ```ros2 run <pkg> <node>```
   * ```ros2 node list```
   * ```ros2 topic list```
   * ```ros2 topic info <topic_name> --verbose``` gives details about QoS.
   * ```ros2 param list```
   * [colcon](pages/colcon.md) is the build tool.
   * ```ros2 doctor --report``` gives tons of information.
* [CMake](pages/cmake.md)
* [Package Documentation](pages/packages.md)
* Status Pages
  * [Foxy Debian Build Status](http://repo.ros2.org/status_page/ros_foxy_default.html)
  * [Rolling Debian Build Status](http://repo.ros2.org/status_page/ros_rolling_default.html)
  * [Compare Foxy/Rolling](http://repo.ros2.org/status_page/compare_foxy_rolling.html)

## License

<a rel="license" href="http://creativecommons.org/publicdomain/zero/1.0/">
  <img src="http://i.creativecommons.org/p/zero/1.0/88x31.png" style="border-style: none;" alt="CC0" />
</a>

The contents of this cookbook are placed in the Public Domain through the CC0 license.
You can use code snippets in your code without attribution and without impact to your
license choice. This cookbook is provided as-is and without any warranties of any kind.
See the full text of the
[CC0 license](https://creativecommons.org/publicdomain/zero/1.0/legalcode).

While not required by license, if you want to copy the entire cookbook somewhere, please
give some attribution.
