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
* Status Pages
  * [Foxy Debian Build Status](http://repo.ros2.org/status_page/ros_foxy_default.html)
  * [Rolling Debian Build Status](http://repo.ros2.org/status_page/ros_rolling_default.html)
  * [Compare Foxy/Rolling](http://repo.ros2.org/status_page/compare_foxy_rolling.html)
