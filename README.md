# ROS2 Cookbook

 * [Getting Started](pages/getting_started.md)
 * Client Libraries
   * rclcpp [API](http://docs.ros2.org/latest/api/rclcpp/)
     * [Logging](rclcpp/logging.md)
     * [Nodes and Components](rclcpp/nodes.md)
     * [Parameters](rclcpp/parameters.md)
     * [Point Clouds](rclcpp/pcl.md)
     * [Time](rclcpp/time.md)
     * [TF2](rclcpp/tf2.md)
     * [Workarounds](rclcpp/workarounds.md)
   * rclpy [API](http://docs.ros2.org/latest/api/rclpy/)
     * [Nodes](rclpy/nodes.md)
     * [Services](rclpy/services.md)
     * [Parameters](rclpy/parameters.md)
     * [Time](rclpy/time.md)
     * [TF2](rclpy/tf2.md)
 * [ros2launch](pages/launch.md)
   * [Python-Based Launch Files](pages/launch.md#python-based-launch-files)
   * [Making a Launch File Executable](pages/launch.md#making-a-launch-file-executable)
   * [Loading Parameters From a File](pages/launch.md#loading-parameters-from-a-file)
   * [Including Python Launch Files](pages/launch.md#including-python-launch-files)
   * [Loading a URDF](pages/launch.md#loading-a-urdf)
   * [Debugging a Node in GDB](pages/launch.md#debugging-a-node-in-gdb)
   * [Installing Launch Files](pages/launch.md#installing-launch-files)
 * [rosbag2](pages/bag.md)
   * [Common CLI Commands](pages/bag.md#common-command-line)
   * [API Tutorials](pages/bag.md#api-tutorials)
   * [Converting Bags from ROS 1 to ROS 2](pages/bag.md#converting-bag-files-from-ros-1)
   * [Removing TF From a Bagfile](pages/bag.md#removing-tf-from-a-bagfile)
 * [Networking (DDS & CycloneDDS)](pages/networking.md)
   * [Clearpath Robotics has a great Network Troubleshooting Flow Chart](https://docs.clearpathrobotics.com/docs/ros/networking/network_troubleshooting/troubleshooting_overview)
 * [Documenting with rosdoc2](pages/rosdoc2.md)
 * [CMake](pages/cmake.md)
   * [ROS 1 Style Linting Configuration](pages/cmake.md#linting-configuration)
   * [Safely Defaulting to Release Build](pages/cmake.md#safely-defaulting-to-release-build)
   * [Installing Python Scripts](pages/cmake.md#installing-python-scripts)
   * [Depending o Messages in Same Package](pages/cmake.md#depending-on-messages-in-same-package)
   * [Removing Boost from Pluginlib](pages/cmake.md#removing-boost-from-pluginlib)
 * [Quality of Service (QoS)](pages/qos.md)
   * [REP-2003 Recommended Practices](pages/qos.md#recommended-practices)
   * [QoS Overrides](pages/qos.md#overrides)
 * Command Line Tools
   * ```ros2 run <pkg> <node>```
   * ```ros2 node list```
   * ```ros2 topic list```
   * ```ros2 topic info <topic_name> --verbose``` gives details about QoS.
   * ```ros2 param list```
   * [colcon](pages/colcon.md) is the build tool.
   * ```ros2 doctor --report``` gives tons of information.
   * ```ros2 pkg prefix <pkg>``` tells where a package is located.
 * Packaging
   * [Setting bloom/git to always use ssh](https://answers.ros.org/question/234494/diagnosing-issues-with-bloom-github-two-factor-authentication/)
     * ```git config --global url.ssh://git@github.com/.insteadOf https://github.com/```
   * ```rosdep install --from-paths src --ignore-src --rosdistro=jazzy -y```
 * [Package Documentation](pages/packages.md)
 * Limitations / Bugs
   * [Cannot generate messages and call ament_python_install_package() in same package](https://github.com/ros2/rosidl_python/issues/141)
 * Status Pages
   * [Rolling Debian Build Status](http://repo.ros2.org/status_page/ros_rolling_default.html)
   * [Kilted Debian Build Status](http://repo.ros2.org/status_page/ros_kilted_default.html)
   * [Jazzy Debian Build Status](http://repo.ros2.org/status_page/ros_jazzy_default.html)
   * [Humble Debian Build Status](http://repo.ros2.org/status_page/ros_humble_default.html)

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
