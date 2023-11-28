# ROS2 Cookbook

 * Client Libraries
   * rclcpp [API](http://docs.ros2.org/latest/api/rclcpp/)
     * [Logging](docs/client_libraries/rclcpp/logging.md)
     * [Nodes and Components](docs/client_libraries/rclcpp/nodes.md)
     * [Parameters](docs/client_libraries/rclcpp/parameters.md)
     * [Point Clouds](docs/client_libraries/rclcpp/pcl.md)
     * [Time](docs/client_libraries/rclcpp/time.md)
     * [TF2](docs/client_libraries/rclcpp/tf2.md)
     * [Workarounds](docs/client_libraries/rclcpp/workarounds.md)
   * rclpy [API](http://docs.ros2.org/latest/api/rclpy/)
     * [Nodes](docs/client_libraries/rclpy/nodes.md)
     * [Parameters](docs/client_libraries/rclpy/parameters.md)
     * [Time](docs/client_libraries/rclpy/time.md)
     * [TF2](docs/client_libraries/rclpy/tf2.md)
 * [ros2launch](docs/tools/launch.md)
   * [Python-Based Launch Files](docs/tools/launch.md#python-based-launch-files)
   * [Making a Launch File Executable](docs/tools/launch.md#making-a-launch-file-executable)
   * [Loading Parameters From a File](docs/tools/launch.md#loading-parameters-from-a-file)
   * [Including Python Launch Files](docs/tools/launch.md#including-python-launch-files)
   * [Loading a URDF](docs/tools/launch.md#loading-a-urdf)
   * [Installing Launch Files](docs/tools/launch.md#installing-launch-files)
 * [rosbag2](docs/tools/bag.md)
   * [Common CLI Commands](docs/tools/bag.md#common-command-line)
   * [API Tutorials](docs/tools/bag.md#api-tutorials)
   * [Converting Bags from ROS 1 to ROS 2](docs/tools/bag.md#converting-bag-files-from-ros-1)
   * [Removing TF From a Bagfile](docs/tools/bag.md#removing-tf-from-a-bagfile)
 * [Networking (DDS & CycloneDDS)](docs/tools/networking.md)
 * Command Line Tools
   * ```ros2 run <pkg> <node>```
   * ```ros2 node list```
   * ```ros2 topic list```
   * ```ros2 topic info <topic_name> --verbose``` gives details about QoS.
   * ```ros2 param list```
   * [colcon](docs/tools/colcon.md) is the build tool.
   * ```ros2 doctor --report``` gives tons of information.
* [CMake](docs/tools/cmake.md)
* Packaging
   * [Setting bloom/git to always use ssh](https://answers.ros.org/question/234494/diagnosing-issues-with-bloom-github-two-factor-authentication/)
     * ```git config --global url.ssh://git@github.com/.insteadOf https://github.com/```
   * ```rosdep install --from-paths src --ignore-src --rosdistro=humble -y```
* [Package Documentation](docs/documentation.md)
* Status Pages
  * [Rolling Debian Build Status](http://repo.ros2.org/status_page/ros_rolling_default.html)
  * [Iron Debian Build Status](http://repo.ros2.org/status_page/ros_iron_default.html)
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
