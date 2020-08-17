# rclcpp: Time

The _rclcpp::Time_ and _rclcpp::Duration_ are a significant departure from
their ROS1 equivalents, but are more closely related to
[std::chrono](https://en.cppreference.com/w/cpp/chrono).

When porting certain ROS1 libraries, there may be significant usage of
timestamps as floating-point seconds. To get floating point seconds from
an rclcpp::Time:

```
double seconds = static_cast<double>(t.nanoseconds()) / 1e9;
```

Converting the opposite direction is equally easy:
```
rclcpp::Time t(static_cast<uin64_t>(seconds * 1e9));
```

	Time t;
	double seconds = t.seconds();  // since the epoch


    Duration d = Duration::from_seconds(1.0);
    double seconds = d.seconds();

https://github.com/ros2/rclcpp/commit/0ccac1e3bd0f4314e11c86202c0c3ade57aed8ed#diff-ccbb86dadc423b17f11c0c3edbea8249 - was added here


rclcpp::Node::now()
