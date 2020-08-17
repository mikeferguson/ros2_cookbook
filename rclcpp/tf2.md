# rclcpp: TF2

The TF2 library provides easy access to transformations. All of the examples below
require a dependency on the _tf2_ros_ package.

## Broadcasting Transforms

```cpp
#include <tf2_ros/transform_broadcaster.h>
std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;

broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nodePtr);

geometry_msgs::msg::TransformStamped transform;
transform.header.stamp = node->now();
transform.header.frame_id = "odom";
transform.child_frame_id = "base_link";

// Fill in transform.transform.translation
// Fill in transform.transform.rotation

broadcaster->sendTransform(transform);
```

## Listening for Transforms

```cpp
#include "tf2_ros/transform_listener.h"

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

rclcpp::Node node("name_of_node");

tf_buffer.reset(new tf2_ros::Buffer(node.get_clock()));
tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer_));
```

## Applying Transforms

TF2 can be extended by packages that provide implementations of _transform_.
The _tf2_geometry_msgs package provides these for _geometry_msgs_. The example
below uses _geometry_msgs::msg::PointStamped_ - but this should work for any
data type in _geometry_msgs_:

```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msg::msgs::PointStamped in, out;
in.header.frame_id = "source_frame";

try
{
  tf_buffer->transform(in, out, "target_frame");
}
catch (const tf2::TransformException& ex)
{
  RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Could not transform point.");
}
```

The _transform_ function can also take a timeout. It will then wait for the
transform to be available up to that amount of time:

```cpp
tf_buffer->transform(in, out, "target_frame", tf2::durationFromSec(1.0));
```

## Get Latest Transform

A common work flow is to get the "latest" transform. In ROS2, this can be
accomplished using _tf2::TimePointZero_, but requires using _lookupTransform_
and then calling _doTransform_ (which is basically what _transform_ does
internally):

```cpp
geometry_msgs::msg::PointStamped in, out;

geometry_msgs::msg::TransformStamped transform =
   tf_buffer->lookupTransform("target_frame",
                              in.header.frame_id,
                              tf2::TimePointZero);

tf2::doTransform(in, out, transform);
```
