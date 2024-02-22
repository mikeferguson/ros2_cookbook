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

rclcpp::Node node("name_of_node");

auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node.get_clock());
auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
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

A common work flow is to get the "latest" transform. In ROS 2, this can be
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

## Constructing Transform from Euler Angles
There are numerous ways to do this (using Eigen, KDL, etc) - but it is also possible with only tf2 APIs:

```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

tf2::Quaternion quat;
// While the documentation for tf2 orders the names of these as yaw, pitch, roll,
// it specifies that yaw = rotation about Y, which is not what most of us expect
quat.setEuler(pitch, roll, yaw);

geometry_msgs::msg::TransformStamped transform;
transform.transform.rotation = tf2::toMsg(quat);
// Probably also fill in transform.header and transform.transform.translation

// Can now use the transform with tf2::doTransform()
```

## Getting Yaw Angle from Quaternion

```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

geometry_msgs::msg::Pose pose;
double yaw = tf2::getYaw(pose.orientation);
```

> [!NOTE]
> ```tf2::getYaw``` requires some pieces from ```tf2_geometery_msgs/tf2_geometry_msgs.hpp```
> but cannot depend on them because it would create a circular dependency. This means you
> absolutely need to include tf2_geometry_msgs BEFORE tf2/utils or you will get an
> undefined reference to ```tf2::fromMsg```

## Transform from Eigen::Isometry3d

```cpp
#include <tf2_eigen/tf2_eigen.hpp>

Eigen::Isometry3d map_to_odom;
geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(map_to_odom);
// Fill in header, child_frame_id
```
