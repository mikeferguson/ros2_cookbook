# rclcpp: Point Clouds

The sensor_msgs/PointCloud2 is a very common type of ROS message for processing
perception data in ROS. It is also one of the most complex messages to actually
interpret.

The complexity of the message derives from the fact that it holds arbitrary
fields in a single giant data store. This allows the PointCloud2 message to
work with any type of cloud (for instance, XYZ points only, XYZRGB, or even
XYZI), but adds a bit of complexity in accessing the data in the cloud.

In ROS1, there was a simpler PointCloud message, but this has been
[deprecated](https://github.com/ros2/common_interfaces/issues/105) and will
be removed in ROS2-G.

## Using the PointCloud2Iterator

The sensor_msgs package provides a C++ PointCloud2Iterator which can be used
to create, modify and access PointCloud2 messages.

To create a new message:

```cpp
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

sensor_msgs::msg::PointCloud2 msg;

// Fill in the size of the cloud
msg.height = 480;
msg.width = 640;

// Create the modifier to setup the fields and memory
sensor_msgs::PointCloud2Modifier mod(msg);

// Set the fields that our cloud will have
mod.setPointCloud2FieldsByString(2, "xyz", "rgb");

// Set up memory for our points
mod.resize(msg.height * msg.width);

// Now create iterators for fields
sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
{
  *iter_x = 0.0;
  *iter_y = 0.0;
  *iter_z = 0.0;
  *iter_r = 0;
  *iter_g = 255;
  *iter_b = 0;
}
```

## Using PCL

For a number of operations, you might want to convert to a pcl::PointCloud
in order to use the extensive API of the
[Point Cloud Library](https://pointclouds.org).

In ROS1, the pcl_ros package allowed you to write a subscriber whose callback
took a pcl::PointCloud directly, however this package has not yet been
ported to ROS2. Regardless, it is pretty straight forward to do the conversion
yourself with the pcl_conversions package:

```cpp
#include "pcl_conversions/pcl_conversions.h"

void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // PCL still uses boost::shared_ptr internally
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *cloud);
}
```

You can also go in the opposite direction:

```cpp
#include "pcl_conversions/pcl_conversions.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
sensor_msgs::msg::PointCloud2 msg;

pcl::toROSMsg(*cloud, msg);
cloud_publisher->publish(msg);
```
