# rclcpp: Point Clouds

The sensor_msgs/PointCloud2 is a very common type of ROS message for processing
perception data in ROS. It is also one of the most complex messages to actually
interpret.

The complexity of the message derives from the fact that it holds arbitrary
fields in a single giant data store. This allows the PointCloud2 message to
work with any type of cloud (for instance, XYZ points only, XYZRGB, or even
XYZI), but adds a bit of complexity in accessing the data in the cloud.

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
