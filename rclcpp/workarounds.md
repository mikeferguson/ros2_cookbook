# rclcpp: Workarounds

## Lazy Publishers

ROS2 does not yet have subscriber connect callbacks. This code creates
a function which is called periodically to check if we need to start
or stop subscribers:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class LazyPublisherEx : rclcpp::Node
{
public:
  LazyPublisherEx(const rclcpp::NodeOptions & options) :
    Node("lazy_ex", options)
  {
    // Setup timer
    timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LazyPublisher::periodic, this));
  }

private:
  void periodic()
  {
    if (pub_.get_subscription_count() > 0)
    {
        // We have a subscriber, do any setup required
    }
    else
    {
        // Subscriber has disconnected, do any shutdown
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

The same can be done when using image transport, you simply
have to change from _get_subscription_count_ to
_getNumSubscribers_:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

class LazyPublisherEx : rclcpp::Node
{
public:
  LazyPublisherEx(const rclcpp::NodeOptions & options) :
    Node("lazy_ex", options)
  {
    // Setup timer
    timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LazyPublisher::periodic, this));
  }

private:
  void periodic()
  {
    if (pub_.getNumSubscribers() > 0)
    {
        // We have a subscriber, do any setup required
    }
    else
    {
        // Subscriber has disconnected, do any shutdown
    }
  }

  image_transport::CameraPublisher pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```