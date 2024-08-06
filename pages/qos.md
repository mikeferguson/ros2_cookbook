# QoS (Quality of Service)

A full description of the available QoS settings, and compatability of different settings can be found
in the [ROS 2 Documentation](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

## Recommended Practices

 * [REP-2003](https://ros.org/reps/rep-2003.html) defines recommended practices for maps and sensor drivers.
 * The suggested practice for "sensor drivers" is also pretty reasonable for most vision pipelines, and
   and that is to:
   * Publish with ``SystemDefaultQoS``
   * Subscribe with ``SensorDataQoS``

Subscribing with ``SensorDataQoS`` means that we will get unreliable transport but be more immune to lossy networks.
By publishing with ``SystemDefaultQoS`` we will still be able to connect to any subscriber as long as they are
specifying ``volatile`` rather than ``transient local``.

## Overrides

``rclcpp`` offers a consistent way to define QoS overrides as parameters. There is a (somewhat dated/inaccurate)
[design doc](http://design.ros2.org/articles/qos_configurability.html) on this feature. In your code:

```cpp
// Allow overriding QoS settings (history, depth, reliability)
rclcpp::PublisherOptions pub_options;
pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
node->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(10), pub_options);
```

Then, in your parameter file, you can define:

```yaml
node_name:
  ros__parameters:
    qos_overrides:
      fully_resolved_topic_name:
        publisher:
          reliability: reliable
          depth: 100
          history: keep_last
```

> [!NOTE]
> While some documentation uses `history_depth`, the actual parameter needs to just be `depth` in order to work. 

The same workflow works for subscribers, you just use ``rclcpp::SubscriptionOptions`` instance and change ``publisher`` to ``subscription`` in the YAML file.

## Magic Numbers

> [!NOTE]
> This is fixed in later releases, and QoS profile will show "Infinite"

If you perform `ros2 topic info -v /some/topic` and you examine the QoS settings, you may note that several fields are set to the magic number 2147483651294967295 (or 2,147,483,651,294,967,295). e.g. 

    QoS profile:
        Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
        Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
        Lifespan: 2147483651294967295 nanoseconds
        Deadline: 2147483651294967295 nanoseconds
        Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
        Liveliness lease duration: 2147483651294967295 nanoseconds

What is the significance of this number, which is approximately equal to 68 years? It's not $2^n - 1$ as you might expect for a magic number. However, durations are defined as 

    [builtin_interfaces/Duration]
    # Seconds component, range is valid over any possible int32 value.
    int32 sec

    # Nanoseconds component in the range of [0, 1e9).
    uint32 nanosec

The max value for an `int32` is $2^{31} - 1$. The max value for an `uint32` is $2^{32} - 1$. (Note: According to the definition, any value for `nanosec` over 1e9 is invalid.)

So the magic number 2147483651294967295 is the number of nanoseconds equivalent to $2^{31} -1$ seconds ($2147483647$ seconds) added to $2^{32} - 1$ nanoseconds ($4.294967295$ seconds). 
