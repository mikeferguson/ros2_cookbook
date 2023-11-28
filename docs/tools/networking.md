# Networking

ROS2 uses DDS for message transport.

Set the environment variable RMW_IMPLEMENTATION to select a DDS implementation
(RMW = robotic middleware). For instance:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

To check which RMW implementation is being used:

```
ros2 doctor --report | grep middleware
```

## DDS Discovery

There is no _rosmaster_ like in ROS1. Node discovery is peer-to-peer with nodes
announcing their topics on startup and periodically after that. By default, any
machines on the same network will see each other if they have the same
ROS_DOMAIN_ID.

ROS_DOMAIN_ID can be any number between 0 and 253, although it is recommended
to use numbers less than 128.

In addition to the ROS_DOMAIN_ID, CycloneDDS supports a domain tag, which allows
nearly infinite partitioning of the network (see below).

If you want to limit communication to the localhost set ROS_LOCALHOST_ONLY,
which is [available since Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/#new-features-in-this-ros-2-release).

## CycloneDDS

Cyclone can be configured with XML. This can be stored in a file or passed
directly in the environment variable CYCLONEDDS_URI. A full list of
supported options can be found in the
[eclipse-cyclonedds repo](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/options.md).
See also the
[Guide to Configuration](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst).

### CycloneDDS: Multiple Interfaces

Cyclone currently only works with a single network interface. If you have multiple
interfaces, specify which one to use in the NetworkInterfaceAddress:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>wlp2s0</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
```

### CycloneDDS: Disabling Multicast (Except Discovery)

Some network hardware can perform poorly with multicast (especially with
WIFI). You can limit multicast to just discovery:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

### CycloneDDS: Domain Tag

CycloneDDS also defines a "Domain Tag" which allows to drastically partition
the network with a custom string:

```xml
<CycloneDDS>
  <Domain>
    <Discovery>
      <Tag>my_robot_name</Tag>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### Example

The above tags can all be combined:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <!-- Explicitly set network interface -->
      <NetworkInterfaceAddress>wlp2s0</NetworkInterfaceAddress>
      <!-- Use multicast for discovery only -->
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <!-- This tag has to be the same on each machine -->
      <Tag>my_robot_name</Tag>
    </Discovery>
  </Domain>
</CycloneDDS>
```
