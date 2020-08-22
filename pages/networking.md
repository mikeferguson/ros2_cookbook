# Networking

ROS2 uses DDS for message transport.

## DDS Discovery

There is no _rosmaster_ like in ROS1. Node discovery is peer-to-peer with nodes
announcing their topics on startup and periodically after that. By default, any
machines on the same network will see each other if they have the same
ROS_DOMAIN_ID.

ROS_DOMAIN_ID can be any number between 0 and 253, although it is recommended
to use numbers less than 128.

In addition to the ROS_DOMAIN_ID, CycloneDDS supports a domain tag, which allows
nearly infinite partitioning of the network (see below).

## CycloneDDS

Cyclone can be configured with XML. This can be stored in a file or passed
directly in the environment variable CYCLONEDDS_URI.

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
