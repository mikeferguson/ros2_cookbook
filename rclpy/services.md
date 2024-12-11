# rclpy: Services

## Synchronous Service Call

By default, services in `rclpy` are asynchronous. For more info see this
[How-to Guide](https://docs.ros.org/en/rolling/How-To-Guides/Sync-Vs-Async.html).

```python
from my_ros_package.srv import ServiceName
from threading import Thread

cli = my_node.create_client(ServiceName, 'service_name')
cli.wait_for_service()

# Need to spin before making the service call or we will deadlock
spin_thread = Thread(target=rclpy.spin, args=(my_node,))
spin_thread.start()

req = ServiceName.Request()
```
