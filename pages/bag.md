# rosbag2

## Common Command Line

 * ```ros2 bag record /topic1 /topic2```
 * When playing a bagfile back, you usually want clock topic:
   * ```ros2 bag play <bagfile> --clock```
 * For nodes to use the clock topic, specify use_sim_time:
   * ```ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true```

## API Tutorials

The ROS documentation has tutorials for recording and playback, which
are linked below. There is no official tutorial for reading from a
bagfile in Python, the linked one comes from an MCAP tutorial:

 * [Record in C++](https://docs.ros.org/en/rolling/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html)
 * [Playback in C++](https://docs.ros.org/en/rolling/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html)
 * [Record in Python](https://docs.ros.org/en/rolling/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html)
 * [Playback in Python](https://mcap.dev/guides/python/ros2)

## Removing TF from a Bagfile

I have often found that I needed to remove problematic TF data from a bagfile,
usually filtering out the ``odom->base_link`` transform so that I can replace
it with a more sophisticated filtered one. In this particular example we just
toss the whole message out - but you could selectively remove the individual
transform within the message and serialize the edited message back up.

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri='input_bag', storage_id='mcap'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr')
)

writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri='output_bag', storage_id='mcap'),
    rosbag2_py.ConverterOptions('', '')
)

# First preprocess the topics
topic_types = reader.get_all_topics_and_types()
for topic_type in topic_types:
    # Setup output bagfile - same topics as input
    writer.create_topic(topic_type)
    # Note the type if this is our TF data
    if topic_type.name == '/tf':
        tf_typename = topic_type.type

# Now iterate through messages, copying over the ones we don't filter
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    filter_out = False
    # Filter out odom tf messages
    if topic == '/tf':
        # Deserialize the message so we can look at it
        msg_type = get_message(tf_typename)
        msg = deserialize_message(data, msg_type)
        # Iterate through the transforms in the message
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom':
                # Toss this message
                filter_out = True
                break
    # Copy over message if it isn't odom
    if not filter_out:
        writer.write(topic, data, timestamp)
```
