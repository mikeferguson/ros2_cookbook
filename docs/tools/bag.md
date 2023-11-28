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

## Converting Bag Files from ROS 1

The following works, assuming you are mainly using standard topics. For instance,
I have converted a number of bagfiles intended for SLAM testing, which mainly
consist of sensor_msgs::LaserScan, TF, and nav_msgs::Odometry data.

The easiest route for converting bagfiles is to use ``rosbags``:

```
sudo pip3 install rosbags
rosbag-convert bagfile_name.bag
```

This will create a new folder with the name ``bagfile_name`` containing the
SQLite file and the index file. At this point, you can inspect the bagfile:

```
ros2 bag info bagfile_name

Files:             bagfile_name.db3
Bag size:          65.7 MiB
Storage id:        sqlite3
ROS Distro:        rosbags
Duration:          122.298s
Start:             Jun 15 2014 21:41:49.861 (1402882909.861)
End:               Jun 15 2014 21:43:52.159 (1402883032.159)
Messages:          35187
Topic information: Topic: odom | Type: nav_msgs/msg/Odometry | Count: 12141 | Serialization Format: cdr
                   Topic: tf | Type: tf2_msgs/msg/TFMessage | Count: 16939 | Serialization Format: cdr
                   Topic: base_scan | Type: sensor_msgs/msg/LaserScan | Count: 4884 | Serialization Format: cdr
                   Topic: odom_combined | Type: nav_msgs/msg/Odometry | Count: 1223 | Serialization Format: cdr
```

This bagfile is now useable in ROS 2. However, you can also go a bit
further by compressing the bagfile, and migrating it to the new MCAP
file format. First, create a YAML file to define the output format:

```yaml
# output_format.yaml
output_bags:
- uri: bagfile_name_compressed
  all: true
  compression_mode: file
  compression_format: zstd
```

Now, run the conversion:

```
ros2 bag convert -i bagfile_name -o output_format.yaml
```

Inspecting the new bag, we can see that compression is very nice - a
75% reduction in file size for my typical SLAM bag files:

```
ros2 bag info bagfile_name_compressed

Files:             bagfile_name_compressed.mcap.zstd
Bag size:          16.7 MiB
Storage id:        mcap
ROS Distro:        rolling
Duration:          122.298s
Start:             Jun 15 2014 21:41:49.861 (1402882909.861)
End:               Jun 15 2014 21:43:52.159 (1402883032.159)
Messages:          35187
Topic information: Topic: base_scan | Type: sensor_msgs/msg/LaserScan | Count: 4884 | Serialization Format: cdr
                   Topic: odom | Type: nav_msgs/msg/Odometry | Count: 12141 | Serialization Format: cdr
                   Topic: odom_combined | Type: nav_msgs/msg/Odometry | Count: 1223 | Serialization Format: cdr
                   Topic: tf | Type: tf2_msgs/msg/TFMessage | Count: 16939 | Serialization Format: cdr
```

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
