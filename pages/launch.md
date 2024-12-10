# roslaunch2

See also [rosetta_launch](https://github.com/MetroRobots/rosetta_launch) for a number of examples.

## Python-Based Launch Files

Python-based launch files all pretty much follow the same structure.

> [!NOTE]
> Prior to _Foxy_, the parameters _name_, _namespace_, and _executable_ were
> [prefixed with node\_](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/#launch-ros)

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='node_runtime_name',
            package='ros2_package_name',
            executable='name_of_executable',
            parameters=[{'name_of_int_param': 1,
                         'name_of_str_param': 'value'}],
            remappings=[('from', 'to')],
            output='screen',
        ),
        # More Nodes!
    ])
```

## Making a Launch File Executable

Normally, launch files are run with:

```
ros2 launch pkg launch.py
```

But, sometimes you want an executable launch file (for instance to
put in a systemd job). Assuming you follow the default pattern shown
above, all you need to add:

```python
#!/usr/bin/env python3

import sys
from launch import LaunchService

# define generate_launch_description() as above

if __name__ == '__main__':
    desc = generate_launch_description()
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(desc)
    return service.run()
```

## Loading Parameters From a File

Some nodes have many parameters, it's easier to put them in a YAML file:

```yaml
node_name:
  ros__parameters:
      some_int_param: 1
      some_str_param: "the_value"
```

To load this:

```python
from ament_index_python.packages import get_package_share_directory

# Assuming you have a file called package_name/config/params.yaml
node_params = os.path.join(
    get_package_share_directory('package_name'),
    'config',
    'params.yaml'
)

# Add this to your LaunchDescription
Node(
    name='node_runtime_name',
    package='ros2_package_name',
    executable='name_of_executable',
    parameters=[{'another_param': 42.0},
                 node_params]
)
```

## Including Python Launch Files

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Assuming you have a file called package_name/launch/my_launch.launch.py
my_launch_file = os.path.join(
    get_package_share_directory('package_name'),
    'launch',
    'my_launch.launch.py'
)

# Add this to your LaunchDescription
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([my_launch_file])
),
```

## Loading a URDF

Most robots need to load their URDF into the robot_state_publisher,
and maybe other nodes as well:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# Load the URDF into a parameter
desc_dir = get_package_share_directory('robot_description_pkg')
urdf_path = os.path.join(desc_dir, 'robots', 'my_urdf.urdf')
urdf = open(urdf_path).read()

# Add this to your LaunchDescription
Node(
    name='robot_state_publisher',
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': urdf}],
)
```

## Debugging a Node in GDB

If you are using a computer with a graphical system, it's pretty
easy to have gdb launch in a new terminal, by adding the following:

```python
Node(
    # Configure the node as above
    # Add a prefix so the node is started in GDB in a new xterm
    prefix=['xterm -e gdb --args'],
)
```

However, many robots lack a graphical system. In this case,
you can launch GDB in a server configuration:

```python
Node(
    # Configure the node as above
    # Add a prefix so the node is started in a gdbserver
    prefix=['gdbserver localhost:2345'],
)
```

Then, in a separate terminal you can connect to the target, and start
the node by running `continue`:

```
$> gdb
(gdb) target remote localhost:2345
(gdb) continue
```

## Installing Launch Files

 * [Launch System Tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-system.html#creating-the-structure-to-hold-launch-files)
