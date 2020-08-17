# Command Line: Colcon

_colcon_ is used to build ROS2 packages in a workspace.

Build all packages:

```
colcon build
```

To avoid having to rebuild when tweaking Python scripts:

```
colcon build --symlink-install
```

To test and get results to screen:

```
colcon test
colcon test-result --verbose
```

Build/test a single package:

```
colcon <verb> --packages-select <package-name>
```

Get the output to the screen:

```
colcon <verb> --event-handlers console_direct+
```

Verb documentation:

 * [build](https://colcon.readthedocs.io/en/released/reference/verb/build.html)
 * [test](https://colcon.readthedocs.io/en/released/reference/verb/test.html)
