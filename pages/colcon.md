# Command Line: Colcon

## Build

_colcon_ is used to build ROS2 packages in a workspace.

Build all packages:

```
colcon build
```

To avoid having to rebuild when tweaking Python scripts,
config files, and launch files:

```
colcon build --symlink-install
```

To fix most build issues, especially if you have added or removed packages
to your workspace or installed new RMW implementations, clean the CMake
cache. See this
[ROS Answers](https://answers.ros.org/question/333534/when-to-use-cmake-cleanconfigure/)
post for more details.

```
colcon build --cmake-clean-cache
```

## CMake Arguments

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Test

To test and get results to screen:

```
colcon test
colcon test-result --verbose
```

Build/test a single package:

```
colcon <verb> --packages-select <package-name>
```

## Formatting

Get the output to the screen:

```
colcon <verb> --event-handlers console_direct+
```

## Verb Documentation

 * [build](https://colcon.readthedocs.io/en/released/reference/verb/build.html)
 * [test](https://colcon.readthedocs.io/en/released/reference/verb/test.html)
