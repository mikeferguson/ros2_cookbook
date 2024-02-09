# Command Line: Colcon

## My Aliases

I hate typing - so these are the aliases in my ``~/.bashrc`` for my most common workflow:

```
alias build2="colcon build --symlink-install"
alias test2="colcon test --event-handlers console_direct+"
```

## Build

_colcon_ is used to build ROS 2 packages in a workspace.

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

## Be Careful With Workspaces

You should probably build your workspace in a window where you have NOT sourced the
setup.bash of that workspace. For more details on why, see
[this ticket](https://github.com/colcon/colcon-core/issues/194).

## Verb Documentation

 * [build](https://colcon.readthedocs.io/en/released/reference/verb/build.html)
 * [test](https://colcon.readthedocs.io/en/released/reference/verb/test.html)
