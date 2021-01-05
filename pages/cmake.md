# CMake

While you don't need to know everything about CMake to use ROS2, knowing a bit
will really be helpful. You might be interested in the
[CMake tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
which explains the basics of CMake.

## Ament

Ament is a set of CMake modules specifically designed for ROS2 with the intent
of making CMake easier to use. See also the
[Ament CMake](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/)
documentation.

The basic structure of an ament package:

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_package_name)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include our own headers
include_directories(include)

# Create a node
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node
  rclcpp
  # Other ament dependencies
  # This sets up include and linker paths
)

add_library(my_library src/my_library.cpp)
ament_target_dependencies(my_library
  rclcpp
)

# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install our node and library
install(TARGETS my_node my_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PACKAGE_NAME}
)

# Install some Python scripts
install(
  PROGRAMS
    scripts/my_script.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# Tell downstream packages where to find our headers
ament_export_include_directories(include)
# Tell downstream packages our libraries to link against
ament_export_libraries(my_library)
# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  rclcpp
)
ament_package()
```

## Linting Configuration

I prefer a more ROS1-style code style. To allow braces to be on their
own lines:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()
```

## Installing Python Scripts

```cmake
install(
  PROGRAMS
    scripts/script1.py
    scripts/script2.py
  DESTINATION lib/${PROJECT_NAME}
)
```

## Depending on Messages in Same Package

It is generally best practice to put messages in separate packages, but sometimes,
especially for drivers, you want the messages in the same package.

```cmake
find_package(rosidl_default_generators REQUIRED)

# Generate some messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
)

# Add a node which uses the messages
add_executable(my_node my_node.cpp)
rosidl_target_interfaces(my_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
```

## Removing Boost from Pluginlib

Pluginlib supports both boost::shared_ptrs and std::shared_ptrs by default,
if you want to avoid depending on Boost in your shiny new ROS2 library, you
need to specifically tell pluginlib not to include the Boost versions:

```cmake
target_compile_definitions(your_library PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
```

## Using Eigen3

Add _eigen_ to your package.xml as a dependency, and then:

```cmake
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
```

## Building Python Extensions in C++

The example below is based on the
[etherbotix](https://github.com/mikeferguson/etherbotix) package.

```cmake
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED python)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(
  my_python SHARED
  ${SOURCE_FILES}
)
set_target_properties(
  my_python PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
  PREFIX ""
)
target_link_libraries(my_python
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

install(
  TARGETS my_python
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
```
