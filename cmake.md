# CMake

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

## Building Python Extensions in C++

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