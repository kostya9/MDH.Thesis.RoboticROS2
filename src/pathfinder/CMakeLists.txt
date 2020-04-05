cmake_minimum_required(VERSION 3.5)
project(pathfinder)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

set(msg_files
  "msg/Move.msg"
  "msg/Rotate.msg"
)

rosidl_generate_interfaces(pathfinder
  ${msg_files}
)

 # This should be lower than generate interfaces due to a bug  https://github.com/ros2/rosidl/issues/402
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VTK installation on ubuntu is bugged https://forum.cb-geo.com/t/cmake-error-due-to-broken-vtk-in-ubuntu-18-04/132
find_package(VTK)
if (VTK_FOUND)
 include(${VTK_USE_FILE})
  link_libraries(${VTK_LIBRARIES})
  add_definitions("-DUSE_VTK")
  set(mpm_vtk ${mpm_SOURCE_DIR}/src/vtk_writer.cc)
endif()

set(target "main")
add_executable(${target} src/main.cpp)
ament_target_dependencies(${target}
  "rclcpp"
  "rclcpp_components"
  "gazebo_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "rcutils"
  "std_msgs"
  "rosidl_default_generators"
  "pcl_conversions"
#  "tf2_geometry_msgs"
  "rcl_interfaces"
#  "pcl"
  )


target_link_libraries(${target}
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

ament_export_dependencies(rosidl_default_runtime)

## To use msg in the same pkg
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(${target}
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
