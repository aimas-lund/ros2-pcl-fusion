cmake_minimum_required(VERSION 3.8)
project(fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

set(_fusion_ros_distro_defines "")
if(DEFINED ENV{ROS_DISTRO})
  set(_fusion_ros_distro "$ENV{ROS_DISTRO}")
  if(_fusion_ros_distro STREQUAL "humble")
    list(APPEND _fusion_ros_distro_defines FUSION_ROS_DISTRO_HUMBLE=1)
  elseif(_fusion_ros_distro STREQUAL "jazzy")
    list(APPEND _fusion_ros_distro_defines FUSION_ROS_DISTRO_JAZZY=1)
  endif()
endif()

add_library(fusion_node SHARED
  src/fusion_node.cpp
)
if(_fusion_ros_distro_defines)
  target_compile_definitions(fusion_node PUBLIC ${_fusion_ros_distro_defines})
endif()
target_compile_features(fusion_node PUBLIC cxx_std_17)
target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fusion_node
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  message_filters
  tf2_sensor_msgs
)

add_executable(fusion_node_exe src/main.cpp)
target_link_libraries(fusion_node_exe fusion_node)
if(_fusion_ros_distro_defines)
  target_compile_definitions(fusion_node_exe PUBLIC ${_fusion_ros_distro_defines})
endif()
ament_target_dependencies(fusion_node_exe
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  message_filters
  tf2_sensor_msgs
)

install(TARGETS
  fusion_node
  fusion_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
