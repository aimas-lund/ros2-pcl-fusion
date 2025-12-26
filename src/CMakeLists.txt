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

add_library(fusion_node SHARED
  src/fusion_node.cpp
)
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
