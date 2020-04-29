cmake_minimum_required(VERSION 3.5)
project(ground_plane_estimator)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(PCL 1.7 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ground_plane_estimator_node src/ground_plane_estimator_node.cpp)
ament_target_dependencies(ground_plane_estimator_node rclcpp rclcpp_components std_msgs nav_msgs sensor_msgs cv_bridge tf2 tf2_ros pcl_conversions image_transport pcl_msgs plc_ros)

target_link_libraries(ground_plane_estimator_node ${Boost_LIBRARIES}  pcl_apps pcl_common pcl_features pcl_filters pcl_io pcl_kdtree pcl_keypoints pcl_octree pcl_registration pcl_sample_consensus pcl_search pcl_segmentation pcl_surface pcl_visualization )

install(TARGETS
  ground_plane_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()