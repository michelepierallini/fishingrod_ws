cmake_minimum_required(VERSION 3.8)
project(sea_plugin_package)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_ROS_INCLUDE_DIRS}
)

# Add plugin library
add_library(SerialElasticPlugin SHARED src/SerialElasticPlugin.cc)
ament_target_dependencies(SerialElasticPlugin rclcpp gazebo_ros gazebo_dev)
target_link_libraries(SerialElasticPlugin ${GAZEBO_LIBRARIES})

add_library(SerialElasticPluginTri SHARED src/SerialElasticPluginTri.cc)
ament_target_dependencies(SerialElasticPluginTri rclcpp gazebo_ros gazebo_dev)
target_link_libraries(SerialElasticPluginTri ${GAZEBO_LIBRARIES})
target_include_directories(SerialElasticPluginTri PRIVATE ${EIGEN3_INCLUDE_DIR})


# Install plugin library
install(
  TARGETS SerialElasticPlugin SerialElasticPluginTri
  LIBRARY DESTINATION lib
)

# Linting and Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate an ament package
ament_package()

# export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/michele/michele_try_ws/fishingrod_ws/src/sea_plugin_package/build
