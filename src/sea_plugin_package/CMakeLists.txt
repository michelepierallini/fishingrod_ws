# cmake_minimum_required(VERSION 3.8)
# project(sea_plugin_package)

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(gazebo_dev REQUIRED)
# find_package(gazebo_ros REQUIRED)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# include_directories(include ${GAZEBO_INCLUDE_DIRS} ${GAZEBO_ROS_INCLUDE_DIRS})

# # Add plugin library
# add_library(SerialElasticPlugin SHARED src/SerialElasticPlugin.cc)
# ament_target_dependencies(SerialElasticPlugin rclcpp gazebo_ros gazebo_dev)
# target_link_libraries(SerialElasticPlugin ${GAZEBO_LIBRARIES})

# # Install plugin library
# install(
#   TARGETS SerialElasticPlugin
#   LIBRARY DESTINATION lib
# )

# # Linting and Testing (optional)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# # Generate an ament package
# ament_package()

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

# Install plugin library
install(
  TARGETS SerialElasticPlugin
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
