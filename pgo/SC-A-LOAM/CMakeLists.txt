cmake_minimum_required(VERSION 3.5)
project(sc_pgo)

# Set build type and standards
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
# Declare a C++ executable
add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)

# Compile options
target_compile_options(alaserPGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)

# Link libraries
ament_target_dependencies(alaserPGO 
  rclcpp
  rclpy
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  rosbag2
  tf2
  tf2_ros
  cv_bridge
  image_transport
  pcl_conversions
  PCL
  OpenCV
  GTSAM
)

# Specify libraries to link a library or executable target against
target_link_libraries(alaserPGO 
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

# Install targets
install(TARGETS alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()


