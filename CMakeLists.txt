cmake_minimum_required(VERSION 3.8)
project(whycon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set some options
option(ENABLE_FULL_UNDISTORT "Undistort the whole frame" OFF)
option(ENABLE_RANDOMIZED_THRESHOLD "Use rand() instead of binary-like search for threshold" OFF)
option(ENABLE_VERBOSE "Enable verbose console messages during detection" OFF)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_SOURCE_DIR}/include/whycon/config.h)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

### SYSTEM DEPENDENCIES ###
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV version ${OpenCV_VERSION}")

### TARGETS ###
include_directories(include)

add_library(whycon SHARED src/lib/circle_detector.cpp src/lib/many_circle_detector.cpp src/lib/localization_system.cpp)
target_link_libraries(whycon ${OpenCV_LIBS})

add_executable(whycon-node src/ros2/whycon_node.cpp src/ros2/whycon_ros.cpp)
set_target_properties(whycon-node PROPERTIES OUTPUT_NAME whycon)

ament_target_dependencies(whycon-node 
rclcpp 
camera_info_manager 
sensor_msgs 
geometry_msgs 
tf2_geometry_msgs 
tf2 
cv_bridge 
image_transport 
image_geometry 
std_srvs 
)

target_link_libraries(whycon-node whycon)

### INSTALL ###

### Mark cpp header files for installation

install(DIRECTORY include/
DESTINATION include/${PROJECT_NAME}
)
## Mark executables and/or libraries for installation
install(TARGETS
  whycon 
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS whycon-node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(
  include
)

ament_export_libraries(
  whycon
)

ament_package()
