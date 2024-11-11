cmake_minimum_required(VERSION 3.5)
project(insta360_ros2_driver)

# Set library and include directories after installation
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/lib")
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")


# Find ament packages and dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Optional: AVCodec, AVFormat, and AVUtil (if using FFmpeg libraries)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)

# Include directories
include_directories(${PROJECT_NAME}_node PRIVATE 
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  include
  lib
)


# Define executable and include directories
add_executable(${PROJECT_NAME}_node src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp cv_bridge image_transport sensor_msgs std_msgs OpenCV AVCODEC AVFORMAT AVUTIL)


# Link libraries to the executable
target_link_libraries(${PROJECT_NAME}_node 
  "${PROJECT_SOURCE_DIR}/lib/libCameraSDK.so"
  ${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES}
)

# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/compression.py
  scripts/undistortion.py
  scripts/live_processing.py
  scripts/get_images.py
  scripts/record.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  lib
  DESTINATION lib/${PROJECT_NAME}
)
# Package configuration
ament_package()


