cmake_minimum_required(VERSION 3.8)
project(native_stereo_camera)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cleaning_robot_interfaces REQUIRED)

set(NATIVE_CAMERA_ROOT "/opt/bestmow/native_camera/releases/20260820-x5-native-v1"
    CACHE PATH "Root of the native camera release")
set(NATIVE_CAMERA_OUT "${NATIVE_CAMERA_ROOT}/out")
find_library(NATIVE_CAMERA_LIBRARY camera PATHS "${NATIVE_CAMERA_OUT}" NO_DEFAULT_PATH REQUIRED)
find_library(NATIVE_LOG_LIBRARY log PATHS "${NATIVE_CAMERA_OUT}" NO_DEFAULT_PATH REQUIRED)

add_executable(native_stereo_camera_node src/native_stereo_camera_node.cpp)
target_compile_features(native_stereo_camera_node PUBLIC cxx_std_17)
target_include_directories(native_stereo_camera_node PRIVATE
  "${NATIVE_CAMERA_ROOT}/apps/communication/camera/External_Header"
  "${NATIVE_CAMERA_ROOT}/common/robot_data_model/External_Header"
  "${NATIVE_CAMERA_ROOT}/apps/utility_tools/log/include"
)
target_link_libraries(native_stereo_camera_node
  "${NATIVE_CAMERA_LIBRARY}" "${NATIVE_LOG_LIBRARY}")
ament_target_dependencies(native_stereo_camera_node
  rclcpp sensor_msgs std_msgs cleaning_robot_interfaces)
set_target_properties(native_stereo_camera_node PROPERTIES
  BUILD_RPATH "${NATIVE_CAMERA_OUT};/usr/hobot/lib;/usr/hobot/lib/sensor"
  INSTALL_RPATH "${NATIVE_CAMERA_OUT};/usr/hobot/lib;/usr/hobot/lib/sensor"
)

install(TARGETS native_stereo_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
ament_package()
