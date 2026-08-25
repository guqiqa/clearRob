cmake_minimum_required(VERSION 3.8)
project(s90_vpu_input)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cleaning_robot_interfaces REQUIRED)

set(S90_VPU_SDK_ROOT "/opt/bestmow/native_camera/releases/20260820-x5-native-v1/3rdparty/D_Robotics_X5"
    CACHE PATH "Hobot X5 media SDK root")
find_path(S90_VPU_INCLUDE_DIR hb_media_codec.h
  PATHS "${S90_VPU_SDK_ROOT}/include" /usr/hobot/include
  NO_DEFAULT_PATH)
find_library(S90_VPU_LIBRARY NAMES multimedia multimedia.so.1
  PATHS /usr/hobot/lib "${S90_VPU_SDK_ROOT}/lib/aarch64"
  NO_DEFAULT_PATH)
if(NOT S90_VPU_LIBRARY AND EXISTS "/usr/hobot/lib/libmultimedia.so.1")
  set(S90_VPU_LIBRARY "/usr/hobot/lib/libmultimedia.so.1")
endif()

add_executable(s90_vpu_input_node src/s90_vpu_input_node.cpp)
target_compile_features(s90_vpu_input_node PUBLIC cxx_std_17)
target_link_libraries(s90_vpu_input_node pthread)
ament_target_dependencies(s90_vpu_input_node rclcpp sensor_msgs cleaning_robot_interfaces)

if(S90_VPU_INCLUDE_DIR AND S90_VPU_LIBRARY)
  message(STATUS "Hobot VPU SDK enabled: ${S90_VPU_INCLUDE_DIR}, ${S90_VPU_LIBRARY}")
  target_compile_definitions(s90_vpu_input_node PRIVATE S90_VPU_HAS_SDK=1)
  target_include_directories(s90_vpu_input_node PRIVATE "${S90_VPU_INCLUDE_DIR}")
  target_link_libraries(s90_vpu_input_node "${S90_VPU_LIBRARY}")
else()
  message(WARNING "Hobot VPU SDK not found; building a diagnostic stub")
endif()

set_target_properties(s90_vpu_input_node PROPERTIES
  BUILD_RPATH "/usr/hobot/lib;${S90_VPU_SDK_ROOT}/lib/aarch64"
  INSTALL_RPATH "/usr/hobot/lib;${S90_VPU_SDK_ROOT}/lib/aarch64")

install(TARGETS s90_vpu_input_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
ament_package()
