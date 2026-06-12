cmake_minimum_required(VERSION 3.8)
project(vision_detector)

# ---- find dependencies ------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---- generate custom messages -----------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# ---- Python package ---------------------------------------------------
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}"
)

# ---- install config & launch ------------------------------------------
install(DIRECTORY config models launch
  DESTINATION share/${PROJECT_NAME}
)

# ---- ament ------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
