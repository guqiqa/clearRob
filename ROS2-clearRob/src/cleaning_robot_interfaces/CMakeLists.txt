cmake_minimum_required(VERSION 3.8)
project(cleaning_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TaskMode.msg"
  "msg/TaskArea.msg"
  "msg/CleanStrategy.msg"
  "msg/MotionCommand.msg"
  "msg/UnifiedCommand.msg"
  "msg/Path.msg"
  "msg/PlannerState.msg"
  "msg/ObstacleItem.msg"
  "msg/ObstacleList.msg"
  "msg/Fusion3DTarget.msg"
  "msg/FusionDecision.msg"
  "msg/MotorStatus.msg"
  "msg/FaultStatus.msg"
  "msg/CleaningProgress.msg"
  "msg/CleaningStats.msg"
  "msg/BrushStatus.msg"
  "msg/DustFull.msg"
  "msg/VisionStatus.msg"
  "msg/Heartbeat.msg"
  "msg/Alert.msg"
  "msg/RCRawSignal.msg"
  "msg/AppCommand.msg"
  "msg/StereoDistance.msg"
)

set(srv_files
  "srv/SetModel.srv"
  "srv/GetStatus.srv"
  "srv/SetPriority.srv"
  "srv/SetMaxSpeed.srv"
  "srv/GetCurrentPose.srv"
  "srv/RequestMap.srv"
  "srv/SetScanFreq.srv"
  "srv/GetStereoDistance.srv"
)

set(action_files
  "action/NavigateToGoal.action"
  "action/FollowPath.action"
  "action/ExecuteCleanup.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
