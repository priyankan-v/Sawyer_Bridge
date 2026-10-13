cmake_minimum_required(VERSION 3.16)
project(intera_motion_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(intera_core_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndpointTrackingError.msg"
  "msg/InterpolatedPath.msg"
  "msg/JointTrackingError.msg"
  "msg/MotionStatus.msg"
  "msg/TrackingOptions.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryAnalysis.msg"
  "msg/TrajectoryOptions.msg"
  "msg/Waypoint.msg"
  "msg/WaypointOptions.msg"
  "msg/WaypointSimple.msg"
  "action/MotionCommand.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces action_msgs intera_core_msgs)
install(FILES bridge_mapping.yaml DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
