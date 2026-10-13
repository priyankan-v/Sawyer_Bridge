cmake_minimum_required(VERSION 3.16)
project(intera_core_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnalogIOState.msg"
  "msg/AnalogIOStates.msg"
  "msg/AnalogOutputCommand.msg"
  "msg/CameraControl.msg"
  "msg/CameraSettings.msg"
  "msg/CollisionAvoidanceState.msg"
  "msg/CollisionDetectionState.msg"
  "msg/DigitalIOState.msg"
  "msg/DigitalIOStates.msg"
  "msg/DigitalOutputCommand.msg"
  "msg/EndpointNamesArray.msg"
  "msg/EndpointState.msg"
  "msg/EndpointStates.msg"
  "msg/HeadPanCommand.msg"
  "msg/HeadState.msg"
  "msg/HomingCommand.msg"
  "msg/HomingState.msg"
  "msg/IOComponentCommand.msg"
  "msg/IOComponentConfiguration.msg"
  "msg/IOComponentStatus.msg"
  "msg/IODataStatus.msg"
  "msg/IODeviceConfiguration.msg"
  "msg/IODeviceStatus.msg"
  "msg/IONodeConfiguration.msg"
  "msg/IONodeStatus.msg"
  "msg/IOStatus.msg"
  "msg/InteractionControlCommand.msg"
  "msg/InteractionControlState.msg"
  "msg/JointCommand.msg"
  "msg/JointLimits.msg"
  "msg/NavigatorState.msg"
  "msg/NavigatorStates.msg"
  "msg/RobotAssemblyState.msg"
  "msg/SEAJointState.msg"
  "msg/URDFConfiguration.msg"
  "srv/IOComponentCommandSrv.srv"
  "srv/SolvePositionFK.srv"
  "srv/SolvePositionIK.srv"
  "action/CalibrationCommand.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces action_msgs)
install(FILES bridge_mapping.yaml DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
