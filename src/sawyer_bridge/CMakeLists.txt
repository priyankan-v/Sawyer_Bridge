cmake_minimum_required(VERSION 3.5)

project(sawyer_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# find ROS 1 packages
set(cmake_extras_files cmake/find_ros1_package.cmake cmake/find_ros1_interface_packages.cmake)
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(WARNING "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  if(NOT ROS1_PACKAGES_UNAVAILABLE)
    # Only warn if ROS1 packages were expected to be available
    message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  endif()
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package(
    CONFIG_EXTRAS ${cmake_extras_files}
  )
  return()
endif()

find_ros1_package(std_msgs REQUIRED)

# Dependency that we should only look for if ROS 1 is installed (it's not present on a ROS 2
# system; see https://github.com/ros2/ros1_bridge/pull/331#issuecomment-1188111510)
find_package(xmlrpcpp REQUIRED)

# find ROS 1 packages with messages / services
include(cmake/find_ros1_interface_packages.cmake)
find_ros1_interface_packages(ros1_message_packages)

set(prefixed_ros1_message_packages "")
foreach(ros1_message_package ${ros1_message_packages})
  # TODO(karsten1987): This is currently a workaround to work with ROS 2 classloader
  # rather than ROS 1 classloader.
  if(NOT "${ros1_message_package}" STREQUAL "nodelet")
    find_ros1_package(${ros1_message_package} REQUIRED)
    list(APPEND prefixed_ros1_message_packages "ros1_${ros1_message_package}")
  endif()
endforeach()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS bin/configure_bridge bin/check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS ${cmake_extras_files}
)

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/get_factory.cpp")
list(APPEND generated_files "${generated_path}/get_mappings.cpp")

# generate per interface compilation units to keep the memory usage low
ament_index_get_resources(ros2_interface_packages "rosidl_interfaces")
# A Sawyer package build limits discovery to core and Intera interfaces.
# Unset this variable to retain the upstream discovery behavior.
if(DEFINED ENV{SAWYER_BRIDGE_PACKAGES})
  set(ros2_interface_packages "$ENV{SAWYER_BRIDGE_PACKAGES}")
endif()
# actionlib_msgs is deprecated, but we will quiet the warning until the bridge has support for
# ROS actions: https://github.com/ros2/design/issues/195
set(actionlib_msgs_DEPRECATED_QUIET TRUE)
foreach(package_name ${ros2_interface_packages})
  find_package(${package_name} QUIET REQUIRED)
  message(STATUS "Found ${package_name}: ${${package_name}_VERSION} (${${package_name}_DIR})")
  if(NOT "${package_name}" STREQUAL "builtin_interfaces")
    list(APPEND generated_files "${generated_path}/${package_name}_factories.cpp")
    list(APPEND generated_files "${generated_path}/${package_name}_factories.hpp")
    foreach(interface_file ${${package_name}_IDL_FILES})
      file(TO_CMAKE_PATH "${interface_file}" interface_name)
      get_filename_component(interface_basename "${interface_name}" NAME_WE)
      # skipping actions and request and response messages of services
      if(NOT "${interface_name}" MATCHES "^(msg|srv)/" OR "${interface_basename}" MATCHES "_(Request|Response)$")
        continue()
      endif()
      string(REPLACE "/" "__" interface_name "${interface_name}")
      get_filename_component(interface_name "${interface_name}" NAME_WE)
      list(APPEND generated_files "${generated_path}/${package_name}__${interface_name}__factories.cpp")
    endforeach()
  endif()
endforeach()

set(target_dependencies
  "bin/ros1_bridge_generate_factories"
  "resource/get_factory.cpp.em"
  "resource/get_mappings.cpp.em"
  "resource/interface_factories.cpp.em"
  "resource/pkg_factories.cpp.em"
  "resource/pkg_factories.hpp.em"
  "ros1_bridge/__init__.py")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND Python3::Interpreter
  ARGS bin/ros1_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
  DEPENDS ${target_dependencies}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating factories for interface types")

if(NOT WIN32)
  # ignore warning in ROS 1 message headers
  set_source_files_properties(${generated_files}
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter")
endif()

include_directories(include ${generated_path})

function(custom_executable target)
  cmake_parse_arguments(
    ARG "ROS1_DEPENDENCIES" "" "TARGET_DEPENDENCIES" ${ARGN})

  add_executable(${target}
    ${ARG_UNPARSED_ARGUMENTS})
  ament_target_dependencies(${target}
    "rclcpp"
    ${ARG_TARGET_DEPENDENCIES})
  if(ARG_ROS1_DEPENDENCIES)
    ament_target_dependencies(${target}
      "ros1_roscpp"
      "ros1_std_msgs")
  endif()
  if(ARG_DEPENDENCIES)
    add_dependencies(${target} ${ARG_DEPENDENCIES})
  endif()

  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

add_library(${PROJECT_NAME} SHARED
  "src/builtin_interfaces_factories.cpp"
  "src/convert_builtin_interfaces.cpp"
  "src/bridge.cpp"
  ${generated_files})
ament_target_dependencies(${PROJECT_NAME}
  ${prefixed_ros1_message_packages}
  ${ros2_interface_packages}
  "rclcpp"
  "ros1_roscpp"
  "ros1_std_msgs")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

custom_executable(parameter_bridge
  "src/parameter_bridge.cpp"
  "src/actions.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${message_packages})
ament_target_dependencies(parameter_bridge rclcpp_action intera_core_msgs intera_motion_msgs)
target_link_libraries(parameter_bridge
  ${PROJECT_NAME})

custom_executable(dynamic_bridge
  "src/dynamic_bridge.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${ros2_interface_packages})
target_link_libraries(dynamic_bridge
  ${PROJECT_NAME})

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
