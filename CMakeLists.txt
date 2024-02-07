cmake_minimum_required(VERSION 3.4)
project(franka_sara_shield_controller)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  joint_limits_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  urdf
  visualization_msgs
  franka_gazebo
)

# when sara-shield is only build locally, use the next line, otherwise comment out
set(safety_shield_DIR /home/user/sara-shield/safety_shield/build)

find_package(Eigen3 REQUIRED)
find_package(safety_shield REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
  cfg/dual_arm_compliance_param.cfg
  cfg/teleop_param.cfg
  cfg/teleop_gripper_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_sara_shield_controller
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
    franka_gazebo
  DEPENDS Franka
)

add_library(franka_sara_shield_controller
  src/sara_shield_controller.cpp
)

add_dependencies(franka_sara_shield_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_sara_shield_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_sara_shield_controller SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(franka_sara_shield_controller PUBLIC
  include
)


add_executable(sara_shield_node
  src/sara_shield_ros_node.cpp
)

add_dependencies(sara_shield_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  safety_shield
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(sara_shield_node PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  safety_shield
)

target_include_directories(sara_shield_node SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(sara_shield_node PUBLIC
  include
)

#add_executable(teleop_gripper_node
#  src/teleop_gripper_node.cpp
#)
#target_include_directories(teleop_gripper_node PUBLIC
#  ${catkin_INCLUDE_DIRS}
#)
#target_link_libraries(teleop_gripper_node PUBLIC
#  ${Franka_LIBRARIES}
#  ${catkin_LIBRARIES}
#)
#add_dependencies(teleop_gripper_node
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${PROJECT_NAME}_gencpp
#  ${PROJECT_NAME}_gencfg
#)

## Installation
install(TARGETS franka_sara_shield_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_sara_shield_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_install_python(
#  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_sara_shield_controller FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_sara_shield_controller
    FILES ${SOURCES}
    DEPENDS franka_sara_shield_controller
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()
