cmake_minimum_required(VERSION 2.8.3)
project(herkulex_controllers)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(
  FILES
  RestartController.srv
  SetSpeed.srv
  SetTorqueLimit.srv
  StartController.srv
  StopController.srv
  TorqueEnable.srv
  ClearErrors.srv
)

catkin_python_setup()

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  herkulex_driver
  std_msgs
  sensor_msgs
  trajectory_msgs
  diagnostic_msgs
  herkulex_msgs
  control_msgs
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/controller_manager.py
  nodes/controller_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
