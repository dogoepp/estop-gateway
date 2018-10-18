cmake_minimum_required(VERSION 2.8.3)
project(estop_gateway)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

add_message_files(
  FILES Heartbeat.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  message_runtime
  std_msgs
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})