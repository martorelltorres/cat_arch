cmake_minimum_required(VERSION 2.8.3)
project(cat_arch)


## Find catkin and any catkin packages
find_package(catkin REQUIRED
	joy
	roscpp
	rospy
	sensor_msgs
	std_msgs
  std_srvs
  message_generation)


add_message_files(
  FILES
  Setpoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  joy
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_runtime)