cmake_minimum_required(VERSION 2.8.3)
project(xiroi)


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
  TotalTime.msg
  Current.msg 
)

add_service_files(
  FILES
  RecoveryAction.srv
  MissionTimeout.srv
)

catkin_python_setup()

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

include_directories(${catkin_INCLUDE_DIRS})

add_executable(nav_sat_pub src/sensors/nav_sat_pub.cpp)
target_link_libraries(nav_sat_pub ${catkin_LIBRARIES})
