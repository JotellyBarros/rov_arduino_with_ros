cmake_minimum_required(VERSION 2.8.3)

project(simple_drive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs actionlib_msgs std_msgs
)

catkin_install_python(
  PROGRAMS nodes/simple_drive 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
