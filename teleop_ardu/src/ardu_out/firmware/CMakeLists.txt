cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(move
  SRCS movement.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega
  PORT /dev/ttyUSB0
)
