cmake_minimum_required(VERSION 2.8.3)
project(uxa_serial)

find_package(catkin REQUIRED COMPONENTS
  uxa_serial_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uxa_serial
  CATKIN_DEPENDS roscpp uxa_serial_msgs
  DEPENDS system_lib
)

include_directories(include
)

add_executable(uxa_serial src/uxa_serial.cpp)
 target_link_libraries(uxa_serial
   ${catkin_LIBRARIES}
   
 )

add_dependencies(uxa_serial uxa_serial_msgs_generate_messages_cpp)
