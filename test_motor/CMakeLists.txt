cmake_minimum_required(VERSION 2.8.3)
project(test_motor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)

add_message_files(
  FILES
  test.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_motor
  CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
#  DEPENDS system_lib
)

#link_directories(/usr/lib)
#link_directories(/usr/local/lib)
link_directories(/home/iclab/catkin_ws/src/test_motor/libs)
set(MOTORS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/Motors.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(Motors ${MOTORS_SRCS})

add_executable(test_motor src/test_motor.cpp)
add_dependencies(test_motor test_motor_generate_messages_cpp)
#target_link_libraries(test_motor Motors EposCmd5 ${catkin_LIBRARIES})
target_link_libraries(test_motor Motors EPOS ${catkin_LIBRARIES})
