cmake_minimum_required(VERSION 2.8.3)
project(system)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_message_files(
  FILES
  test.msg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    roscpp
)

###########
## Build ##
###########
link_directories(/home/iclab/catkin_ws/src/system/libs)
set(MOTORS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/Motors.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(Motor_Lib ${MOTORS_SRCS})

add_executable(motion src/motion.cpp)
add_dependencies(motion motion_generate_messages_cpp)
target_link_libraries(motion Motor_Lib EPOS ${catkin_LIBRARIES})
