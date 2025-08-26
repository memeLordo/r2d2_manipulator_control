cmake_minimum_required(VERSION 3.0.2)
project(manipulator_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  r2d2_msg_pkg 
)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  # include/${PROJECT_NAME}/utils
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/IConfigJson.cpp
  src/ManipulatorControl.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/manipulator_control_node.cpp # ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(FILES
  launch/manipulator_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulator_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
