cmake_minimum_required(VERSION 3.0.2)
project(manipulator_control)

add_compile_options(-std=c++14)

catkin_package(
CATKIN_DEPENDS r2d2_msg_pkg
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  r2d2_msg_pkg 
)

###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ConfigJson.cpp
  src/ManipulatorControl.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/manipulator_control_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############



#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulator_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
