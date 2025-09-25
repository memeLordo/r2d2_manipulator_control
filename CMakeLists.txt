cmake_minimum_required(VERSION 3.0.2)
project(manipulator_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  r2d2_msg_pkg
  r2d2_utils_pkg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp roslib r2d2_msg_pkg r2d2_utils_pkg
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
  src/ConfigMath.cpp
  src/ManipulatorControl.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulator_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
