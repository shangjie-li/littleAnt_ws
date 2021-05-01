cmake_minimum_required(VERSION 2.8.3)
project(driverless)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  diagnostic_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TrackingState.msg
)

## Generate actions in the 'action' folder
add_action_files(DIRECTORY action 
  FILES 
  DoReverse.action
  DoDriverlessTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/auto_drive_base.cpp 
                                    src/driverless_node.cpp 
                                    src/driverless_fun.cpp
                                    src/avoiding.cpp
                                    src/path_tracking.cpp
                                    src/car_following.cpp
                                    src/extern_control.cpp
                                    src/reverse_drive.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_node PRIVATE ${OpenCV_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /usr/local/lib/libtinyxml2.a)

add_executable(actionlib_client_node src/actionlib_client.cpp)
add_dependencies(actionlib_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(actionlib_client_node ${catkin_LIBRARIES})

