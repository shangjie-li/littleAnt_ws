cmake_minimum_required(VERSION 3.0.2)
project(noval_convert_xy)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  perception_msgs
  serial
  roslib
  roscpp
  rosconsole
  tf
  fake_gps_msgs
  nav_msgs
  sensor_msgs
  geographic_msgs
  geodesy
)


find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Boost REQUIRED COMPONENTS 
	system 
	filesystem 
	thread
)

catkin_package(
)

set(EIGEN_INC_DIR /usr/include/eigen3/)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INC_DIR}
)

add_executable(noval_convert_xy src/noval_convert_xy.cpp)
target_link_libraries(noval_convert_xy ${catkin_LIBRARIES})


