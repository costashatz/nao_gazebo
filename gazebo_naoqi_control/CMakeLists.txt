cmake_minimum_required(VERSION 2.8.3)
project(gazebo_naoqi_control)


# Tell CMake where to find "FindNAOqiSIM.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Tell Program where to find NAOqi/NAOqiSIM bin
add_definitions(-DNAOQI_SDK="$ENV{AL_DIR}")
add_definitions(-DNAOQI_SIM_SDK="$ENV{AL_SIM_DIR}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  control_toolbox
)

# Load NAOqi dependencies
find_package(NAOqiSIM REQUIRED COMPONENTS 
	alnaosim 
	alsimutils
	alrobotmodel
)

if(NAOqiSIM_FOUND)
	message(STATUS "NAOqi Simulator SDK found!")
endif()

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${NAOqiSIM_INCLUDE_DIRS} include)

add_library(gazebo_naoqi_control src/gazebo_naoqi_control_plugin.cpp)
target_link_libraries(gazebo_naoqi_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${NAOqiSIM_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    control_toolbox
)
