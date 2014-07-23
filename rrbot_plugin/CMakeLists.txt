cmake_minimum_required(VERSION 2.8.3)
project(rrbot_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  gazebo_ros
  std_msgs
  sensor_msgs
  kdl_parser
  )

# cisst
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstRobot
  cisstNumerical
  )

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

# KDL
find_package(orocos_kdl REQUIRED)


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS})


# --------- Start targets -----------------
add_library(${PROJECT_NAME} SHARED src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(rrbot_gravity SHARED
  src/rrbot_gravity_plugin.cpp
  )
target_link_libraries(rrbot_gravity
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  )
cisst_target_link_libraries (rrbot_gravity ${REQUIRED_CISST_LIBRARIES})

catkin_package(
  DEPENDS
  roscpp
  gazebo_ros
  )
