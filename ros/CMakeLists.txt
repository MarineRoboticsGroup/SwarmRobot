# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.10.2)
project(swarm_robot)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

# Find catkin packages and libraries for catkin and system dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  cmake_modules
)

# Declare catkin specific configuration to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_srvs sensor_msgs trajectory_msgs geometry_msgs
)

# Build Teleop
set(TELEOP_INC_DIR ./include)
set(TELEOP_SRC_DIR ./src)
set(teleop_headers
 ${TELEOP_INC_DIR}/swarm_teleop.h
)
set(teleop_src
 ${TELEOP_SRC_DIR}/swarm_teleop.cpp
)
add_library(teleop_lib ${teleop_src})
target_link_libraries(teleop_lib ${catkin_LIBRARIES})
target_include_directories(teleop_lib
  PUBLIC
    ${TELEOP_INC_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_teleop_node
  ${TELEOP_SRC_DIR}/swarm_teleop_node.cpp
  ${TELEOP_SRC_DIR}/swarm_teleop.cpp
)

target_include_directories(swarm_teleop_node
  PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${TELEOP_INC_DIR}
)
target_link_libraries(swarm_teleop_node
    ${catkin_LIBRARIES}
    ${teleop_lib}
)

add_dependencies(swarm_teleop_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#################################################################################
## Install
#################################################################################
install(TARGETS swarm_teleop_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
