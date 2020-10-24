################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.10.2)
project(swarm_robot)

set(CMAKE_CXX_STANDARD 17)

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

################################################################################
# Build
################################################################################


add_executable(swarmbot_traj_wheel_operator
  src/traj_wheel_operator.cpp
)
add_dependencies(swarmbot_traj_wheel_operator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(swarmbot_traj_wheel_operator
  PUBLIC
    /include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(swarmbot_traj_wheel_operator
  PUBLIC
    ${catkin_LIBRARIES}
)

#################################################################################
## Install
#################################################################################
install(TARGETS swarmbot_traj_wheel_operator
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
