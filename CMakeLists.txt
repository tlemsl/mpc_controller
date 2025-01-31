cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_subdirectory(3rdparty/MPC)

set(MPC_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/3rdparty/MPC/includes)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compile options
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  roscpp
  tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_controller
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    roscpp
    tf2
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${MPC_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/includes
)

add_executable(mpc_controller 
  src/Controller.cpp
  src/Ackermann.cpp
  src/Node.cpp
)
target_link_libraries(mpc_controller
  ${catkin_LIBRARIES}
  MJMPC
  pthread
  Eigen3::Eigen
)

