cmake_minimum_required(VERSION 2.8.3)
project(px4_getPoint)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_extras
  geometry_msgs
  angles
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(px4_getPoint src/px4_getpoint_node.cpp)


target_link_libraries(px4_getPoint
   ${catkin_LIBRARIES}
 )

