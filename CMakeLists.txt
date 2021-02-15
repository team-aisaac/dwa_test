cmake_minimum_required(VERSION 3.0.2)
project(dwa_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  PRIVATE /usr/include/python2.7
)


add_library(${PROJECT_NAME}
  src/dwa_planner.cpp
)

add_executable(${PROJECT_NAME}_node src/planning_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  python2.7
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
