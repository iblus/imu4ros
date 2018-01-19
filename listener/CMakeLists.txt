cmake_minimum_required(VERSION 2.8.3)
project(leador_imu_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  leador_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_listener_node listener.cpp)
target_link_libraries(${PROJECT_NAME}_listener_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_listener_node PROPERTIES OUTPUT_NAME listener_node PREFIX "")

install(TARGETS ${PROJECT_NAME}_listener_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


