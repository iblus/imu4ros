cmake_minimum_required(VERSION 2.8.3)
project(leador_imu_uart)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  leador_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_imu_node src/node.cpp src/imu.cpp src/uart.cpp src/loop_queue.cpp)
target_link_libraries(${PROJECT_NAME}_imu_node ${catkin_LIBRARIES} pthread)
set_target_properties(${PROJECT_NAME}_imu_node PROPERTIES OUTPUT_NAME imu_node PREFIX "")

install(TARGETS ${PROJECT_NAME}_imu_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

