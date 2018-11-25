cmake_minimum_required(VERSION 2.8)
project(base_control)
SET(CMAKE_CXX_FLAGS "-O3 -pthread -std=c++11")
SET(LIB_SERIAL_DIR ${PROJECT_SOURCE_DIR}/thirdparty/libserial/lib)
find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    realtime_tools
    tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)
include_directories(
  include
)
include_directories(SYSTEM
  include/imu
  ${LIB_SERIAL_DIR}/../include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${LIB_SERIAL_DIR}/../include)
link_directories(${LIB_SERIAL_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME} SHARED src/base_controller.cpp src/odometry.cpp src/imu/packet.c src/imu/imu_data_decode.c)
add_executable(${PROJECT_NAME}_node src/serial_port_read.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${LIB_SERIAL_DIR}/libserial.so)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

