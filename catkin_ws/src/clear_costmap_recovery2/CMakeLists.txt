cmake_minimum_required(VERSION 2.8.3)
project(clear_costmap_recovery2)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            nav_core
            pluginlib
            roscpp
            tf
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES clear_costmap_recovery2
    CATKIN_DEPENDS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf
)

add_library(clear_costmap_recovery2 src/clear_costmap_recovery2.cpp)
target_link_libraries(clear_costmap_recovery2 ${catkin_LIBRARIES})


## Configure Tests
#if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
#  find_package(rostest REQUIRED)

  # Add the test folder to the include directories
#  include_directories(test)

#  include_directories(${GTEST_INCLUDE_DIRS})
#  link_directories(${GTEST_LIBRARY_DIRS})

  # Create targets for test executables
#  add_rostest_gtest(clear_tester test/clear_tests.launch test/clear_tester.cpp)
#  add_dependencies(tests clear_tester ${catkin_LIBRARIES})
#  target_link_libraries(clear_tester clear_costmap_recovery2 ${GTEST_LIBRARIES})
#endif()


install(TARGETS clear_costmap_recovery2
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(FILES ccr_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
