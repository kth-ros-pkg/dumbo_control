cmake_minimum_required(VERSION 2.8.3)
project(dumbo_hw_control_loop)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    dumbo_hardware_interface
    controller_manager
    realtime_tools
    control_msgs
    std_srvs
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS  roscpp dumbo_hardware_interface controller_manager realtime_tools control_msgs std_srvs std_msgs
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(dumbo_hw_control_loop src/dumbo_hw_control_loop.cpp)
target_link_libraries(dumbo_hw_control_loop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS dumbo_hw_control_loop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
