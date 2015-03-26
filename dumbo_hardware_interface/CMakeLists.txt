cmake_minimum_required(VERSION 2.8.3)
project(dumbo_hardware_interface)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    hardware_interface
    dumbo_powercube_chain
    dumbo_force_torque_sensor
)


find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface dumbo_powercube_chain dumbo_force_torque_sensor
  LIBRARIES dumbo_hw
  DEPENDS
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(dumbo_hw 
  src/dumbo_hw.cpp 
  src/schunk_arm_hw.cpp 
  src/pg70_hw.cpp 
  src/force_torque_sensor_hw.cpp)

target_link_libraries(dumbo_hw ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS dumbo_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)