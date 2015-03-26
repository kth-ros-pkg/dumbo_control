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
    diagnostic_msgs
    diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/dumbo_hw_control_loop.cpp)
target_link_libraries(${PROJECT_NAME} rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_custom_command(
  TARGET ${PROJECT_NAME}
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/set_uid ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/${PROJECT_NAME} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/run_hw_control_loop scripts/set_uid DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
