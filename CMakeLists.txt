cmake_minimum_required (VERSION 2.8.12)
project (agni_serial_protocol)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tactile_msgs
)

find_package(Boost 1.38 REQUIRED COMPONENTS system program_options)
find_package(Curses REQUIRED)

if(catkin_FOUND)
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES agni_serial_protocol
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tactile_msgs
  #  DEPENDS system_lib
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
  add_definitions(-DHAVE_ROS=1)
endif(catkin_FOUND)

include_directories(
  libio
  ./
  ${CURSES_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
)

# add sub libraries *after* calling catking_package()
add_subdirectory (libio)

if(catkin_FOUND)
  add_executable(${PROJECT_NAME}_node node/agni_serial_protocol_node.cpp)
  target_link_libraries(${PROJECT_NAME}_node
    agni_serial_protocol_io
    ${catkin_LIBRARIES}
    ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
    ${CURSES_NCURSES_LIBRARY}
  )
endif(catkin_FOUND)

add_executable(test_libio test/test_libio.cpp)
target_link_libraries(test_libio
  agni_serial_protocol_io
  ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
  ${CURSES_NCURSES_LIBRARY}
)

add_executable(mpu9250 test/mpu9250.cpp)
target_link_libraries(mpu9250
  agni_serial_protocol_io
  ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
  ${CURSES_NCURSES_LIBRARY}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(d launch registration)
  install(DIRECTORY ${d} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach(d)
