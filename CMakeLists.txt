cmake_minimum_required (VERSION 2.8.12)
project (agni_serial_protocol)

#find_package(Boost 1.38 REQUIRED COMPONENTS thread program_options regex)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
#find_package(Qt54 COMPONENTS Core Gui Svg Xml SerialPort)

# add sub libraries *after* calling catking_package()
add_subdirectory (libio)

find_package(Boost 1.38 REQUIRED COMPONENTS program_options)

find_package(Curses QUIET)
if(CURSES_FOUND)
   add_definitions(-DHAVE_CURSES=1)
endif()


include_directories(
  libio
  ${CURSES_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
)
add_executable(test_libio test_libio.cpp)
target_link_libraries(test_libio 
  agni_serial_protocol_io
  ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
  ${CURSES_NCURSES_LIBRARY}
)
