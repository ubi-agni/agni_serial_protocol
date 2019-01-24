file(GLOB SOURCES *.h *.cpp)

find_package(yaml-cpp)

include_directories(
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_io STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME}_io
                      ${YAML_CPP_LIBRARIES}
                      ${catkin_LIBRARIES}
)
