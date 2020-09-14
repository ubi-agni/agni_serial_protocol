file(GLOB SOURCES *.h *.cpp)

find_package(yaml-cpp)

include_directories(
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_io STATIC ${SOURCES})
add_dependencies(${PROJECT_NAME}_io ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_io
                      ${YAML_CPP_LIBRARIES}
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_io
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
