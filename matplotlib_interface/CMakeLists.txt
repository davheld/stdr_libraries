cmake_minimum_required(VERSION 2.8.3)
project(matplotlib_interface)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED)
find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS EIGEN PYTHON
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_library(${PROJECT_NAME}
  include/matplotlib_interface/matplotlib_interface.h
  src/matplotlib_interface.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(matplotlib_interface_example src/example.cpp)
target_link_libraries(matplotlib_interface_example ${PROJECT_NAME})
set_target_properties(matplotlib_interface_example
                      PROPERTIES OUTPUT_NAME example
                      PREFIX "")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
