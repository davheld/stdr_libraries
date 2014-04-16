cmake_minimum_required(VERSION 2.8.3)
project(matplotlib_interface)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

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

add_executable(example src/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
