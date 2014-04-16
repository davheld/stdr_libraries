cmake_minimum_required(VERSION 2.8.3)
project(fastjpeg)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS roscpp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
find_package(libjpeg REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS LIBJPEG
)

include_directories(include ${catkin_INCLUDE_DIRS} ${LIBJPEG_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/fastjpeg.cpp include/fastjpeg/fastjpeg.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LIBJPEG_LIBRARIES})
