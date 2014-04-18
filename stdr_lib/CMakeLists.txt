cmake_minimum_required(VERSION 2.8.3)
project(stdr_lib)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS filesystem)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stdr_lib
  CATKIN_DEPENDS roscpp
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/stdr_lib/exception.h
  include/stdr_lib/roslib_helpers.h
  include/stdr_lib/unit_conversions.h
  include/stdr_lib/rosparam_helpers.h

  src/roslib_helpers.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_find_file src/test_find_file.cpp)
target_link_libraries(test_find_file ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

