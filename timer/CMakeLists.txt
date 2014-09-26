cmake_minimum_required(VERSION 2.8.3)
project(timer)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(timer
  include/timer/timer.h
  src/lib/timer.cpp
)
target_link_libraries(timer ${catkin_LIBRARIES})

if(NOT APPLE)
  target_link_libraries(timer rt)
endif()

if(CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(test_timer src/test/test_timer.cpp)
  target_link_libraries(test_timer timer)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
