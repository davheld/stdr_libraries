cmake_minimum_required(VERSION 2.8.3)
project(stdr_grid)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS bag_of_tricks timer)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bag_of_tricks timer
)


include_directories(include ${catkin_INCLUDE_DIRS})


# This executable is a dummy one. It's purpose is to make sure that the
# templated classes in the header files compile ok
add_executable(sanity_check
  src/sanity_check.cpp
  include/stdr_grid/grid.h
  include/stdr_grid/rolling_grid.h
)


# This does not perform API tests on grid, but simply verify that our custom
# floor function is faster that the standard one.
add_executable(test_floor_time src/test_floor_time.cpp)
target_link_libraries(test_floor_time ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_grid src/test_grid.cpp)
endif(CATKIN_ENABLE_TESTING)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

