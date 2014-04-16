cmake_minimum_required(VERSION 2.8.3)
project(eigen_extensions)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS bag_of_tricks)
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eigen_extensions
  CATKIN_DEPENDS bag_of_tricks
  DEPENDS EIGEN Boost
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/eigen_extensions.h
  include/${PROJECT_NAME}/random.h
  src/random.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES})


add_executable(cat src/cat.cpp)
target_link_libraries(cat ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(convert src/convert.cpp)
target_link_libraries(convert ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_eigen_extensions src/test_eigen_extensions.cpp)
  target_link_libraries(test_eigen_extensions ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES})

  catkin_add_gtest(test_eigen_extensions_random src/test_random.cpp)
  target_link_libraries(test_eigen_extensions_random ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES})
endif()
