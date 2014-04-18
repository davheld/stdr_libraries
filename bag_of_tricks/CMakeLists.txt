cmake_minimum_required(VERSION 2.8.3)
project(bag_of_tricks)
set(CMAKE_BUILD_TYPE RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS roscpp serializable timer)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp serializable timer
  DEPENDS Boost OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/bag_of_tricks/bag_of_tricks.h
  include/bag_of_tricks/connected_components.h
  include/bag_of_tricks/glob.h
  include/bag_of_tricks/image_region_iterator.h
  include/bag_of_tricks/next_path.h
  src/image_region_iterator.cpp
  src/connected_components.cpp
  src/next_path.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
if(NOT APPLE)
target_link_libraries(${PROJECT_NAME} rt)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
