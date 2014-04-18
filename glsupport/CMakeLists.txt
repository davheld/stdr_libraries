cmake_minimum_required(VERSION 2.8.3)
project(glsupport)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS angles stdr_lib)

find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(Eigen REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(FTGL REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS angles stdr_lib
  DEPENDS OpenCV EIGEN OPENGL FREETYPE GLUT FTGL
)

find_package(GLUT REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${FTGL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

add_library(${PROJECT_NAME}
  include/glsupport/fontRenderer.h
  include/glsupport/glsupport.h
  include/glsupport/textures.h

  src/textures.cpp
  src/glsupport.cpp
  src/helvetica.h
  src/fontRenderer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${FREETYPE_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${FTGL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
