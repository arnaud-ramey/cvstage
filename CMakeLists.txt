### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(cvstage)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(ROSDEPS  image_geometry  cv_bridge  rosbag)
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
find_package( OpenCV REQUIRED )
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# nothing to do
### 5 - Invoke message/service/action generation (generate_messages())
# nothing to do
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROSDEPS}
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(test_cvstage.exe  src/test_cvstage.cpp
  include/cvstage/cvstage.h
  include/cvstage/plugins/draw_costmap.h
  include/cvstage/plugins/draw_ellipses.h
  include/cvstage/plugins/draw_xy_lines.h
  include/cvstage/plugins/reproject_image.h
)
target_link_libraries(test_cvstage.exe  ${catkin_LIBRARIES} ${OpenCV_LIBS})
### 8 - Tests to build (catkin_add_gtest())
add_subdirectory(test)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
