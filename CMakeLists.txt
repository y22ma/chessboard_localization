cmake_minimum_required(VERSION 2.8.3)
project(chessboard_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs sensor_msgs image_transport cv_bridge tf
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES chessboard_localization
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs image_transport cv_bridge tf
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(chessboard_localization_node
  src/chessboard_localization.cpp src/chessboard_localization_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(chessboard_localization_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
