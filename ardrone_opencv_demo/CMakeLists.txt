cmake_minimum_required(VERSION 2.8)
project(ardrone_opencv_demo)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
	cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
	ardrone_autonomy
)

#catkin_python_setup()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ../../devel/include
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/kar_mavros_offboard_tutorial_node.cpp)
add_executable(opencv_demo src/opencv_demo.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(opencv_demo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
