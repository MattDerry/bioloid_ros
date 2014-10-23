cmake_minimum_required(VERSION 2.8.3)
project(bioloid_ros_driver)

find_package(catkin REQUIRED COMPONENTS  roscpp  rospy  std_msgs  message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_service_files( FILES allcmd.srv )

generate_messages (  DEPENDENCIES  std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bioloid_ros_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ~/serial_library/include
)

find_library(ZGB_LIBRARY zgb ~/serial_library/lib)
add_executable(bioloid_driver src/bioloid_driver.cpp)
add_dependencies(bioloid_driver bioloid_ros_driver_generate_messages_cpp)
add_dependencies(bioloid_driver bioloid_ros_driver_gencpp)
target_link_libraries(bioloid_driver ${catkin_LIBRARIES})
target_link_libraries(bioloid_driver ${ZGB_LIBRARY})
target_link_libraries(bioloid_driver ${Boost})
