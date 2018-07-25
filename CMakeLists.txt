cmake_minimum_required(VERSION 2.8.3)
project(um7_driver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  serial
)

find_package(Boost REQUIRED)

catkin_package(
    #INCLUDE_DIRS include
  LIBRARIES um7_driver
  CATKIN_DEPENDS message_generation rospy roscpp sensor_msgs serial
  DEPENDS Boost
)


include_directories(#include
  ${catkin_INCLUDE_DIRS}
)

add_executable(um7_driver_node src/um7_driver.cpp)
target_link_libraries(um7_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(um7_driver_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

