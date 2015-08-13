cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(DoorPlugin SHARED DoorPlugin.cc)
target_link_libraries(DoorPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Install the library
install (TARGETS DoorPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib COMPONENT shlib)
