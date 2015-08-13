cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(TeleopR2Plugin SHARED TeleopR2Plugin.cc)
target_link_libraries(TeleopR2Plugin
 ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
)

# Install the library
install (TARGETS TeleopR2Plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib COMPONENT shlib)
