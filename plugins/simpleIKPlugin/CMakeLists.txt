cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED COMPONENTS system)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
#link_directories(${Eigen3_LIBRARY_DIRS})

find_package(orocos_kdl REQUIRED COMPONENTS system)
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(SimpleIKPlugin SHARED SimpleIKPlugin.cc)
target_link_libraries(SimpleIKPlugin
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES}
)

# Install the library
install (TARGETS SimpleIKPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib COMPONENT shlib)
