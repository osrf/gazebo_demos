cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED COMPONENTS system)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})

find_package(orocos-kdl REQUIRED COMPONENTS system)
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} orocos_kdl)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(SimpleIKPlugin SHARED SimpleIKPlugin.cc)
target_link_libraries(SimpleIKPlugin
 ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
)

# Install the library
install (TARGETS SimpleIKPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib COMPONENT shlib)
