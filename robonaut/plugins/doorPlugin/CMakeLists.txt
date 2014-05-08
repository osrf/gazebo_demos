cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(DoorPlugin SHARED DoorPlugin.cc)
target_link_libraries(DoorPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Install the library
set_target_properties(DoorPlugin PROPERTIES SOVERSION ${PROJECT_MAJOR_VERSION} VERSION ${PROJECT_VERSION_FULL})
install (TARGETS DoorPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib COMPONENT shlib)