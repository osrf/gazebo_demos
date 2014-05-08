# setup.sh with some environment variables for plugins, models, world files
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.sh.in ${PROJECT_BINARY_DIR}/cmake/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo_demos/)