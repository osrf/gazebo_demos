# Script to launch the Atlas demo.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gazebo_demo_atlas.in ${PROJECT_BINARY_DIR}/atlas/bin/gazebo_demo_atlas @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gazebo_demo_atlas
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)
