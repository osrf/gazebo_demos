# Script to launch the Robonaut demo.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gazebo_demo_robonaut.in ${PROJECT_BINARY_DIR}/robonaut/bin/gazebo_demo_robonaut @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gazebo_demo_robonaut
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo_demos/bin
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)