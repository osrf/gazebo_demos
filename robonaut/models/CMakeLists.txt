set (dirs
  monkey_wrench_demo
  robonaut_hydra
  taskboard_demo
)

install(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo_demos/models/)