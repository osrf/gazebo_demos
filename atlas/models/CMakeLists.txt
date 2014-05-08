set (dirs
  atlas_hydra
  bin
  multisense_sl
)

install(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo_demos/models/)
