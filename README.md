Prerequisites
==

$ sudo apt-get install ros-hydro-orocos-kdl

Compilation
==

$ mkdir build

$ cd build

$ cmake ..

$ make

$ sudo make install

Running
==
Source Gazebo setup.sh file. Example:

$ . /usr/share/gazebo/setup.bash

Source gazebo_demos setup.sh file. Example:

$ . /usr/share/gazebo_demos/setup.bash

All the demos start with the prefix gazebo_demo, for xample:

$ gazebo_demo_atlas