Prerequisites
==

$ sudo apt-get install libeigen3-dev

$ cd /tmp

$ git clone http://github.com/orocos/orocos_kinematics_dynamics.git

$ cd /tmp/orocos_kinematics_dynamics/orocos-kdl

$ mkdir build; cd build

$ cmake ../ -DCMAKE_INSTALL_PREFIX=/usr; make 

$ sudo make install


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
