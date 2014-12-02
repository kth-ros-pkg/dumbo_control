#!/bin/bash

# assumes that the dumbo_hw_control_loop executable is located in ~/catkin_ws/devel/lib/dumbo_hw_control_loop/dumbo_hw_control_loop

cd $1 # cd to the directory with the dumbo_hw_control_loop node
sudo chown root:root $2 # change ownship to root
sudo chmod a+rx $2     # set as executable by all
sudo chmod u+s $2       # set the setuid bit
