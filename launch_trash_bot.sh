#!/bin/sh
# activate the rplidar 
chmod 666 /dev/ttyUSB0

echo "launched rp lidar, roomba pycreate ros stack ... "

echo "launched roomba create_2, launching nav stack launch file given by cmd line arg" 

roslaunch $1 & 

echo "launching rviz " 
rviz 


