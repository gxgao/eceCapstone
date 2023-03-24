#!/bin/sh
# activate the rplidar 
chmod 666 /dev/ttyUSB0

roslaunch rplidar_ros rplidar.launch &

echo "launched rp lidar, roomba pycreate ros stack ... "
roslaunch create_bringup create_2.launch & 

echo "launched roomba create_2, launching nav stack launch file given by cmd line arg" 

roslaunch $1 & 

echo "launching rviz " 
rviz 


