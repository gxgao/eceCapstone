/*
 * Automatic Addison
 * Website: https://automaticaddison.com
 *   ROS node that converts the user's desired initial pose and goal location
 *   into a usable format.
 * Subscribe:
 *   initialpose : The initial position and orientation of the robot using 
 *                 quaternions. (geometry_msgs/PoseWithCovarianceStamped)
 *   move_base_simple/goal : Goal position and 
 *                           orientation (geometry_msgs::PoseStamped)
 * Publish: This node publishes to the following topics:   
 *   goal_2d : Goal position and orientation (geometry_msgs::PoseStamped)
 *   initial_2d : The initial position and orientation of the robot using 
 *                Euler angles. (geometry_msgs/PoseStamped)
 * From Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include statements 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>

using namespace std;
 
// Initialize ROS publishers
ros::Publisher pub;
void handle_laser(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  sensor_msgs::LaserScan new_scan;
  new_scan.header.stamp = scan->header.stamp;
  new_scan.header.frame_id = "laser_180";
  new_scan.angle_min = -2.19;
  new_scan.angle_max = 2.19;
  new_scan.angle_increment = scan->angle_increment;
  new_scan.time_increment = scan->time_increment;
  new_scan.range_min = scan->range_min;
  new_scan.range_max= scan->range_max;

  int SCAN_LEN = 1147;
  int SIZE = 800;
  new_scan.intensities.resize(SIZE);
  new_scan.ranges.resize(SIZE);

  int RIGHT_END = SIZE/2;
  for (int i = 0; i < RIGHT_END; i++) {
    new_scan.ranges[i + RIGHT_END] = scan->ranges[i];
    new_scan.intensities[i + RIGHT_END] = scan->intensities[i];
  }

  int LEFT_START = SCAN_LEN - (SIZE/2) + (SIZE & 1);
  for (int i = LEFT_START; i < SCAN_LEN; i++) {
    new_scan.ranges[i - LEFT_START] = scan->ranges[i];
    new_scan.intensities[i - LEFT_START] = scan->intensities[i];
  }
  std::cout << "callback " << std::endl;
  pub.publish(new_scan);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "new_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(10);
  pub = node.advertise<sensor_msgs::LaserScan>("/scan_180", 1000);
  ros::Subscriber sub = node.subscribe("/scan", 0, handle_laser);
  ros::spin();
  return 0;
}

