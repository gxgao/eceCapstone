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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>

using namespace std;
 
// Initialize ROS publishers
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher move_base_goal_pub; 
 
// Take move_base_simple/goal as input and publish goal_2d
void handle_goal(const geometry_msgs::PoseStamped &goal) {
  geometry_msgs::PoseStamped rpyGoal;
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = goal.header.stamp;
  rpyGoal.pose.position.x = goal.pose.position.x;
  rpyGoal.pose.position.y = goal.pose.position.y;
  rpyGoal.pose.position.z = 0; 
  std::cout << "handling goal with " << goal.pose.position.x << "," << goal.pose.position.y << std::endl; 
  tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rpyGoal.pose.orientation.x = 0;
  rpyGoal.pose.orientation.y = 0;
  rpyGoal.pose.orientation.z = yaw;
  rpyGoal.pose.orientation.w = 0;
  pub.publish(rpyGoal);
}
 
// Take initialpose as input and publish initial_2d
void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStamped &pose) {
  geometry_msgs::PoseStamped rpyPose;
  rpyPose.header.frame_id = "map";
  rpyPose.header.stamp = pose.header.stamp;
  rpyPose.pose.position.x = pose.pose.pose.position.x;
  rpyPose.pose.position.y = pose.pose.pose.position.y;
  rpyPose.pose.position.z = 0;
  std::cout << "handling initial pose with " << rpyPose.pose.position.x << "," << rpyPose.pose.position.y << std::endl; 
  tf::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rpyPose.pose.orientation.x = 0;
  rpyPose.pose.orientation.y = 0;
  rpyPose.pose.orientation.z = yaw;
  rpyPose.pose.orientation.w = 0;
  pub2.publish(rpyPose);
}

// Take initialpose as input and publish initial_2d
void handle_pose_goal_norviz(float_t x, float_t y, float_t yaw) {
  geometry_msgs::PoseStamped rpyPose;
  ROS_INFO("PUBLISHING POSE %f %f %f", x, y, yaw); 
  rpyPose.header.frame_id = "map";
  rpyPose.header.stamp = ros::Time::now();
  rpyPose.pose.position.x = x;
  rpyPose.pose.position.y = y;
  rpyPose.pose.position.z = 0;
  tf::Quaternion q(0, 0, 0, 0);
  tf::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  rpyPose.pose.orientation.x = 0;
  rpyPose.pose.orientation.y = 0;
  rpyPose.pose.orientation.z = yaw;
  rpyPose.pose.orientation.w = 0;
  move_base_goal_pub.publish(rpyPose);
  pub.publish(rpyPose);
}
 
void handle_pose_init_norviz(float_t x, float_t y, float_t yaw) {
  geometry_msgs::PoseStamped rpyPose;
  ROS_INFO("INITIALIZING STARTING POSE %f %f %f", x, y, yaw); 
  rpyPose.header.frame_id = "map";
  rpyPose.header.stamp = ros::Time::now();
  rpyPose.pose.position.x = x;
  rpyPose.pose.position.y = y;
  rpyPose.pose.position.z = 0;
  tf::Quaternion q(0, 0, 0, 0);
  tf::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  rpyPose.pose.orientation.x = 0;
  rpyPose.pose.orientation.y = 0;
  rpyPose.pose.orientation.z = yaw;
  rpyPose.pose.orientation.w = 0;
  pub2.publish(rpyPose);
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_click_to_2d");
  ros::NodeHandle node;
  pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 0);
  pub2 = node.advertise<geometry_msgs::PoseStamped>("initial_2d", 0);
  move_base_goal_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 0);
  ros::Subscriber sub = node.subscribe("move_base_simple/goal", 0, handle_goal);
  ros::Subscriber sub2 = node.subscribe("initialpose", 0, handle_initial_pose);
  ros::Rate loop_rate(10);
  int cnt = 0;  
  while (ros::ok()) {
	if (cnt == 300) {

		handle_pose_init_norviz(-0.05025903135538101, -0.035147152841091156, 0.1114344522356987); 	
	}	
	if (cnt == 400) {

		handle_pose_goal_norviz(-0.9060239195823669, 0.55168217420578, 3.017787456512451); 	
	}
        ros::spinOnce();
        loop_rate.sleep();
	cnt++; 
  }
  return 0;
}
