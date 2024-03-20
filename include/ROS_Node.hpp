#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string.h>

extern bool is_ros_connected;

void Rosserial_init();
void ROS_Update();
float get_linear_velocity();
float get_angular_velocity();
void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg);
// void publish_odom(const nav_msgs::Odometry odom);
void publish_odom();
void print_on_terminal(String msg, float value);



// void publish_odom(const nav_msgs::Odometry odom);




#endif