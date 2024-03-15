#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string.h>

void Rosserial_init();
void ROS_Update();
float get_linear_velocity();
float get_angular_velocity();
void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg);
void print_on_terminal(float msg);
void publish_odom(const nav_msgs::Odometry odom);




#endif