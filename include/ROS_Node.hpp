#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <mechanism.hpp>

extern bool is_ros_connected;

void Rosserial_init();
void ROS_Update();
float get_linear_velocity();
float get_angular_velocity();
void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg);
void goal_distance_cb(const std_msgs::Int16 &msg);
void publish_battery_percentage(float battery_percentage);
extern int get_mechanism();
void publish_mechanism_states(int states_num);

// void publish_odom();
void publish_odom(std::pair<geometry_msgs::Pose, geometry_msgs::Twist> odometry);
void print_on_terminal(String msg, float value = 0);
void laser_pointer_cb(const std_msgs::Bool &pointer_msg);

ros::NodeHandle get_nh();

// void publish_odom(const nav_msgs::Odometry odom);

#endif