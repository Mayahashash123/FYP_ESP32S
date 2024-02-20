#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string.h>

void Rosserial_init();
void ROS_Update();
void Get_cmd_vel();
void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg);

#endif