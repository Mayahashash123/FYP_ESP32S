#include <ROS_Node.hpp>

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);

float linear_velocity, angular_velocity;

void Rosserial_init()
{
    nh.initNode();
    nh.subscribe(sub);
}

void ROS_Update()
{
    nh.spinOnce();
    delay(1);
}

void Get_cmd_vel(float &linear_x, float & angular_z)
{
    linear_x = linear_velocity;
    angular_z = angular_velocity;
    
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg)
{
    //   nh.loginfo(("x: " + String(msg)).c_str());

    linear_velocity = cmd_msg.linear.x, // m/s
    angular_velocity = cmd_msg.angular.z; // rad/s
}
