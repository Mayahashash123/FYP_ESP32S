#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
#include <ROS_Node.hpp>

unsigned long previous_millis, time_interval = 1;

void setup()
{
  Serial.begin(57600);
  bluetooth_init();
  Odrive_init();
  Rosserial_init();
  pinMode(2, OUTPUT);
}
 //TODO: make state machine for the connection and disconnection of the ros node
//  TODO: make software architecture to the code (class based)!!!


void loop()
{

  // drive motors using bluetooth
  driveMotors_RC(Read_RC_Input());

  if (!is_ros_connected)
  {
    driveMotors(0, 0);
  }
  else
  {
    // drive motors from laptop
    // driveMotors(get_linear_velocity(), get_angular_velocity());
    
    if (millis() - previous_millis > time_interval)
    {
      previous_millis = millis();
    }
      publish_odom();
  }

  // ros_spin
  ROS_Update();
}

// #include <ros.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Twist.h>
// // #include <nav_msgs/Odometry.h>
// // #define USE_USBCON

// ros::NodeHandle  nh1;

// // std_msgs::String str_msg;
// geometry_msgs::Pose Pose_msg;
// geometry_msgs::Twist twist_msg;

// // geometry_msgs::
// // nav_msgs::Odometry odom_msg1;

// // ros::Publisher chatter("chatter", &str_msg);
// ros::Publisher pose_pub("pose",&Pose_msg);
// ros::Publisher twist_pub("twist", &twist_msg);

// // ros::Publisher pose_pub("odom",&odom_msg1);

// void setup()
// {
//   Serial.begin(57600);
//   nh1.initNode();
//   nh1.getHardware()->setBaud(57600);
//   nh1.advertise(pose_pub);
//   nh1.advertise(twist_pub);
// }

// void loop()
// {
//   Pose_msg.position.x = random();
//   Pose_msg.position.y = random();
//   Pose_msg.position.z = 0;
//   Pose_msg.orientation.x = 0;
//   Pose_msg.orientation.y = 0;
//   Pose_msg.orientation.z = random();
//   Pose_msg.orientation.w = random();

//   twist_msg.linear.x = random();
//   twist_msg.angular.z = random();

// // odom_msg1.pose = Pose_msg;



//   pose_pub.publish( &Pose_msg );
//   twist_pub.publish( &twist_msg );
//   nh1.spinOnce();
//   delay(1);
// }
