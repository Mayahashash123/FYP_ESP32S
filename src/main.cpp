#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
#include <ROS_Node.hpp>

unsigned long previous_millis, time_interval = 50;

void setup()
{
  Serial.begin(57600);
  bluetooth_init();
  Odrive_init();
  Rosserial_init();
  pinMode(2, OUTPUT);
}
/*
 TODO: make state machine for the connection and disconnection of the ros node
 TODO: make software architecture to the code (class based)!!!
*/

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
    driveMotors(get_linear_velocity(), get_angular_velocity());
    
    if (millis() - previous_millis > time_interval)
    {
      publish_odom();
      previous_millis = millis();
    }
  }

  // ros_spin
  ROS_Update();
}

// #include <ros.h>
// #include <std_msgs/String.h>
// #define USE_USBCON

// ros::NodeHandle  nh1;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// char hello[13] = "hello world!";

// void setup()
// {
//   Serial.begin(57600);
//   nh1.initNode();
//   nh1.advertise(chatter);
// }

// void loop()
// {
//   str_msg.data = hello;
//   chatter.publish( &str_msg );
//   nh1.spinOnce();
//   delay(1000);
// }
