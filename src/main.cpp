#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
#include <ROS_Node.hpp>

unsigned long previous_millis, time_interval = 50;
float linear = 0, angular = 0;

void setup()
{
  Serial.begin(57600);
  bluetooth_init();
  Odrive_init();
  Rosserial_init();
  pinMode(2, OUTPUT);
}
std::pair<float, float> RC_Cmd(0, 0);
std::pair<float, float> Ros_Cmd;

void loop()
{
  RC_Control(RC_Cmd.first, RC_Cmd.second); // zeros if not RC connected

  if (!is_ros_connected && !is_bluetooth_connected)
  {
    linear = angular = 0;
  }
  else
  {

    if (!is_ros_connected) // RC but no ROS
    {
      linear = RC_Cmd.first;
      angular = RC_Cmd.second;
    }
    else // ROS connected
    {
      Ros_Cmd.first = get_linear_velocity();
      Ros_Cmd.second = get_angular_velocity();

      if (Ros_Cmd.first == 0 && Ros_Cmd.second == 0)
      { // in case bt is not connected RC_Cmd is 0 and in case it is connected it will take the last value of RC_Cmd which it should be for the first time 0
        linear = RC_Cmd.first;
        angular = RC_Cmd.second;
      }
      else
      {
        linear = Ros_Cmd.first;
        angular = Ros_Cmd.second;
      }

      if (millis() - previous_millis > time_interval)
      {
        publish_odom(getOdom());
        previous_millis = millis();
      }
    }
  }

  driveMotors(linear, angular);

  // ros_spin
  ROS_Update();
}
