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

void loop()
{

  if (!is_ros_connected && !is_bluetooth_connected)
  {
    linear = angular = 0;
  }

  else // ros is connected
  {
    // drive motors from laptop
    linear = get_linear_velocity();
    angular = get_angular_velocity();

    if (millis() - previous_millis > time_interval)
    {
      publish_odom(getOdom());
      previous_millis = millis();
    }
  }

  RC_Control(linear, angular);

  driveMotors(linear, angular);

  // ros_spin
  ROS_Update();
}

