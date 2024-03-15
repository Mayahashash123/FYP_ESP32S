#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
#include <ROS_Node.hpp>

unsigned long previous_millis, time_interval = 100;

void setup()
{
  Serial.begin(9600);
  Serial.println("hi");
  bluetooth_init();
  Odrive_init();
  // Rosserial_init();
}

void loop()
{
  // drive motors using bluetooth
  driveMotors_RC(Read_RC_Input());

  // drive motors from laptop
  //  driveMotors(get_linear_velocity(), get_angular_velocity());

  if (millis() - previous_millis > time_interval)
  {
    publish_odom(getOdom());
    previous_millis = millis();
  }

  // ros_spin
  //  ROS_Update();
}
