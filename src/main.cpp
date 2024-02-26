#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
// // #include <ROS_Node.hpp>

void setup()
{
  Serial.begin(9600);
  Serial.print("hi");
  bluetooth_init();
  Odrive_init();
}

void loop()
{

  // Serial_Motor_Test(Read_RC_Input());
  driveMotors(Read_RC_Input());

  //   // ROS_Update();

  //
}
