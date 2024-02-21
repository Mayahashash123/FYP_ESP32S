#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
// // #include <ROS_Node.hpp>

void setup()
{
  Serial.begin(115200);

  // bluetooth_init();
  Odrive_init();
}

void loop()
{

  Serial_Motor_Test();

  //   // RC_data();

  //   // ROS_Update();

  //
}
