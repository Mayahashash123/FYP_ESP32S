#ifndef RC_CONTROL_HPP
#define RC_CONTROL_HPP

#include "BluetoothSerial.h"

void bluetooth_init();
// char Read_RC_Input();
void RC_Control(float &linear_velocity, float &angular_velocity);
extern bool is_bluetooth_connected;


#endif