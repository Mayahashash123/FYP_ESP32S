#ifndef _ODRIVE_CONTROL_HPP_
#define _ODRIVE_CONTROL_HPP_

#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define SoftwareSerial_PIN_RX 18
#define SoftwareSerial_PIN_TX 19

#define ESP32_UART2_PIN_TX 17                                      
#define ESP32_UART2_PIN_RX 16

#define BAUDRATE 115200

#define WheelRadius 0.0825
#define RobotBase 0.26

#define BackRightMotor 0
#define BackLeftMotor 1
#define FrontRightMotor 1
#define FrontLeftMotor 0

extern float right_velocity;
extern float left_velocity;

void Odrive_init();
void driveMotors(float linear_velocity, float angular_velocity);
void Serial_Motor_Test();


// Printing with stream operator helper functions
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}

#endif