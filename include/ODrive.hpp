#ifndef _ODRIVE_CONTROL_HPP_
#define _ODRIVE_CONTROL_HPP_

#include <ODriveArduino.h>
#include <HardwareSerial.h>

#define ESP32_UART0_PIN_TX 1
#define ESP32_UART0_PIN_RX 3

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
void testMotor();



#endif