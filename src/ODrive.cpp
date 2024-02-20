#include <ODrive.hpp>

// HardwareSerial &back_odrive_serial = Serial1;
// HardwareSerial &front_odrive_serial = Serial1;

ODriveArduino back_odrive(Serial2);
// ODriveArduino front_odrive(front_odrive_serial);

void Odrive_init()
{
    // front_odrive_serial.begin(BAUDRATE, SERIAL_8N1, ESP32_UART0_PIN_TX, ESP32_UART0_PIN_RX);
    Serial1.begin(BAUDRATE, SERIAL_8N1, ESP32_UART2_PIN_TX, ESP32_UART2_PIN_RX);
}

void testMotor()
{
    float velocity = 0.2;
    
    // if (Serial.available())
    // {

    //     if (Serial.read() == 'f' && velocity < 0.2){
    //         velocity = velocity + 0.;
    //         Serial.println("f");
    //     }
    //     else if (Serial.read()== 'b' && velocity > -0.2){
    //         velocity = velocity - 0.01;
    //         Serial.println("b");
    //     }

    // }
    Serial.println(velocity);
    back_odrive.SetVelocity(BackLeftMotor, velocity);
    back_odrive.SetVelocity(BackRightMotor, velocity);

}

void driveMotors(float linear_velocity, float angular_velocity)
{

    right_velocity = (linear_velocity + angular_velocity * RobotBase / 2.0) / WheelRadius;
    left_velocity = (linear_velocity - angular_velocity * RobotBase / 2.0) / WheelRadius;

    back_odrive.SetVelocity(BackRightMotor, right_velocity);
    back_odrive.SetVelocity(FrontRightMotor, right_velocity);

    back_odrive.SetVelocity(BackLeftMotor, left_velocity);
    back_odrive.SetVelocity(FrontLeftMotor, left_velocity);
}