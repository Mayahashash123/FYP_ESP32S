#include <ODrive.hpp>

HardwareSerial &back_odrive_serial = Serial2;
SoftwareSerial front_odrive_serial(SoftwareSerial_PIN_RX, SoftwareSerial_PIN_TX); // RX, TX

ODriveArduino back_odrive(back_odrive_serial);
ODriveArduino front_odrive(front_odrive_serial);

void Odrive_init()
{
    Serial.begin(BAUDRATE);
    front_odrive_serial.begin(BAUDRATE);
    back_odrive_serial.begin(BAUDRATE);
    Serial.println("odrive initialized");
}

float velocity, linear_velocity, angular_velocity, right_velocity, left_velocity;
float velocity_limit = 1.0, cmd_velocity_limit = 0.1;

void Serial_Motor_Test(char input)
{
    switch (input)
    {
    case 's':
        velocity = 0.0;
        break;

    case 'f':
        velocity = velocity == -velocity_limit ? 0.0 : velocity == 0.0 ? velocity_limit
                                                                       : velocity + 0.1;
        break;

    case 'b':
        velocity = velocity == velocity_limit ? 0.0 : velocity == 0.0 ? -velocity_limit: velocity - 0.1;
        break;
    }

    Serial.println(velocity);
    back_odrive.SetVelocity(BackLeftMotor, -velocity);
    back_odrive.SetVelocity(BackRightMotor, velocity);
    front_odrive.SetVelocity(FrontLeftMotor, -velocity);
    front_odrive.SetVelocity(FrontRightMotor, velocity);
}

// void driveMotors(float linear_velocity, float angular_velocity)
// {

//     right_velocity = (linear_velocity + angular_velocity * RobotBase / 2.0) / WheelRadius;
//     left_velocity = (linear_velocity - angular_velocity * RobotBase / 2.0) / WheelRadius;

//     back_odrive.SetVelocity(BackRightMotor, right_velocity);
//     back_odrive.SetVelocity(FrontRightMotor, right_velocity);

//     back_odrive.SetVelocity(BackLeftMotor, left_velocity);
//     back_odrive.SetVelocity(FrontLeftMotor, left_velocity);
// }

void driveMotors(char input)
{
    switch (input)
    {
    case 's':
        linear_velocity = 0.0;
        angular_velocity = 0.0;
        break;

    case 'f':
        // linear_velocity = linear_velocity == -cmd_velocity_limit ? 0.0 : linear_velocity == 0.0 ? cmd_velocity_limit: velocity + 0.01;
        linear_velocity += 0.02;
        break;

    case 'b':
        // linear_velocity = linear_velocity == cmd_velocity_limit ? 0.0 : linear_velocity == 0.0 ? -cmd_velocity_limit: velocity - 0.01;
        linear_velocity -= 0.02;
        break;

    case 'l':
        angular_velocity += 0.07;
        break;

    case 'r':
        angular_velocity -= 0.07;
        break;
    }

    right_velocity = (linear_velocity + angular_velocity * RobotBase / 2.0) / WheelRadius;
    left_velocity = (linear_velocity - angular_velocity * RobotBase / 2.0) / WheelRadius;

    Serial.println(right_velocity);
    Serial.println(left_velocity);
    back_odrive.SetVelocity(BackLeftMotor, - left_velocity);
    back_odrive.SetVelocity(BackRightMotor, right_velocity);
    front_odrive.SetVelocity(FrontLeftMotor, - left_velocity);
    front_odrive.SetVelocity(FrontRightMotor, right_velocity);
}