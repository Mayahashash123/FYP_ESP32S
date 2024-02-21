#include <ODrive.hpp>

HardwareSerial &back_odrive_serial = Serial2;
SoftwareSerial front_odrive_serial(SoftwareSerial_PIN_RX, SoftwareSerial_PIN_TX); // RX, TX

ODriveArduino back_odrive(back_odrive_serial);
ODriveArduino front_odrive(front_odrive_serial);

void Odrive_init()
{
    front_odrive_serial.begin(BAUDRATE);
    back_odrive_serial.begin(BAUDRATE);
    Serial.println("odrive initialized");
}

float velocity;

void Serial_Motor_Test()
{
    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
        case 'v':
            back_odrive_serial << "back vbus_voltage\n";
            Serial << "Vbus voltage: " << back_odrive.readFloat() << '\n';

            front_odrive_serial << "front vbus_voltage\n";
            Serial << "Vbus voltage: " << front_odrive.readFloat() << '\n';
            break;

        case 's':
            velocity = 0.0;
            break;

        case 'f':
            velocity = velocity == 0.0 ? 0.5 : velocity + 0.1;
            break;

        case 'r':
            velocity = velocity == 0.0 ? -0.5 : velocity - 0.1;
            break;

        default:
            velocity = 0;
            break;
        }

        Serial.println(velocity);
        back_odrive.SetVelocity(BackLeftMotor, velocity);
        back_odrive.SetVelocity(BackRightMotor, velocity);
        front_odrive.SetVelocity(FrontLeftMotor, velocity);
        front_odrive.SetVelocity(FrontRightMotor, velocity);
    }
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