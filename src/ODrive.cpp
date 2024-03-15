#include <ODrive.hpp>
#include <ROS_Node.hpp>

HardwareSerial &back_odrive_serial = Serial2;
SoftwareSerial front_odrive_serial(SoftwareSerial_PIN_RX, SoftwareSerial_PIN_TX); // RX, TX

ODriveArduino back_odrive(back_odrive_serial);
ODriveArduino front_odrive(front_odrive_serial);

float linear_velocity, angular_velocity, right_velocity, left_velocity;
float velocity, velocity_limit = 1.0, cmd_velocity_limit = 0.1;
float front_right_position, front_left_position, back_right_position, back_left_position, average_right_position, average_left_position;
std::pair<float, float> previous_average_position, average_position; // right, left
float actual_x, actual_y, orientation;
const float tyre_circumference = 2 * 3.14159 * WheelRadius;

void Odrive_init()
{
    Serial.begin(9600);
    front_odrive_serial.begin(BAUDRATE);
    back_odrive_serial.begin(BAUDRATE);
    // Serial.println("odrive initialized");
}

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
        velocity = velocity == velocity_limit ? 0.0 : velocity == 0.0 ? -velocity_limit
                                                                      : velocity - 0.1;
        break;
    }

    back_odrive.SetVelocity(BackLeftMotor, -velocity);
    back_odrive.SetVelocity(BackRightMotor, velocity);
    front_odrive.SetVelocity(FrontLeftMotor, -velocity);
    front_odrive.SetVelocity(FrontRightMotor, velocity);
}

void driveMotors(float linear_velocity, float angular_velocity)
{
    right_velocity = (linear_velocity + angular_velocity * RobotBase / 2.0) / WheelRadius;
    left_velocity = (linear_velocity - angular_velocity * RobotBase / 2.0) / WheelRadius;

    back_odrive.SetVelocity(BackLeftMotor, -left_velocity);
    back_odrive.SetVelocity(BackRightMotor, right_velocity);
    front_odrive.SetVelocity(FrontLeftMotor, -left_velocity);
    front_odrive.SetVelocity(FrontRightMotor, right_velocity);
}

void driveMotors_RC(char input)
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

    back_odrive.SetVelocity(BackLeftMotor, -left_velocity);
    back_odrive.SetVelocity(BackRightMotor, right_velocity);
    front_odrive.SetVelocity(FrontLeftMotor, -left_velocity);
    front_odrive.SetVelocity(FrontRightMotor, right_velocity);
}

void getPosition(bool debug = false)
{
    front_right_position = front_odrive.GetPosition(FrontRightMotor);
    front_left_position = front_odrive.GetPosition(FrontLeftMotor);

    back_right_position = back_odrive.GetPosition(BackRightMotor);
    back_left_position = back_odrive.GetPosition(BackLeftMotor);

    previous_average_position = average_position;

    average_position.first = (front_right_position + back_right_position) / 2.0;
    average_position.second = (front_left_position + back_left_position) / 2.0;

    if (debug)
    {
        Serial.println("front right: " + String(front_right_position));
        Serial.println("front left: " + String(front_left_position));
        Serial.println("-------");
    }
}

void euler_to_quat(float x, float y, float z, float *q)
{
    q[0] = sin(x / 2) * cos(y / 2) * cos(z / 2) - cos(x / 2) * sin(y / 2) * sin(z / 2);
    q[1] = cos(x / 2) * sin(y / 2) * cos(z / 2) + sin(x / 2) * cos(y / 2) * sin(z / 2);
    q[2] = cos(x / 2) * cos(y / 2) * sin(z / 2) - sin(x / 2) * sin(y / 2) * cos(z / 2);
    q[3] = cos(x / 2) * cos(y / 2) * cos(z / 2) + sin(x / 2) * sin(y / 2) * sin(z / 2);
}

nav_msgs::Odometry getOdom()
{
    nav_msgs::Odometry odom;

    // position in meters
    float right_position_in_meter = average_position.first * tyre_circumference;
    float left_position_in_meter = average_position.second * tyre_circumference;

    float previous_right_position_in_meter = previous_average_position.first * tyre_circumference;
    float previous_left_position_in_meter = previous_average_position.second * tyre_circumference;

    // Distance travelled
    float left_distance_traveled = right_position_in_meter - previous_right_position_in_meter;
    float right_distance_traveled = left_position_in_meter - previous_left_position_in_meter;

    // average distance and angle
    float distance = (left_distance_traveled + right_distance_traveled) / 2.0;
    float theta = (right_distance_traveled - left_distance_traveled) / RobotBase;

    // get distance in x and y (projection)
    float xd = cos(theta) * distance;
    float yd = -sin(theta) * distance;

    actual_x += cos(theta) * xd - sin(theta) * yd;
    actual_y += sin(theta) * xd + cos(theta) * yd;
    orientation = (orientation + theta);

    if (orientation > 2 * 3.14159)
        orientation = 0;

    odom.pose.pose.position.x = actual_x;
    odom.pose.pose.position.y = actual_y;

    float q[4];
    euler_to_quat(0, 0, orientation, q);

    odom.pose.pose.orientation.z = q[2];
    odom.pose.pose.orientation.w = q[3];

    return odom;
}