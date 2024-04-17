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
std::pair<float, float> motor_speed;                                 // right, left
float actual_x, actual_y, orientation;
const float tyre_circumference = 2 * 3.14159 * WheelRadius;
float feed_forward;

void Odrive_init()
{
    // Serial.begin(9600);
    front_odrive_serial.begin(BAUDRATE);
    back_odrive_serial.begin(BAUDRATE);
    // Serial.println("odrive initialized");


}


float getvoltage()
{
    front_odrive_serial << "r vbus_voltage\n";
    return front_odrive.readFloat();
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
    // if (linear_velocity==0 && angular_velocity==0)
    //     feed_forward=0.0;
    // else
    //     feed_forward=1.5;
    feed_forward = (linear_velocity == 0 && angular_velocity == 0) ? 0.0 : 0.5;

    right_velocity = (linear_velocity + angular_velocity * RobotBase / 2.0) / tyre_circumference;
    left_velocity = (linear_velocity - angular_velocity * RobotBase / 2.0) / tyre_circumference;

    back_odrive.SetVelocity(BackLeftMotor, -left_velocity, -feed_forward);
    back_odrive.SetVelocity(BackRightMotor, right_velocity, feed_forward);
    front_odrive.SetVelocity(FrontLeftMotor, -left_velocity, -feed_forward);
    front_odrive.SetVelocity(FrontRightMotor, right_velocity, feed_forward);
}

void getPosition(bool debug = false)
{
    // front_right_position = front_odrive.GetPosition(FrontRightMotor);
    // front_left_position = -1 * front_odrive.GetPosition(FrontLeftMotor);

    back_right_position = back_odrive.GetPosition(BackRightMotor);
    back_left_position = -1 * back_odrive.GetPosition(BackLeftMotor);

    previous_average_position = average_position;

    // average_position.first = (front_right_position + back_right_position) / 2.0;
    // average_position.second = (front_left_position + back_left_position) / 2.0;

    average_position.first = back_right_position;
    average_position.second = back_left_position;

    // motor_speed.first = back_odrive.GetVelocity(BackRightMotor);
    // motor_speed.second = -1 * back_odrive.GetVelocity(BackLeftMotor);

    if (debug)
    {
        // Serial.println("front right: " + String(front_right_position));
        // Serial.println("front left: " + String(front_left_position));
        Serial.println("back right: " + String(back_right_position));
        Serial.println("back left: " + String(back_left_position));
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

std::pair<geometry_msgs::Pose, geometry_msgs::Twist> getOdom()
{
    getPosition(false);
    // nav_msgs::Odometry odom;
    geometry_msgs::Pose _pose;
    geometry_msgs::Twist _twist;
    // calculate velocity
    float linear_x = tyre_circumference * (motor_speed.second + motor_speed.first) / (2.0);
    float angular_z = tyre_circumference * (motor_speed.first - motor_speed.second) / (RobotBase);

    // position in meters
    float right_position_in_meter = average_position.first * tyre_circumference;
    float left_position_in_meter = average_position.second * tyre_circumference;

    float previous_right_position_in_meter = previous_average_position.first * tyre_circumference;
    float previous_left_position_in_meter = previous_average_position.second * tyre_circumference;

    // Distance travelled
    float right_distance_traveled = right_position_in_meter - previous_right_position_in_meter;
    float left_distance_traveled = left_position_in_meter - previous_left_position_in_meter;

    // average distance and angle
    float distance = (left_distance_traveled + right_distance_traveled) / 2.0;
    float delta_theta = (right_distance_traveled - left_distance_traveled) / RobotBase;

    // print_on_terminal("distance: ",distance);
    // print_on_terminal("delta_theta",delta_theta);

    // get distance in x and y (projection)
    float xd = cos(delta_theta) * distance;
    float yd = -sin(delta_theta) * distance;

    orientation += delta_theta;

    actual_x += cos(orientation) * xd - sin(orientation) * yd;
    actual_y += sin(orientation) * xd + cos(orientation) * yd;

    // print_on_terminal("actual_x",actual_x);
    // print_on_terminal("actual_y",actual_y);
    // print_on_terminal("\n---------------------",0);

    if (orientation > 2 * 3.14159)
        orientation = 0;

    _pose.position.x = actual_x;
    _pose.position.y = actual_y;

    float q[4];
    euler_to_quat(0, 0, orientation, q);

    _pose.orientation.z = q[2];
    _pose.orientation.w = q[3];

    _twist.linear.x = linear_x;
    _twist.angular.z = angular_z;

    return {_pose, _twist};
}

// std::pair<geometry_msgs::Pose, geometry_msgs::Twist> getOdom()
// {
//     getPosition();
//     // nav_msgs::Odometry odom;
//     geometry_msgs::Pose _pose;
//     geometry_msgs::Twist _twist;

//     float linear_x = tyre_circumference * (motor_speed.second + motor_speed.first) / (2.0);
//     float angular_z = tyre_circumference * (motor_speed.first - motor_speed.second) / (WheelBase);

//     // position in meters
//     float right_position_in_meter = average_position.first * tyre_circumference;
//     float left_position_in_meter = average_position.second * tyre_circumference;

//     float previous_right_position_in_meter = previous_average_position.first * tyre_circumference;
//     float previous_left_position_in_meter = previous_average_position.second * tyre_circumference;

//     // Distance travelled
//     float right_distance_traveled = right_position_in_meter - previous_right_position_in_meter;
//     float left_distance_traveled = left_position_in_meter - previous_left_position_in_meter;

//     // average distance and angle
//     float distance = (left_distance_traveled + right_distance_traveled) / 2.0;
//     float theta = (right_distance_traveled - left_distance_traveled) / WheelBase;

//     // get distance in x and y (projection)
//     float xd = cos(theta) * distance;
//     float yd = -sin(theta) * distance;

//     orientation = (orientation + theta);

//     actual_x += cos(orientation) * xd - sin(orientation) * yd;
//     actual_y += sin(orientation) * xd + cos(orientation) * yd;

//     if (orientation > 2 * 3.14159)
//         orientation = 0;

//     _pose.position.x = actual_x;
//     _pose.position.y = actual_y;

//     float q[4];
//     euler_to_quat(0, 0, orientation, q);

//     _pose.orientation.z = q[2];
//     _pose.orientation.w = q[3];

//     _twist.linear.x = linear_x;
//     _twist.angular.z = angular_z;

//     return {_pose, _twist};
// }