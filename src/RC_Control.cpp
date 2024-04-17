#include <RC_Control.hpp>
BluetoothSerial SerialBT;
bool is_bluetooth_connected;

void bluetooth_init()
{
    SerialBT.begin("ESP32s");
}
char rc_input = ' ';

// void RC_Control(float &linear_velocity, float &angular_velocity, int &mechanism_height)
void RC_Control(float &linear_velocity, float &angular_velocity)
{
    is_bluetooth_connected = SerialBT.connected();
    if (!is_bluetooth_connected)
    {
        rc_input = 's';
    }

    if (SerialBT.available())
    {
        rc_input = SerialBT.read();
    }
    switch (rc_input)
    {
    case 's':
        linear_velocity = 0.0;
        angular_velocity = 0.0;
        break;

    case 'f':
        // linear_velocity = linear_velocity == -cmd_velocity_limit ? 0.0 : linear_velocity == 0.0 ? cmd_velocity_limit: velocity + 0.01;
        linear_velocity += 0.05;
        break;

    case 'b':
        // linear_velocity = linear_velocity == cmd_velocity_limit ? 0.0 : linear_velocity == 0.0 ? -cmd_velocity_limit: velocity - 0.01;
        linear_velocity -= 0.05;
        break;

    case 'l':
        angular_velocity += 0.1;
        break;

    case 'r':
        angular_velocity -= 0.1;
        break;

    // case 'u':
    //     mechanism_height = 90;
    //     break;

    // case 'd':
    //     mechanism_height = 0;
    //     break;

    }
    // return rc_input;
}
