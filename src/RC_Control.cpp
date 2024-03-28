#include <RC_Control.hpp>
BluetoothSerial SerialBT;
bool is_bluetooth_connected;

void bluetooth_init()
{
    SerialBT.begin("ESP32s");
}

void 

RC_Control(float &linear_velocity, float &angular_velocity)
{
    is_bluetooth_connected = SerialBT.connected();
    if (!is_bluetooth_connected)
    {
        return;
    }

    char rc_input = 'e';

    if (SerialBT.available())
    {
        rc_input = SerialBT.read();
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
            angular_velocity += 0.07;
            break;

        case 'r':
            angular_velocity -= 0.07;
            break;
        }
    }
    // return rc_input;
}
