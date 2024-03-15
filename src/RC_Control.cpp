#include <RC_Control.hpp>
BluetoothSerial SerialBT;

void bluetooth_init()
{
    SerialBT.begin("ESP32s");
}

char Read_RC_Input()
{
    char rc_input= 'e';
    if (SerialBT.available())
    {
        rc_input = SerialBT.read();
    }
    return rc_input;
}
