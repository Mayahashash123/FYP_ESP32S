#include <RC_Control.hpp>
BluetoothSerial SerialBT;

void bluetooth_init(){
    SerialBT.begin("ESP32s");
}

void RC_data(){
    if (Serial.available()) {
        SerialBT.write(Serial.read());
    }
    if (SerialBT.available()) {
        Serial.write(SerialBT.read());
    }
}