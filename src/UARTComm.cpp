// src/UARTComm.cpp
#include "UARTComm.h"

UARTComm::UARTComm(HardwareSerial &ser) : serial(ser) {}

void UARTComm::sendData(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float temp1, float temp2, float temp3) {
    serial.print("AX:"); serial.print(ax); serial.print(";");
    serial.print("AY:"); serial.print(ay); serial.print(";");
    serial.print("AZ:"); serial.print(az); serial.print(";");
    serial.print("GX:"); serial.print(gx); serial.print(";");
    serial.print("GY:"); serial.print(gy); serial.print(";");
    serial.print("GZ:"); serial.print(gz); serial.print(";");
    serial.print("TEMP1:"); serial.print(temp1); serial.print(";");
    serial.print("TEMP2:"); serial.print(temp2); serial.print(";");
    serial.print("TEMP3:"); serial.print(temp3); serial.println(";");
}