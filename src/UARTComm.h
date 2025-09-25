// src/UARTComm.h
#ifndef UARTCOMM_H
#define UARTCOMM_H

#include <Arduino.h>
#include <HardwareSerial.h>

class UARTComm {
private:
    HardwareSerial &serial;  // Ссылка на UART

public:
    UARTComm(HardwareSerial &ser);  // Конструктор
    void sendData(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float temp1, float temp2, float temp3);  // Отправка форматированных данных
};

#endif