// src/UARTComm.h
#ifndef UARTCOMM_H
#define UARTCOMM_H

#include <Arduino.h>
#include <HardwareSerial.h>

class UARTComm {
private:
    HardwareSerial &serial;  // Ссылка на UART
    char buffer[256];  // Буфер для команд
    int bufferIndex;

public:
    UARTComm(HardwareSerial &ser);  // Конструктор

    void sendData(float temp1, float temp2, float temp3, 
        float voltage, float current, float pwm, 
        float vibrationLevel, int errorCode);  // Отправка форматированных данных

    bool receiveForecast(float& predTEMP1, float& predTEMP2, float& predTEMP3, 
                              float& predVOLT, float& predCURR, float& predPWM, float& predVib);

};
#endif