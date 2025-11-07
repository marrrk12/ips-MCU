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

    void sendData(int16_t ax, int16_t ay, int16_t az, 
        int16_t gx, int16_t gy, int16_t gz, 
        float temp1, float temp2, float temp3, 
        float voltage, float current, float pwm, 
        float vibrationLevel, int errorCode);  // Отправка форматированных данных

    bool receiveForecast(float& predAX, float& predAY, float& predAZ, 
                        float& predGX, float& predGY, float& predGZ,
                         float& predTEMP1, float& predTEMP2, float& predTEMP3, 
                         float& predVOLT, float& predCURR, float& predPWM);

};
#endif