// src/UARTComm.cpp
#include "UARTComm.h"

UARTComm::UARTComm(HardwareSerial &ser) : serial(ser), bufferIndex(0) {
    memset(buffer, 0, sizeof(buffer));
}

void UARTComm::sendData(int16_t ax, int16_t ay, int16_t az, 
                        int16_t gx, int16_t gy, int16_t gz, 
                        float temp1, float temp2, float temp3, 
                        float voltage, float current, 
                        float pwm, int errorCode) {
    serial.print("AX:"); serial.print(ax); serial.print(";");
    serial.print("AY:"); serial.print(ay); serial.print(";");
    serial.print("AZ:"); serial.print(az); serial.print(";");
    serial.print("GX:"); serial.print(gx); serial.print(";");
    serial.print("GY:"); serial.print(gy); serial.print(";");
    serial.print("GZ:"); serial.print(gz); serial.print(";");
    serial.print("TEMP1:"); serial.print(temp1); serial.print(";");
    serial.print("TEMP2:"); serial.print(temp2); serial.print(";");
    serial.print("TEMP3:"); serial.print(temp3); serial.println(";");
    serial.print("VOLT:"); serial.print(voltage); serial.print(";");
    serial.print("CURR:"); serial.print(current); serial.println(";");
    serial.print("PWM:"); serial.print(pwm); serial.print(";");
    serial.print("ERR:"); serial.print(errorCode); serial.println(";");
    serial.flush();
}

bool UARTComm::receiveForecast(float& predAX, float& predAY, float& predAZ, 
                              float& predGX, float& predGY, float& predGZ,
                              float& predTEMP1, float& predTEMP2, float& predTEMP3, 
                              float& predVOLT, float& predCURR, float& predPWM) {
    if (bufferIndex >= sizeof(buffer) - 1) {
        bufferIndex = 0;
        memset(buffer, 0, sizeof(buffer));
        return false;  // Сброс и игнор
    }
    while (serial.available() > 0) {
        char c = serial.read();
        if (c == '\n' || c == ';') {  // Конец команды
            if (bufferIndex > 0) {
                buffer[bufferIndex] = '\0';  // Завершающий нуль
                // Ручное копирование вместо strncpy
                char tempBuffer[16];
                int tempIndex = 0;
                char* token = buffer;
                while (*token) {
                    if (*token == ':' && tempIndex < sizeof(tempBuffer) - 1) {
                        tempBuffer[tempIndex] = '\0';
                        token++;  // Пропускаем ':'
                        if (strncmp(tempBuffer, "PRED_AX", 7) == 0) predAX = atof(token);
                        else if (strncmp(tempBuffer, "PRED_AY", 7) == 0) predAY = atof(token);
                        else if (strncmp(tempBuffer, "PRED_AZ", 7) == 0) predAZ = atof(token);
                        else if (strncmp(tempBuffer, "PRED_GX", 7) == 0) predGX = atof(token);
                        else if (strncmp(tempBuffer, "PRED_GY", 7) == 0) predGY = atof(token);
                        else if (strncmp(tempBuffer, "PRED_GZ", 7) == 0) predGZ = atof(token);
                        else if (strncmp(tempBuffer, "PRED_TEMP1", 10) == 0) predTEMP1 = atof(token);
                        else if (strncmp(tempBuffer, "PRED_TEMP2", 10) == 0) predTEMP2 = atof(token);
                        else if (strncmp(tempBuffer, "PRED_TEMP3", 10) == 0) predTEMP3 = atof(token);
                        else if (strncmp(tempBuffer, "PRED_VOLT", 9) == 0) predVOLT = atof(token);
                        else if (strncmp(tempBuffer, "PRED_CURR", 9) == 0) predCURR = atof(token);
                        else if (strncmp(tempBuffer, "PRED_PWM", 8) == 0) predPWM = atof(token);
                        tempIndex = 0;
                    } else if (tempIndex < sizeof(tempBuffer) - 1) {
                        tempBuffer[tempIndex++] = *token;
                    }
                    token++;
                }
                if (predPWM < 1000.0f || predPWM > 2000.0f) {
                    return false;  // Игнор прогноза
                }
                bufferIndex = 0;  // Сброс буфера
                memset(buffer, 0, sizeof(buffer));
                return true;
            }
        } else if (bufferIndex < sizeof(buffer) - 1) {
            buffer[bufferIndex++] = c;  // Добавление символа в буфер
        }
    }
    return false;  // Нет данных
}

