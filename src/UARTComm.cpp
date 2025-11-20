// src/UARTComm.cpp
#include "UARTComm.h"

UARTComm::UARTComm(HardwareSerial &ser) : serial(ser), bufferIndex(0) {
    memset(buffer, 0, sizeof(buffer));
}

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
        crc &= 0xFF;
    }
    return crc;
}

void UARTComm::sendData(int16_t ax, int16_t ay, int16_t az, 
                        int16_t gx, int16_t gy, int16_t gz, 
                        float temp1, float temp2, float temp3, 
                        float voltage, float current, 
                        float pwm, float vibrationLevel, int errorCode) {

    // // ОТЛАДКА: печатаем входные данные
    // serial.print("DEBUG: ax="); serial.print(ax);
    // serial.print(" ay="); serial.print(ay);
    // serial.print(" az="); serial.print(az);
    // serial.print(" gx="); serial.print(gx);
    // serial.print(" gy="); serial.print(gy);
    // serial.print(" gz="); serial.print(gz);
    // serial.print(" temp1="); serial.print(temp1);
    // serial.print(" temp2="); serial.print(temp2);
    // serial.print(" temp3="); serial.print(temp3);
    // serial.print(" voltage="); serial.print(voltage);
    // serial.print(" current="); serial.print(current);
    // serial.print(" pwm="); serial.print(pwm);
    // serial.print(" vib="); serial.print(vibrationLevel);
    // serial.print(" errorCode="); serial.print(errorCode);
    // serial.println();

    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
        // "AX:%.2f;AY:%.2f;AZ:%.2f;"
        // "GX:%.1f;GY:%.1f;GZ:%.1f;"
        "TEMP1:%.1f;TEMP2:%.1f;TEMP3:%.1f;"
        "VOLT:%.2f;CURR:%.1f;PWM:%.0f;VIB:%.2f;ERR:%d",
        // ax / 100.0f, ay / 100.0f, az / 100.0f,  // из int16_t * 100 → float
        // (float)gx, (float)gy, (float)gz,
        temp1, temp2, temp3,
        voltage, current, pwm, vibrationLevel, errorCode
    );

    // Формируем пакет: STX (0x02) + ДАННЫЕ + ETX (0x03) + CRC8
    uint8_t packet[260];
    packet[0] = 0x02;                    // STX
    memcpy(&packet[1], buffer, len);     // Данные
    packet[1 + len] = 0x03;               // ETX
    packet[2 + len] = crc8((uint8_t*)buffer, len); // CRC

    // Отправляем весь пакет
    // serial.println(buffer);
    serial.write(packet, len + 3);
    serial.flush();

    // Частота: 2 Гц → 500 мс
    // delay(500);
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

