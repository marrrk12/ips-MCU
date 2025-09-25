#include "Sensors.h"
#include "UARTComm.h"

Sensors sensors(PA1);  // Пин для 1-Wire
HardwareSerial Serial1(PA10, PA9);
UARTComm uart(Serial1);  // Используем существующий Serial1

void setup() {
    Serial1.begin(115200);  // Инициализация UART
    if (!sensors.init()) {
        while (1) delay(10);  // Остановка при ошибке
    }
}

void loop() {
    float ax, ay, az, gx, gy, gz;
    float temp1, temp2, temp3;
    sensors.readMPU(ax, ay, az, gx, gy, gz);
    sensors.readDS(temp1, temp2, temp3);

    uart.sendData(ax, ay, az, gx, gy, gz, temp1, temp2, temp3);

    delay(500);
}