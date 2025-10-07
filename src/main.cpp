#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"
#include "SystemLogic.h"

Sensors sensors(PB3, PA0, PA1);  // Пин для 1-Wire
HardwareSerial Serial1(PA10, PA9);
UARTComm uart(Serial1);  // Используем существующий Serial1
MotorControl motorControl(PA8, 100.0f);
SystemLogic systemLogic(sensors, motorControl, uart);  // Логика системы

void setup() {
    Serial1.begin(115200);  // Инициализация UART
    if (!sensors.init()) {
        while (1) delay(10);  // Остановка при ошибке
    }
}

void loop() {
    systemLogic.update();  // Обновление логики
    delay(500);  // Задержка для стабильности
}