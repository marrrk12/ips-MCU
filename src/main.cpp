#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"
#include "SystemLogic.h"

#define LED_PIN PC13

Sensors sensors(PA1, PB0, PA7);  // Пин для 1-Wire, напряжение, ток
HardwareSerial Serial1(PA10, PA9); // Uart to RPI
UARTComm uart(Serial1);  // Используем существующий Serial1
MotorControl motorControl(PA8, 100.0f); // PWM
SystemLogic systemLogic(sensors, motorControl, uart, BATTERY_TYPE);  // Логика системы


void setup() {
    pinMode(LED_PIN, OUTPUT); // Инициализация светодиода
    digitalWrite(LED_PIN, HIGH); // Выключен (инвертированная логика на PC13)
    Serial1.begin(115200);
    if (!sensors.init()) {
        while (1) {
            // Индикация ошибки инициализации: 3 быстрых моргания
            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_PIN, LOW); // Вкл
                delay(200);
                digitalWrite(LED_PIN, HIGH); // Выкл
                delay(200);
            }
            delay(1000);
        }
    }
}

void loop() {
    systemLogic.update();  // Обновление логики
    delay(500);  // Задержка для стабильности
}