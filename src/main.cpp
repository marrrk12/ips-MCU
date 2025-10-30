#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"
#include "SystemLogic.h"

#define LED_PIN PC13

#include "stm32f1xx_hal.h"         
IWDG_HandleTypeDef hiwdg;

Sensors sensors(PA1, PB0, PA7);  // Пин для 1-Wire, напряжение, ток
HardwareSerial Serial1(PA10, PA9); // Uart to RPI
UARTComm uart(Serial1);  // Используем существующий Serial1
MotorControl motorControl(PA8, 100.0f); // PWM
SystemLogic systemLogic(sensors, motorControl, uart, BATTERY_TYPE);  // Логика системы


void setup() {
    pinMode(LED_PIN, OUTPUT); // Инициализация светодиода
    digitalWrite(LED_PIN, HIGH); // Выключен (инвертированная логика на PC13)
    pinMode(PB8, INPUT); // Вход для ШИМ от полётного контроллера
    Serial1.begin(115200);

    // IWDG: 5 секунд
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;   // 40kHz / 32 = 1.25 kHz
    hiwdg.Init.Reload    = 12500;                // 12500 / 1.25 kHz = 10 сек
    HAL_IWDG_Init(&hiwdg);

    // Тест UART: "живой"
    Serial1.println("IPS-MCU START");

    if (!sensors.init()) {
        Serial1.println("INIT FAILED");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }

    // КАЛИБРОВКА ПО КОМАНДЕ (опционально)
    sensors.calibrateMPU();
}

void loop() {
    // HAL_IWDG_Refresh(&hiwdg);
    systemLogic.update();  // Обновление логики
    // HAL_IWDG_Refresh(&hiwdg);
    delay(500);  // Задержка для стабильности
}