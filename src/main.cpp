#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"
#include "SystemLogic.h"
#include "Timer.h"

// Создаем таймеры для разных задач

#define LED_PIN PC13

// #include "stm32f1xx_hal.h"         
// IWDG_HandleTypeDef hiwdg;

Sensors sensors(PA1, PB0, PA7);  // Пин для 1-Wire, напряжение, ток
HardwareSerial Serial1(PA10, PA9); // Uart to RPI
UARTComm uart(Serial1);  // Используем существующий Serial1
MotorControl motorControl(PA8, 100.0f); // PWM
SystemLogic systemLogic(sensors, motorControl, uart, BATTERY_TYPE);  // Логика системы
Timer mainLoopTimer(500, true);      // Основной цикл - 500мс
Timer uartSendTimer(100, true);      // Отправка данных - 100мс



void setup() {
    pinMode(LED_PIN, OUTPUT); // Инициализация светодиода
    digitalWrite(LED_PIN, HIGH); // Выключен (инвертированная логика на PC13)
    pinMode(PB8, INPUT); // Вход для ШИМ от полётного контроллера
    // pinMode(PB6, INPUT_PULLUP);
    // pinMode(PB7, INPUT_PULLUP);
    Serial1.begin(115200);

    // // IWDG: 5 секунд
    // hiwdg.Instance = IWDG;
    // hiwdg.Init.Prescaler = IWDG_PRESCALER_32;   // 40kHz / 32 = 1.25 kHz
    // hiwdg.Init.Reload    = 25000;                // 12500 / 1.25 kHz = 10 сек
    // HAL_IWDG_Init(&hiwdg);

    // Тест UART: "живой"
    Serial1.println("IPS-MCU START");

    Serial1.println("I2C SCAN...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial1.print("Found I2C: 0x");
            if (addr < 16) Serial1.print("0");
            Serial1.println(addr, HEX);
        }
        // HAL_IWDG_Refresh(&hiwdg);
    }
    Serial1.println("SCAN DONE");

    if (!sensors.init()) {
        Serial1.println("INIT FAILED");
        while (1) {
            if (uartSendTimer.update()){

                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                // delay(100);
                // HAL_IWDG_Refresh(&hiwdg);
                
            }
        }
    }
    Serial1.println("SYSTEM READY");

}

void loop() {
     if (mainLoopTimer.update()) {
        Serial1.println("LOOP START");
        
        if (Serial1.available()) {
            String cmd = Serial1.readStringUntil('\n');
            cmd.trim();
            if (cmd == "c") {
                Serial1.println("STARTING CALIBRATION...");
                sensors.calibrateMPU();
                Serial1.println("CALIBRATION DONE & SAVED");
            }
        }

        systemLogic.update();  // Обновление логики
        // HAL_IWDG_Refresh(&hiwdg);
        // delay(500);  // Задержка для стабильности
        Serial1.println("LOOP END");
    }
}