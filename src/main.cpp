#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"
#include "SystemLogic.h"
#include "Timer.h"

// Создаем таймеры для разных задач

#define LED_PIN PC13

// #include "stm32f1xx_hal.h"         
// IWDG_HandleTypeDef hiwdg;

Sensors sensors(PB0, PA1, PA7);  // Пин для 1-Wire, напряжение, ток
// HardwareSerial Serial1(PA10, PA9); // Uart to RPI
UARTComm uart(Serial);  // Используем существующий Serial1
MotorControl motor(PA8, PB8, 80.0f); // PWM
SystemLogic systemLogic(sensors, motor, uart, BATTERY_TYPE, PB8);  // Логика системы
// Timer mainLoopTimer(500, true);      // Основной цикл - 500мс
// Timer uartSendTimer(100, true);      // Отправка данных - 100мс



void setup() {
    Serial.println("Start");
    pinMode(LED_PIN, OUTPUT); // Инициализация светодиода
    digitalWrite(LED_PIN, HIGH); // Выключен (инвертированная логика на PC13)

    // pinMode(PB6, INPUT_PULLUP);
    // pinMode(PB7, INPUT_PULLUP);
    Serial.begin(115200);

    // // IWDG: 5 секунд
    // hiwdg.Instance = IWDG;
    // hiwdg.Init.Prescaler = IWDG_PRESCALER_32;   // 40kHz / 32 = 1.25 kHz
    // hiwdg.Init.Reload    = 25000;                // 12500 / 1.25 kHz = 10 сек
    // HAL_IWDG_Init(&hiwdg);

    // Тест UART: "живой"
    // Serial.println("IPS-MCU START");

    // Serial.println("I2C SCAN...");
    // for (uint8_t addr = 1; addr < 127; addr++) {
    //     Wire.beginTransmission(addr);
    //     if (Wire.endTransmission() == 0) {
    //         Serial.print("Found I2C: 0x");
    //         if (addr < 16) Serial.print("0");
    //         Serial.println(addr, HEX);
    //     }
    //     // HAL_IWDG_Refresh(&hiwdg);
    // }
    // Serial.println("SCAN DONE");

    if (!sensors.init()) {
        Serial.println("INIT FAILED");
        // while(1) delay(100);
        // while (1) {
        //     if (uartSendTimer.update()){

        //         digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        //         // delay(100);
        //         // HAL_IWDG_Refresh(&hiwdg);
                
        //     }
        // }
    }
    systemLogic.begin();
    Serial.println("SYSTEM READY");

}

void loop() {
     
        
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd == "c") {
                Serial.println("STARTING CALIBRATION...");
                sensors.calibrateMPU();
                Serial.println("CALIBRATION DONE & SAVED");
            }
        }

        systemLogic.update();  // Обновление логики
        // HAL_IWDG_Refresh(&hiwdg);
        // delay(10);  // Задержка для стабильности
        // Serial.println("LOOP END");
    
}