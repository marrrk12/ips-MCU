#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"
#include "SystemLogic.h"
#include "Timer.h"

// Создаем таймеры для разных задач
#define PWM_OUT_PIN PA11    // ← Выход на ESC
#define PWM_IN_PIN  PA8     // ← Вход от полётника
#define LED_PIN PC13

// #include "stm32f1xx_hal.h"         
// IWDG_HandleTypeDef hiwdg;

Sensors sensors(PB0, PA1, PA7);  // Пин для 1-Wire, напряжение, ток
// HardwareSerial Serial1(PA10, PA9); // Uart to RPI
UARTComm uart(Serial);  // Используем существующий Serial1
MotorControl motor(PA11, PA8, 80.0f); // PWM
SystemLogic systemLogic(sensors, motor, uart, BATTERY_TYPE, PB8);  // Логика системы
// Timer mainLoopTimer(500, true);      // Основной цикл - 500мс
// Timer uartSendTimer(100, true);      // Отправка данных - 100мс

// Глобальные переменные — только для ШИМ
volatile uint16_t output_pwm_us = 1100;     // ← Это и есть выход на ESC его можно корректировать, 
                                            //   на испытания поставил низкие 1100, а так можно и 1500 воткнуть 
volatile bool pwm_update_needed = false;

// === Прерывание таймера — строго каждые 20 мс (50 Гц) ===
void pwmTimerISR() {
    if (pwm_update_needed) {
        analogWrite(PWM_OUT_PIN, output_pwm_us);   // ← Только здесь!
        pwm_update_needed = false;
    }
}


void setup() {
    Serial.println("Start");
    pinMode(LED_PIN, OUTPUT); // Инициализация светодиода
    digitalWrite(LED_PIN, HIGH); // Выключен (инвертированная логика на PC13)

    
    Serial.begin(115200);

    // === ВЫХОД ШИМ: PA11, 50 Гц, 1000–2000 мкс ===
    pinMode(PWM_OUT_PIN, OUTPUT);
    // Настраиваем таймер на 50 Гц (период 20 мс)
    // Для этого используем Timer 1 (PA11 = TIM1_CH4)
    HardwareTimer *ht = new HardwareTimer(TIM1);
    ht->setOverflow(20000, MICROSEC_FORMAT);         // 20 000 мкс = 20 мс
    ht->attachInterrupt(pwmTimerISR);
    ht->resume();

    // Теперь analogWrite(PA11, 1000..2000) = 1000–2000 мкс!
    analogWrite(PWM_OUT_PIN, 1100);

    // pinMode(PB6, INPUT_PULLUP);
    // pinMode(PB7, INPUT_PULLUP);
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
     
    systemLogic.update();
    
    uint16_t desired = systemLogic.getDesiredPWM();
    if (desired != output_pwm_us) {
        noInterrupts();
        output_pwm_us = constrain(desired, 1000, 2000);
        pwm_update_needed = true;
        interrupts();
    }
    
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "c") {
            Serial.println("STARTING CALIBRATION...");
            sensors.calibrateMPU();
            Serial.println("CALIBRATION DONE & SAVED");
        }
    }

          // Обновление логики
        // HAL_IWDG_Refresh(&hiwdg);
        // delay(10);  // Задержка для стабильности
        // Serial.println("LOOP END");
    
}