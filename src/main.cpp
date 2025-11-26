#include "Sensors.h"
#include "UARTComm.h"
#include "SystemLogic.h"
#include <Servo.h>
// #include "MotorControl.h"

#define PWM_IN PA8
#define PWM_OUT PA11

Sensors sensors(PB0, PA1, PA7);         // Пин для 1-Wire, напряжение, ток // HardwareSerial Serial1(PA10, PA9); // Uart to RPI
UARTComm uart(Serial);                  // Используем существующий Serial1
SystemLogic systemLogic(sensors, uart); // Логика системы
Servo esc;

// Глобальные для входного PWM (прерывания CHANGE)
volatile unsigned long pulseInWidth = 1100; // Входной ШИМ
volatile bool newPulse = false;
volatile unsigned long riseTime = 0;

void pulseHandler();

// Глобальный режим (по умолчанию BYPASS - чистый пасsthrough)
enum Mode
{
    MODE_BYPASS,
    MODE_NORMAL
};
Mode currentMode = MODE_NORMAL;

// Delta от логики (volatile для безопасного доступа)
// volatile int lastDelta = 0; // Текущая дельта от логики

void setup()
{
    Serial.println("Start");
    Serial.begin(115200);

    esc.attach(PWM_OUT); // автоматически настраивает таймер на 50 Гц
    esc.writeMicroseconds(1000);
    newPulse = false;

    pinMode(PWM_IN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PWM_IN), pulseHandler, CHANGE);

    systemLogic.begin();

    if (!sensors.init())
    {
        Serial.println("INIT FAILED");
        // while (1) {
        //     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        //     delay(100);
        // }
    }
}

void loop()
{

    static unsigned long lastValidPWM = 1100;
    static unsigned long lastSensorsFast = 0;
    static unsigned long lastTemps = 0;
    static unsigned long lastForecast = 0;
    static unsigned long lastCalculations = 0;
    static unsigned long lastSend = 0;
    static unsigned long lastRamp = 0;
    unsigned long now = millis();

    // 1. Мгновенная трансляция ШИМ (пасsthrough + delta)
    if (newPulse)
    {
        lastValidPWM = pulseInWidth;
        newPulse = false;
    }

    int outputPWM = lastValidPWM;
    if (currentMode == MODE_NORMAL)
    {
        outputPWM += lastDelta; // Применяем дельту
    }
    esc.writeMicroseconds(constrain(outputPWM, 1000, 2300));

    if (now - lastSensorsFast >= 10)
    {
        systemLogic.updateSensorsFast();
        lastSensorsFast = now;
    }

    if (now - lastTemps >= 100)
    {
        systemLogic.updateTemperatures();
        lastTemps = now;
    }

    if (now - lastForecast >= 50)
    {
        systemLogic.updateForecast();
        lastForecast = now;
    }

    if (now - lastCalculations >= 100)
    {
        systemLogic.updateCalculations(lastValidPWM);
        lastCalculations = now;
    }

    if (now - lastSend >= 200)
    {
        systemLogic.sendData(lastValidPWM);
        lastSend = now;
    }

    if (now - lastRamp >= 20)
    {
        systemLogic.rampDelta();
        lastRamp = now;
    }

    // Обработка UART команд (для тестов/переключений)
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "c")
        {
            Serial.println("CALIBRATING MPU...");
            sensors.calibrateMPU();
            Serial.println("CALIBRATION DONE & SAVED");
        }
        else if (cmd == "bypass")
        {
            currentMode = MODE_BYPASS;
            Serial.println("MODE: BYPASS (чистый пасsthrough)");
        }
        else if (cmd == "normal")
        {
            currentMode = MODE_NORMAL;
            Serial.println("MODE: NORMAL (ИИ-коррекция активна)");
        }
        else if (cmd == "calib")
        {
            Serial.println("ESC calib...");
            esc.writeMicroseconds(1000);
            delay(3000);
            esc.writeMicroseconds(2000);
            delay(3000);
            esc.writeMicroseconds(1500);
            delay(2000);
        }
    }

    //   delay(1);  // ~500 Гц цикл
}

void pulseHandler()
{
    if (digitalRead(PWM_IN) == HIGH)
    {
        riseTime = micros();
    }
    else
    {
        unsigned long fall = micros();
        unsigned long width = (fall >= riseTime) ? fall - riseTime : (0xFFFFFFFF - riseTime) + fall;
        if (width >= 800 && width <= 2300)
        {
            pulseInWidth = width;
            newPulse = true;
        }
    }
}