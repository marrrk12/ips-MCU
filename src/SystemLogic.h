// src/SystemLogic.h
#ifndef SYSTEMLOGIC_H
#define SYSTEMLOGIC_H

#include "Sensors.h"
#include "UARTComm.h"
#include "MotorControl.h"

class SystemLogic {
private:
    Sensors& sensors;
    MotorControl& motor;
    UARTComm& uart;
    float maxTemp = 50.0f;       // Максимальная температура (°C)
    float maxVibration = 15.0f;  // Максимальная вибрация (m/s²)
    float minVoltage;            // Динамический порог напряжения
    float maxCurrent;            // Максимальный ток из MotorControl
    float predVoltThreshold = 0.9f;  // Порог прогноза напряжения (90% от minVoltage)
    float predCurrThreshold = 0.9f;  // Порог прогноза тока (90% от maxCurrent)
    float predPWMThreshold = 2000.0f; // Максимальный прогноз ШИМ (μs)
    float pwmAdjustStep = 10.0f;     // Шаг корректировки ШИМ (μs)
    const int LED_PIN = PC13;
    unsigned long lastLedUpdate = 0;
    int ledBlinkCount = 0; // Счётчик морганий
    int ledTargetBlinks = 0; // Сколько раз моргнуть
    bool ledState = false; // Текущее состояние (вкл/выкл)
    unsigned long ledNextToggle = 0; // Время следующего переключения
    void updateLED(int errorCode, bool uartOk);
    void ledBlinkPattern(int blinks, int onTime, int offTime, int cycleTime);

public:
    SystemLogic(Sensors& sens, MotorControl& mot, UARTComm& u, int batteryType);
    void update();  // Обновление логики
    // Коды ошибок (зарезервированные значения для sendData)
    static const int ERROR_NONE = -1;           // Нормальная работа
    static const int ERROR_OVERHEAT = -2;       // Перегрев >50°C
    static const int ERROR_HIGH_VIBRATION = -3; // Вибрация >15 m/s²
    static const int ERROR_LOW_VOLTAGE = -4;    // Напряжение <В
    static const int ERROR_OVERCURRENT = -5;    // Ток >100A
    static const int ERROR_INIT_FAIL = -6;      // Ошибка инициализации
    static const int ERROR_CRITICAL = -999;     // Критическая ошибка


private:
    float calculateVibration(float ax, float ay, float az);  // Расчёт вибраций
    int getErrorCode(float temp1, float vibration, float voltage, float current, float predVolt, float predCurr, float predPWM);
    int adjustPWM(float predPWM, float predVolt, float predCurr, float voltage, float current);

};

#endif