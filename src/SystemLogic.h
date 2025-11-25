// src/SystemLogic.h
#ifndef SYSTEMLOGIC_H
#define SYSTEMLOGIC_H

#include "Sensors.h"
#include "UARTComm.h"
// #include "MotorControl.h"

#define EMERGENCY_PWM 1100  // Предполагаемый ШИМ для экстренных ситуаций
#define RAMP_STEP 50

#define LED_PIN PC13

extern volatile int lastDelta;

class SystemLogic {
private:
    Sensors& sensors;
    // MotorControl& motor;
    UARTComm& uart;
    float maxTempMotor = 80.0f;       // Максимальная температура (°C)
    float maxTempBattery = 50.0f;
    float maxVibration = 3.7f;  // Максимальная вибрация (m/s²)
    float minVoltage = 35.0f;            // Динамический порог напряжения
    float maxCurrent = 70.0f;            // Максимальный ток из MotorControl
    float predThreshold = 0.9f;
    int errorCode = ERROR_NONE;

    // Кэшированные значения (обновляются в update())
    float t1 = 0, t2 = 0, t3 = 0;
    float voltage = 0, current = 0, vibration = 0;
    float predT1 = 0, predT2 = 0, predT3 = 0, predV = 0, predI = 0, predPWM = 0, predVib = 0;

    uint32_t lastForecastTime = 0;
    const uint32_t forecastTimeout = 2000;
    // int currentCorrectedPWM = 1000;  // с ramp

    // Асинх DS
    uint32_t lastTempRequest = 0;
    bool tempsRequested = false;
    const uint32_t tempConversionTime = 94;  // 9bit

    int targetDelta = 0;  // Целевая дельта от calculate

    unsigned long lastLedUpdate = 0;
    int ledBlinkCount = 0;
    int ledTargetBlinks = 0;
    bool ledState = false;
    unsigned long ledNextToggle = 0;
    
    int calculateDelta(int basePWM);
    void updateLED(int errorCode);
    void ledBlinkPattern(int blinks, int onTime, int offTime, int cycleTime);
    int getErrorCode(float temp1, float temp2, float temp3, 
        float vibration, float voltage, float current, float pwm,
        float predTEMP1, float predTEMP2, float predTEMP3, 
        float predVibration, float predVolt, float predCurr, float predPWM);

public:
    
    SystemLogic(Sensors& s, UARTComm& u) : sensors(s), uart(u) {}
    void begin();
    void update(unsigned long basePWM);     // ← теперь принимает basePWM
    int getDesiredPWM(int basePWM) { return basePWM + lastDelta; }  // Для main, но мы используем lastDelta глобально    void rampDelta();
    void rampDelta();
    

    // Коды ошибок (зарезервированные значения для sendData)
    static const int ERROR_NONE = -1;                   // Нормальная работа
    static const int ERROR_OVERHEAT_MOTOR = -2;         // Перегрев >50°C
    static const int ERROR_HIGH_VIBRATION = -3;         // Вибрация >15 m/s²
    static const int ERROR_LOW_VOLTAGE = -4;            // Напряжение <В
    static const int ERROR_OVERCURRENT = -5;            // Ток >100A
    static const int ERROR_INIT_FAIL = -6;              // Ошибка инициализации
    static const int ERROR_OVERHEAT_BATTERY = -7;       // Новая ошибка для АКБ
    static const int ERROR_CRITICAL = -999;             // Критическая ошибка


};

#endif