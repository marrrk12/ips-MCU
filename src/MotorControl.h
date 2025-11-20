// src/MotorControl.h
#include <Arduino.h>
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
public:
    enum Mode {
        MODE_NORMAL,    // Наша логика (ток, плавность)
        MODE_BYPASS     // Прямая транзитная передача ШИМ с входа на выход
    };

private:
    const uint8_t pwmOutPin;     // Выход на драйвер (например PA8)
    const uint8_t pwmInPin;      // Вход ШИМ-сигнала (например PB8)

    // Текущие значения
    volatile unsigned long riseTime = 0;
    volatile unsigned long pulseWidth = 0;
    volatile bool newDataAvailable = false;

    int currentPWM = 0;
    float maxCurrent = 70.0f;
    Mode currentMode = MODE_NORMAL;

    // Статические функции для прерываний (должны быть static)
    static void handleRiseWrapper();
    static void handleFallWrapper();

    // Указатель на экземпляр (для прерываний)
    static MotorControl* instance;
    

public:
    MotorControl(uint8_t outPin, uint8_t inPin, float maxCurrent = 70.0f);
    void begin();
    void setMode(Mode mode);

    // Основные методы
    bool setPWM(int value);                    // 0-255
    bool setCurrent(float current);            // A
    void adjustPWM(int delta);                 // Плавное изменение
    int getCurrentPWM() const;
    unsigned long getInputPulseWidth() const;  // Для отладки
    float getMaxCurrent() const;

private:
    void handleRise();
    void handleFall();
    void updateOutput();  // Обновляет выход в зависимости от режима
};

#endif