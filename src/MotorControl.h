// src/MotorControl.h
#include <Arduino.h>
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
private:
    int pwmPin;
    float maxCurrent;
    int currentPWM;  // Текущее значение PWM
    const int minPWM = 0;
    const int maxPWM = 255;

public:
    MotorControl(int pin, float maxCurrent);  // Конструктор
    void setPWM(int value);  // Установка значения ШИМ
    bool setCurrent(float current);  // Установка тока с ограничением
    void adjustPWM(int delta);  // Плавное изменение PWM
    int getCurrentPWM();  // Получение текущего PWM
};

#endif