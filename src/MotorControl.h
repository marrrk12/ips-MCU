// src/MotorControl.h
#include <Arduino.h>
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
private:
    int pwmPin;
    float maxCurrent = 100.0f;
    int currentPWM;  // Текущее значение PWM
    const int minPWM = 0;
    const int maxPWM = 255;

public:
    MotorControl(int pin, float maxCurrent);  // Конструктор
    bool setPWM(int value);  // Установка значения ШИМ
    bool setCurrent(float current);  // Установка тока с ограничением
    void adjustPWM(int delta);  // Плавное изменение PWM
    int getCurrentPWM();  // Получение текущего PWM
    float getMaxCurrent(); // Добавляем публичный геттер для maxCurrent

};

#endif