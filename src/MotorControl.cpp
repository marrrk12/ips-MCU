// src/MotorControl.cpp
#include "MotorControl.h"

MotorControl::MotorControl(int pin, float maxCurrent) : pwmPin(pin), maxCurrent(maxCurrent), currentPWM(0) {
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, 0);  // Начальное значение
}

void MotorControl::setPWM(int value) {
    currentPWM = constrain(value, minPWM, maxPWM);  // Ограничение
    analogWrite(pwmPin, currentPWM);
}

bool MotorControl::setCurrent(float current) {
    if (current < 0 || current > maxCurrent) {
        setPWM(0);  // Остановка при превышении
        return false;
    }
    int targetPWM = static_cast<int>((current / maxCurrent) * maxPWM);
    adjustPWM(targetPWM - currentPWM);  // Плавное изменение
    return true;
}

void MotorControl::adjustPWM(int delta) {
    int newPWM = currentPWM + delta;
    setPWM(newPWM);  // Ограничение произойдёт внутри
}

int MotorControl::getCurrentPWM() {
    return currentPWM;
}