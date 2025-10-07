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
    float maxTemp;  // Максимальная температура (например, 50°C)
    float maxVibration;  // Максимальная вибрация (в m/s², например, 15)

public:
    SystemLogic(Sensors& sens, MotorControl& mot, UARTComm& u);
    void update();  // Обновление логики
    float calculateVibration(float ax, float ay, float az);  // Расчёт вибраций

};

#endif