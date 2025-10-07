// src/SystemLogic.cpp
#include "SystemLogic.h"

SystemLogic::SystemLogic(Sensors& sens, MotorControl& mot, UARTComm& u) :
 sensors(sens), motor(mot), uart(u) {
    maxTemp = 50.0;
    maxVibration = 15.0;
}

void SystemLogic::update() {
    float ax, ay, az, gx, gy, gz;
    float temp1, temp2, temp3;
    float voltage = sensors.readVoltage();
    float current = sensors.readCurrent();
    sensors.readMPU(ax, ay, az, gx, gy, gz);
    sensors.readDS(temp1, temp2, temp3);

    float vibration = calculateVibration(ax, ay, az);  // Расчёт вибраций
    bool motorOk = motor.setCurrent(current);  // Установка тока

    // Прогнозы от RPi
    float predAX, predAY, predAZ, predGX, predGY, predGZ;
    float predTEMP1, predTEMP2, predTEMP3, predVOLT, predCURR;
    if (uart.receiveForecast(predAX, predAY, predAZ, predGX, predGY, predGZ,
                            predTEMP1, predTEMP2, predTEMP3, predVOLT, predCURR)) {
        // Логика на основе прогнозов
        if (predVOLT < 11.0 || predCURR > 100.0) {
            motor.setPWM(0);  // Остановка при низком напряжении или токе
            motorOk = false;
        } else {
            int targetPWM = static_cast<int>((predCURR / 100.0) * 255);  // Прогноз тока в PWM
            motor.setPWM(targetPWM);
        }
    }

    // Логика на основе данных
    if (temp1 > maxTemp || vibration > maxVibration || voltage < 11.0) {
        motor.setPWM(0);  // Остановка при превышении
        motorOk = false;
    } else if (motorOk) {
        // Плавное регулирование (пример)
        int delta = (vibration > 5.0) ? -10 : 0;  // Снижение при вибрациях >5 m/s²
        motor.adjustPWM(delta);
    }

uart.sendData(ax, ay, az, gx, gy, gz, temp1, temp2, temp3, voltage, current);

    if (!motorOk) {
        uart.sendData(0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1);  // Ошибка
    }
}

float SystemLogic::calculateVibration(float ax, float ay, float az) {
    return sqrt(ax * ax + ay * ay + (az - 9.81) * (az - 9.81));  // Корректировка гравитации

}