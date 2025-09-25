// src/Sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class Sensors {
private:
    Adafruit_MPU6050 mpu;  // Обновлённый объект MPU-6050
    OneWire oneWire;  // Шина для DS18B20
    DallasTemperature dsSensors;  // DS18B20
    DeviceAddress dsAddresses[3];  // Адреса трёх датчиков

public:
    Sensors(int oneWirePin);  // Конструктор
    bool init();  // Инициализация с проверкой
    void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);  // Чтение MPU
    void readDS(float &temp1, float &temp2, float &temp3);  // Чтение DS18B20
};

#endif