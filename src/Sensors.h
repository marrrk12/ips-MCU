// src/Sensors.h
#ifndef SENSORS_H
#define SENSORS_H

// Настраиваемые значения (изменяйте здесь для испытаний/полётов)
#define R1_VOLTAGE 300000.0f  // R1 в делителе напряжения
#define R2_VOLTAGE 20000.0f   // R2: 20000 для испытаний (21В), 51000 для полётов (21-48В)

// Для ACS758 (питание 5В)
#define ACS_QUIESCENT 0.6f    // Quiescent Vout при 5В
#define ACS_SENS 0.04f        // Чувствительность 40мВ/А
#define ACS_DIVIDER_SCALE 2.0f  // Масштаб делителя Vout (например, 2 для 10k/10k делителя, чтобы 5В→2.5В)

#define BATTERY_5S   0  // Испытания: 21В номинал, min=18.5В
#define BATTERY_10S  1  // Полёты: 42В номинал, min=37В

// Выберите конфигурацию BATTERY_5S или BATTERY_10S
#define BATTERY_TYPE BATTERY_10S

// Пороги разряда (V)
#define MIN_VOLTAGE_5S  18.5f
#define MIN_VOLTAGE_10S 37.0f

#include <Arduino.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "I2Cdev.h"
#include "MPU6050.h"  // Jeff Rowberg

class Sensors {
private:
    // Adafruit_MPU6050 mpu;  // Обновлённый объект MPU-6050
    MPU6050 mpu;
    OneWire oneWire;  // Шина для DS18B20
    DallasTemperature dsSensors;  // DS18B20
    DeviceAddress dsAddresses[3];  // Адреса трёх датчиков
    int voltagePin;  // Пин для напряжения (делитель)
    int currentPin;  // Пин для тока (ACS758)

public:
    Sensors(int oneWirePin, int voltPin, int currPin);  // Конструктор
    bool init();  // Инициализация с проверкой
    void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);  // Чтение MPU
    void readDS(float &temp1, float &temp2, float &temp3);  // Чтение DS18B20
    float readVoltage();  // Напряжение с делителя
    float readCurrent();  // Ток с ACS758

    // Калибровка (вызывать по кнопке или при старте)
    void calibrateMPU();
    void saveCalibrationToEEPROM();  // Опционально
    void loadCalibrationFromEEPROM(); // Опционально
};

#endif