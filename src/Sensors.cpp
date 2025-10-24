// src/Sensors.cpp
#include "Sensors.h"

Sensors::Sensors(int oneWirePin, int voltPin, int currPin) : oneWire(oneWirePin), dsSensors(&oneWire), voltagePin(voltPin), currentPin(currPin) {
    pinMode(voltagePin, INPUT);
    pinMode(currentPin, INPUT);
}
bool Sensors::init() {
    Wire.begin();  // Инициализация I2C
    if (!mpu.begin(0x68, &Wire)) {
        return false;  // Ошибка инициализации MPU
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // Настройка диапазона
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);  // Гироскоп
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // Фильтр
    dsSensors.begin();  // Инициализация DS18B20
    for (uint8_t i = 0; i < 3; i++) {
        if (!dsSensors.getAddress(dsAddresses[i], i)) {
            return false;  // Ошибка адреса
        }
    }
    return true;
}

void Sensors::readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  // Получение событий
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
}

void Sensors::readDS(float &temp1, float &temp2, float &temp3) {
    dsSensors.requestTemperatures();
    temp1 = dsSensors.getTempC(dsAddresses[0]);
    temp2 = dsSensors.getTempC(dsAddresses[1]);
    temp3 = dsSensors.getTempC(dsAddresses[2]);
}

float Sensors::readVoltage() {
    int analogValue = analogRead(voltagePin);  // 0-4095
    float voltageOut = (analogValue * 3.3f) / 4095.0f;  // V_out при 3.3V ADC
    float voltageIn = voltageOut / (R2_VOLTAGE / (R1_VOLTAGE + R2_VOLTAGE));  // V_in батареи (настраиваемо)
    return voltageIn;
}

float Sensors::readCurrent() { 
    int analogValue = analogRead(currentPin);  // 0-4095
    float voltageOut = (analogValue * 3.3f) / 4095.0f;  // V_out от ADC
    voltageOut *= ACS_DIVIDER_SCALE;  // Коррекция за делитель (если есть, иначе 1.0f)
    float current = (voltageOut - ACS_QUIESCENT) / ACS_SENS;  // Формула для униполярного ACS758 на 5В
    return current;
}