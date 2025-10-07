// src/Sensors.cpp
#include "Sensors.h"

Sensors::Sensors(int oneWirePin, int voltPin, int currPin) : oneWire(oneWirePin), dsSensors(&oneWire), voltagePin(voltPin), currentPin(currPin) {
    pinMode(voltagePin, INPUT);
    pinMode(currentPin, INPUT);
}
bool Sensors::init() {
    Wire.begin();  // Инициализация I2C
    if (!mpu.begin()) {
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
    float R1 = 300000.0;  //  Вот эти значения надо продумать как изменять                          
    float R2 = 51000.0;   // потому что у нас не всегда будет напряжение 21В
    int analogValue = analogRead(voltagePin);  // 0-4095 (12-бит ADC)
    float voltageOut = (analogValue * 3.3) / 4095.0;  // V_out при 3.3V опорном
    float voltageIn = voltageOut / (R2 / (R1 + R2));  // V_in батареи
    return voltageIn;
}

float Sensors::readCurrent() { 
    // ACS758 питается от 5V или 3.3V? Если 5V, центр 2.5V, 
    // скорректируйте формулу (current = (voltageOut - 2.5) / 0.04).
    int analogValue = analogRead(currentPin);  // 0-4095
    float voltageOut = (analogValue * 3.3) / 4095.0;  // V_out
    float current = (voltageOut - 1.65) / 0.04;  // Для ±100A, центр 1.65V при 3.3V
    return current;
}