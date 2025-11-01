// src/Sensors.cpp
#include "Sensors.h"
#include <EEPROM.h>  // Для STM32 — виртуальная EEPROM

Sensors::Sensors(int oneWirePin, int voltPin, int currPin)
    : oneWire(oneWirePin), dsSensors(&oneWire), voltagePin(voltPin), currentPin(currPin), mpu(0x68){
    pinMode(voltagePin, INPUT);
    pinMode(currentPin, INPUT);
}
bool Sensors::init() {
    Serial1.println("Sensors::init() START");

        // Включаем подтяжки (на всякий случай)
    pinMode(PB6, INPUT_PULLUP);
    pinMode(PB7, INPUT_PULLUP);
    
    Wire.begin();  // Инициализация I2C
    Wire.setClock(100000);  // 100 kHz — надёжно

    Serial1.println("Wire.begin() OK");

    mpu.initialize();
    Serial1.println("mpu.initialize() OK");

    bool connected = mpu.testConnection();
    Serial1.print("mpu.testConnection(): ");
    Serial1.println(connected ? "OK" : "FAILED");

    if (!connected) {
        Serial1.println("MPU6050 NOT RESPONDING!");
        return false;
    }
    
    Serial1.println("MPU6050 CONNECTED");

    // Настройка диапазонов
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    // === ЗАГРУЗКА КАЛИБРОВКИ ИЗ EEPROM ===
    loadCalibrationFromEEPROM();

    // === ИЛИ КАЛИБРОВКА ПРИ СТАРТЕ ===
    // calibrateMPU();  

    // if (!mpu.begin(0x68, &Wire)) {
    //     Serial1.println("mpu FAIL");
    //     return false;
        
    // }
    // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // Фильтр 21 Гц
    
    dsSensors.begin();
    for (uint8_t i = 0; i < 3; i++) {
        if (!dsSensors.getAddress(dsAddresses[i], i)) {
            Serial1.print("DS18B20 #");
            Serial1.print(i);
            Serial1.println(" NOT FOUND");
            return false;
        }
    }

    Serial1.println("Sensors::init() SUCCESS");
    return true;
}

void Sensors::readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // ±8g → 4096 LSB/g
    ax = ax_raw / 4096.0f;
    ay = ay_raw / 4096.0f;
    az = az_raw / 4096.0f;

    // ±500 °/s → 65.5 LSB/°/s
    gx = gx_raw / 65.5f;
    gy = gy_raw / 65.5f;
    gz = gz_raw / 65.5f;
}   

void Sensors::calibrateMPU() {
    Serial1.println("MPU CALIBRATION START (6 iterations)");

    // Калибровка акселерометра
    mpu.CalibrateAccel(6);
    Serial1.println("Accel calibrated");

    // Калибровка гироскопа
    mpu.CalibrateGyro(6);
    Serial1.println("Gyro calibrated");

    // Сохраняем в EEPROM
    saveCalibrationToEEPROM();
    Serial1.println("CALIBRATION SAVED TO EEPROM");
}

void Sensors::saveCalibrationToEEPROM() {
    int16_t offsets[6];
    offsets[0] = mpu.getXAccelOffset();
    offsets[1] = mpu.getYAccelOffset();
    offsets[2] = mpu.getZAccelOffset();
    offsets[3] = mpu.getXGyroOffset();
    offsets[4] = mpu.getYGyroOffset();
    offsets[5] = mpu.getZGyroOffset();

    EEPROM.put(0, offsets);
    Serial1.println("Calibration saved to EEPROM");
}
void Sensors::loadCalibrationFromEEPROM() {
    int16_t offsets[6];
    EEPROM.get(0, offsets);

    // Проверка валидности (опционально)
    if (offsets[0] != 0xFFFF && offsets[0] != 0) {
        mpu.setXAccelOffset(offsets[0]);
        mpu.setYAccelOffset(offsets[1]);
        mpu.setZAccelOffset(offsets[2]);
        mpu.setXGyroOffset(offsets[3]);
        mpu.setYGyroOffset(offsets[4]);
        mpu.setZGyroOffset(offsets[5]);
        Serial1.println("Calibration loaded from EEPROM");
    } else {
        Serial1.println("No valid calibration in EEPROM");
    }
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

