// src/Sensors.cpp
#include "Sensors.h"
#include <EEPROM.h>  // Для STM32 — виртуальная EEPROM
#include "Timer.h"

Sensors::Sensors(int oneWirePin, int voltPin, int currPin)
    : oneWire(oneWirePin), dsSensors(&oneWire), voltagePin(voltPin), currentPin(currPin), mpu(0x68){
    pinMode(voltagePin, INPUT);
    pinMode(currentPin, INPUT);
}

bool Sensors::init() {
    Serial.println("Sensors::init() START");

    // Включаем подтяжки (на всякий случай)
    pinMode(PB6, INPUT_PULLUP);
    pinMode(PB7, INPUT_PULLUP);
    
    Wire.begin();  // Инициализация I2C
    Wire.setClock(100000);  // 100 kHz — надёжно

    Serial.println("Wire.begin() OK");

    mpu.initialize();
    Serial.println("mpu.initialize() OK");

    bool connected = mpu.testConnection();
    Serial.print("mpu.testConnection(): ");
    Serial.println(connected ? "OK" : "FAILED");

    
    Serial.println("MPU6050 CONNECTED");

    // Настройка диапазонов
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);

    loadCalibration();
    applyOffsets();
    
    dsSensors.begin();
    for (uint8_t i = 0; i < 3; i++) {
        if (!dsSensors.getAddress(dsAddresses[i], i)) {
            Serial.print("DS18B20 #");
            Serial.print(i);
            Serial.println(" NOT FOUND");
            connected = false;
        }
    }
    
    if (!connected) {
        Serial.println("MPU6050 OR DS18B20 NOT RESPONDING!");
        return false;
    }
    
    Serial.println("Sensors::init() SUCCESS");
    return true;
}

void Sensors::readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    ax = (raw_ax - offsets[0]) / 4096.0f;  // ±8g
    ay = (raw_ay - offsets[1]) / 4096.0f;
    az = (raw_az - offsets[2]) / 4096.0f;
    gx = (raw_gx - offsets[3]) / 32.8f;    // ±1000°/s
    gy = (raw_gy - offsets[4]) / 32.8f;
    gz = (raw_gz - offsets[5]) / 32.8f;

    // === ВИБРАЦИЯ ===
    float acc_mag = sqrt(ax*ax + ay*ay + az*az);
    float g_force = acc_mag;
    float vib_g = fabs(g_force - 1.0f);

    sumSq += (long)(vib_g * 1000);
    samples++;

    uint32_t now = millis();
    if (now - lastVibTime >= VIB_WINDOW_MS) {
        float rms_g = sqrt((float)sumSq / samples) / 1000.0f;
        vibrationLevel = constrain(rms_g * 100, 0, 100);
        vibrationDetected = (vibrationLevel > VIB_THRESHOLD);

        sumSq = 0; samples = 0; lastVibTime = now;
    }
}   

float Sensors::getVibrationLevel() { return vibrationLevel; }
bool Sensors::isVibrationDetected() { return vibrationDetected; }

void Sensors::calibrateMPU() {
    Serial.println("MPU CALIBRATION START");


    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int ax_o = 0, ay_o = 0, az_o = 0, gx_o = 0, gy_o = 0, gz_o = 0;

    mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0); mpu.setYGyroOffset(0); mpu.setZGyroOffset(0);

    meansensors(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

    ax_o = -mean_ax / 8;
    ay_o = -mean_ay / 8;
    az_o = (4096 - mean_az) / 8;  // ±8g
    gx_o = -mean_gx / 4;
    gy_o = -mean_gy / 4;
    gz_o = -mean_gz / 4;

    // Итерации
    for (int i = 0; i < 50; i++) {
        mpu.setXAccelOffset(ax_o); mpu.setYAccelOffset(ay_o); mpu.setZAccelOffset(az_o);
        mpu.setXGyroOffset(gx_o); mpu.setYGyroOffset(gy_o); mpu.setZGyroOffset(gz_o);
        meansensors(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

        if (abs(mean_ax) > 8) ax_o -= mean_ax / 8;
        if (abs(mean_ay) > 8) ay_o -= mean_ay / 8;
        if (abs(4096 - mean_az) > 8) az_o += (4096 - mean_az) / 8;
        if (abs(mean_gx) > 2) gx_o -= mean_gx / 4;
        if (abs(mean_gy) > 2) gy_o -= mean_gy / 4;
        if (abs(mean_gz) > 2) gz_o -= mean_gz / 4;
        delay(2);
    }

    offsets[0] = ax_o; offsets[1] = ay_o; offsets[2] = az_o;
    offsets[3] = gx_o; offsets[4] = gy_o; offsets[5] = gz_o;
    EEPROM.put(EEPROM_START, offsets);
    applyOffsets();

    Serial.println("CALIBRATION SAVED TO EEPROM");
}

void Sensors::loadCalibration() {
  EEPROM.get(EEPROM_START, offsets);
  if (offsets[0] < -30000 || offsets[0] > 30000) {
    Serial.println("Нет калибровки. Отправь 'c'.");
    memset(offsets, 0, sizeof(offsets));
  } else {
    Serial.println("Калибровка загружена.");
  }
}

void Sensors::applyOffsets() {
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
}

void Sensors::meansensors(int* max, int* may, int* maz, int* mgx, int* mgy, int* mgz) {
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int16_t tax, tay, taz, tgx, tgy, tgz;

  int count = BUFFER_SIZE + 100;
  Timer sampleTimer(2, true);  

  for (int i = 0; i < count; i++) {
    mpu.getMotion6(&tax, &tay, &taz, &tgx, &tgy, &tgz);
    if (i >= 100) {
      sum_ax += tax; sum_ay += tay; sum_az += taz;
      sum_gx += tgx; sum_gy += tgy; sum_gz += tgz;
    }
    while (!sampleTimer.update()) {
            // Можно выполнять другие задачи здесь
            // HAL_IWDG_Refresh(&hiwdg); // Обновляем Watchdog
        }
  }

  *max = sum_ax / BUFFER_SIZE;
  *may = sum_ay / BUFFER_SIZE;
  *maz = sum_az / BUFFER_SIZE;
  *mgx = sum_gx / BUFFER_SIZE;
  *mgy = sum_gy / BUFFER_SIZE;
  *mgz = sum_gz / BUFFER_SIZE;
}

void Sensors::saveCalibrationToEEPROM() { EEPROM.put(EEPROM_START, offsets); }


void Sensors::readDS(float &temp1, float &temp2, float &temp3) {
    dsSensors.requestTemperatures();
    temp1 = dsSensors.getTempC(dsAddresses[0]); // motor
    temp2 = dsSensors.getTempC(dsAddresses[1]); // battery
    temp3 = dsSensors.getTempC(dsAddresses[2]); // outside
}

float Sensors::readVoltage() {
    const int samples = 50;
    long sum = 0;

    for (int i = 0; i < samples; i++) {
        sum += analogRead(voltagePin);
        delay(1);
    }

    int analogValue = sum / samples;
    float voltageOut = (analogValue * V_REF_CALIB) / 4095.0f * ADC_CALIB;
    float dividerRatio = R2_VOLTAGE / (R1_VOLTAGE + R2_VOLTAGE);
    float voltageIn = voltageOut / dividerRatio;

    
    // Serial.print("Analog: "); Serial.print(analogValue);
    // Serial.print(" | V_out: "); Serial.print(voltageOut, 3);
    // Serial.print("V | V_in: "); Serial.print(voltageIn, 2);
    // Serial.println("V");

    return voltageIn;
}

float Sensors::readCurrent() {
    const int NUM_SAMPLES = 10;
    static int samples[NUM_SAMPLES] = {0};
    static int sampleIndex = 0;

    // === ТВОЯ ТОЧНАЯ КАЛИБРОВКА ===
    const int ZERO_ADC = 515;                    // ← При 0 А
    const float SENSITIVITY_ADC_PER_A = 4.0f;    // ← 4 единицы ADC на 1 А
    // =====================================

    // Чтение с фильтром
    samples[sampleIndex] = analogRead(currentPin);
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += samples[i];
    }
    int avgADC = sum / NUM_SAMPLES;

    // === РАСЧЁТ ТОКА ===
    int deltaADC = avgADC - ZERO_ADC;
    float current = deltaADC / SENSITIVITY_ADC_PER_A;

    // Защита от шума
    if (current < 0.05f) current = 0.0f;

    // // Отладка (можно убрать потом)
    // static uint32_t lastPrint = 0;
    // if (millis() - lastPrint > 1000) {
    //     Serial.print(F("CURR: ADC="));
    //     Serial.print(avgADC);
    //     Serial.print(F(" → "));
    //     Serial.print(current, 2);
    //     Serial.println(F("A"));
    //     lastPrint = millis();
    // }

    return current;
}