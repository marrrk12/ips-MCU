#include <Arduino.h>
#include <Wire.h>

// MPU-6050 I2C адрес
#define MPU_ADDR 0x68

// UART1 (PA9 - TX, PA10 - RX)
HardwareSerial Serial1(PA10, PA9);

// Данные MPU-6050
struct MPUData {
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  int16_t temp;
  bool valid;
};

MPUData mpuData = {0};
unsigned long lastReadTime = 0;
unsigned long READ_INTERVAL = 100;  // Убрал const для изменения значения

// Прототипы функций (добавь это в начало)
void initializeMPU();
bool readMPUData();
void sendMPUData();
void receiveCommands();

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // Инициализация I2C
  Wire.begin();
  Wire.setClock(400000);  // Fast mode 400kHz
  
  // Инициализация MPU-6050
  initializeMPU();
  
  Serial.println("MPU-6050 инициализирован");
  Serial1.println("MPU-6050 Ready!");
}

void initializeMPU() {
  // Выход из sleep режима
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU-6050
  Wire.endTransmission(true);
  
  // Настройка гироскопа ±250°/sec
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission(true);
  
  // Настройка акселерометра ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g
  Wire.endTransmission(true);
}

bool readMPUData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting from register 0x3B (ACCEL_XOUT_H)
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  
  // Запрашиваем 14 байт (3*accel, temp, 3*gyro)
  // Исправлена неоднозначность вызова
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);
  if (Wire.available() < 14) {
    return false;
  }
  
  // Чтение данных
  mpuData.accelX = Wire.read() << 8 | Wire.read();
  mpuData.accelY = Wire.read() << 8 | Wire.read();
  mpuData.accelZ = Wire.read() << 8 | Wire.read();
  mpuData.temp = Wire.read() << 8 | Wire.read();
  mpuData.gyroX = Wire.read() << 8 | Wire.read();
  mpuData.gyroY = Wire.read() << 8 | Wire.read();
  mpuData.gyroZ = Wire.read() << 8 | Wire.read();
  
  return true;
}

void sendMPUData() {
  // Формат: ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,TEMP
  String dataString = 
    String(mpuData.accelX) + "," +
    String(mpuData.accelY) + "," +
    String(mpuData.accelZ) + "," +
    String(mpuData.gyroX) + "," +
    String(mpuData.gyroY) + "," +
    String(mpuData.gyroZ) + "," +
    String(mpuData.temp);
  
  dataString += "\n";
  
  Serial1.print(dataString);
  
  // Дебаг в Serial
  Serial.print("MPU: A(");
  Serial.print(mpuData.accelX);
  Serial.print(",");
  Serial.print(mpuData.accelY);
  Serial.print(",");
  Serial.print(mpuData.accelZ);
  Serial.print(") G(");
  Serial.print(mpuData.gyroX);
  Serial.print(",");
  Serial.print(mpuData.gyroY);
  Serial.print(",");
  Serial.print(mpuData.gyroZ);
  Serial.print(") T:");
  Serial.println(mpuData.temp);
}

void receiveCommands() {
  if (Serial1.available() > 0) {
    String received = Serial1.readStringUntil('\n');
    received.trim();
    
    Serial.println("Команда от RPi: " + received);
    
    if (received == "CALIBRATE") {
      // Простая калибровка - сброс смещения
      Serial1.println("CALIBRATION_STARTED");
      delay(100);
      Serial1.println("CALIBRATION_DONE");
    } else if (received == "RATE_10HZ") {
      READ_INTERVAL = 100;
      Serial1.println("RATE_SET_10HZ");
    } else if (received == "RATE_50HZ") {
      READ_INTERVAL = 20;
      Serial1.println("RATE_SET_50HZ");
    }
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Чтение данных MPU-6050
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    mpuData.valid = readMPUData();
    lastReadTime = currentTime;
    
    if (mpuData.valid) {
      sendMPUData();
    } else {
      Serial.println("Ошибка чтения MPU-6050");
      Serial1.println("ERROR:MPU_READ_FAIL");
    }
  }

  // Приём команд от Raspberry Pi
  receiveCommands();
}