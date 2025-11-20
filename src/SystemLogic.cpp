// src/SystemLogic.cpp
#include "SystemLogic.h"

SystemLogic::SystemLogic(Sensors& sens, MotorControl& mot, UARTComm& u, int batteryType, int inputPWM_Pin) :
    sensors(sens), motor(mot), uart(u), maxCurrent(mot.getMaxCurrent()), inputPWM_PIN(inputPWM_Pin) {

    minVoltage = (batteryType == BATTERY_5S) ? MIN_VOLTAGE_5S : MIN_VOLTAGE_10S;
    // Инициализация LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Выключен по умолчанию

}

void SystemLogic::begin() {
    motor.begin();
    // Можно задать стартовый режим
    motor.setMode(MotorControl::MODE_BYPASS);
    Serial.println("SystemLogic: NORMAL MODE активирован");
}

void SystemLogic::update() {
    

    float ax, ay, az, gx, gy, gz;
    float temp1, temp2, temp3;
    float voltage = sensors.readVoltage();
    float current = sensors.readCurrent();
    sensors.readMPU(ax, ay, az, gx, gy, gz);
    sensors.readDS(temp1, temp2, temp3);

    float vibration = sensors.getVibrationLevel();  // 0–100

    float predAX = 0, predAY = 0, predAZ = 0, predGX = 0, predGY = 0, predGZ = 0;
    float predTEMP1 = temp1, predTEMP2 = temp2, predTEMP3 = temp3;
    float predVOLT = voltage, predCURR = current, predPWM = motor.getCurrentPWM() * 1000.0f;
    bool useForecast = uart.receiveForecast(predAX, predAY, predAZ, predGX, predGY, predGZ,
                                           predTEMP1, predTEMP2, predTEMP3, predVOLT, predCURR, predPWM);
    bool uartOk = useForecast;

    // === ВХОДНОЙ ШИМ ОТ ПОЛЁТНИКА (через MotorControl) ===
    unsigned long inputPulseUs = motor.getInputPulseWidth();
    float inputPWM_us = (inputPulseUs >= 1000 && inputPulseUs <= 2000) ? inputPulseUs : 1500.0f;

    // === ЛОГИКА УПРАВЛЕНИЯ ===
    int targetPWM = 0;
    int errorCode = ERROR_NONE;
    bool systemOk = true;

    // if (useForecast && predPWM >= 1000 && predPWM <= 2000) {
    //     // ИСП работает — используем безопасный минимум
    //     float safePWM = min(inputPWM_us, predPWM);
    //     targetPWM = map(constrain((int)safePWM, 1000, 2000), 1000, 2000, 0, 255);
    //     motor.setMode(MotorControl::MODE_NORMAL);
    // } else {
    //     motor.setMode(MotorControl::MODE_NORMAL);
    //     targetPWM = map(constrain((int)inputPWM_us,1000,2000),1000,2000,0,255);
        
        
    //     // // ИСП не отвечает → аварийный режим BYPASS
    //     // Serial.println(F("ИСП НЕ ОТВЕЧАЕТ → ПЕРЕКЛЮЧЕНИЕ В BYPASS MODE"));
    //     // motor.setMode(MotorControl::MODE_BYPASS);
    //     // targetPWM = map(constrain((int)inputPWM_us, 1000, 2000), 1000, 2000, 0, 255);
    //     // systemOk = false;
    // }


    errorCode = getErrorCode(temp1, temp2, temp3, vibration, voltage, current, predVOLT, predCURR, predPWM, predTEMP2);
    if (errorCode == ERROR_CRITICAL) {
        targetPWM = 1300;
        motor.setPWM(1300);
    } else {
        motor.setPWM(targetPWM);
    }

    // Обновление светодиода с приоритетом ошибок
    updateLED(errorCode, systemOk);

    // if (errorCode != ERROR_NONE || !motorOk) {
    //     motor.setPWM(EMERGENCY_PWM);
    //     analogWrite(PA8, EMERGENCY_PWM);
    // }

    
    uart.sendData(static_cast<int16_t>(ax * 100), static_cast<int16_t>(ay * 100), static_cast<int16_t>(az * 100),
                  static_cast<int16_t>(gx * 100), static_cast<int16_t>(gy * 100), static_cast<int16_t>(gz * 100),
                  temp1, temp2, temp3, voltage, current, motor.getCurrentPWM(), vibration, errorCode);

    static uint32_t lastDebugPrint = 0;
    const uint32_t debugInterval = 500; // каждые 500 мс

    // // === ОТЛАДКА В SERIAL ===
    // if (millis() - lastDebugPrint >= debugInterval) {
    //     Serial.print(F("IN: "));
    //     Serial.print(inputPWM_us);
    //     Serial.print(F("us → OUT: "));
    //     Serial.print(motor.getCurrentPWM());
    //     Serial.print(F(" | Mode: "));
    //     Serial.print(motor.getCurrentPWM() == map(constrain((int)inputPWM_us,1000,2000),1000,2000,0,255) ? F("BYPASS") : F("NORMAL"));
    //     Serial.print(F(" | Forecast: "));
    //     Serial.print(useForecast ? F("YES") : F("NO"));
    //     Serial.print(F(" | ERR: "));
    //     Serial.println(errorCode);
    //     lastDebugPrint = millis();
    // }
}


int SystemLogic::getErrorCode(float temp1, float temp2, float temp3, float vibration, float voltage, float current, float predVolt, float predCurr, float predPWM, float predTEMP2) {
    if (temp2 > maxTempBattery) return ERROR_OVERHEAT_BATTERY;  // АКБ перегрев
    if (temp1 > maxTemp) return ERROR_OVERHEAT_MOTOR;
    // if (vibration > maxVibration) return ERROR_HIGH_VIBRATION;
    if (sensors.isVibrationDetected()) return ERROR_HIGH_VIBRATION;
    if (voltage < minVoltage) return ERROR_LOW_VOLTAGE;
    if (current > maxCurrent) return ERROR_OVERCURRENT;
    if (predVolt < minVoltage * predVoltThreshold) return ERROR_LOW_VOLTAGE;
    if (predCurr > maxCurrent * predCurrThreshold) return ERROR_OVERCURRENT;
    if (predPWM > predPWMThreshold) return ERROR_CRITICAL;
    if (predTEMP2 > maxTempBattery * 0.9f) return ERROR_OVERHEAT_BATTERY;  // Упреждающий перегрев АКБ
    return ERROR_NONE;
}




float SystemLogic::calculateVibration(float ax, float ay, float az) {
    // Текущая норма гравитации (должна быть ~9.81)
    float currentMagnitude = sqrt(ax*ax + ay*ay + az*az);
    
    // Отклонение от 9.81 м/с²
    float vibration = fabs(currentMagnitude - 9.81f);
    
    // Фильтр (обязательно!)
    static float filtered = 0.0f;
    const float alpha = 0.7f;
    filtered = alpha * filtered + (1.0f - alpha) * vibration;
    Serial.println("MPU filtred vibronation: " + String(filtered) + "");
    return filtered;
}

void SystemLogic::updateLED(int errorCode, bool systemOk) {
    if (errorCode == ERROR_CRITICAL) {
        digitalWrite(LED_PIN, LOW);  // Постоянно горит
        ledBlinkCount = ledTargetBlinks = 0;
        ledState = true;
    } else if (errorCode != ERROR_NONE) {
        ledBlinkPattern(4, 150, 150, 2000);
    } else if (!systemOk) {
        ledBlinkPattern(2, 200, 200, 2000);
    } else {
        ledBlinkPattern(1, 100, 1900, 2000);
    }
}

void SystemLogic::ledBlinkPattern(int blinks, int onTime, int offTime, int cycleTime) {
    unsigned long now = millis();
    if (now - lastLedUpdate >= cycleTime && ledBlinkCount == 0) {
        ledTargetBlinks = blinks;
        ledBlinkCount = 0;
        ledState = true;
        digitalWrite(LED_PIN, LOW);
        ledNextToggle = now + onTime;
        lastLedUpdate = now;
        return;
    }
    if (ledTargetBlinks > 0 && now >= ledNextToggle) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        ledNextToggle = now + (ledState ? onTime : offTime);
        if (!ledState) ledBlinkCount++;
        if (ledBlinkCount >= ledTargetBlinks) {
            ledTargetBlinks = 0;
            digitalWrite(LED_PIN, HIGH);
        }
    }
}



