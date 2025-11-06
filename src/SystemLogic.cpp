// src/SystemLogic.cpp
#include "SystemLogic.h"

SystemLogic::SystemLogic(Sensors& sens, MotorControl& mot, UARTComm& u, int batteryType) :
    sensors(sens), motor(mot), uart(u), maxCurrent(mot.getMaxCurrent()) {
    minVoltage = (batteryType == BATTERY_5S) ? MIN_VOLTAGE_5S : MIN_VOLTAGE_10S;
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

    // Чтение ШИМ от полётного контроллера
    float inputPWM = readInputPWM(); // В μs
    // Комбинируем inputPWM и predPWM (например, берём минимум для безопасности)
    float effectivePWM = useForecast ? min(inputPWM, predPWM) : inputPWM;

    int errorCode = getErrorCode(temp1, temp2, temp3, vibration, voltage, current, predVOLT, predCURR, predPWM, predTEMP2);
    int targetPWM = useForecast ? adjustPWM(effectivePWM, predVOLT, predCURR, voltage, current, predTEMP2, temp2) 
    : adjustPWM(motor.getCurrentPWM() * 1000.0f, voltage, current, voltage, current, predTEMP2, temp2);
    bool motorOk = motor.setPWM(targetPWM);

    // Обновление светодиода с приоритетом ошибок
    updateLED(errorCode, uartOk && motorOk);

    if (errorCode != ERROR_NONE || !motorOk) {
        motor.setPWM(0);
        analogWrite(PA8, 0);
    }

    float pwm_value = motor.getCurrentPWM() * 1000.0f;
    uart.sendData(static_cast<int16_t>(ax * 100), static_cast<int16_t>(ay * 100), static_cast<int16_t>(az * 100),
                  static_cast<int16_t>(gx * 100), static_cast<int16_t>(gy * 100), static_cast<int16_t>(gz * 100),
                  temp1, temp2, temp3, voltage, current, pwm_value, errorCode, vibration);
}

float SystemLogic::readInputPWM() {
    unsigned long pulse = pulseIn(INPUT_PWM_PIN, HIGH, 25000); // Ожидаем импульс до 25 мс
    if (pulse == 0) return motor.getCurrentPWM() * 1000.0f; // Если нет сигнала, вернуть текущий ШИМ
    return constrain(pulse, 1000.0f, 2000.0f); // Ограничиваем 1000–2000 μs
}

int SystemLogic::getErrorCode(float temp1, float temp2, float temp3, float vibration, float voltage, float current, float predVolt, float predCurr, float predPWM, float predTEMP2) {
    if (temp2 > maxTempBattery) return ERROR_OVERHEAT_BATTERY;  // АКБ перегрев
    if (temp1 > maxTemp) return ERROR_OVERHEAT;
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


int SystemLogic::adjustPWM(float effectivePWM, float predVolt, float predCurr, float voltage, float current, float predTEMP2, float temp2) {
    float targetPWM = static_cast<int>(effectivePWM / 1000.0f * 255.0f); // Конвертация μs в 0-255

    // Защита батареи: снижение ШИМ при низком напряжении
    if (predVolt < minVoltage * predVoltThreshold || voltage < minVoltage) {
        targetPWM = max(targetPWM - pwmAdjustStep, 0.0f);
    }
    // Защита мотора: снижение ШИМ при высоком токе
    else if (predCurr > maxCurrent * predCurrThreshold || current > maxCurrent) {
        targetPWM = max(targetPWM - pwmAdjustStep, 0.0f);
    }
    // Повышение ШИМ, если прогноз безопасен
    else if (predVolt > minVoltage && predCurr < maxCurrent * 0.7f && effectivePWM <= predPWMThreshold) {
        targetPWM = min(targetPWM + pwmAdjustStep, 255.0f);
    }
    if (predTEMP2 > maxTempBattery * 0.9f || temp2 > maxTempBattery * 0.8f) {
        targetPWM = max(targetPWM - pwmAdjustStep * 2, 0.0f);  // Усиленное понижение при перегреве АКБ
    }

    return constrain(static_cast<int>(targetPWM), 0, 255); // вот это надо тестить, потому что не факт что конвертация нужна 
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
    Serial1.println("MPU filtred vibronation: " + String(filtered) + "");
    return filtered;
}

void SystemLogic::updateLED(int errorCode, bool systemOk) {
    unsigned long currentTime = millis();
    
    // Сбрасываем состояние при изменении errorCode или systemOk
    if (errorCode != lastErrorCode || systemOk != lastSystemOk) {
        ledBlinkCount = 0;
        ledTargetBlinks = 0;
        ledState = false;
        digitalWrite(LED_PIN, HIGH); // Выкл (инвертированная логика)
        lastLedUpdate = currentTime;
        lastErrorCode = errorCode;
        lastSystemOk = systemOk;
    }

    if (errorCode == ERROR_CRITICAL) {
        digitalWrite(LED_PIN, LOW); // Постоянно горит
        ledBlinkCount = 0;
        ledTargetBlinks = 0;
        ledState = true;
    } else if (errorCode != ERROR_NONE) {
        ledBlinkPattern(4, 200, 200, 2000); // 4 быстрых моргания
    } else if (!systemOk) {
        ledBlinkPattern(2, 200, 200, 2000); // 2 быстрых моргания
    } else {
        ledBlinkPattern(1, 200, 1800, 2000); // 1 моргание
    }
}

void SystemLogic::ledBlinkPattern(int blinks, int onTime, int offTime, int cycleTime) {
    unsigned long currentTime = millis();

    // Начало нового цикла
    if (currentTime - lastLedUpdate >= cycleTime && ledBlinkCount == 0) {
        ledBlinkCount = 0;
        ledTargetBlinks = blinks;
        ledState = true;
        digitalWrite(LED_PIN, LOW); // Вкл
        ledNextToggle = currentTime + onTime;
        lastLedUpdate = currentTime;
        return;
    }

    // Переключение в активном цикле
    if (ledTargetBlinks > 0 && currentTime >= ledNextToggle) {
        if (ledState) {
            digitalWrite(LED_PIN, HIGH); // Выкл
            ledState = false;
            ledNextToggle = currentTime + offTime;
            ledBlinkCount++;
        } else if (ledBlinkCount < ledTargetBlinks) {
            digitalWrite(LED_PIN, LOW); // Вкл
            ledState = true;
            ledNextToggle = currentTime + onTime;
        } else {
            ledTargetBlinks = 0;
            ledBlinkCount = 0;
            digitalWrite(LED_PIN, HIGH); // Выкл
        }
    }
}