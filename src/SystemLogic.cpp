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

    float vibration = calculateVibration(ax, ay, az);  // Расчёт вибраций
    // bool motorOk = motor.setCurrent(current);  // Установка тока

    

    // Прогнозы от RPi
    float predAX = 0, predAY = 0, predAZ = 0, predGX = 0, predGY = 0, predGZ = 0;
    float predTEMP1 = temp1, predTEMP2 = temp2, predTEMP3 = temp3;
    float predVOLT = voltage, predCURR = current, predPWM = motor.getCurrentPWM() * 1000.0f;
    bool useForecast = uart.receiveForecast(predAX, predAY, predAZ, predGX, predGY, predGZ,
                                           predTEMP1, predTEMP2, predTEMP3, predVOLT, predCURR, predPWM);
    
    bool uartOk = useForecast; // UART OK, если прогноз получен
    int errorCode = getErrorCode(temp1, vibration, voltage, current, predVOLT, predCURR, predPWM);

    // Корректировка ШИМ
    int targetPWM = useForecast ? adjustPWM(predPWM, predVOLT, predCURR, voltage, current) : motor.getCurrentPWM();
    bool motorOk = motor.setPWM(targetPWM);

    // Остановка при ошибке
    if (errorCode != ERROR_NONE || !motorOk) {
        motor.setPWM(0);
    }

    // Обновление светодиода
    updateLED(errorCode, uartOk);

    float pwm_value = motor.getCurrentPWM() * 1000.0f;
    // Отправка данных на RPi
    uart.sendData(static_cast<int16_t>(ax * 100), static_cast<int16_t>(ay * 100), static_cast<int16_t>(az * 100),
                  static_cast<int16_t>(gx * 100), static_cast<int16_t>(gy * 100), static_cast<int16_t>(gz * 100),
                  temp1, temp2, temp3, voltage, current, pwm_value, errorCode);
}


int SystemLogic::getErrorCode(float temp1, float vibration, float voltage, float current, float predVolt, float predCurr, float predPWM) {
    if (current > maxCurrent) return ERROR_OVERCURRENT;
    if (temp1 > maxTemp) return ERROR_OVERHEAT;
    if (vibration > maxVibration) return ERROR_HIGH_VIBRATION;
    if (voltage < minVoltage) return ERROR_LOW_VOLTAGE;
    if (predVolt < minVoltage * predVoltThreshold) return ERROR_LOW_VOLTAGE;
    if (predCurr > maxCurrent * predCurrThreshold) return ERROR_OVERCURRENT;
    if (predPWM > predPWMThreshold) return ERROR_CRITICAL;
    return ERROR_NONE;
}

int SystemLogic::adjustPWM(float predPWM, float predVolt, float predCurr, float voltage, float current) {
    float currentPWM = static_cast<float>(motor.getCurrentPWM());
    float targetPWM = predPWM / 1000.0f * 255.0f; // Конвертация μs в 0-255

    // Защита батареи: снижение ШИМ при низком напряжении
    if (predVolt < minVoltage * predVoltThreshold || voltage < minVoltage) {
        targetPWM = max(currentPWM - pwmAdjustStep, 0.0f);
    }
    // Защита мотора: снижение ШИМ при высоком токе
    else if (predCurr > maxCurrent * predCurrThreshold || current > maxCurrent * 0.8f) {
        targetPWM = max(currentPWM - pwmAdjustStep, 0.0f);
    }
    // Повышение ШИМ, если прогноз безопасен
    else if (predVolt > minVoltage && predCurr < maxCurrent * 0.7f && predPWM <= predPWMThreshold) {
        targetPWM = min(currentPWM + pwmAdjustStep, 255.0f);
    }

    return constrain(static_cast<int>(targetPWM), 0, 255);
}

float SystemLogic::calculateVibration(float ax, float ay, float az) {
    return sqrt(ax * ax + ay * ay + (az - 9.81f) * (az - 9.81f));
}

void SystemLogic::updateLED(int errorCode, bool uartOk) {
    if (errorCode == ERROR_CRITICAL) {
        digitalWrite(LED_PIN, LOW); // Постоянно горит
        ledBlinkCount = 0;
        ledTargetBlinks = 0;
        ledState = true;
    } else if (errorCode != ERROR_NONE) {
        ledBlinkPattern(4, 200, 200, 2000); // 4 быстрых моргания
    } else if (!uartOk) {
        ledBlinkPattern(2, 200, 200, 2000); // 2 быстрых моргания
    } else {
        ledBlinkPattern(1, 200, 1800, 2000); // 1 короткое моргание
    }
}

void SystemLogic::ledBlinkPattern(int blinks, int onTime, int offTime, int cycleTime) {
    unsigned long currentTime = millis();

    // Начало нового цикла
    if (currentTime - lastLedUpdate >= cycleTime) {
        ledBlinkCount = 0;
        ledTargetBlinks = blinks;
        ledState = true;
        digitalWrite(LED_PIN, LOW); // Вкл (инвертированная логика)
        ledNextToggle = currentTime + onTime;
        lastLedUpdate = currentTime;
        return;
    }

    // Если цикл активен, проверяем переключение
    if (ledTargetBlinks > 0 && currentTime >= ledNextToggle) {
        if (ledState) {
            // Выключаем светодиод
            digitalWrite(LED_PIN, HIGH); // Выкл
            ledState = false;
            ledNextToggle = currentTime + offTime;
            ledBlinkCount++;
        } else if (ledBlinkCount < ledTargetBlinks) {
            // Включаем светодиод для следующего моргания
            digitalWrite(LED_PIN, LOW); // Вкл
            ledState = true;
            ledNextToggle = currentTime + onTime;
        } else {
            // Завершаем цикл морганий
            ledTargetBlinks = 0;
            ledBlinkCount = 0;
            digitalWrite(LED_PIN, HIGH); // Выкл
        }
    }
}