// src/SystemLogic.cpp
#include "SystemLogic.h"

volatile int lastDelta = 0;

// SystemLogic::SystemLogic(Sensors& sens, UARTComm& u) : sensors(sens), uart(u) {}

void SystemLogic::begin()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Выключен по умолчанию
    Serial.println("SystemLogic: INITIALIZED");
    for (uint8_t i = 0; i < 3; i++) {
    sensors.dsSensors.setResolution(sensors.dsAddresses[i], 9);
  }
}

void SystemLogic::update(unsigned long basePWM)
{
    unsigned long now = millis();

  // Асинх DS18B20
  if (!tempsRequested) {
    sensors.dsSensors.requestTemperatures();
    lastTempRequest = now;
    tempsRequested = true;
  } else if (now - lastTempRequest >= tempConversionTime) {
    sensors.readDS(t1, t2, t3);
    tempsRequested = false;
  }
    // sensors.readDS(t1, t2, t3);
    voltage = sensors.readVoltage();
    current = sensors.readCurrent();
    float ax, ay, az, gx, gy, gz;
    sensors.readMPU(ax, ay, az, gx, gy, gz);
    vibration = sensors.getVibrationLevel();

    // Холд прогноза
    if (uart.receiveForecast(predT1, predT2, predT3, predV, predI, predPWM, predVib))
    { // предпоследний — predPWM (не используется)
        lastForecastTime = millis();
    }
    bool forecastValid = (millis() - lastForecastTime < forecastTimeout);
    float useT1 = forecastValid ? predT1 : t1;
    float useT2 = forecastValid ? predT2 : t2;
    float useT3 = forecastValid ? predT3 : t3;
    float useV = forecastValid ? predV : voltage;
    float useI = forecastValid ? predI : current;
    float usePWM = forecastValid ? predPWM : basePWM;
    float useVib = forecastValid ? predVib : vibration;


    errorCode = getErrorCode(t1, t2, t3,  voltage, current, basePWM, vibration,
                           predT1, predT2, predT3, predV, predI, predPWM, predVib);
    voltage = 37.5;
    t2=30;
    t3=20;
    predV = 37;
    targetDelta = calculateDelta(basePWM);
    // int target = basePWM + delta;
    // // Плавный ramp
    // if (target > currentCorrectedPWM)
    //     currentCorrectedPWM = min(currentCorrectedPWM + 100, target);
    // else
    //     currentCorrectedPWM = max(currentCorrectedPWM - 100, target);

    // currentCorrectedPWM = constrain(currentCorrectedPWM, 800, 2300);

    uart.sendData(t1, t2, t3, voltage, current, basePWM, vibration, errorCode);
    updateLED(errorCode);

    // if (errorCode != ERROR_NONE || !motorOk) {
    //     motor.setPWM(EMERGENCY_PWM);
    //     analogWrite(PA8, EMERGENCY_PWM);
    // }

    // static uint32_t lastDebugPrint = 0;
    // const uint32_t debugInterval = 500; // каждые 500 мс

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

void SystemLogic::rampDelta() {
  if (targetDelta > lastDelta) {
    lastDelta = min(lastDelta + RAMP_STEP, targetDelta);
  } else if (targetDelta < lastDelta) {
    lastDelta = max(lastDelta - RAMP_STEP, targetDelta);
  }
}

int SystemLogic::calculateDelta(int basePWM)
{
    int delta = 0;

    // Текущие
    if (t1 > maxTempMotor) // все цифры надо изменить на зарезервированные значения, чтобы их можно было менять глобально
        delta -= 200; 
    if (t2 > maxTempBattery)
        delta -= 150;
    if (vibration > maxVibration)
        delta -= 100;
    if (voltage < minVoltage)
        delta -= 300;
    if (current > maxCurrent)
        delta -= 250;

    // Прогнозы
    if (predT1 > maxTempMotor)
        delta -= 70;
    if (predT2 > maxTempBattery)
        delta -= 40;
    if (predVib > maxVibration)
        delta -= 15;
    if (predV < minVoltage)
        delta -= 90;
    if (predI > maxCurrent)
        delta -= 80;

    // Если холодно — можно чуть поддать
    if (t3 < 5.0f && t2 < 5.0f)
        delta += 60;
    
    Serial.print(F("delta: "));
    Serial.println(delta);
    return delta;
}

int SystemLogic::getErrorCode(float temp1, float temp2, float temp3,
                              float vibration, float voltage, float current, float pwm,
                              float predTEMP1, float predTEMP2, float predTEMP3,
                              float predVibration, float predVolt, float predCurr, float predPWM)
{
    if (temp2 > maxTempBattery)
        return ERROR_OVERHEAT_BATTERY; // АКБ перегрев
    if (temp1 > maxTempMotor)
        return ERROR_OVERHEAT_MOTOR;
    if (sensors.isVibrationDetected())
        return ERROR_HIGH_VIBRATION;
    if (voltage < minVoltage)
        return ERROR_LOW_VOLTAGE;
    if (current > maxCurrent)
        return ERROR_OVERCURRENT;
    // if (predVolt < minVoltage * predVoltThreshold) return ERROR_LOW_VOLTAGE;
    // if (predCurr > maxCurrent * predCurrThreshold) return ERROR_OVERCURRENT;
    // if (predPWM > predPWMThreshold) return ERROR_CRITICAL;
    // if (predTEMP2 > maxTempBattery * 0.9f) return ERROR_OVERHEAT_BATTERY;  // Упреждающий перегрев АКБ
    return ERROR_NONE;
}

void SystemLogic::updateLED(int errorCode)
{
    if (errorCode == ERROR_CRITICAL)
    {
        digitalWrite(LED_PIN, LOW); // Постоянно горит
        ledBlinkCount = ledTargetBlinks = 0;
        ledState = true;
    }
    else if (errorCode != ERROR_NONE)
    {
        ledBlinkPattern(4, 150, 150, 2000);
        // } else if (!systemOk) {
        //     ledBlinkPattern(2, 200, 200, 2000);
    }
    else
    {
        ledBlinkPattern(1, 100, 1900, 2000);
    }
}

void SystemLogic::ledBlinkPattern(int blinks, int onTime, int offTime, int cycleTime)
{
    unsigned long now = millis();
    if (now - lastLedUpdate >= cycleTime && ledBlinkCount == 0)
    {
        ledTargetBlinks = blinks;
        ledBlinkCount = 0;
        ledState = true;
        digitalWrite(LED_PIN, LOW);
        ledNextToggle = now + onTime;
        lastLedUpdate = now;
        return;
    }
    if (ledTargetBlinks > 0 && now >= ledNextToggle)
    {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        ledNextToggle = now + (ledState ? onTime : offTime);
        if (!ledState)
            ledBlinkCount++;
        if (ledBlinkCount >= ledTargetBlinks)
        {
            ledTargetBlinks = 0;
            digitalWrite(LED_PIN, HIGH);
        }
    }
}
