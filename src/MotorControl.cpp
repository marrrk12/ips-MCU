#include "MotorControl.h"

// Статический указатель
MotorControl* MotorControl::instance = nullptr;

MotorControl::MotorControl(uint8_t outPin, uint8_t inPin, float maxCurrent)
    : pwmOutPin(outPin), pwmInPin(inPin), maxCurrent(maxCurrent) {
    instance = this;
}

void MotorControl::begin() {
    // pinMode(pwmOutPin, OUTPUT);
    // analogWrite(pwmOutPin, 0);

    pinMode(pwmInPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pwmInPin), handleRiseWrapper, RISING);
}

unsigned long MotorControl::getInputPulseWidth() const {
    noInterrupts();
    unsigned long pw = pulseWidth;
    interrupts();
    return pw;
}

void MotorControl::setMode(Mode mode) {
    currentMode = mode;
    if (mode == MODE_BYPASS) {
        // Serial.println("BYPASS MODE ACTIVATED — прямая передача ШИМ");
    } else {
        // Serial.println("NORMAL MODE — управление током");
    }
    updateOutput();
}

// ===================== ПРЕРЫВАНИЯ =====================
void MotorControl::handleRiseWrapper() {
    if (instance) instance->handleRise();
}

void MotorControl::handleFallWrapper() {
    if (instance) instance->handleFall();
}

void MotorControl::handleRise() {
    riseTime = micros();
    attachInterrupt(digitalPinToInterrupt(pwmInPin), handleFallWrapper, FALLING);
}

void MotorControl::handleFall() {
    pulseWidth = micros() - riseTime;
    newDataAvailable = true;
    attachInterrupt(digitalPinToInterrupt(pwmInPin), handleRiseWrapper, RISING);

    // В режиме BYPASS — сразу передаём
    if (currentMode == MODE_BYPASS) {
        updateOutput();
    }
}

// ===================== УПРАВЛЕНИЕ =====================
void MotorControl::updateOutput() {
    if (currentMode == MODE_BYPASS && newDataAvailable) {
        noInterrupts();
        unsigned long pw = pulseWidth;
        newDataAvailable = false;
        interrupts();


        if (pw >= 900 && pw <= 2100) {  // Защита
            analogWrite(pwmOutPin, pw);    // ← ПИШЕМ МИКРОСЕКУНДЫ!
            currentPWM = pw;            // Храним в мкс
        } else {
            analogWrite(pwmOutPin, 1000);  // Стоп
            currentPWM = 1000;
        }

    //     // Переводим 1000–2000 мкс → 0–255
    //     if (pw >= 1000 && pw <= 2000) {
    //         int pwm = map(pw, 1000, 2000, 0, 255);
    //         analogWrite(pwmOutPin, pwm);
    //         currentPWM = pwm;
    //     } else if (pw < 900 || pw > 2100) {
    //         analogWrite(pwmOutPin, 0);  // Защита
    //         currentPWM = 0;
    //     }
    }
}

bool MotorControl::setPWM(int value) {
    if (currentMode != MODE_NORMAL) return false;
    // currentPWM = constrain(value, 0, 255);
    currentPWM = constrain(value, 1000, 2000);
    analogWrite(pwmOutPin, currentPWM);
    return true;
}

bool MotorControl::setCurrent(float current) {
    if (currentMode != MODE_NORMAL) return false;
    if (current <= 0) {
        setPWM(0);
        return true;
    }
    if (current > maxCurrent) {
        setPWM(0);
        return false;
    }
    int targetPWM = (int)((current / maxCurrent) * 255);
    adjustPWM(targetPWM - currentPWM);
    return true;
}

void MotorControl::adjustPWM(int delta) {
    if (currentMode != MODE_NORMAL) return;
    setPWM(currentPWM + delta);
}

int MotorControl::getCurrentPWM() const {
    return currentPWM;
}



float MotorControl::getMaxCurrent() const {
    return maxCurrent;
}