#include "Timer.h"

Timer::Timer(unsigned long intervalMs, bool autoStart) 
    : interval(intervalMs), lastTime(0), enabled(autoStart) {
    if (autoStart) {
        lastTime = millis();
    }
}

void Timer::setInterval(unsigned long intervalMs) {
    interval = intervalMs;
}

bool Timer::update() {
    if (!enabled) return false;
    
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
        lastTime = currentTime;
        return true;
    }
    return false;
}

void Timer::start() {
    enabled = true;
    lastTime = millis();
}

void Timer::stop() {
    enabled = false;
}

bool Timer::isEnabled() {
    return enabled;
}

unsigned long Timer::getElapsed() {
    return millis() - lastTime;
}

unsigned long Timer::getRemaining() {
    if (!enabled) return 0;
    unsigned long elapsed = millis() - lastTime;
    return (elapsed >= interval) ? 0 : interval - elapsed;
}