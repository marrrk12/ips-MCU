#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer {
private:
    unsigned long interval;
    unsigned long lastTime;
    bool enabled;

public:
    Timer(unsigned long intervalMs = 1000, bool autoStart = true);
    void setInterval(unsigned long intervalMs);
    bool update();  // Возвращает true, если время вышло
    void start();
    void stop();
    bool isEnabled();
    unsigned long getElapsed();
    unsigned long getRemaining();
};

#endif