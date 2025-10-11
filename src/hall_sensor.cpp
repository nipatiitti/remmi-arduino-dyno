#include "hall_sensor.h"

#include "config.h"

// Interrupt variables must be volatile
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseTime = 0;
volatile double flywheelRPM = 0;

// Initialize hall sensor interrupt
void initHallSensor() {
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    // Attach interrupt - trigger on FALLING edge
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallEffect,
                    FALLING);
}

// Get current flywheel RPM
double getFlywheelRPM() { return flywheelRPM; }

// Get last pulse time (for timeout checking)
unsigned long getLastPulseTime() { return lastPulseTime; }

// Check if RPM has timed out and reset to 0
void checkRPMTimeout() {
    if (micros() - lastPulseTime > TIMEOUT) {
        flywheelRPM = 0;
    }
}

// Hall effect interrupt service routine
void hallEffect() {
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastPulseTime;

    // Ignore pulses that come too quickly (debouncing)
    if (interval > DEBOUNCE_TIME) {
        pulseTime = interval;
        lastPulseTime = currentTime;

        // Calculate RPM with bounds checking
        // At 6000 RPM, pulseTime should be 10000 microseconds
        // Ignore anything faster than 10000 RPM (pulse time < 6000
        // microseconds)
        if (pulseTime > 6000) {
            flywheelRPM = (60.0 * 1000000.0) / pulseTime;
        }
    }
}
