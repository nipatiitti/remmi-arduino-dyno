#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <Arduino.h>

// Initialize hall sensor interrupt
void initHallSensor();

// Get current flywheel RPM (volatile, updated by interrupt)
double getFlywheelRPM();

// Check if RPM has timed out and reset to 0
void checkRPMTimeout();

// Interrupt service routine (called by hardware interrupt)
void hallEffect();

// Get last pulse time (for timeout checking)
unsigned long getLastPulseTime();

#endif  // HALL_SENSOR_H
