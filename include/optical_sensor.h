#ifndef OPTICAL_SENSOR_H
#define OPTICAL_SENSOR_H

#include <Arduino.h>

// Initialize optical sensor interrupt
void initOpticalSensor();

// Get current RPM from optical sensor (volatile, updated by interrupt)
double getOpticalRPM();

// Check if optical RPM has timed out and reset to 0
void checkOpticalRPMTimeout();

// Interrupt service routine (called by hardware interrupt)
void opticalSensorISR();

// Get last optical pulse time (for timeout checking)
unsigned long getLastOpticalPulseTime();

// Get total pulse count (for debugging/diagnostics)
unsigned long getOpticalPulseCount();

// Get roller angular acceleration in rad/s^2
float getOpticalAngularAcceleration();

// Get roller RPM from optical sensor
float getOpticalRollerRPM();

// Get calculated rear wheel RPM
float getOpticalRearWheelRPM();

// Get calculated force at roller (in Newtons)
float getOpticalForceAtRoller();

// Get calculated rear wheel torque (in Nm)
float getOpticalRearWheelTorque();

#endif  // OPTICAL_SENSOR_H
