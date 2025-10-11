#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <Arduino.h>

// Initialize serial communication
void initSerial();

// Check for serial RPM override input
void checkSerialInput();

// Get the current target RPM (either from serial or potentiometer)
float getTargetRPM();

// Check if using serial RPM override
bool isUsingSerialRPM();

// Send telemetry data to serial (in Teleplot format)
void sendSerialTelemetry(float torque, float force, unsigned long engineRPM,
                         float setpoint, uint16_t adcValue);

#endif  // SERIAL_INTERFACE_H
