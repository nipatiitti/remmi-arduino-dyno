#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <Arduino.h>
#include <mcp_canbus.h>

// Initialize CAN bus
bool initCAN();

// Check if CAN is initialized
bool isCANInitialized();

// Send dyno telemetry data over CAN bus
void sendCANTelemetry(unsigned long engineRPM, float torque, float force,
                      int servoPosition, bool pidActive);

// Send optical dyno telemetry data over CAN bus
void sendOpticalDynoTelemetry(float rearWheelTorque, float rearWheelRPM,
                              float rollerRPM, float angularAcceleration);

#endif  // CAN_BUS_H
