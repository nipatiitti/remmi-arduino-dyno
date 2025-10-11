#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

// Initialize PID controller and servo
void initPIDController();

// Update PID controller with current RPM and target RPM
void updatePIDController(double currentRPM, double targetRPM);

// Get current servo position
int getServoPosition();

// Update servo position (rate limited)
void updateServo();

// Get PID output value
double getPIDOutput();

// Check if PID is in automatic mode
bool isPIDAutomatic();

#endif  // PID_CONTROLLER_H
