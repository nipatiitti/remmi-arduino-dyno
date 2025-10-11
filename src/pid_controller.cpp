#include "pid_controller.h"

#include <PID_v1.h>

#include "config.h"

/*
TODO:
Kompensointi
- torque(rpm) = A*rpm
- torque(d) = B*d*-4
*/

// PID variables
static double Setpoint, Input, Output;
static PID controller(&Input, &Output, &Setpoint, PID_KP, PID_KI, PID_KD,
                      DIRECT);

// Servo object
static Servo brakeServo;

// Servo update timing
static unsigned long lastServoUpdate = 0;
static int currentServoPosition = 0;

// Initialize PID controller and servo
void initPIDController() {
    // Initialize PID
    Input = 0;
    Setpoint = 0;
    controller.SetMode(AUTOMATIC);

    // Initialize servo
    brakeServo.attach(SERVO_PIN);
    brakeServo.write(0);
    currentServoPosition = 0;
}

// Update PID controller with current RPM and target RPM
void updatePIDController(double currentRPM, double targetRPM) {
    Input = currentRPM;
    Setpoint = targetRPM;
    controller.Compute();
}

// Get current servo position
int getServoPosition() { return currentServoPosition; }

// Update servo position (rate limited)
void updateServo() {
    if (millis() - lastServoUpdate < SERVO_UPDATE_INTERVAL) {
        return;
    }

    // Map PID output (0-255) to servo position (SERVO_MIN-SERVO_MAX)
    // double cmd = anti_nonlinearize(Output);  // Optional anti-nonlinearity
    currentServoPosition = map(Output, 0, 255, SERVO_MIN, SERVO_MAX);
    currentServoPosition =
        constrain(currentServoPosition, SERVO_MIN, SERVO_MAX);

    brakeServo.write(currentServoPosition);
    lastServoUpdate = millis();
}

// Get PID output value
double getPIDOutput() { return Output; }

// Check if PID is in automatic mode
bool isPIDAutomatic() { return controller.GetMode() == AUTOMATIC; }
