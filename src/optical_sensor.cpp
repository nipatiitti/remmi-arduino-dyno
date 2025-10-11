#include "optical_sensor.h"

#include "calculations.h"
#include "config.h"

// Number of teeth on the cog
const int TEETH_COUNT = 20;

// Interrupt variables must be volatile
volatile unsigned long lastOpticalPulseTime = 0;
volatile unsigned long opticalPulseTime = 0;
volatile double opticalRPM = 0;
volatile unsigned long opticalPulseCount = 0;

// Variables for angular acceleration calculation
volatile float lastAngularVelocity = 0.0;
volatile float currentAngularVelocity = 0.0;
volatile float angularAcceleration = 0.0;
volatile unsigned long lastVelocityUpdateTime = 0;

// Initialize optical sensor interrupt
void initOpticalSensor() {
    pinMode(OPTICAL_SENSOR_PIN, INPUT_PULLUP);
    // Attach interrupt - trigger on FALLING edge (when beam is blocked)
    attachInterrupt(digitalPinToInterrupt(OPTICAL_SENSOR_PIN), opticalSensorISR,
                    FALLING);
}

// Get current optical RPM
double getOpticalRPM() { return opticalRPM; }

// Get last pulse time (for timeout checking)
unsigned long getLastOpticalPulseTime() { return lastOpticalPulseTime; }

// Get total pulse count (for debugging)
unsigned long getOpticalPulseCount() { return opticalPulseCount; }

// Check if optical RPM has timed out and reset to 0
void checkOpticalRPMTimeout() {
    if (micros() - lastOpticalPulseTime > TIMEOUT) {
        opticalRPM = 0;
        angularAcceleration = 0.0;
        currentAngularVelocity = 0.0;
        lastAngularVelocity = 0.0;
    }
}

// Optical sensor interrupt service routine
void opticalSensorISR() {
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastOpticalPulseTime;

    // Ignore pulses that come too quickly (debouncing)
    // With 20 teeth at 10000 RPM: 10000 * 20 / 60 = 3333 pulses/sec
    // Minimum interval = 1000000 / 3333 = 300 microseconds
    // Use 200 microseconds as debounce threshold for safety
    if (interval > 200) {
        opticalPulseTime = interval;
        lastOpticalPulseTime = currentTime;
        opticalPulseCount++;

        // Calculate RPM based on pulse interval and number of teeth
        // RPM = (60 seconds * 1000000 microseconds/second) / (pulseTime *
        // TEETH_COUNT) At 6000 RPM with 20 teeth: pulseTime should be 500
        // microseconds per tooth Ignore anything faster than 12000 RPM (pulse
        // time < 250 microseconds)
        if (opticalPulseTime > 250) {
            // Each pulse represents 1/TEETH_COUNT of a revolution
            opticalRPM = (60.0 * 1000000.0) / (opticalPulseTime * TEETH_COUNT);

            // Calculate angular velocity and acceleration
            // Convert flywheel RPM to roller RPM first
            float rollerRPM = opticalRPM / I_TRANSMISSION;
            currentAngularVelocity = (rollerRPM * 2.0 * PI) / 60.0;  // rad/s

            // Calculate angular acceleration: α = Δω / Δt
            if (lastVelocityUpdateTime > 0) {
                float deltaTime = (currentTime - lastVelocityUpdateTime) /
                                  1000000.0;  // convert to seconds
                if (deltaTime > 0) {
                    angularAcceleration =
                        (currentAngularVelocity - lastAngularVelocity) /
                        deltaTime;
                }
            }

            lastAngularVelocity = currentAngularVelocity;
            lastVelocityUpdateTime = currentTime;
        }
    }
}

// Get roller angular acceleration in rad/s^2
float getOpticalAngularAcceleration() { return angularAcceleration; }

// Get roller RPM from optical sensor
float getOpticalRollerRPM() { return calculateRollerRPM(opticalRPM); }

// Get calculated rear wheel RPM
float getOpticalRearWheelRPM() {
    float rollerRPM = getOpticalRollerRPM();
    return calculateRearWheelRPM(rollerRPM);
}

// Get calculated force at roller (in Newtons)
float getOpticalForceAtRoller() {
    return calculateForceAtRoller(angularAcceleration);
}

// Get calculated rear wheel torque (in Nm)
float getOpticalRearWheelTorque() {
    float force = getOpticalForceAtRoller();
    return calculateRearWheelTorque(force);
}
