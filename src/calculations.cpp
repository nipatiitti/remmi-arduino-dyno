#include "calculations.h"

#include <Arduino.h>

#include "config.h"

// Map the potentiometer value of 0-1023 to a target RPM
float mapPotValueToRPM(int potValue) {
    return map(potValue, 0.0, 1023.0, 0.0, MOTOR_MAX_RPM);
}

// Convert flywheel RPM to engine RPM
float FtoE(float flywheelRPM) { return flywheelRPM / GEAR_RATIO; }

// Convert engine RPM to flywheel RPM
float EtoF(float engineRPM) { return engineRPM * GEAR_RATIO; }

// Convert kilograms to Newton-meters (torque)
float KGtoNM(float kg) {
    float newtons = kg * 9.81;
    return newtons * L;  // Nm
}

// Calculate force at roller from torque at flywheel
float forceAtRoller(float torque_at_flywheel) {
    float torque_at_roller = Z_DF / Z_DR * torque_at_flywheel;
    return torque_at_roller / (D_D / 1000.0);  // Force in newtons
}

// Anti-nonlinearity compensation function
double anti_nonlinearize(double cmd, double rpm) {
    double u = 255 * COEFF_A * rpm * COEFF_B * pow((cmd / 255), (1 / COEFF_E));
    if (u > 2000) {
        u = 2000;
    }
    return u;
}

// Calculate roller RPM from flywheel (optical sensor) RPM
float calculateRollerRPM(float rpm_flywheel) {
    return rpm_flywheel / I_TRANSMISSION;
}

// Calculate roller angular velocity in rad/s
float calculateRollerAngularVelocity(float rpm_roller) {
    return (rpm_roller * 2.0 * PI) / 60.0;
}

// Calculate force at roller from angular acceleration
float calculateForceAtRoller(float angular_acceleration) {
    // F = (J * α) / R
    // where J = inertia, α = angular acceleration, R = radius
    return (J_AT_ROLLER * angular_acceleration) / R_ROLLER;
}

// Calculate rear wheel RPM from roller RPM
float calculateRearWheelRPM(float rpm_roller) {
    return rpm_roller * R_ROLLER / R_REARWHEEL;
}

// Calculate rear wheel torque from roller force
float calculateRearWheelTorque(float force_roller) {
    // Torque = Force × Radius
    return force_roller * R_REARWHEEL;
}
