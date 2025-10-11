#ifndef CALCULATIONS_H
#define CALCULATIONS_H

// Map the potentiometer value of 0-1023 to a target RPM
float mapPotValueToRPM(int potValue);

// Convert flywheel RPM to engine RPM
float FtoE(float flywheelRPM);

// Convert engine RPM to flywheel RPM
float EtoF(float engineRPM);

// Convert kilograms to Newton-meters (torque)
float KGtoNM(float kg);

// Calculate force at roller from torque at flywheel
float forceAtRoller(float torque_at_flywheel);

// Anti-nonlinearity compensation function
double anti_nonlinearize(double cmd, double rpm = 1);

// Dyno optical sensor calculations
// Calculate roller RPM from flywheel (optical sensor) RPM
float calculateRollerRPM(float rpm_flywheel);

// Calculate roller angular velocity in rad/s
float calculateRollerAngularVelocity(float rpm_roller);

// Calculate force at roller from angular acceleration
float calculateForceAtRoller(float angular_acceleration);

// Calculate rear wheel RPM from roller RPM
float calculateRearWheelRPM(float rpm_roller);

// Calculate rear wheel torque from roller force
float calculateRearWheelTorque(float force_roller);

#endif  // CALCULATIONS_H
