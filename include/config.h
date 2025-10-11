#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pin Definitions
const int POT_PIN = A0;                  // Analog pin A0
const int HALL_SENSOR_PIN = 7;           // Digital pin 7
const int OPTICAL_SENSOR_PIN = 2;        // Digital pin 2 (interrupt capable)
const int SERVO_PIN = 4;                 // Digital pin 4
const int PRESSURE_SENSOR_AMP_PIN = A3;  // Analog pin A3

// Servo Configuration
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;  // Increase this to increase servo travel

// Motor Configuration
const int MOTOR_MAX_RPM = 5000;  // Maximum RPM of the motor

// Timing Configuration
const unsigned long TIMEOUT = 2 * 1000000;  // 2 seconds in microseconds
const unsigned long DEBOUNCE_TIME = 2000;   // 2ms in microseconds
const unsigned long SERIAL_TARGET_TIMEOUT =
    10000;                                        // 10 seconds in milliseconds
const unsigned long SERVO_UPDATE_INTERVAL = 100;  // ms between updates (10 Hz)
const unsigned long CAN_BUS_UPDATE_INTERVAL =
    100;  // ms between CAN bus updates (10 Hz)
const unsigned long SERIAL_PRINT_INTERVAL = 100;  // ms between serial prints

// Dyno Mechanical Configuration
const float Z_E = 18;     // ENGINE_SPROCKET_TEETH
const float Z_W = 149;    // REAR_WHEEL_SPROCKET_TEETH
const float D_W = 480.0;  // mm, diameter REAR_WHEEL_DIA
const float D_D = 58.0;   // mm, diameter DYNO_ROLL_DIA
const float Z_DF = 13;    // DYNO_FRONT_SPROCKET_TEETH
const float Z_DR = 30;    // DYNO_REAR_SPROCKET_TEETH

// Lever arm length for torque calculation
const float L = 0.234;  // m, length of the lever arm

// Total ratio from engine to dyno flywheel
const float GEAR_RATIO = Z_E / Z_W * D_W / D_D * Z_DF / Z_DR;

// Wheel parameters for force calculation
const float WHEEL_RADIUS_M =
    D_W / 2000.0;  // Convert wheel diameter from mm to radius in meters

// Dyno Optical Sensor Configuration (10/2025)
const float Z_ROLLER = 20.0;                         // teeth on roller gear
const float Z_FLYWHEEL = 29.0;                       // teeth on flywheel gear
const float I_TRANSMISSION = Z_ROLLER / Z_FLYWHEEL;  // transmission ratio
const float R_ROLLER = 0.110;      // radius of roller in meters
const float R_REARWHEEL = 0.240;   // radius of rear wheel in meters
const float Z_SENSORWHEEL = 20.0;  // teeth on sensor wheel

// Inertia at roller based on measurements in 10/2025
const float J_AT_ROLLER = 0.7545;  // kg*m^2, inertia at roller

// PID Controller Parameters
const double PID_KP = 1;
const double PID_KI = 1;
const double PID_KD = 0.1;

// Anti-nonlinear parameters
// T = A*rpm + B*d**E
const double COEFF_A = 1;     // rpm coefficient
const double COEFF_B = 1;     // magnet distance coefficient
const double COEFF_E = -2.0;  // magnet distance exponent

// ADC Configuration
#define ADC_ADDR 0x48           // 7-bit I2C address (0x90 >> 1)
const uint8_t ADC_CHANNEL = 0;  // Default ADC channel for pressure sensor

// CAN Bus Configuration
#define SPI_CS_PIN 10
const uint32_t CAN_SEND_ID =
    0x300;  // 768 decimal - fits in 11-bit Standard CAN
const uint32_t CAN_OPTICAL_DYNO_ID =
    0x301;  // 769 decimal - optical dyno telemetry
const int MAX_CAN_INIT_ATTEMPTS = 10;

// Serial Configuration
const unsigned long SERIAL_BAUD_RATE = 115200;

#endif  // CONFIG_H
