#include <Arduino.h>
#include <SPI.h>

// Include all module headers
#include "config.h"
#include "adc_sensor.h"
#include "calculations.h"
#include "can_bus.h"
#include "hall_sensor.h"
#include "pid_controller.h"
#include "pressure_sensor.h"
#include "serial_interface.h"

void setup() {
    // Initialize serial communication
    initSerial();

    // Initialize I2C ADC sensor
    initADC();

    // Initialize GPIO pins
    pinMode(POT_PIN, INPUT);
    pinMode(PRESSURE_SENSOR_AMP_PIN, INPUT);

    // Initialize hall sensor with interrupt
    initHallSensor();

    // Initialize PID controller and servo
    initPIDController();

    // Print system information
    Serial.print("Engine to flywheel ratio: ");
    Serial.println(GEAR_RATIO);

    // Initialize CAN Bus
    initCAN();
}

void loop() {
    // Check for serial input (RPM override)
    checkSerialInput();

    // Check if wheel has stopped (timeout)
    checkRPMTimeout();

    // Get current flywheel RPM and convert to engine RPM
    double flywheelRPM = getFlywheelRPM();
    unsigned long engineRPM = FtoE(flywheelRPM);

    // Get target RPM (from serial or potentiometer)
    float targetRPM = getTargetRPM();

    // Update PID controller
    updatePIDController(engineRPM, targetRPM);

    // Update servo position (rate limited)
    updateServo();

    // Read pressure sensor and calculate torque
    uint16_t adcValue = readADC(ADC_CHANNEL);
    float pressureKG = pressureSensorToKG(adcValue);
    float torque = KGtoNM(pressureKG);

    // Calculate force at roller
    float forceNewtons = forceAtRoller(torque);

    // Send CAN telemetry if initialized
    if (isCANInitialized()) {
        sendCANTelemetry(engineRPM, torque, forceNewtons, 
                        getServoPosition(), isPIDAutomatic());
    }

    // Send serial telemetry
    sendSerialTelemetry(torque, forceNewtons, engineRPM, targetRPM, adcValue);
}
