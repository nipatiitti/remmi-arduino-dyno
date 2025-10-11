#include "serial_interface.h"

#include "calculations.h"
#include "config.h"

// Serial target RPM override
static float serialTargetRPM = -1;  // -1 means no override, use potentiometer
static unsigned long lastSerialTargetTime = 0;
static unsigned long lastPrint = 0;

// Initialize serial communication
void initSerial() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial);  // Wait for serial port to connect (needed for native USB)
}

// Check for serial RPM override input
void checkSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove whitespace and newline characters

        float receivedRPM = input.toFloat();
        if (receivedRPM > 0 && receivedRPM <= MOTOR_MAX_RPM) {
            serialTargetRPM = receivedRPM;
            lastSerialTargetTime = millis();
        }
    }

    // Check if serial target RPM has expired
    if (serialTargetRPM > 0 &&
        (millis() - lastSerialTargetTime > SERIAL_TARGET_TIMEOUT)) {
        serialTargetRPM = -1;  // Reset to use potentiometer
    }
}

// Get the current target RPM (either from serial or potentiometer)
float getTargetRPM() {
    if (serialTargetRPM > 0) {
        // Use serial override value
        return serialTargetRPM;
    } else {
        // Default to potentiometer value
        int potValue = analogRead(POT_PIN);
        return mapPotValueToRPM(potValue);
    }
}

// Check if using serial RPM override
bool isUsingSerialRPM() { return serialTargetRPM > 0; }

// Send telemetry data to serial (in Teleplot format)
void sendSerialTelemetry(float torque, float force, unsigned long engineRPM,
                         float setpoint, uint16_t adcValue) {
    if (millis() - lastPrint < SERIAL_PRINT_INTERVAL) {
        return;
    }

    // Data in Teleplot format
    Serial.print(">Torque:");
    Serial.println(torque);

    Serial.print(">Force:");
    Serial.println(force);

    Serial.print(">Engine:");
    Serial.println(engineRPM);

    Serial.print(">Target:");
    Serial.println(setpoint);

    Serial.print(">ADC:");
    Serial.println(adcValue);

    lastPrint = millis();
}
