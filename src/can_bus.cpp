#include "can_bus.h"

#include "config.h"
#include "hall_sensor.h"

/*
   Arduino Leonardo SPI CS Pin - you can use any digital pin
   Using pin 10 as it's the standard SPI CS pin
   If pin 10 doesn't work, try: 9, 8, 7, 6, 5, 4, 3, 2
   Leonardo SPI pins (fixed):
   - MOSI: Pin 16 (ICSP-4)
   - MISO: Pin 14 (ICSP-1)
   - SCK:  Pin 15 (ICSP-3)
   - CS:   Any digital pin (currently pin 10)
*/

// CAN bus object
MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin

// Track CAN initialization status
static bool canInitialized = false;

// Last CAN update time
static unsigned long lastCANUpdate = 0;

// Initialize CAN bus
bool initCAN() {
    Serial.println("Starting CAN Bus initialization...");
    Serial.print("CAN CS Pin: ");
    Serial.println(SPI_CS_PIN);

    int initAttempts = 0;

    while (CAN_OK != CAN.begin(CAN_1000KBPS) &&
           initAttempts < MAX_CAN_INIT_ATTEMPTS) {
        initAttempts++;
        Serial.print("CAN BUS Shield init fail - Attempt ");
        Serial.print(initAttempts);
        Serial.print("/");
        Serial.println(MAX_CAN_INIT_ATTEMPTS);
        delay(500);
    }

    if (initAttempts >= MAX_CAN_INIT_ATTEMPTS) {
        Serial.println(
            "ERROR: CAN Bus initialization failed after maximum attempts!");
        Serial.println(
            "Hardware issue detected - continuing without CAN "
            "functionality...");
        canInitialized = false;
        return false;
    } else {
        Serial.println("CAN BUS Shield init ok!");
        canInitialized = true;
        return true;
    }
}

// Check if CAN is initialized
bool isCANInitialized() { return canInitialized; }

// Send dyno telemetry data over CAN bus
void sendCANTelemetry(unsigned long engineRPM, float torque, float force,
                      int servoPosition, bool pidActive) {
    if (!canInitialized) {
        return;
    }

    if (millis() - lastCANUpdate < CAN_BUS_UPDATE_INTERVAL) {
        return;
    }

    // Prepare CAN message data (8 bytes)
    // Byte 0-1: Engine RPM (uint16_t, 0-65535 RPM)
    // Byte 2-3: Torque in cNm (uint16_t, centinewton-meters, 0-655.35 Nm)
    // Byte 4-5: Force in cN (uint16_t, centinewtons, 0-655.35 N)
    // Byte 6: Servo position (uint8_t, 0-255)
    // Byte 7: Status flags (uint8_t, bit flags)
    unsigned char canData[8] = {0};

    // Pack Engine RPM (bytes 0-1) - Big-endian format for MoTeC
    uint16_t rpmData = constrain(engineRPM, 0, 65535);
    canData[0] = (rpmData >> 8) & 0xFF;  // MSB first
    canData[1] = rpmData & 0xFF;         // LSB second

    // Pack Torque in centinewton-meters (bytes 2-3) - Big-endian
    uint16_t torqueData = constrain(torque * 100, 0, 65535);
    canData[2] = (torqueData >> 8) & 0xFF;  // MSB first
    canData[3] = torqueData & 0xFF;         // LSB second

    // Pack Force in centinewtons (bytes 4-5) - Big-endian
    uint16_t forceData = constrain(force * 100, 0, 65535);
    canData[4] = (forceData >> 8) & 0xFF;  // MSB first
    canData[5] = forceData & 0xFF;         // LSB second

    // Pack Servo position (byte 6)
    canData[6] = constrain(servoPosition, 0, 255);

    // Pack Status flags (byte 7)
    uint8_t statusFlags = 0;
    if (canInitialized) statusFlags |= (1 << 0);  // Bit 0: CAN initialized
    if (digitalRead(HALL_SENSOR_PIN) == LOW)
        statusFlags |= (1 << 1);  // Bit 1: Hall sensor active
    // Bit 2 will be set in main.cpp for serial target
    if (pidActive) statusFlags |= (1 << 3);  // Bit 3: PID active
    canData[7] = statusFlags;

    // Send standard CAN frame
    CAN.sendMsgBuf(CAN_SEND_ID, 0, 8, canData);

    lastCANUpdate = millis();
}

// Send optical dyno telemetry data over CAN bus
void sendOpticalDynoTelemetry(float rearWheelTorque, float rearWheelRPM,
                              float rollerRPM, float angularAcceleration) {
    if (!canInitialized) {
        return;
    }

    if (millis() - lastCANUpdate < CAN_BUS_UPDATE_INTERVAL) {
        return;
    }

    // Prepare CAN message data (8 bytes)
    // Byte 0-1: Rear Wheel Torque in cNm (uint16_t, centinewton-meters,
    // 0-655.35 Nm) Byte 2-3: Rear Wheel RPM (uint16_t, 0-65535 RPM) Byte 4-5:
    // Roller RPM (uint16_t, 0-65535 RPM) Byte 6-7: Angular Acceleration in 0.1
    // °/s² (int16_t, -3276.8 to 3276.7 °/s²)
    unsigned char canData[8] = {0};

    // Pack Rear Wheel Torque in centinewton-meters (bytes 0-1) - Big-endian
    uint16_t torqueData = constrain(rearWheelTorque * 100, 0, 65535);
    canData[0] = (torqueData >> 8) & 0xFF;  // MSB first
    canData[1] = torqueData & 0xFF;         // LSB second

    // Pack Rear Wheel RPM (bytes 2-3) - Big-endian
    uint16_t rearRPMData = constrain(rearWheelRPM, 0, 65535);
    canData[2] = (rearRPMData >> 8) & 0xFF;  // MSB first
    canData[3] = rearRPMData & 0xFF;         // LSB second

    // Pack Roller RPM (bytes 4-5) - Big-endian
    uint16_t rollerRPMData = constrain(rollerRPM, 0, 65535);
    canData[4] = (rollerRPMData >> 8) & 0xFF;  // MSB first
    canData[5] = rollerRPMData & 0xFF;         // LSB second

    // Convert angular acceleration from rad/s² to deg/s² (1 rad = 57.29578 deg)
    // Pack as uint16_t with 0.1 °/s² resolution and -3276.8 offset
    // Formula: raw_value = (deg/s² - offset) / scale = (deg/s² + 3276.8) / 0.1
    float accelDegrees =
        angularAcceleration * 57.29578;  // Convert rad/s² to deg/s²
    uint16_t accelData = constrain((accelDegrees + 3276.8) * 10, 0, 65535);
    canData[6] = (accelData >> 8) & 0xFF;  // MSB first
    canData[7] = accelData & 0xFF;         // LSB second

    // Send standard CAN frame
    CAN.sendMsgBuf(CAN_OPTICAL_DYNO_ID, 0, 8, canData);

    lastCANUpdate = millis();
}
