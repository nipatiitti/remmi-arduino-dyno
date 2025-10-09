#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <mcp_canbus.h>

/*
TODO:
Kompensointi
- torque(rpm) = A*rpm
- torque(d) = B*d*-4
 */
double Setpoint, Input, Output;

double Kp = 1;
double Ki = 1;
double Kd = 0.1;
PID controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Anti-nonlinear parameters
// T = A*rpm + B*d**E
// double A = 0.0088; // rpm coefficient
// double B = 170; // magnet distance coefficient
double A = 1;     // rpm coefficient
double B = 1;     // magnet distance coefficient
double E = -2.0;  // magnet distance exponent0

Servo brakeServo;

const int servoMin = 0;
const int servoMax = 180;  // Increase this to increase servo travel

const int MOTOR_MAX_RPM = 5000;  // Maximum RPM of the motor

// Map the potentiometer value of 0-1023 to a target RPM
float mapPotValueToRPM(int potValue) {
    return map(potValue, 0.0, 1023.0, 0.0, MOTOR_MAX_RPM);
}

// Servo myservo;  // create Servo object to control a servo

const int POT_PIN = A0;                  // Analog pin A0
const int HALL_SENSOR_PIN = 7;           // Digital pin 7
const int SERVO_PIN = 4;                 // Digital pin 4
const int PRESSURE_SENSOR_AMP_PIN = A3;  // Analog pin A3

const unsigned long TIMEOUT = 2 * 1000000;  // 2 seconds in microseconds

// Serial target RPM override
float serialTargetRPM = -1;  // -1 means no override, use potentiometer
unsigned long lastSerialTargetTime = 0;
const unsigned long SERIAL_TARGET_TIMEOUT =
    10000;  // 10 seconds in milliseconds

#define ADC_ADDR 0x48  // 7-bit I2C address (0x90 >> 1)

// Function to read one channel (0-7) from the ADC
uint16_t readADC(uint8_t channel) {
    // The ADS7828 command byte structure:
    // Bit 7-4: Command (1000 = single-ended)
    // Bit 3-1: Channel selection bits (000 to 111)
    // Bit 0: Power management

    // For internal reference, use:
    // PD1=0, PD0=1 (internal reference ON, ADC ON)

    // Map channel (0-7) to the correct command bits (SD, C2, C1, C0)
    // Channel bit pattern to differential input mapping:
    // CH0: C2=0, C1=0, C0=0 -> maps to +CH0, -CH1
    // CH1: C2=0, C1=0, C0=1 -> maps to +CH2, -CH3
    // etc...

    // For single-ended mode, use this mapping:
    static const uint8_t channelMap[] = {
        0x00,  // CH0 = +IN0, -COM
        0x04,  // CH1 = +IN1, -COM
        0x01,  // CH2 = +IN2, -COM
        0x05,  // CH3 = +IN3, -COM
        0x02,  // CH4 = +IN4, -COM
        0x06,  // CH5 = +IN5, -COM
        0x03,  // CH6 = +IN6, -COM
        0x07   // CH7 = +IN7, -COM
    };

    // 0x80 = 1000xxxx (single-ended mode)
    // 0x08 = xxx01xxx (power mode: internal reference ON)
    // channelMap[channel] = xxxx[C2,C1,C0]x
    uint8_t command = 0x80 | 0x08 | (channelMap[channel] << 1);

    Wire.beginTransmission(ADC_ADDR);
    Wire.write(command);
    Wire.endTransmission();

    // Allow conversion time
    delay(2);

    // Request 2 bytes from the ADC
    Wire.requestFrom(ADC_ADDR, (uint8_t)2);
    if (Wire.available() < 2) {
        Serial.println("Error: not enough data received");
        return 0;
    }

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();

    // Combine to get 12-bit result
    uint16_t adcValue = ((uint16_t)msb << 8) | lsb;
    return adcValue;
}

// Calculations to move between engine rpm and flywheel rpm
const float Z_E = 18;     // ENGINE_SPROCKET_TEETH
const float Z_W = 149;    // REAR_WHEEL_SPROCKET_TEETH
const float D_W = 480.0;  // mm, diameter REAR_WHEEL_DIA
const float D_D = 58.0;   // mm, diameter DYNO_ROLL_DIA
const float Z_DF = 13;    // DYNO_FRONT_SPROCKET_TEETH
const float Z_DR = 30;    // DYNO_REAR_SPROCKET_TEETH

// Wheel parameters for force calculation
const float WHEEL_RADIUS_M =
    D_W / 2000.0;  // Convert wheel diameter from mm to radius in meters

// Total ratio from engine to dyno flywheel
const float I = Z_E / Z_W * D_W / D_D * Z_DF / Z_DR;

const float FLYWHEEL_TO_ENGINE_RATIO = 0.5;  // Old
float FtoE(float flywheelRPM) { return flywheelRPM / I; }
float EtoF(float engineRPM) { return engineRPM * I; }

// Interrupt variables must be volatile
const unsigned long DEBOUNCE_TIME = 2000;  // 2ms in microseconds
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseTime = 0;
volatile double flywheelRPM = 0;
void hallEffect() {
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastPulseTime;

    // Ignore pulses that come too quickly (debouncing)
    if (interval > DEBOUNCE_TIME) {
        pulseTime = interval;
        lastPulseTime = currentTime;

        // Calculate RPM with bounds checking
        // At 6000 RPM, pulseTime should be 10000 microseconds
        // Ignore anything faster than 10000 RPM (pulse time < 6000
        // microseconds)
        if (pulseTime > 6000) {
            flywheelRPM = (60.0 * 1000000.0) / pulseTime;
        }
    }
}

const int REFERENCES = 14;
const int REFERENCE_VALUES[REFERENCES][2] = {
    {808, 0},     {838, 350},   {858, 700},   {880, 1050},  {914, 1400},
    {932, 1750},  {957, 2100},  {988, 2450},  {1011, 2800}, {1062, 3150},
    {1084, 3500}, {1120, 3850}, {1153, 4200}, {1180, 4550}};

// Linear interpolation
float pressureSensorToKG(int pressureSensorValue) {
    // Handle case where pressure value is below the minimum reference
    if (pressureSensorValue <= REFERENCE_VALUES[0][0]) {
        return REFERENCE_VALUES[0][1] / 1000.0;
    }

    // Handle case where pressure value is above the maximum reference
    if (pressureSensorValue >= REFERENCE_VALUES[13][0]) {
        return REFERENCE_VALUES[13][1] / 1000.0;
    }

    int i = 0;
    while (i < REFERENCES - 1 && pressureSensorValue > REFERENCE_VALUES[i][0]) {
        i++;
    }

    // Linear interpolation
    int x0 = REFERENCE_VALUES[i - 1][0];
    int x1 = REFERENCE_VALUES[i][0];
    int y0 = REFERENCE_VALUES[i - 1][1];
    int y1 = REFERENCE_VALUES[i][1];

    float grams = y0 + (pressureSensorValue - x0) * (y1 - y0) / (x1 - x0);
    return grams / 1000.0;  // Convert grams to kg
}

const float L = 0.234;  // m, length of the lever arm
float KGtoNM(float kg) {
    float newtons = kg * 9.81;
    return newtons * L;  // Nm
}

double anti_nonlinearize(double cmd, double rpm = 1) {
    double u = 255 * A * rpm * B * pow((cmd / 255), (1 / E));
    if (u > 2000) {
        u = 2000;
    }
    // Serial.println(u);
    return u;
}

// CAN bus stuff
/*
   Arduino Leonardo SPI CS Pin - you can use any digital pin
   Using pin 9 as it's the standard SPI CS pin
   If pin 9 doesn't work, try: 10, 8, 7, 6, 5, 4, 3, 2
   Leonardo SPI pins (fixed):
   - MOSI: Pin 16 (ICSP-4)
   - MISO: Pin 14 (ICSP-1)
   - SCK:  Pin 15 (ICSP-3)
   - CS:   Any digital pin (currently pin 9)
*/
#define SPI_CS_PIN 10
MCP_CAN CAN(SPI_CS_PIN);       // Set CS pin
uint32_t CAN_SEND_ID = 0x300;  // 768 decimal - fits in 11-bit Standard CAN
const int CAN_BUS_UPDATE_INTERVAL = 100;  // ms between CAN bus updates // 10 Hz
bool canInitialized = false;              // Track CAN initialization status

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for serial port to connect. Needed for native USB
                      // in Leonardo

    Wire.begin();  // Initialize I2C bus (works for Arduino Leonardo)

    pinMode(POT_PIN, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    pinMode(PRESSURE_SENSOR_AMP_PIN, INPUT);

    brakeServo.attach(SERVO_PIN);
    brakeServo.write(0);

    // Attach interrupt - trigger on FALLING edge
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallEffect,
                    FALLING);

    // Print engine to flywheel ratio
    Serial.print("Engine to flywheel ratio: ");
    Serial.println(I);

    // initialize the variables we're linked to
    Input = 0;
    Setpoint = 0;
    controller.SetMode(AUTOMATIC);

    // CAN Bus stuff
    Serial.println("Starting CAN Bus initialization...");
    Serial.print("CAN CS Pin: ");
    Serial.println(SPI_CS_PIN);

    int initAttempts = 0;
    const int MAX_INIT_ATTEMPTS = 10;

    while (CAN_OK != CAN.begin(CAN_1000KBPS) &&
           initAttempts < MAX_INIT_ATTEMPTS) {
        initAttempts++;
        Serial.print("CAN BUS Shield init fail - Attempt ");
        Serial.print(initAttempts);
        Serial.print("/");
        Serial.println(MAX_INIT_ATTEMPTS);
        delay(500);
    }

    if (initAttempts >= MAX_INIT_ATTEMPTS) {
        Serial.println(
            "ERROR: CAN Bus initialization failed after maximum attempts!");
        Serial.println(
            "Hardware issue detected - continuing without CAN "
            "functionality...");
        canInitialized = false;
    } else {
        Serial.println("CAN BUS Shield init ok!");
        canInitialized = true;
    }
}

void loop() {
    // Check for serial input for target RPM override
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

    // Check if wheel has stopped (no pulses for TIMEOUT period)
    if (micros() - lastPulseTime > TIMEOUT) {
        flywheelRPM = 0;
    }

    // Determine target RPM source
    if (serialTargetRPM > 0) {
        // Use serial override value
        Setpoint = serialTargetRPM;
    } else {
        // Default to potentiometer value
        int potValue = analogRead(POT_PIN);
        Setpoint = mapPotValueToRPM(potValue);
    }

    unsigned long engineRPM = FtoE(flywheelRPM);
    Input = engineRPM;
    controller.Compute();

    static unsigned long lastServoUpdate = 0;
    const unsigned long SERVO_UPDATE_INTERVAL =
        100;  // ms between updates // 10 Hz

    // Map PID output (0-255) to servo position (servoMin-servoMax)
    // double cmd = anti_nonlinearize(Output);
    int servoPosition = map(Output, 0, 255, servoMin, servoMax);  // 0-255 cmd
    servoPosition = constrain(servoPosition, servoMin, servoMax);

    if (millis() - lastServoUpdate > SERVO_UPDATE_INTERVAL) {
        brakeServo.write(servoPosition);
        lastServoUpdate = millis();
    }

    // Torque calculations
    uint8_t channel = 0;  // Change this to 0-7 to read other channels
    uint16_t value = readADC(channel);
    float pressureKG = pressureSensorToKG(value);
    float torque =
        KGtoNM(pressureKG);  // This is used only for future calculations

    // CAN Bus transmission
    static unsigned long lastCANUpdate = 0;
    if (canInitialized && millis() - lastCANUpdate > CAN_BUS_UPDATE_INTERVAL) {
        float forceNewtons = torque / WHEEL_RADIUS_M;

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
        uint16_t forceData = constrain(forceNewtons * 100, 0, 65535);
        canData[4] = (forceData >> 8) & 0xFF;  // MSB first
        canData[5] = forceData & 0xFF;         // LSB second

        // Pack Servo position (byte 6)
        canData[6] = constrain(servoPosition, 0, 255);

        // Pack Status flags (byte 7)
        uint8_t statusFlags = 0;
        if (canInitialized) statusFlags |= (1 << 0);  // Bit 0: CAN initialized
        if (digitalRead(HALL_SENSOR_PIN) == LOW)
            statusFlags |= (1 << 1);  // Bit 1: Hall sensor active
        if (serialTargetRPM > 0)
            statusFlags |= (1 << 2);  // Bit 2: RPM source (1=serial, 0=pot)
        if (controller.GetMode() == AUTOMATIC)
            statusFlags |= (1 << 3);  // Bit 3: PID active
        canData[7] = statusFlags;

        // Send standard CAN frame
        CAN.sendMsgBuf(CAN_SEND_ID, 0, 8, canData);

        lastCANUpdate = millis();
    }

    // Send to serial every 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        // Data in Teleplotter format
        Serial.print(">Torque_(Nm):");
        Serial.println(torque);

        Serial.print(">Flywheel_RPM:");
        Serial.println(flywheelRPM);

        Serial.print(">Engine_RPM:");
        Serial.println(engineRPM);

        Serial.print(">RPM_Target:");
        Serial.println(Setpoint);

        Serial.print(">ADC:");
        Serial.println(value);

        // Display comprehensive CAN data being sent
        float forceNewtons = torque / WHEEL_RADIUS_M;

        Serial.print(">CAN_RPM:");
        Serial.println(engineRPM);

        Serial.print(">CAN_Torque_(cNm):");
        Serial.println(torque * 100);

        Serial.print(">CAN_Force_(cN):");
        Serial.println(forceNewtons * 100);

        Serial.print(">CAN_Servo_Pos:");
        Serial.println(servoPosition);

        lastPrint = millis();
    }
}