#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

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

void setup() {
    Serial.begin(115200);
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
}

void loop() {
    // Check if wheel has stopped (no pulses for TIMEOUT period)
    if (micros() - lastPulseTime > TIMEOUT) {
        flywheelRPM = 0;
    }

    int potValue = analogRead(POT_PIN);
    Setpoint = mapPotValueToRPM(potValue);

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

    // DEBUG PRINTS
    // Print every 100ms
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

        Serial.print(">Servo_target:");
        Serial.println(servoPosition);

        Serial.print(">PID_Output:");
        Serial.println(Output);

        Serial.print(">RPM_Hall_status:");
        Serial.println(digitalRead(HALL_SENSOR_PIN));

        Serial.print(">ADC:");
        Serial.println(value);

        lastPrint = millis();
    }
}