#include "pressure_sensor.h"

// Calibration reference values for pressure sensor
// [ADC_VALUE, GRAMS]
const int REFERENCES = 14;
const int REFERENCE_VALUES[14][2] = {
    {808, 0},     {838, 350},   {858, 700},   {880, 1050},  {914, 1400},
    {932, 1750},  {957, 2100},  {988, 2450},  {1011, 2800}, {1062, 3150},
    {1084, 3500}, {1120, 3850}, {1153, 4200}, {1180, 4550}};

// Linear interpolation to convert pressure sensor ADC value to kilograms
float pressureSensorToKG(int pressureSensorValue) {
    // Handle case where pressure value is below the minimum reference
    if (pressureSensorValue <= REFERENCE_VALUES[0][0]) {
        return REFERENCE_VALUES[0][1] / 1000.0;
    }

    // Handle case where pressure value is above the maximum reference
    if (pressureSensorValue >= REFERENCE_VALUES[REFERENCES - 1][0]) {
        return REFERENCE_VALUES[REFERENCES - 1][1] / 1000.0;
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
