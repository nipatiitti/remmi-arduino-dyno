#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

// Convert pressure sensor ADC value to kilograms using linear interpolation
float pressureSensorToKG(int pressureSensorValue);

#endif  // PRESSURE_SENSOR_H
