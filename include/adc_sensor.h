#ifndef ADC_SENSOR_H
#define ADC_SENSOR_H

#include <Arduino.h>

// Initialize the ADC sensor (I2C)
void initADC();

// Read one channel (0-7) from the ADC
uint16_t readADC(uint8_t channel);

#endif  // ADC_SENSOR_H
