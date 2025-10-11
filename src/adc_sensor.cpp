#include "adc_sensor.h"

#include <Wire.h>

#include "config.h"

// Initialize the ADC sensor (I2C)
void initADC() {
    Wire.begin();  // Initialize I2C bus
}

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
