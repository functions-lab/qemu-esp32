#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_SENS "esp32s3.sens"
#define ESP32S3_SENS(obj) OBJECT_CHECK(ESP32S3SensState, (obj), TYPE_ESP32S3_SENS)

#define ESP32S3_SENS_SIZE           0x200
#define ESP32S3_ADC_UNIT_CHANNELS   10
/* Analog inputs 0-9 are ADC1 channels 0-9, 10-19 are ADC2 channels 0-9 */
#define ESP32S3_ADC_INPUTS          (2 * ESP32S3_ADC_UNIT_CHANNELS)

typedef struct ESP32S3AdcWaveform {
    uint16_t *samples;
    int count;
    uint64_t period_ns;
} ESP32S3AdcWaveform;

typedef struct ESP32S3SensState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[ESP32S3_SENS_SIZE / 4];
    /* Pin voltage as a 12-bit code on a 3.3 V full scale */
    uint16_t input[ESP32S3_ADC_INPUTS];
    ESP32S3AdcWaveform waveform[ESP32S3_ADC_INPUTS];
} ESP32S3SensState;

void esp32s3_sens_set_input(ESP32S3SensState *s, int input, uint16_t code);
/* Periodic input: `count` codes evenly spaced from t = 0 to t = period_ns; count 0 clears it */
void esp32s3_sens_set_waveform(ESP32S3SensState *s, int input, const uint16_t *samples, int count,
                               uint64_t period_ns);
