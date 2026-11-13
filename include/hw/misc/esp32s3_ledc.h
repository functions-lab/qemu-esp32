#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_LEDC "esp32s3.ledc"
#define ESP32S3_LEDC(obj) OBJECT_CHECK(ESP32S3LEDCState, (obj), TYPE_ESP32S3_LEDC)

#define ESP32S3_LEDC_CHANNELS 8
#define ESP32S3_LEDC_TIMERS   4

/* Output: a channel's duty or frequency took effect; the line is the channel */
#define ESP32S3_LEDC_UPDATE "esp32s3_ledc_update"

typedef struct ESP32S3LEDCState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq update[ESP32S3_LEDC_CHANNELS];

    uint32_t ch_conf0[ESP32S3_LEDC_CHANNELS];
    uint32_t ch_hpoint[ESP32S3_LEDC_CHANNELS];
    uint32_t ch_duty[ESP32S3_LEDC_CHANNELS];
    uint32_t ch_conf1[ESP32S3_LEDC_CHANNELS];
    /* The duty in effect, with 4 fractional bits (DUTY_R) */
    uint32_t ch_duty_r[ESP32S3_LEDC_CHANNELS];
    /* The host reads timer_freq[] and timer_conf[] as one table */
    uint32_t timer_freq[ESP32S3_LEDC_TIMERS];
    uint32_t timer_conf[ESP32S3_LEDC_TIMERS];
    /* Duty in effect as a percentage of the PWM period */
    float duty_pct[ESP32S3_LEDC_CHANNELS];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf;
} ESP32S3LEDCState;
