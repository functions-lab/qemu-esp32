#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_ANA "esp32s3.ana"
#define ESP32S3_ANA(obj) OBJECT_CHECK(ESP32S3AnaState, (obj), TYPE_ESP32S3_ANA)

#define DR_REG_I2C_MST_BASE 0x6000E000
#define ESP32S3_ANA_SIZE    0x1000

typedef struct ESP32S3AnaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[ESP32S3_ANA_SIZE / 4];
    /* Registers of the analog blocks, by slave ID and register address */
    uint8_t analog[256][256];
} ESP32S3AnaState;
