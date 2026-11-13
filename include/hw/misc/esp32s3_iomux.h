#pragma once

#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_IOMUX "esp32s3.iomux"
#define ESP32S3_IOMUX(obj) OBJECT_CHECK(ESP32S3IOMuxState, (obj), TYPE_ESP32S3_IOMUX)

#define ESP32S3_IOMUX_PAD_COUNT 49

REG32(IO_MUX_PIN_CTRL, 0x00)
REG32(IO_MUX_GPIO0, 0x04)
    FIELD(IO_MUX_GPIO0, FUN_WPD, 7, 1)
    FIELD(IO_MUX_GPIO0, FUN_WPU, 8, 1)
    FIELD(IO_MUX_GPIO0, FUN_IE, 9, 1)
    FIELD(IO_MUX_GPIO0, FUN_DRV, 10, 2)
    FIELD(IO_MUX_GPIO0, MCU_SEL, 12, 3)
REG32(IO_MUX_DATE, 0xFC)

typedef struct ESP32S3IOMuxState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t pin_ctrl;
    /* IO_MUX_GPIOn_REG, indexed by GPIO number */
    uint32_t pads[ESP32S3_IOMUX_PAD_COUNT];
} ESP32S3IOMuxState;
