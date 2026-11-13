#pragma once

#include "hw/sysbus.h"
#include "hw/dma/esp_gdma.h"

#define TYPE_ESP32S3_GPSPI "esp32s3.gpspi"
#define ESP32S3_GPSPI(obj) OBJECT_CHECK(ESP32S3GpspiState, (obj), TYPE_ESP32S3_GPSPI)

/* Named GPIO outputs: chip-select lane levels, one line per CS0..CS2 */
#define ESP32S3_GPSPI_CS        "esp32s3_gpspi_cs"
#define ESP32S3_GPSPI_CS_COUNT  3

#define ESP32S3_GPSPI_REG_COUNT (0x100 / 4)

typedef struct ESP32S3GpspiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs[ESP32S3_GPSPI_CS_COUNT];
    ESPGdmaState *gdma;
    /* Bus number the host's SPI events carry */
    uint8_t bus;

    uint32_t regs[ESP32S3_GPSPI_REG_COUNT];
    /* CS lanes held active after a CS_KEEP_ACTIVE transaction */
    uint8_t cs_held;
} ESP32S3GpspiState;
