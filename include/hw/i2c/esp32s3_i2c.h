#pragma once

#include "hw/sysbus.h"
#include "qemu/fifo8.h"

#define TYPE_ESP32S3_I2C "esp32s3.i2c"
#define ESP32S3_I2C(obj) OBJECT_CHECK(ESP32S3I2CState, (obj), TYPE_ESP32S3_I2C)

#define ESP32S3_I2C_FIFO_LEN 32
#define ESP32S3_I2C_CMD_COUNT 8
#define ESP32S3_I2C_REG_COUNT (0x100 / 4)

typedef struct ESP32S3I2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    /* Bus number the host's I2C events carry */
    uint8_t bus;

    uint32_t regs[ESP32S3_I2C_REG_COUNT];
    Fifo8 tx_fifo;
    Fifo8 rx_fifo;
    /* Bytes ever pushed into / popped from each FIFO, for FIFO_ST */
    uint32_t tx_pushed, tx_popped, rx_pushed, rx_popped;
    /* Between a START and its STOP */
    bool bus_busy;
    /* The next byte written is an address byte */
    bool expect_addr;
    /* The addressed slave ACKed its address */
    bool slave_engaged;
    uint8_t addr;
    uint32_t int_raw;
} ESP32S3I2CState;
