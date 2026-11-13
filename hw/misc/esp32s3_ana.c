/*
 * ESP32-S3 analog configuration master (I2C_MST at 0x6000E000)
 *
 * The internal I2C master the ROM's regi2c functions use to program the
 * BBPLL and other analog blocks. The analog blocks are not modeled: their
 * registers hold what was written, and BBPLL and PHY calibrations complete at
 * once.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32s3_ana.h"

/* I2C0_CTRL and I2C1_CTRL share one layout: one analog register access each */
REG32(I2C0_CTRL, 0x00)
    FIELD(I2C_CTRL, SLAVE_ID, 0, 8)
    FIELD(I2C_CTRL, ADDR, 8, 8)
    FIELD(I2C_CTRL, DATA, 16, 8)
    FIELD(I2C_CTRL, WR, 24, 1)
    FIELD(I2C_CTRL, BUSY, 25, 1)
REG32(I2C1_CTRL, 0x04)
REG32(I2C_MST_ANA_CONF0, 0x40)
    FIELD(I2C_MST_ANA_CONF0, BBPLL_CAL_DONE, 24, 1)
REG32(I2C_MST_ANA_CONF2, 0x4C)
REG32(I2C_MST_STATUS, 0x50)
REG32(I2C_MST_CAL_STATUS, 0xC0)

/* Analog calibration status bits the PHY waits for */
#define ANA_STATUS_DONE 0x07000000

static uint64_t esp32s3_ana_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3AnaState *s = ESP32S3_ANA(opaque);
    uint32_t r = s->regs[addr / 4];

    if (addr == A_I2C_MST_ANA_CONF0) {
        r |= R_I2C_MST_ANA_CONF0_BBPLL_CAL_DONE_MASK;
    } else if (addr == A_I2C_MST_STATUS) {
        r |= ANA_STATUS_DONE;
    } else if (addr == A_I2C_MST_ANA_CONF2) {
        r |= 0xFF000000;
    } else if (addr == A_I2C_MST_CAL_STATUS) {
        r = 0xFFFFFFFF;
    }
    return r;
}

static void esp32s3_ana_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3AnaState *s = ESP32S3_ANA(opaque);
    if (addr == A_I2C0_CTRL || addr == A_I2C1_CTRL) {
        /* The access completes at once: a write stores the data, a read returns it */
        uint8_t *reg = &s->analog[FIELD_EX32(value, I2C_CTRL, SLAVE_ID)][FIELD_EX32(value, I2C_CTRL, ADDR)];
        if (FIELD_EX32(value, I2C_CTRL, WR)) {
            *reg = FIELD_EX32(value, I2C_CTRL, DATA);
        } else {
            value = FIELD_DP32(value, I2C_CTRL, DATA, *reg);
        }
        value = FIELD_DP32(value, I2C_CTRL, BUSY, 0);
    }
    s->regs[addr / 4] = value;
}

static const MemoryRegionOps esp32s3_ana_ops = {
    .read = esp32s3_ana_read,
    .write = esp32s3_ana_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_ana_init(Object *obj)
{
    ESP32S3AnaState *s = ESP32S3_ANA(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_ana_ops, s, TYPE_ESP32S3_ANA, ESP32S3_ANA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const TypeInfo esp32s3_ana_info = {
    .name = TYPE_ESP32S3_ANA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3AnaState),
    .instance_init = esp32s3_ana_init,
};

static void esp32s3_ana_register_types(void)
{
    type_register_static(&esp32s3_ana_info);
}

type_init(esp32s3_ana_register_types)
