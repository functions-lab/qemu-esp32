/*
 * ESP32-S3 IO MUX
 *
 * Pad configuration registers, stored for the guest and for the host, which
 * reads the pulls (FUN_WPU / FUN_WPD) through the picsimlab bridge.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32s3_iomux.h"

#define IO_MUX_DATE_VALUE 0x1907160
/* FUN_IE set, FUN_DRV = 2 */
#define PAD_RESET_VALUE   0x00000A00

static uint64_t esp32s3_iomux_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(opaque);

    if (addr >= A_IO_MUX_GPIO0 && addr < A_IO_MUX_GPIO0 + 4 * ESP32S3_IOMUX_PAD_COUNT) {
        return s->pads[(addr - A_IO_MUX_GPIO0) / 4];
    }
    switch (addr) {
    case A_IO_MUX_PIN_CTRL:
        return s->pin_ctrl;
    case A_IO_MUX_DATE:
        return IO_MUX_DATE_VALUE;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32s3.iomux: read of unmodeled register 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_iomux_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(opaque);

    if (addr >= A_IO_MUX_GPIO0 && addr < A_IO_MUX_GPIO0 + 4 * ESP32S3_IOMUX_PAD_COUNT) {
        s->pads[(addr - A_IO_MUX_GPIO0) / 4] = value;
        return;
    }
    switch (addr) {
    case A_IO_MUX_PIN_CTRL:
        s->pin_ctrl = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32s3.iomux: write of unmodeled register 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps esp32s3_iomux_ops = {
    .read = esp32s3_iomux_read,
    .write = esp32s3_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_iomux_reset_hold(Object *obj, ResetType type)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(obj);

    s->pin_ctrl = 0x7FF;
    for (int n = 0; n < ESP32S3_IOMUX_PAD_COUNT; n++) {
        s->pads[n] = PAD_RESET_VALUE;
    }
}

static void esp32s3_iomux_init(Object *obj)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_iomux_ops, s, TYPE_ESP32S3_IOMUX, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void esp32s3_iomux_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_iomux_reset_hold;
}

static const TypeInfo esp32s3_iomux_info = {
    .name = TYPE_ESP32S3_IOMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3IOMuxState),
    .instance_init = esp32s3_iomux_init,
    .class_init = esp32s3_iomux_class_init,
};

static void esp32s3_iomux_register_types(void)
{
    type_register_static(&esp32s3_iomux_info);
}

type_init(esp32s3_iomux_register_types)
