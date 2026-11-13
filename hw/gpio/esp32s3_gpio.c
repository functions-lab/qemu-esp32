/*
 * ESP32-S3 GPIO and GPIO matrix
 *
 * Register map from the ESP32-S3 Technical Reference Manual, chapter
 * "IO MUX and GPIO Matrix".
 *
 * A pin keeps one level, which both the outside world (ESP32S3_GPIOS_IN) and
 * an enabled GPIO output write; GPIO_IN reads it back. GPIO_OUT reaches a pin
 * only while the pin's FUNCn_OUT_SEL selects the GPIO_OUT signal.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32s3_gpio.h"

#define GPIO_DATE_VALUE 0x1907040

enum {
    INT_TYPE_DISABLED,
    INT_TYPE_RISING,
    INT_TYPE_FALLING,
    INT_TYPE_ANY_EDGE,
    INT_TYPE_LOW_LEVEL,
    INT_TYPE_HIGH_LEVEL,
};

#define INT_ENA_CPU BIT(0)
#define INT_ENA_NMI BIT(1)

static bool bank_bit(const uint32_t *bank, int n)
{
    return (bank[n / 32] >> (n % 32)) & 1;
}

static void set_bank_bit(uint32_t *bank, int n, bool value)
{
    if (value) {
        bank[n / 32] |= BIT(n % 32);
    } else {
        bank[n / 32] &= ~BIT(n % 32);
    }
}

static int int_type(ESP32S3GPIOState *s, int n)
{
    return FIELD_EX32(s->pin[n], GPIO_PIN0, INT_TYPE);
}

static bool level_type_matches(ESP32S3GPIOState *s, int n)
{
    bool level = bank_bit(s->in, n);
    switch (int_type(s, n)) {
    case INT_TYPE_LOW_LEVEL:
        return !level;
    case INT_TYPE_HIGH_LEVEL:
        return level;
    default:
        return false;
    }
}

static bool edge_triggers(ESP32S3GPIOState *s, int n, bool old, bool new)
{
    switch (int_type(s, n)) {
    case INT_TYPE_RISING:
        return new && !old;
    case INT_TYPE_FALLING:
        return old && !new;
    case INT_TYPE_ANY_EDGE:
        return old != new;
    default:
        return false;
    }
}

static uint32_t int_status(ESP32S3GPIOState *s, int bank, uint32_t enable_bit)
{
    uint32_t r = 0;
    for (int i = 0; i < 32 && bank * 32 + i < ESP32S3_GPIO_COUNT; i++) {
        int n = bank * 32 + i;
        if ((s->status[bank] & BIT(i)) && (FIELD_EX32(s->pin[n], GPIO_PIN0, INT_ENA) & enable_bit)) {
            r |= BIT(i);
        }
    }
    return r;
}

static void update_irq(ESP32S3GPIOState *s)
{
    qemu_set_irq(s->irq, int_status(s, 0, INT_ENA_CPU) || int_status(s, 1, INT_ENA_CPU));
    qemu_set_irq(s->nmi_irq, int_status(s, 0, INT_ENA_NMI) || int_status(s, 1, INT_ENA_NMI));
}

static void set_pad_level(ESP32S3GPIOState *s, int n, bool level)
{
    bool old = bank_bit(s->in, n);
    set_bank_bit(s->in, n, level);
    if (edge_triggers(s, n, old, level) || level_type_matches(s, n)) {
        set_bank_bit(s->status, n, true);
    }
    update_irq(s);
}

static bool gpio_out_drives_pad(ESP32S3GPIOState *s, int n)
{
    return bank_bit(s->enable, n) &&
           FIELD_EX32(s->out_sel[n], GPIO_FUNC0_OUT_SEL_CFG, OUT_SEL) == ESP32S3_GPIO_SIG_GPIO_OUT;
}

/* The host hears of the level before GPIO_IN reflects it. */
static void drive_pad(ESP32S3GPIOState *s, int n)
{
    bool level = bank_bit(s->out, n);
    qemu_set_irq(s->level_out[n], level);
    set_pad_level(s, n, level);
}

static void write_out(ESP32S3GPIOState *s, int bank, uint32_t value)
{
    uint32_t changed = s->out[bank] ^ value;
    s->out[bank] = value;
    for (int i = 0; i < 32; i++) {
        int n = bank * 32 + i;
        if ((changed & BIT(i)) && n < ESP32S3_GPIO_COUNT && gpio_out_drives_pad(s, n)) {
            drive_pad(s, n);
        }
    }
}

static void write_enable(ESP32S3GPIOState *s, int bank, uint32_t value)
{
    uint32_t changed = s->enable[bank] ^ value;
    s->enable[bank] = value;
    for (int i = 0; i < 32; i++) {
        int n = bank * 32 + i;
        if (!(changed & BIT(i)) || n >= ESP32S3_GPIO_COUNT) {
            continue;
        }
        if (gpio_out_drives_pad(s, n)) {
            drive_pad(s, n);
        }
        qemu_set_irq(s->dir_out[n], bank_bit(s->enable, n));
    }
}

static void clear_status(ESP32S3GPIOState *s, int bank, uint32_t mask)
{
    s->status[bank] &= ~mask;
    for (int i = 0; i < 32; i++) {
        int n = bank * 32 + i;
        if ((mask & BIT(i)) && n < ESP32S3_GPIO_COUNT && level_type_matches(s, n)) {
            s->status[bank] |= BIT(i);
        }
    }
    update_irq(s);
}

static void write_out_sel(ESP32S3GPIOState *s, int n, uint32_t value)
{
    bool drove = gpio_out_drives_pad(s, n);
    s->out_sel[n] = value;
    qemu_set_irq(s->matrix_out[n], FIELD_EX32(value, GPIO_FUNC0_OUT_SEL_CFG, OUT_SEL));
    if (!drove && gpio_out_drives_pad(s, n)) {
        drive_pad(s, n);
    }
}

static void write_pin_reg(ESP32S3GPIOState *s, int n, uint32_t value)
{
    s->pin[n] = value;
    if (level_type_matches(s, n)) {
        set_bank_bit(s->status, n, true);
    }
    update_irq(s);
}

static uint64_t esp32s3_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);

    if (addr >= A_GPIO_PIN0 && addr < A_GPIO_PIN0 + 4 * ESP32S3_GPIO_COUNT) {
        return s->pin[(addr - A_GPIO_PIN0) / 4];
    }
    if (addr >= A_GPIO_FUNC0_IN_SEL_CFG && addr < A_GPIO_FUNC0_IN_SEL_CFG + 4 * ESP32S3_GPIO_IN_SIGNALS) {
        return s->in_sel[(addr - A_GPIO_FUNC0_IN_SEL_CFG) / 4];
    }
    if (addr >= A_GPIO_FUNC0_OUT_SEL_CFG && addr < A_GPIO_FUNC0_OUT_SEL_CFG + 4 * ESP32S3_GPIO_COUNT) {
        return s->out_sel[(addr - A_GPIO_FUNC0_OUT_SEL_CFG) / 4];
    }

    switch (addr) {
    case A_GPIO_OUT:
        return s->out[0];
    case A_GPIO_OUT1:
        return s->out[1];
    case A_GPIO_ENABLE:
        return s->enable[0];
    case A_GPIO_ENABLE1:
        return s->enable[1];
    case A_GPIO_STRAP:
        return s->strap_mode;
    case A_GPIO_IN:
        qemu_irq_raise(s->sync);
        return s->in[0];
    case A_GPIO_IN1:
        qemu_irq_raise(s->sync);
        return s->in[1];
    case A_GPIO_STATUS:
        return s->status[0];
    case A_GPIO_STATUS1:
        return s->status[1];
    case A_GPIO_PCPU_INT:
        return int_status(s, 0, INT_ENA_CPU);
    case A_GPIO_PCPU_NMI_INT:
        return int_status(s, 0, INT_ENA_NMI);
    case A_GPIO_PCPU_INT1:
        return int_status(s, 1, INT_ENA_CPU);
    case A_GPIO_PCPU_NMI_INT1:
        return int_status(s, 1, INT_ENA_NMI);
    case A_GPIO_CLOCK_GATE:
        return s->clock_gate;
    case A_GPIO_DATE:
        return GPIO_DATE_VALUE;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32s3.gpio: read of unmodeled register 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_gpio_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);

    if (addr >= A_GPIO_PIN0 && addr < A_GPIO_PIN0 + 4 * ESP32S3_GPIO_COUNT) {
        write_pin_reg(s, (addr - A_GPIO_PIN0) / 4, value);
        return;
    }
    if (addr >= A_GPIO_FUNC0_IN_SEL_CFG && addr < A_GPIO_FUNC0_IN_SEL_CFG + 4 * ESP32S3_GPIO_IN_SIGNALS) {
        s->in_sel[(addr - A_GPIO_FUNC0_IN_SEL_CFG) / 4] = value;
        return;
    }
    if (addr >= A_GPIO_FUNC0_OUT_SEL_CFG && addr < A_GPIO_FUNC0_OUT_SEL_CFG + 4 * ESP32S3_GPIO_COUNT) {
        write_out_sel(s, (addr - A_GPIO_FUNC0_OUT_SEL_CFG) / 4, value);
        return;
    }

    switch (addr) {
    case A_GPIO_OUT:
        write_out(s, 0, value);
        break;
    case A_GPIO_OUT_W1TS:
        write_out(s, 0, s->out[0] | value);
        break;
    case A_GPIO_OUT_W1TC:
        write_out(s, 0, s->out[0] & ~value);
        break;
    case A_GPIO_OUT1:
        write_out(s, 1, value);
        break;
    case A_GPIO_OUT1_W1TS:
        write_out(s, 1, s->out[1] | value);
        break;
    case A_GPIO_OUT1_W1TC:
        write_out(s, 1, s->out[1] & ~value);
        break;
    case A_GPIO_ENABLE:
        write_enable(s, 0, value);
        break;
    case A_GPIO_ENABLE_W1TS:
        write_enable(s, 0, s->enable[0] | value);
        break;
    case A_GPIO_ENABLE_W1TC:
        write_enable(s, 0, s->enable[0] & ~value);
        break;
    case A_GPIO_ENABLE1:
        write_enable(s, 1, value);
        break;
    case A_GPIO_ENABLE1_W1TS:
        write_enable(s, 1, s->enable[1] | value);
        break;
    case A_GPIO_ENABLE1_W1TC:
        write_enable(s, 1, s->enable[1] & ~value);
        break;
    case A_GPIO_STRAP:
        s->strap_mode = value;
        break;
    case A_GPIO_STATUS:
        s->status[0] = value;
        update_irq(s);
        break;
    case A_GPIO_STATUS_W1TS:
        s->status[0] |= value;
        update_irq(s);
        break;
    case A_GPIO_STATUS_W1TC:
        clear_status(s, 0, value);
        break;
    case A_GPIO_STATUS1:
        s->status[1] = value;
        update_irq(s);
        break;
    case A_GPIO_STATUS1_W1TS:
        s->status[1] |= value;
        update_irq(s);
        break;
    case A_GPIO_STATUS1_W1TC:
        clear_status(s, 1, value);
        break;
    case A_GPIO_CLOCK_GATE:
        s->clock_gate = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32s3.gpio: write of unmodeled register 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps esp32s3_gpio_ops = {
    .read = esp32s3_gpio_read,
    .write = esp32s3_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_gpio_set_input(void *opaque, int n, int level)
{
    set_pad_level(ESP32S3_GPIO(opaque), n, level != 0);
}

static void esp32s3_gpio_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);

    memset(s->out, 0, sizeof(s->out));
    memset(s->enable, 0, sizeof(s->enable));
    memset(s->status, 0, sizeof(s->status));
    memset(s->pin, 0, sizeof(s->pin));
    memset(s->in_sel, 0, sizeof(s->in_sel));
    for (int n = 0; n < ESP32S3_GPIO_COUNT; n++) {
        s->out_sel[n] = ESP32S3_GPIO_SIG_GPIO_OUT;
    }
    s->clock_gate = 1;
    qemu_irq_lower(s->irq);
    qemu_irq_lower(s->nmi_irq);
}

static void esp32s3_gpio_init(Object *obj)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_gpio_ops, s, TYPE_ESP32S3_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->nmi_irq);
    qdev_init_gpio_out_named(DEVICE(s), s->level_out, ESP32S3_GPIOS, ESP32S3_GPIO_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), s->dir_out, ESP32S3_GPIOS_DIR, ESP32S3_GPIO_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), s->matrix_out, ESP32S3_GPIOS_MATRIX, ESP32S3_GPIO_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), &s->sync, ESP32S3_GPIOS_SYNC, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_gpio_set_input, ESP32S3_GPIOS_IN, ESP32S3_GPIO_COUNT);
}

static Property esp32s3_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", ESP32S3GPIOState, strap_mode, ESP32S3_STRAP_MODE_FLASH_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_gpio_reset_hold;
    device_class_set_props(dc, esp32s3_gpio_properties);
}

static const TypeInfo esp32s3_gpio_info = {
    .name = TYPE_ESP32S3_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GPIOState),
    .instance_init = esp32s3_gpio_init,
    .class_init = esp32s3_gpio_class_init,
};

static void esp32s3_gpio_register_types(void)
{
    type_register_static(&esp32s3_gpio_info);
}

type_init(esp32s3_gpio_register_types)
