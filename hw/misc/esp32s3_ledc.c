/*
 * ESP32-S3 LED PWM controller (LEDC)
 *
 * The PWM waveforms are not generated: the model keeps each timer's frequency
 * and each channel's duty in effect, and signals the board when either
 * changes. A duty update latches DUTY when PARA_UP is written with
 * DUTY_START set; fades are rejected. A timer held in reset or without a
 * clock does not count, so its channels output no PWM: they report duty 0.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32s3_ledc.h"
#include "hw/xtensa/picsimlab.h"

#define CH_STRIDE 0x14

REG32(CH_CONF0, 0x00)
    FIELD(CH_CONF0, TIMER_SEL, 0, 2)
    FIELD(CH_CONF0, PARA_UP, 4, 1)
    FIELD(CH_CONF0, OVF_CNT_EN, 15, 1)
REG32(CH_HPOINT, 0x04)
REG32(CH_DUTY, 0x08)
REG32(CH_CONF1, 0x0C)
    FIELD(CH_CONF1, DUTY_SCALE, 0, 10)
    FIELD(CH_CONF1, DUTY_START, 31, 1)
REG32(CH_DUTY_R, 0x10)
REG32(TIMER_CONF, 0xA0)
    FIELD(TIMER_CONF, DUTY_RES, 0, 4)
    FIELD(TIMER_CONF, CLK_DIV, 4, 18)
    FIELD(TIMER_CONF, PAUSE, 22, 1)
    FIELD(TIMER_CONF, RST, 23, 1)
    FIELD(TIMER_CONF, TICK_SEL, 24, 1)
    FIELD(TIMER_CONF, PARA_UP, 25, 1)
REG32(TIMER_VALUE, 0xA4)
REG32(INT_RAW, 0xC0)
REG32(INT_ST, 0xC4)
REG32(INT_ENA, 0xC8)
REG32(INT_CLR, 0xCC)
REG32(CONF, 0xD0)
    FIELD(CONF, APB_CLK_SEL, 0, 2)
REG32(DATE, 0xFC)

#define LEDC_DATE_VALUE 0x19040200
#define INT_DUTY_CHNG_END(ch) BIT(4 + (ch))
#define INT_DUTY_CHNG_END_ALL (0xFF << 4)

QEMU_BUILD_BUG_ON(offsetof(ESP32S3LEDCState, timer_conf) !=
                  offsetof(ESP32S3LEDCState, timer_freq) + sizeof(uint32_t) * ESP32S3_LEDC_TIMERS);

static void update_irq(ESP32S3LEDCState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

/* Timer clock in Hz; 0 when no clock is selected */
static uint64_t timer_clock_hz(ESP32S3LEDCState *s, int timer)
{
    if (FIELD_EX32(s->timer_conf[timer], TIMER_CONF, TICK_SEL)) {
        return 1000000;     /* REF_TICK */
    }
    switch (FIELD_EX32(s->conf, CONF, APB_CLK_SEL)) {
    case 0:
        return 0;
    case 1:
        return 80000000;    /* APB */
    case 2:
        return 0;           /* RC_FAST: rejected when a timer is updated on it */
    case 3:
        return 40000000;    /* XTAL */
    default:
        g_assert_not_reached();
    }
}

static void compute_timer_freq(ESP32S3LEDCState *s, int timer)
{
    uint32_t conf = s->timer_conf[timer];
    /* CLK_DIV has 8 fractional bits: f = clk * 256 / (CLK_DIV * 2^DUTY_RES) */
    uint64_t div = (uint64_t)FIELD_EX32(conf, TIMER_CONF, CLK_DIV) << FIELD_EX32(conf, TIMER_CONF, DUTY_RES);
    uint64_t clk = timer_clock_hz(s, timer);

    s->timer_freq[timer] = div ? (clk * 256 + div / 2) / div : 0;
}

static void compute_duty_pct(ESP32S3LEDCState *s, int ch)
{
    int timer = FIELD_EX32(s->ch_conf0[ch], CH_CONF0, TIMER_SEL);
    uint32_t res = FIELD_EX32(s->timer_conf[timer], TIMER_CONF, DUTY_RES);

    if (FIELD_EX32(s->timer_conf[timer], TIMER_CONF, RST) || s->timer_freq[timer] == 0) {
        s->duty_pct[ch] = 0;
        return;
    }
    s->duty_pct[ch] = (float)((double)s->ch_duty_r[ch] / 16.0 / (double)(1u << res) * 100.0);
}

static void update_channel(ESP32S3LEDCState *s, int ch)
{
    if (FIELD_EX32(s->ch_conf1[ch], CH_CONF1, DUTY_START)) {
        if (FIELD_EX32(s->ch_conf1[ch], CH_CONF1, DUTY_SCALE) != 0) {
            picsimlab_reject("esp32s3_ledc: channel %d: duty fades are not modeled", ch);
            return;
        }
        s->ch_duty_r[ch] = s->ch_duty[ch];
        s->ch_conf1[ch] = FIELD_DP32(s->ch_conf1[ch], CH_CONF1, DUTY_START, 0);
        s->int_raw |= INT_DUTY_CHNG_END(ch);
    }
    compute_duty_pct(s, ch);
    qemu_irq_raise(s->update[ch]);
}

static void refresh_timer(ESP32S3LEDCState *s, int timer)
{
    compute_timer_freq(s, timer);
    for (int ch = 0; ch < ESP32S3_LEDC_CHANNELS; ch++) {
        if (FIELD_EX32(s->ch_conf0[ch], CH_CONF0, TIMER_SEL) == timer) {
            compute_duty_pct(s, ch);
            qemu_irq_raise(s->update[ch]);
        }
    }
}

static void update_timer(ESP32S3LEDCState *s, int timer)
{
    if (!FIELD_EX32(s->timer_conf[timer], TIMER_CONF, TICK_SEL) &&
        FIELD_EX32(s->conf, CONF, APB_CLK_SEL) == 2) {
        picsimlab_reject("esp32s3_ledc: timer %d: the RC_FAST clock is not modeled", timer);
        return;
    }
    refresh_timer(s, timer);
}

static uint64_t esp32s3_ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(opaque);

    if (addr < A_TIMER_CONF) {
        int ch = addr / CH_STRIDE;
        switch (addr % CH_STRIDE) {
        case A_CH_CONF0:
            return s->ch_conf0[ch];
        case A_CH_HPOINT:
            return s->ch_hpoint[ch];
        case A_CH_DUTY:
            return s->ch_duty[ch];
        case A_CH_CONF1:
            return s->ch_conf1[ch];
        case A_CH_DUTY_R:
            return s->ch_duty_r[ch];
        default:
            g_assert_not_reached();
        }
    }
    if (addr < A_INT_RAW) {
        int timer = (addr - A_TIMER_CONF) / 8;
        if ((addr - A_TIMER_CONF) % 8 == 0) {
            return s->timer_conf[timer];
        }
        picsimlab_reject("esp32s3_ledc: reading timer %d's counter is not modeled", timer);
        return 0;
    }
    switch (addr) {
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_CONF:
        return s->conf;
    case A_DATE:
        return LEDC_DATE_VALUE;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32s3_ledc: read of unknown register 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_ledc_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(opaque);

    if (addr < A_TIMER_CONF) {
        int ch = addr / CH_STRIDE;
        switch (addr % CH_STRIDE) {
        case A_CH_CONF0:
            if (FIELD_EX32(value, CH_CONF0, OVF_CNT_EN)) {
                picsimlab_reject("esp32s3_ledc: channel %d: overflow counting is not modeled", ch);
                return;
            }
            s->ch_conf0[ch] = FIELD_DP32(value, CH_CONF0, PARA_UP, 0);
            if (FIELD_EX32(value, CH_CONF0, PARA_UP)) {
                update_channel(s, ch);
            }
            break;
        case A_CH_HPOINT:
            s->ch_hpoint[ch] = value;
            break;
        case A_CH_DUTY:
            s->ch_duty[ch] = value & 0x7FFFF;
            break;
        case A_CH_CONF1:
            s->ch_conf1[ch] = value;
            break;
        case A_CH_DUTY_R:
            break;
        default:
            g_assert_not_reached();
        }
        update_irq(s);
        return;
    }
    if (addr < A_INT_RAW) {
        int timer = (addr - A_TIMER_CONF) / 8;
        if ((addr - A_TIMER_CONF) % 8 == 0) {
            if (FIELD_EX32(value, TIMER_CONF, PAUSE)) {
                picsimlab_reject("esp32s3_ledc: timer %d: pausing is not modeled", timer);
                return;
            }
            bool rst_changed = FIELD_EX32(value ^ s->timer_conf[timer], TIMER_CONF, RST);

            s->timer_conf[timer] = FIELD_DP32(value, TIMER_CONF, PARA_UP, 0);
            if (FIELD_EX32(value, TIMER_CONF, PARA_UP)) {
                update_timer(s, timer);
            } else if (rst_changed) {
                refresh_timer(s, timer);
            }
        }
        return;
    }
    switch (addr) {
    case A_INT_ENA:
        if (value & ~INT_DUTY_CHNG_END_ALL) {
            picsimlab_reject("esp32s3_ledc: interrupts 0x%" PRIx64 " are not modeled", value & ~INT_DUTY_CHNG_END_ALL);
            return;
        }
        s->int_ena = value;
        break;
    case A_INT_CLR:
        s->int_raw &= ~value;
        break;
    case A_CONF:
        s->conf = value;
        for (int timer = 0; timer < ESP32S3_LEDC_TIMERS; timer++) {
            uint32_t old_freq = s->timer_freq[timer];
            compute_timer_freq(s, timer);
            if (s->timer_freq[timer] != old_freq) {
                refresh_timer(s, timer);
            }
        }
        break;
    case A_INT_RAW:
    case A_INT_ST:
    case A_DATE:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32s3_ledc: write to unknown register 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    update_irq(s);
}

static const MemoryRegionOps esp32s3_ledc_ops = {
    .read = esp32s3_ledc_read,
    .write = esp32s3_ledc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_ledc_reset_hold(Object *obj, ResetType type)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(obj);

    memset(s->ch_conf0, 0, sizeof(s->ch_conf0));
    memset(s->ch_hpoint, 0, sizeof(s->ch_hpoint));
    memset(s->ch_duty, 0, sizeof(s->ch_duty));
    memset(s->ch_conf1, 0, sizeof(s->ch_conf1));
    memset(s->ch_duty_r, 0, sizeof(s->ch_duty_r));
    memset(s->timer_freq, 0, sizeof(s->timer_freq));
    memset(s->duty_pct, 0, sizeof(s->duty_pct));
    for (int timer = 0; timer < ESP32S3_LEDC_TIMERS; timer++) {
        s->timer_conf[timer] = R_TIMER_CONF_RST_MASK;
    }
    s->int_raw = 0;
    s->int_ena = 0;
    s->conf = 0;
    update_irq(s);
}

static void esp32s3_ledc_init(Object *obj)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_ledc_ops, s, TYPE_ESP32S3_LEDC, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->update, ESP32S3_LEDC_UPDATE, ESP32S3_LEDC_CHANNELS);
}

static void esp32s3_ledc_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_ledc_reset_hold;
}

static const TypeInfo esp32s3_ledc_info = {
    .name = TYPE_ESP32S3_LEDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3LEDCState),
    .instance_init = esp32s3_ledc_init,
    .class_init = esp32s3_ledc_class_init,
};

static void esp32s3_ledc_register_types(void)
{
    type_register_static(&esp32s3_ledc_info);
}

type_init(esp32s3_ledc_register_types)
