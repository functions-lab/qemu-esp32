/*
 * ESP32-S3 SENS: the SAR ADCs' RTC controller
 *
 * A one-shot conversion completes as soon as MEASn_START_SAR rises. The ADC is
 * linear: a pin voltage V reads V * 4095 / full_scale(atten), saturating at
 * 4095, with the full scales the worker's eFuse calibration image assumes.
 * The temperature sensor reads a fixed room temperature as soon as it is
 * dumped. Other SENS registers hold what was written.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32s3_sens.h"

REG32(SAR_READER1_CTRL, 0x00)
    FIELD(SAR_READER1_CTRL, SAR1_DATA_INV, 28, 1)
REG32(SAR_MEAS1_CTRL2, 0x0C)
REG32(SAR_ATTEN1, 0x14)
REG32(SAR_READER2_CTRL, 0x24)
    FIELD(SAR_READER2_CTRL, SAR2_DATA_INV, 29, 1)
REG32(SAR_MEAS2_CTRL2, 0x30)
REG32(SAR_ATTEN2, 0x38)
REG32(SAR_SLAVE_ADDR1, 0x40)
    FIELD(SAR_SLAVE_ADDR1, MEAS_STATUS, 22, 8)
REG32(SAR_TSENS_CTRL, 0x50)
    FIELD(SAR_TSENS_CTRL, OUT, 0, 8)
    FIELD(SAR_TSENS_CTRL, READY, 8, 1)
    FIELD(SAR_TSENS_CTRL, DUMP_OUT, 24, 1)
REG32(SARDATE, 0x1FC)

/* MEAS1_CTRL2 and MEAS2_CTRL2 share one layout */
FIELD(SAR_MEAS_CTRL2, DATA, 0, 16)
FIELD(SAR_MEAS_CTRL2, DONE, 16, 1)
FIELD(SAR_MEAS_CTRL2, START, 17, 1)
FIELD(SAR_MEAS_CTRL2, EN_PAD, 19, 12)

#define SENS_DATE_VALUE 0x2101180
/* About 25 C with IDF's conversion 0.4386 * raw - 20.52 */
#define TSENS_ROOM_CODE 104

/* Full-scale input in mV per attenuation: 0, 2.5, 6 and 11 dB */
static const uint32_t full_scale_mv[4] = { 930, 1240, 1856, 3300 };

static uint16_t input_code(ESP32S3SensState *s, int input)
{
    ESP32S3AdcWaveform *w = &s->waveform[input];

    if (w->count == 0) {
        return s->input[input];
    }
    if (w->count == 1) {
        return w->samples[0];
    }
    /* Linear interpolation on the sample grid, phased from virtual time 0 */
    uint64_t t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) % w->period_ns;
    uint64_t steps = w->count - 1;
    uint64_t pos = t * steps;
    uint64_t i = pos / w->period_ns;
    uint64_t frac = pos % w->period_ns;
    int64_t a = w->samples[i];
    int64_t b = w->samples[i + 1];
    return a + (b - a) * (int64_t)frac / (int64_t)w->period_ns;
}

static void convert(ESP32S3SensState *s, int unit)
{
    const hwaddr ctrl2 = unit == 1 ? A_SAR_MEAS1_CTRL2 : A_SAR_MEAS2_CTRL2;
    const uint32_t atten_reg = s->regs[(unit == 1 ? A_SAR_ATTEN1 : A_SAR_ATTEN2) / 4];
    uint32_t r = s->regs[ctrl2 / 4];
    uint32_t pads = FIELD_EX32(r, SAR_MEAS_CTRL2, EN_PAD);
    bool invert;

    if (pads == 0 || (pads & (pads - 1)) != 0) {
        error_report("esp32s3_sens: ADC%d conversion with pad map 0x%x; one channel at a time is modeled",
                     unit, pads);
        abort();
    }
    int channel = ctz32(pads);
    if (channel >= ESP32S3_ADC_UNIT_CHANNELS) {
        error_report("esp32s3_sens: ADC%d channel %d is not modeled", unit, channel);
        abort();
    }

    int input = (unit == 1 ? 0 : ESP32S3_ADC_UNIT_CHANNELS) + channel;
    uint32_t atten = (atten_reg >> (channel * 2)) & 3;
    uint32_t code = MIN(4095, input_code(s, input) * 3300 / full_scale_mv[atten]);

    if (unit == 1) {
        invert = FIELD_EX32(s->regs[R_SAR_READER1_CTRL], SAR_READER1_CTRL, SAR1_DATA_INV);
    } else {
        invert = FIELD_EX32(s->regs[R_SAR_READER2_CTRL], SAR_READER2_CTRL, SAR2_DATA_INV);
    }
    if (invert) {
        code = ~code & 0xFFF;
    }

    r = FIELD_DP32(r, SAR_MEAS_CTRL2, DATA, code);
    s->regs[ctrl2 / 4] = FIELD_DP32(r, SAR_MEAS_CTRL2, DONE, 1);
}

static void write_meas_ctrl2(ESP32S3SensState *s, int unit, hwaddr addr, uint32_t value)
{
    uint32_t old = s->regs[addr / 4];
    /* DATA and DONE are read-only */
    uint32_t keep = R_SAR_MEAS_CTRL2_DATA_MASK | R_SAR_MEAS_CTRL2_DONE_MASK;

    s->regs[addr / 4] = (value & ~keep) | (old & keep);
    if (!FIELD_EX32(value, SAR_MEAS_CTRL2, START)) {
        s->regs[addr / 4] = FIELD_DP32(s->regs[addr / 4], SAR_MEAS_CTRL2, DONE, 0);
    } else if (!FIELD_EX32(old, SAR_MEAS_CTRL2, START)) {
        convert(s, unit);
    }
}

static uint64_t esp32s3_sens_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3SensState *s = ESP32S3_SENS(opaque);

    switch (addr) {
    case A_SAR_SLAVE_ADDR1:
        /* The controller is never busy */
        return FIELD_DP32(s->regs[R_SAR_SLAVE_ADDR1], SAR_SLAVE_ADDR1, MEAS_STATUS, 0);
    case A_SAR_TSENS_CTRL:
        if (FIELD_EX32(s->regs[R_SAR_TSENS_CTRL], SAR_TSENS_CTRL, DUMP_OUT)) {
            uint32_t r = FIELD_DP32(s->regs[R_SAR_TSENS_CTRL], SAR_TSENS_CTRL, READY, 1);
            return FIELD_DP32(r, SAR_TSENS_CTRL, OUT, TSENS_ROOM_CODE);
        }
        return s->regs[R_SAR_TSENS_CTRL];
    case A_SARDATE:
        return SENS_DATE_VALUE;
    default:
        return s->regs[addr / 4];
    }
}

static void esp32s3_sens_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3SensState *s = ESP32S3_SENS(opaque);

    switch (addr) {
    case A_SAR_MEAS1_CTRL2:
        write_meas_ctrl2(s, 1, addr, value);
        break;
    case A_SAR_MEAS2_CTRL2:
        write_meas_ctrl2(s, 2, addr, value);
        break;
    case A_SAR_TSENS_CTRL:
        s->regs[R_SAR_TSENS_CTRL] = value & ~(R_SAR_TSENS_CTRL_OUT_MASK | R_SAR_TSENS_CTRL_READY_MASK);
        break;
    case A_SARDATE:
        break;
    default:
        s->regs[addr / 4] = value;
        break;
    }
}

static const MemoryRegionOps esp32s3_sens_ops = {
    .read = esp32s3_sens_read,
    .write = esp32s3_sens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

void esp32s3_sens_set_input(ESP32S3SensState *s, int input, uint16_t code)
{
    if (input < 0 || input >= ESP32S3_ADC_INPUTS || code > 4095) {
        error_report("esp32s3_sens: no analog input %d for code %u", input, code);
        abort();
    }
    s->input[input] = code;
}

void esp32s3_sens_set_waveform(ESP32S3SensState *s, int input, const uint16_t *samples, int count,
                               uint64_t period_ns)
{
    ESP32S3AdcWaveform *w;

    if (input < 0 || input >= ESP32S3_ADC_INPUTS || count < 0 || (count > 0 && period_ns == 0)) {
        error_report("esp32s3_sens: invalid waveform for analog input %d", input);
        abort();
    }
    for (int i = 0; i < count; i++) {
        if (samples[i] > 4095) {
            error_report("esp32s3_sens: waveform sample %u is not a 12-bit code", samples[i]);
            abort();
        }
    }
    w = &s->waveform[input];
    g_free(w->samples);
    w->samples = count ? g_memdup2(samples, count * sizeof(uint16_t)) : NULL;
    w->count = count;
    w->period_ns = period_ns;
}

static void esp32s3_sens_reset_hold(Object *obj, ResetType type)
{
    ESP32S3SensState *s = ESP32S3_SENS(obj);

    memset(s->regs, 0, sizeof(s->regs));
    /* SAR1_INT_EN, SAR1_CLK_GATED, SAR1_CLK_DIV = 2 */
    s->regs[R_SAR_READER1_CTRL] = 0x20040002;
    /* SAR2_INT_EN, SAR2_CLK_GATED, SAR2_WAIT_ARB_CYCLE = 1, SAR2_CLK_DIV = 2 */
    s->regs[R_SAR_READER2_CTRL] = 0x40050002;
    s->regs[R_SAR_ATTEN1] = 0xFFFFFFFF;
    s->regs[R_SAR_ATTEN2] = 0xFFFFFFFF;
}

static void esp32s3_sens_init(Object *obj)
{
    ESP32S3SensState *s = ESP32S3_SENS(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_sens_ops, s, TYPE_ESP32S3_SENS, ESP32S3_SENS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void esp32s3_sens_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_sens_reset_hold;
}

static const TypeInfo esp32s3_sens_info = {
    .name = TYPE_ESP32S3_SENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3SensState),
    .instance_init = esp32s3_sens_init,
    .class_init = esp32s3_sens_class_init,
};

static void esp32s3_sens_register_types(void)
{
    type_register_static(&esp32s3_sens_info);
}

type_init(esp32s3_sens_register_types)
