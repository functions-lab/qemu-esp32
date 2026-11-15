/*
 * ESP32-S3 general-purpose SPI controller (GPSPI2), master mode
 *
 * A transaction runs to completion when CMD.USR is written: bus timing is not
 * modeled. Each byte goes to the host's picsimlab SPI bridge, which answers
 * the MISO byte; write-only transactions go to the host in one batch. Chip
 * selects are reported as lane levels on the CS outputs.
 *
 * Modeled: single-line modes 0-3, whose clock edges leave the bytes as they
 * are, and 3-wire half duplex; MSB or LSB first, which the host sees as the
 * bit-reversed byte; command / address / dummy phases of whole bytes, full and
 * half duplex, data through W0-W15 (or W8-W15 alone) or GDMA, and the
 * TRANS_DONE interrupt. Anything else is rejected.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/ssi/esp32s3_gpspi.h"
#include "hw/xtensa/picsimlab.h"

REG32(CMD, 0x00)
    FIELD(CMD, UPDATE, 23, 1)
    FIELD(CMD, USR, 24, 1)
REG32(ADDR, 0x04)
REG32(CTRL, 0x08)
    FIELD(CTRL, LINES, 5, 12)           /* FADDR_*, FCMD_*, FREAD_* */
    FIELD(CTRL, RD_BIT_ORDER, 23, 2)
    FIELD(CTRL, WR_BIT_ORDER, 25, 2)
REG32(CLOCK, 0x0C)
REG32(USER, 0x10)
    FIELD(USER, DOUTDIN, 0, 1)
    FIELD(USER, QPI_MODE, 3, 1)
    FIELD(USER, OPI_MODE, 4, 1)
    FIELD(USER, CK_OUT_EDGE, 9, 1)
    FIELD(USER, FWRITE_LINES, 12, 3)
    FIELD(USER, USR_CONF_NXT, 15, 1)
    FIELD(USER, SIO, 17, 1)
    FIELD(USER, USR_MISO_HIGHPART, 24, 1)
    FIELD(USER, USR_MOSI_HIGHPART, 25, 1)
    FIELD(USER, USR_MOSI, 27, 1)
    FIELD(USER, USR_MISO, 28, 1)
    FIELD(USER, USR_DUMMY, 29, 1)
    FIELD(USER, USR_ADDR, 30, 1)
    FIELD(USER, USR_COMMAND, 31, 1)
REG32(USER1, 0x14)
    FIELD(USER1, USR_DUMMY_CYCLELEN, 0, 8)
    FIELD(USER1, USR_ADDR_BITLEN, 27, 5)
REG32(USER2, 0x18)
    FIELD(USER2, USR_COMMAND_VALUE, 0, 16)
    FIELD(USER2, USR_COMMAND_BITLEN, 28, 4)
REG32(MS_DLEN, 0x1C)
    FIELD(MS_DLEN, MS_DATA_BITLEN, 0, 18)
REG32(MISC, 0x20)
    FIELD(MISC, CS_DIS, 0, 6)
    FIELD(MISC, MASTER_CS_POL, 7, 6)
    FIELD(MISC, CK_IDLE_EDGE, 29, 1)
    FIELD(MISC, CS_KEEP_ACTIVE, 30, 1)
REG32(DMA_CONF, 0x30)
    FIELD(DMA_CONF, DMA_SEG_TRANS_EN, 18, 1)
    FIELD(DMA_CONF, DMA_RX_ENA, 27, 1)
    FIELD(DMA_CONF, DMA_TX_ENA, 28, 1)
    FIELD(DMA_CONF, RX_AFIFO_RST, 29, 1)
    FIELD(DMA_CONF, BUF_AFIFO_RST, 30, 1)
    FIELD(DMA_CONF, DMA_AFIFO_RST, 31, 1)
REG32(DMA_INT_ENA, 0x34)
REG32(DMA_INT_CLR, 0x38)
REG32(DMA_INT_RAW, 0x3C)
    FIELD(DMA_INT, TRANS_DONE, 12, 1)
REG32(DMA_INT_ST, 0x40)
REG32(DMA_INT_SET, 0x44)
REG32(W0, 0x98)
REG32(SLAVE, 0xE0)
    FIELD(SLAVE, SLAVE_MODE, 26, 1)
REG32(CLK_GATE, 0xE8)
REG32(DATE, 0xF0)

#define GPSPI_DATE_VALUE    0x2101040
#define GPSPI_BUF_BYTES     64
/* USR_MOSI_HIGHPART / USR_MISO_HIGHPART: data in W8-W15 only */
#define GPSPI_HIGHPART      32
#define GPSPI_INT_MASK      0x1FFFFF
#define MOSI_IDLE           0xFF

static void update_irq(ESP32S3GpspiState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_DMA_INT_RAW] & s->regs[R_DMA_INT_ENA]) != 0);
}

static void reject(ESP32S3GpspiState *s, const char *what)
{
    picsimlab_reject("esp32s3_gpspi%d: %s is not modeled", s->bus, what);
}

static void set_cs(ESP32S3GpspiState *s, int lane, bool active)
{
    bool active_high = (FIELD_EX32(s->regs[R_MISC], MISC, MASTER_CS_POL) >> lane) & 1;

    qemu_set_irq(s->cs[lane], active == active_high);
}

/* CS0-CS2 lanes the transaction drives; CS3-CS5 reach no host-visible signal */
static uint8_t selected_lanes(ESP32S3GpspiState *s)
{
    return ~FIELD_EX32(s->regs[R_MISC], MISC, CS_DIS) & ((1 << ESP32S3_GPSPI_CS_COUNT) - 1);
}

static void set_lanes(ESP32S3GpspiState *s, uint8_t lanes, bool active)
{
    for (int lane = 0; lane < ESP32S3_GPSPI_CS_COUNT; lane++) {
        if (lanes & (1 << lane)) {
            set_cs(s, lane, active);
        }
    }
}

/* Returns false when the configuration was rejected */
static bool check_supported(ESP32S3GpspiState *s)
{
    uint32_t user = s->regs[R_USER];

    if (FIELD_EX32(s->regs[R_SLAVE], SLAVE, SLAVE_MODE)) {
        reject(s, "slave mode");
        return false;
    }
    if (FIELD_EX32(s->regs[R_DMA_CONF], DMA_CONF, DMA_SEG_TRANS_EN) || FIELD_EX32(user, USER, USR_CONF_NXT)) {
        reject(s, "segmented transfer");
        return false;
    }
    if (FIELD_EX32(s->regs[R_CTRL], CTRL, RD_BIT_ORDER) > 1 || FIELD_EX32(s->regs[R_CTRL], CTRL, WR_BIT_ORDER) > 1) {
        reject(s, "a reserved bit order");
        return false;
    }
    if (FIELD_EX32(s->regs[R_CTRL], CTRL, LINES) || FIELD_EX32(user, USER, FWRITE_LINES) ||
        FIELD_EX32(user, USER, QPI_MODE) || FIELD_EX32(user, USER, OPI_MODE)) {
        reject(s, "dual, quad or octal line mode");
        return false;
    }
    if (FIELD_EX32(user, USER, SIO) && FIELD_EX32(user, USER, DOUTDIN)) {
        reject(s, "full duplex in 3-wire (SIO) mode");
        return false;
    }
    return true;
}

/* Returns false when the phase was rejected */
static bool whole_bytes(ESP32S3GpspiState *s, const char *phase, uint32_t bits, uint32_t *bytes)
{
    if (bits % 8) {
        picsimlab_reject("esp32s3_gpspi%d: %s phase of %u bits is not modeled; whole bytes are", s->bus, phase, bits);
        return false;
    }
    *bytes = bits / 8;
    return true;
}

/* Data buffer of a CPU transfer: W0-W15, or W8-W15 for the high part */
static bool buf_window(ESP32S3GpspiState *s, bool highpart, uint32_t len, uint32_t *base)
{
    *base = highpart ? GPSPI_HIGHPART : 0;
    if (len > GPSPI_BUF_BYTES - *base) {
        reject(s, highpart ? "CPU-buffer transfer over 32 bytes in W8-W15" : "CPU-buffer transfer over 64 bytes");
        return false;
    }
    return true;
}

static uint8_t buf_byte(ESP32S3GpspiState *s, uint32_t i)
{
    return s->regs[R_W0 + i / 4] >> ((i % 4) * 8);
}

static void set_buf_byte(ESP32S3GpspiState *s, uint32_t i, uint8_t byte)
{
    uint32_t *w = &s->regs[R_W0 + i / 4];

    *w = (*w & ~(0xFFu << ((i % 4) * 8))) | ((uint32_t)byte << ((i % 4) * 8));
}

/* The GDMA channel bound to SPI2 in one direction, if its link was started */
static bool gdma_channel(ESP32S3GpspiState *s, int dir, uint32_t *chan)
{
    const uint32_t start = dir == ESP_GDMA_IN_IDX ? R_GDMA_IN_LINK_START_MASK : R_GDMA_OUT_LINK_START_MASK;

    for (uint32_t i = 0; i < ESP_GDMA_GET_CLASS(s->gdma)->m_channel_count; i++) {
        if (FIELD_EX32(s->gdma->ch_conf[dir][i].peripheral, GDMA_PERI_SEL, PERI_SEL) == GDMA_SPI2 &&
            (s->gdma->ch_conf[dir][i].link & start)) {
            *chan = i;
            return true;
        }
    }
    return false;
}

/* Returns false when the transfer was rejected */
static bool fetch_mosi(ESP32S3GpspiState *s, uint8_t *data, uint32_t len)
{
    uint32_t chan, base;

    if (!FIELD_EX32(s->regs[R_DMA_CONF], DMA_CONF, DMA_TX_ENA)) {
        if (!buf_window(s, FIELD_EX32(s->regs[R_USER], USER, USR_MOSI_HIGHPART), len, &base)) {
            return false;
        }
        for (uint32_t i = 0; i < len; i++) {
            data[i] = buf_byte(s, base + i);
        }
    } else if (gdma_channel(s, ESP_GDMA_OUT_IDX, &chan)) {
        if (!esp_gdma_read_channel(s->gdma, chan, data, len)) {
            picsimlab_reject("esp32s3_gpspi%d: GDMA out channel %u could not supply %u bytes", s->bus, chan, len);
            return false;
        }
    } else {
        /* MOSI enabled with no data queued, as in a full-duplex read */
        memset(data, MOSI_IDLE, len);
    }
    return true;
}

static void store_miso(ESP32S3GpspiState *s, uint8_t *data, uint32_t len)
{
    uint32_t chan, base;

    if (!FIELD_EX32(s->regs[R_DMA_CONF], DMA_CONF, DMA_RX_ENA)) {
        if (!buf_window(s, FIELD_EX32(s->regs[R_USER], USER, USR_MISO_HIGHPART), len, &base)) {
            return;
        }
        for (uint32_t i = 0; i < len; i++) {
            set_buf_byte(s, base + i, data[i]);
        }
    } else if (!gdma_channel(s, ESP_GDMA_IN_IDX, &chan) || esp_gdma_write_channel(s->gdma, chan, data, len) < 0) {
        picsimlab_reject("esp32s3_gpspi%d: no GDMA in channel took %u bytes", s->bus, len);
    }
}

static void run_transaction(ESP32S3GpspiState *s)
{
    uint32_t user = s->regs[R_USER];
    bool full_duplex = FIELD_EX32(user, USER, DOUTDIN);
    bool mosi = FIELD_EX32(user, USER, USR_MOSI);
    bool miso = FIELD_EX32(user, USER, USR_MISO);
    uint32_t cmd_len = 0, addr_len = 0, dummy_len = 0;
    uint32_t data_len = 0;

    bool lsb_out = FIELD_EX32(s->regs[R_CTRL], CTRL, WR_BIT_ORDER);
    bool lsb_in = FIELD_EX32(s->regs[R_CTRL], CTRL, RD_BIT_ORDER);

    if (!check_supported(s)) {
        return;
    }
    if (FIELD_EX32(user, USER, USR_COMMAND) &&
        !whole_bytes(s, "command", FIELD_EX32(s->regs[R_USER2], USER2, USR_COMMAND_BITLEN) + 1, &cmd_len)) {
        return;
    }
    if (FIELD_EX32(user, USER, USR_ADDR) &&
        !whole_bytes(s, "address", FIELD_EX32(s->regs[R_USER1], USER1, USR_ADDR_BITLEN) + 1, &addr_len)) {
        return;
    }
    if (FIELD_EX32(user, USER, USR_DUMMY) &&
        !whole_bytes(s, "dummy", FIELD_EX32(s->regs[R_USER1], USER1, USR_DUMMY_CYCLELEN) + 1, &dummy_len)) {
        return;
    }
    if ((mosi || miso) && !whole_bytes(s, "data", FIELD_EX32(s->regs[R_MS_DLEN], MS_DLEN, MS_DATA_BITLEN) + 1,
                                       &data_len)) {
        return;
    }

    /*
     * MOSI stream: command, address, dummy, then data. Full duplex has one
     * data phase that also captures MISO; half duplex sends DOUT, then DIN.
     */
    uint32_t dout_len = (mosi || (full_duplex && miso)) ? data_len : 0;
    uint32_t din_len = (miso && !full_duplex) ? data_len : 0;
    uint32_t head_len = cmd_len + addr_len + dummy_len;
    uint32_t total = head_len + dout_len + din_len;
    g_autofree uint8_t *out = g_malloc(total ? total : 1);
    g_autofree uint8_t *in = g_malloc(data_len ? data_len : 1);
    uint32_t pos = 0;

    /* The command value goes out low byte first, each byte MSB first */
    for (uint32_t i = 0; i < cmd_len; i++) {
        out[pos++] = FIELD_EX32(s->regs[R_USER2], USER2, USR_COMMAND_VALUE) >> (8 * i);
    }
    /* The address goes out from ADDR bit 31 down */
    for (uint32_t i = 0; i < addr_len; i++) {
        out[pos++] = s->regs[R_ADDR] >> (24 - 8 * i);
    }
    memset(out + pos, MOSI_IDLE, dummy_len);
    pos += dummy_len;
    if (mosi) {
        if (!fetch_mosi(s, out + pos, dout_len)) {
            return;
        }
    } else {
        memset(out + pos, MOSI_IDLE, dout_len);
    }
    pos += dout_len;
    memset(out + pos, MOSI_IDLE, din_len);
    /* An LSB-first byte is, to an MSB-first slave, the byte bit-reversed */
    if (lsb_out) {
        for (uint32_t i = 0; i < total; i++) {
            out[i] = revbit8(out[i]);
        }
    }

    uint8_t lanes = selected_lanes(s);
    set_lanes(s, s->cs_held & ~lanes, false);
    set_lanes(s, lanes & ~s->cs_held, true);

    if (!miso) {
        if (total) {
            picsimlab_spi_event_batch(s->bus, out, total);
        }
    } else {
        uint32_t capture_at = full_duplex ? head_len : head_len + dout_len;
        for (uint32_t i = 0; i < total; i++) {
            uint8_t answer = picsimlab_spi_event(s->bus, out[i] << 8);
            if (i >= capture_at && i < capture_at + data_len) {
                in[i - capture_at] = lsb_in ? revbit8(answer) : answer;
            }
        }
        store_miso(s, in, data_len);
    }

    if (FIELD_EX32(s->regs[R_MISC], MISC, CS_KEEP_ACTIVE)) {
        s->cs_held = lanes;
    } else {
        set_lanes(s, lanes, false);
        s->cs_held = 0;
    }
    s->regs[R_DMA_INT_RAW] |= R_DMA_INT_TRANS_DONE_MASK;
}

static uint64_t esp32s3_gpspi_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(opaque);

    switch (addr) {
    case A_DMA_INT_ST:
        return s->regs[R_DMA_INT_RAW] & s->regs[R_DMA_INT_ENA];
    case A_DMA_INT_CLR:
    case A_DMA_INT_SET:
        return 0;
    case A_DATE:
        return GPSPI_DATE_VALUE;
    default:
        return s->regs[addr / 4];
    }
}

static void esp32s3_gpspi_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(opaque);

    switch (addr) {
    case A_CMD:
        /* UPDATE and USR complete at once and read back 0 */
        s->regs[R_CMD] = value & ~(R_CMD_UPDATE_MASK | R_CMD_USR_MASK);
        if (FIELD_EX32(value, CMD, USR)) {
            run_transaction(s);
        }
        break;
    case A_MISC:
        s->regs[R_MISC] = value;
        if (s->cs_held && !FIELD_EX32(value, MISC, CS_KEEP_ACTIVE)) {
            /* Clearing CS_KEEP_ACTIVE outside a transaction releases a held CS */
            set_lanes(s, s->cs_held, false);
            s->cs_held = 0;
        }
        break;
    case A_DMA_CONF:
        /* The FIFO resets act once */
        s->regs[R_DMA_CONF] = value & ~(R_DMA_CONF_RX_AFIFO_RST_MASK | R_DMA_CONF_BUF_AFIFO_RST_MASK |
                                        R_DMA_CONF_DMA_AFIFO_RST_MASK);
        break;
    case A_DMA_INT_CLR:
        s->regs[R_DMA_INT_RAW] &= ~value;
        break;
    case A_DMA_INT_SET:
        s->regs[R_DMA_INT_RAW] |= value & GPSPI_INT_MASK;
        break;
    case A_DMA_INT_RAW:
    case A_DMA_INT_ST:
    case A_DATE:
        break;
    default:
        s->regs[addr / 4] = value;
        break;
    }
    update_irq(s);
}

static const MemoryRegionOps esp32s3_gpspi_ops = {
    .read = esp32s3_gpspi_read,
    .write = esp32s3_gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_gpspi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(obj);

    memset(s->regs, 0, sizeof(s->regs));
    /* CS1-CS5 disabled */
    s->regs[R_MISC] = 0x3E;
    s->cs_held = 0;
    update_irq(s);
}

static void esp32s3_gpspi_init(Object *obj)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_gpspi_ops, s, TYPE_ESP32S3_GPSPI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->cs, ESP32S3_GPSPI_CS, ESP32S3_GPSPI_CS_COUNT);
}

static void esp32s3_gpspi_realize(DeviceState *dev, Error **errp)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(dev);

    if (s->gdma == NULL) {
        error_setg(errp, "esp32s3_gpspi: no GDMA attached");
    }
}

static Property esp32s3_gpspi_properties[] = {
    DEFINE_PROP_UINT8("bus", ESP32S3GpspiState, bus, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_gpspi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32s3_gpspi_realize;
    rc->phases.hold = esp32s3_gpspi_reset_hold;
    device_class_set_props(dc, esp32s3_gpspi_properties);
}

static const TypeInfo esp32s3_gpspi_info = {
    .name = TYPE_ESP32S3_GPSPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GpspiState),
    .instance_init = esp32s3_gpspi_init,
    .class_init = esp32s3_gpspi_class_init,
};

static void esp32s3_gpspi_register_types(void)
{
    type_register_static(&esp32s3_gpspi_info);
}

type_init(esp32s3_gpspi_register_types)
