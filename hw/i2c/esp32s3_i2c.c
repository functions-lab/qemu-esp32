/*
 * ESP32-S3 I2C controller, master mode
 *
 * The command list runs to completion when TRANS_START is written: bus timing
 * is not modeled. Each bus event goes to the host's picsimlab I2C bridge. A
 * repeated START begins a new address phase without ending the transaction;
 * the host sees FINISH only at a STOP, or when a NACK the command did not
 * expect aborts the transaction (the controller then generates the STOP).
 * Address bytes are sent as queued: the controller does not interpret 10-bit
 * address prefixes. LSB-first bytes reach the host bit-reversed, as an
 * MSB-first slave would read them. A write to the full TX FIFO is dropped
 * and raises TXFIFO_OVF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/i2c/esp32s3_i2c.h"
#include "hw/xtensa/picsimlab.h"

REG32(CTR, 0x04)
    FIELD(CTR, MS_MODE, 4, 1)
    FIELD(CTR, TRANS_START, 5, 1)
    FIELD(CTR, TX_LSB_FIRST, 6, 1)
    FIELD(CTR, RX_LSB_FIRST, 7, 1)
    FIELD(CTR, FSM_RST, 10, 1)
    FIELD(CTR, CONF_UPGATE, 11, 1)
REG32(SR, 0x08)
    FIELD(SR, RESP_REC, 0, 1)
    FIELD(SR, BUS_BUSY, 4, 1)
    FIELD(SR, RXFIFO_CNT, 8, 6)
    FIELD(SR, STRETCH_CAUSE, 14, 2)
    FIELD(SR, TXFIFO_CNT, 18, 6)
REG32(FIFO_ST, 0x14)
    FIELD(FIFO_ST, RXFIFO_RADDR, 0, 5)
    FIELD(FIFO_ST, RXFIFO_WADDR, 5, 5)
    FIELD(FIFO_ST, TXFIFO_RADDR, 10, 5)
    FIELD(FIFO_ST, TXFIFO_WADDR, 15, 5)
REG32(FIFO_CONF, 0x18)
    FIELD(FIFO_CONF, RXFIFO_WM_THRHD, 0, 5)
    FIELD(FIFO_CONF, TXFIFO_WM_THRHD, 5, 5)
    FIELD(FIFO_CONF, NONFIFO_EN, 10, 1)
    FIELD(FIFO_CONF, RX_FIFO_RST, 12, 1)
    FIELD(FIFO_CONF, TX_FIFO_RST, 13, 1)
REG32(DATA, 0x1C)
REG32(INT_RAW, 0x20)
    FIELD(INT, RXFIFO_WM, 0, 1)
    FIELD(INT, TXFIFO_WM, 1, 1)
    FIELD(INT, END_DETECT, 3, 1)
    FIELD(INT, BYTE_TRANS_DONE, 4, 1)
    FIELD(INT, TRANS_COMPLETE, 7, 1)
    FIELD(INT, TRANS_START, 9, 1)
    FIELD(INT, NACK, 10, 1)
    FIELD(INT, TXFIFO_OVF, 11, 1)
REG32(INT_CLR, 0x24)
REG32(INT_ENA, 0x28)
REG32(INT_STATUS, 0x2C)
REG32(SCL_START_HOLD, 0x40)
REG32(SCL_RSTART_SETUP, 0x44)
REG32(SCL_STOP_HOLD, 0x48)
REG32(SCL_STOP_SETUP, 0x4C)
REG32(FILTER_CFG, 0x50)
REG32(CLK_CONF, 0x54)
REG32(COMD0, 0x58)
    FIELD(COMD, BYTE_NUM, 0, 8)
    FIELD(COMD, ACK_EN, 8, 1)
    FIELD(COMD, ACK_EXP, 9, 1)
    FIELD(COMD, ACK_VAL, 10, 1)
    FIELD(COMD, OP_CODE, 11, 3)
    FIELD(COMD, DONE, 31, 1)
REG32(SCL_ST_TIME_OUT, 0x78)
REG32(SCL_MAIN_ST_TIME_OUT, 0x7C)
REG32(SCL_SP_CONF, 0x80)
    FIELD(SCL_SP_CONF, SCL_RST_SLV_EN, 0, 1)
REG32(DATE, 0xF8)

#define I2C_DATE_VALUE 0x20070201

enum {
    OP_WRITE = 1,
    OP_STOP = 2,
    OP_READ = 3,
    OP_END = 4,
    OP_RSTART = 6,
};

/* Interrupts set by events and cleared by INT_CLR; the FIFO watermarks follow levels */
#define INT_LATCHED (~(R_INT_RXFIFO_WM_MASK | R_INT_TXFIFO_WM_MASK) & 0x3FFFF)

static void update_irq(ESP32S3I2CState *s)
{
    uint32_t raw = s->int_raw & INT_LATCHED;

    if (fifo8_num_used(&s->rx_fifo) >= FIELD_EX32(s->regs[R_FIFO_CONF], FIFO_CONF, RXFIFO_WM_THRHD)) {
        raw |= R_INT_RXFIFO_WM_MASK;
    }
    if (fifo8_num_used(&s->tx_fifo) <= FIELD_EX32(s->regs[R_FIFO_CONF], FIFO_CONF, TXFIFO_WM_THRHD)) {
        raw |= R_INT_TXFIFO_WM_MASK;
    }
    s->int_raw = raw;
    qemu_set_irq(s->irq, (s->int_raw & s->regs[R_INT_ENA]) != 0);
}

static int host_event(ESP32S3I2CState *s, uint16_t event)
{
    return picsimlab_i2c_event(s->bus, s->addr, event);
}

/* STOP: the transaction ends on the bus */
static void bus_stop(ESP32S3I2CState *s)
{
    if (s->slave_engaged) {
        host_event(s, PICSIMLAB_I2C_FINISH);
    }
    s->bus_busy = false;
    s->expect_addr = false;
    s->slave_engaged = false;
}

static void tx_fifo_reset(ESP32S3I2CState *s)
{
    s->tx_popped += fifo8_num_used(&s->tx_fifo);
    fifo8_reset(&s->tx_fifo);
}

static void rx_fifo_reset(ESP32S3I2CState *s)
{
    s->rx_popped += fifo8_num_used(&s->rx_fifo);
    fifo8_reset(&s->rx_fifo);
}

/* Sends one byte; returns the ACK bit the slave answered (0 = ACK) */
static int write_byte(ESP32S3I2CState *s, uint8_t byte)
{
    if (FIELD_EX32(s->regs[R_CTR], CTR, TX_LSB_FIRST)) {
        byte = revbit8(byte);
    }
    if (s->expect_addr) {
        s->expect_addr = false;
        s->addr = byte >> 1;
        int nack = host_event(s, (byte & 1) ? PICSIMLAB_I2C_START_RECV : PICSIMLAB_I2C_START_SEND) != 0;
        s->slave_engaged = !nack;
        return nack;
    }
    if (!s->slave_engaged) {
        /* Nobody answered the address: SDA stays released */
        return 1;
    }
    return host_event(s, (byte << 8) | PICSIMLAB_I2C_WRITE) != 0;
}

static uint8_t read_byte(ESP32S3I2CState *s, bool master_nack)
{
    /* With no slave addressed, SDA stays released */
    uint8_t byte = s->slave_engaged ? host_event(s, PICSIMLAB_I2C_READ) : 0xFF;

    if (master_nack && s->slave_engaged) {
        host_event(s, PICSIMLAB_I2C_NACK);
    }
    return FIELD_EX32(s->regs[R_CTR], CTR, RX_LSB_FIRST) ? revbit8(byte) : byte;
}

/* Runs one command; returns false when the command list stops here or was rejected */
static bool run_command(ESP32S3I2CState *s, int index)
{
    uint32_t cmd = s->regs[R_COMD0 + index];
    uint32_t count = FIELD_EX32(cmd, COMD, BYTE_NUM);
    bool more = true;

    switch (FIELD_EX32(cmd, COMD, OP_CODE)) {
    case OP_RSTART:
        s->bus_busy = true;
        s->expect_addr = true;
        s->slave_engaged = false;
        break;
    case OP_WRITE:
        for (uint32_t i = 0; i < count; i++) {
            if (fifo8_is_empty(&s->tx_fifo)) {
                picsimlab_reject("esp32s3_i2c%d: a WRITE command with an empty TX FIFO is not modeled", s->bus);
                return false;
            }
            uint8_t byte = fifo8_pop(&s->tx_fifo);
            s->tx_popped++;
            int ack = write_byte(s, byte);
            s->regs[R_SR] = FIELD_DP32(s->regs[R_SR], SR, RESP_REC, ack);
            if (FIELD_EX32(cmd, COMD, ACK_EN) && ack != FIELD_EX32(cmd, COMD, ACK_EXP)) {
                s->int_raw |= R_INT_NACK_MASK;
                bus_stop(s);
                more = false;
                break;
            }
        }
        break;
    case OP_READ:
        for (uint32_t i = 0; i < count; i++) {
            if (fifo8_is_full(&s->rx_fifo)) {
                picsimlab_reject("esp32s3_i2c%d: a READ command overflowing the RX FIFO is not modeled", s->bus);
                return false;
            }
            fifo8_push(&s->rx_fifo, read_byte(s, FIELD_EX32(cmd, COMD, ACK_VAL)));
            s->rx_pushed++;
        }
        break;
    case OP_STOP:
        bus_stop(s);
        s->int_raw |= R_INT_TRANS_COMPLETE_MASK;
        more = false;
        break;
    case OP_END:
        /* The bus is held until the next TRANS_START */
        s->int_raw |= R_INT_END_DETECT_MASK;
        more = false;
        break;
    default:
        picsimlab_reject("esp32s3_i2c%d: command %d's opcode %d is not modeled", s->bus, index,
                         (int)FIELD_EX32(cmd, COMD, OP_CODE));
        return false;
    }
    s->regs[R_COMD0 + index] = FIELD_DP32(cmd, COMD, DONE, 1);
    return more;
}

static void trans_start(ESP32S3I2CState *s)
{
    uint32_t ctr = s->regs[R_CTR];

    if (!FIELD_EX32(ctr, CTR, MS_MODE)) {
        picsimlab_reject("esp32s3_i2c%d: slave mode is not modeled", s->bus);
        return;
    }
    if (FIELD_EX32(s->regs[R_FIFO_CONF], FIFO_CONF, NONFIFO_EN)) {
        picsimlab_reject("esp32s3_i2c%d: non-FIFO mode is not modeled", s->bus);
        return;
    }
    s->int_raw |= R_INT_TRANS_START_MASK;
    for (int i = 0; i < ESP32S3_I2C_CMD_COUNT; i++) {
        if (!run_command(s, i)) {
            update_irq(s);
            return;
        }
    }
    picsimlab_reject("esp32s3_i2c%d: a command list without END or STOP is not modeled", s->bus);
}

static uint64_t esp32s3_i2c_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3I2CState *s = ESP32S3_I2C(opaque);
    uint32_t r;

    switch (addr) {
    case A_SR:
        r = s->regs[R_SR] & R_SR_RESP_REC_MASK;
        r = FIELD_DP32(r, SR, BUS_BUSY, s->bus_busy);
        r = FIELD_DP32(r, SR, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
        r = FIELD_DP32(r, SR, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
        /* No clock stretching */
        return FIELD_DP32(r, SR, STRETCH_CAUSE, 3);
    case A_FIFO_ST:
        r = FIELD_DP32(0, FIFO_ST, RXFIFO_RADDR, s->rx_popped);
        r = FIELD_DP32(r, FIFO_ST, RXFIFO_WADDR, s->rx_pushed);
        r = FIELD_DP32(r, FIFO_ST, TXFIFO_RADDR, s->tx_popped);
        return FIELD_DP32(r, FIFO_ST, TXFIFO_WADDR, s->tx_pushed);
    case A_DATA:
        if (fifo8_is_empty(&s->rx_fifo)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32s3_i2c%d: read of an empty RX FIFO\n", s->bus);
            return 0;
        }
        r = fifo8_pop(&s->rx_fifo);
        s->rx_popped++;
        update_irq(s);
        return r;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_STATUS:
        return s->int_raw & s->regs[R_INT_ENA];
    case A_DATE:
        return I2C_DATE_VALUE;
    default:
        if (addr >= 0x100) {
            picsimlab_reject("esp32s3_i2c%d: direct FIFO RAM access is not modeled", s->bus);
            return 0;
        }
        return s->regs[addr / 4];
    }
}

static void esp32s3_i2c_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3I2CState *s = ESP32S3_I2C(opaque);

    switch (addr) {
    case A_CTR:
        /* TRANS_START, FSM_RST and CONF_UPGATE act once and read back 0 */
        s->regs[R_CTR] = value & ~(R_CTR_TRANS_START_MASK | R_CTR_FSM_RST_MASK | R_CTR_CONF_UPGATE_MASK);
        if (FIELD_EX32(value, CTR, FSM_RST)) {
            bus_stop(s);
        }
        if (FIELD_EX32(value, CTR, TRANS_START)) {
            trans_start(s);
        }
        break;
    case A_FIFO_CONF:
        s->regs[R_FIFO_CONF] = value;
        if (FIELD_EX32(value, FIFO_CONF, TX_FIFO_RST)) {
            tx_fifo_reset(s);
        }
        if (FIELD_EX32(value, FIFO_CONF, RX_FIFO_RST)) {
            rx_fifo_reset(s);
        }
        break;
    case A_DATA:
        if (fifo8_is_full(&s->tx_fifo)) {
            s->int_raw |= R_INT_TXFIFO_OVF_MASK;
            break;
        }
        fifo8_push(&s->tx_fifo, value & 0xFF);
        s->tx_pushed++;
        break;
    case A_INT_CLR:
        s->int_raw &= ~value;
        break;
    case A_INT_ENA:
        s->regs[R_INT_ENA] = value;
        break;
    case A_SCL_SP_CONF:
        /* The bus-clear clocks are sent at once */
        s->regs[R_SCL_SP_CONF] = FIELD_DP32(value, SCL_SP_CONF, SCL_RST_SLV_EN, 0);
        break;
    case A_SR:
    case A_FIFO_ST:
    case A_INT_RAW:
    case A_INT_STATUS:
    case A_DATE:
        break;
    default:
        if (addr >= 0x100) {
            picsimlab_reject("esp32s3_i2c%d: direct FIFO RAM access is not modeled", s->bus);
            return;
        }
        s->regs[addr / 4] = value;
        break;
    }
    update_irq(s);
}

static const MemoryRegionOps esp32s3_i2c_ops = {
    .read = esp32s3_i2c_read,
    .write = esp32s3_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_i2c_reset_hold(Object *obj, ResetType type)
{
    ESP32S3I2CState *s = ESP32S3_I2C(obj);

    memset(s->regs, 0, sizeof(s->regs));
    /* SDA/SCL_FORCE_OUT, RX_FULL_ACK_LEVEL, ARBITRATION_EN */
    s->regs[R_CTR] = 0x20B;
    s->regs[R_FIFO_CONF] = (4 << R_FIFO_CONF_TXFIFO_WM_THRHD_SHIFT) | (11 << R_FIFO_CONF_RXFIFO_WM_THRHD_SHIFT) |
                           (1 << 14);   /* FIFO_PRT_EN */
    s->regs[R_SCL_START_HOLD] = 8;
    s->regs[R_SCL_RSTART_SETUP] = 8;
    s->regs[R_SCL_STOP_HOLD] = 8;
    s->regs[R_SCL_STOP_SETUP] = 8;
    s->regs[R_FILTER_CFG] = 0x300;      /* SCL/SDA_FILTER_EN */
    s->regs[R_CLK_CONF] = 1 << 21;      /* SCLK_ACTIVE */
    s->regs[R_SCL_ST_TIME_OUT] = 16;
    s->regs[R_SCL_MAIN_ST_TIME_OUT] = 16;
    s->regs[0x0C / 4] = 16;             /* TO: TIME_OUT_VALUE */
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    s->tx_pushed = s->tx_popped = s->rx_pushed = s->rx_popped = 0;
    s->bus_busy = false;
    s->expect_addr = false;
    s->slave_engaged = false;
    s->int_raw = 0;
    update_irq(s);
}

static void esp32s3_i2c_init(Object *obj)
{
    ESP32S3I2CState *s = ESP32S3_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_i2c_ops, s, TYPE_ESP32S3_I2C, 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    fifo8_create(&s->tx_fifo, ESP32S3_I2C_FIFO_LEN);
    fifo8_create(&s->rx_fifo, ESP32S3_I2C_FIFO_LEN);
}

static Property esp32s3_i2c_properties[] = {
    DEFINE_PROP_UINT8("bus", ESP32S3I2CState, bus, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_i2c_reset_hold;
    device_class_set_props(dc, esp32s3_i2c_properties);
}

static const TypeInfo esp32s3_i2c_info = {
    .name = TYPE_ESP32S3_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3I2CState),
    .instance_init = esp32s3_i2c_init,
    .class_init = esp32s3_i2c_class_init,
};

static void esp32s3_i2c_register_types(void)
{
    type_register_static(&esp32s3_i2c_info);
}

type_init(esp32s3_i2c_register_types)
