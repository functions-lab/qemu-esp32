/*
 * ESP32-S3 UART
 *
 * Transmission is instantaneous: a byte written to the TX FIFO leaves at once,
 * so the TX FIFO is always empty and the transmitter always idle. Received
 * bytes fill a 128-byte RX FIFO; a byte that finds it full is lost and raises
 * RXFIFO_OVF. The RX timeout fires RX_TOUT_THRHD bit times after the last
 * received byte, at the baud rate CLKDIV and CLK_CONF select.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/char/esp32s3_uart.h"
#include "hw/xtensa/picsimlab.h"

#define UART_DATE_VALUE  0x2008270
#define UART_ID_RESET    0x40000500

/* Interrupts that stay raised until INT_CLR; the others follow FIFO levels */
#define UART_INT_LATCHED (R_UART_INT_RXFIFO_OVF_MASK | R_UART_INT_RXFIFO_TOUT_MASK | \
                          R_UART_INT_TX_DONE_MASK)

static uint64_t sclk_hz(ESP32S3UARTState *s)
{
    switch (FIELD_EX32(s->reg[R_UART_CLK_CONF], UART_CLK_CONF, SCLK_SEL)) {
    case 0:
    case 1:
        return 80000000;    /* APB */
    case 2:
        return 20000000;    /* RC_FAST, as ESP-IDF's uart_ll_get_sclk_freq */
    case 3:
        return 40000000;    /* XTAL */
    default:
        g_assert_not_reached();
    }
}

/* Returns false when the divider was rejected */
static bool bit_ns(ESP32S3UARTState *s, uint64_t *ns)
{
    uint32_t clkdiv = s->reg[R_UART_CLKDIV];
    uint64_t div16 = (FIELD_EX32(clkdiv, UART_CLKDIV, CLKDIV) << 4) | FIELD_EX32(clkdiv, UART_CLKDIV, FRAG);
    uint64_t div_num = FIELD_EX32(s->reg[R_UART_CLK_CONF], UART_CLK_CONF, SCLK_DIV_NUM) + 1;

    if (div16 == 0) {
        picsimlab_reject("esp32s3_uart%d: a CLKDIV of 0, which gives no baud rate, is not modeled", s->index);
        return false;
    }
    /* bit time = (CLKDIV + FRAG/16) * (SCLK_DIV_NUM + 1) / SCLK */
    *ns = div16 * div_num * NANOSECONDS_PER_SECOND / (sclk_hz(s) << 4);
    return true;
}

static void update_irq(ESP32S3UARTState *s)
{
    uint32_t raw = s->reg[R_UART_INT_RAW] & UART_INT_LATCHED;
    uint32_t rx_count = fifo8_num_used(&s->rx_fifo);

    if (rx_count >= FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, RXFIFO_FULL_THRHD)) {
        raw |= R_UART_INT_RXFIFO_FULL_MASK;
    }
    /* The TX FIFO is always empty, so always at or below its threshold */
    raw |= R_UART_INT_TXFIFO_EMPTY_MASK;

    s->reg[R_UART_INT_RAW] = raw;
    s->reg[R_UART_INT_ST] = raw & s->reg[R_UART_INT_ENA];
    qemu_set_irq(s->irq, s->reg[R_UART_INT_ST] != 0);
}

static void arm_rx_timeout(ESP32S3UARTState *s)
{
    uint32_t thrhd = FIELD_EX32(s->reg[R_UART_MEM_CONF], UART_MEM_CONF, RX_TOUT_THRHD);
    uint64_t bit;

    if (!FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, RX_TOUT_EN) || thrhd == 0) {
        timer_del(&s->rx_timeout_timer);
        return;
    }
    if (!bit_ns(s, &bit)) {
        return;
    }
    timer_mod(&s->rx_timeout_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + thrhd * bit);
}

static void rx_timeout_cb(void *opaque)
{
    ESP32S3UARTState *s = ESP32S3_UART(opaque);

    if (!fifo8_is_empty(&s->rx_fifo)) {
        s->reg[R_UART_INT_RAW] |= R_UART_INT_RXFIFO_TOUT_MASK;
        update_irq(s);
    }
}

void esp32s3_uart_receive_byte(ESP32S3UARTState *s, uint8_t byte)
{
    if (fifo8_is_full(&s->rx_fifo)) {
        s->reg[R_UART_INT_RAW] |= R_UART_INT_RXFIFO_OVF_MASK;
    } else {
        fifo8_push(&s->rx_fifo, byte);
        s->rx_pushed++;
    }
    arm_rx_timeout(s);
    update_irq(s);
}

static void transmit(ESP32S3UARTState *s, uint8_t byte)
{
    if (FIELD_EX32(s->reg[R_UART_CONF0], UART_CONF0, LOOPBACK)) {
        esp32s3_uart_receive_byte(s, byte);
    } else {
        qemu_chr_fe_write_all(&s->chr, &byte, 1);
        picsimlab_uart_tx_event(s->index, byte);
    }
    s->reg[R_UART_INT_RAW] |= R_UART_INT_TX_DONE_MASK;
}

static void rx_fifo_reset(ESP32S3UARTState *s)
{
    s->rx_popped += fifo8_num_used(&s->rx_fifo);
    fifo8_reset(&s->rx_fifo);
}

static uint64_t esp32s3_uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3UARTState *s = ESP32S3_UART(opaque);
    uint32_t r;

    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_is_empty(&s->rx_fifo)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32s3_uart%d: read of an empty RX FIFO\n", s->index);
            return 0;
        }
        r = fifo8_pop(&s->rx_fifo);
        s->rx_popped++;
        update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        return r;
    case A_UART_STATUS:
        r = FIELD_DP32(0, UART_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
        /* Idle lines and deasserted modem inputs/outputs */
        r = FIELD_DP32(r, UART_STATUS, DSRN, 1);
        r = FIELD_DP32(r, UART_STATUS, CTSN, 1);
        r = FIELD_DP32(r, UART_STATUS, RXD, 1);
        r = FIELD_DP32(r, UART_STATUS, DTRN, 1);
        r = FIELD_DP32(r, UART_STATUS, RTSN, 1);
        r = FIELD_DP32(r, UART_STATUS, TXD, 1);
        return r;
    case A_UART_MEM_RX_STATUS:
        r = FIELD_DP32(0, UART_MEM_RX_STATUS, APB_RX_RADDR, 0x200 + s->rx_popped % ESP32S3_UART_FIFO_LENGTH);
        return FIELD_DP32(r, UART_MEM_RX_STATUS, RX_WADDR, 0x200 + s->rx_pushed % ESP32S3_UART_FIFO_LENGTH);
    case A_UART_MEM_TX_STATUS:
    case A_UART_FSM_STATUS:
        /* Empty TX FIFO, idle transmitter and receiver */
        return 0;
    case A_UART_LOWPULSE:
    case A_UART_HIGHPULSE:
    case A_UART_POSPULSE:
    case A_UART_NEGPULSE:
        /* No edges measured on RXD */
        return 0xFFF;
    case A_UART_DATE:
        return UART_DATE_VALUE;
    default:
        return s->reg[addr / 4];
    }
}

static void esp32s3_uart_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3UARTState *s = ESP32S3_UART(opaque);

    switch (addr) {
    case A_UART_FIFO:
        transmit(s, value & 0xFF);
        break;
    case A_UART_INT_CLR:
        s->reg[R_UART_INT_RAW] &= ~((uint32_t)value & UART_INT_LATCHED);
        break;
    case A_UART_INT_ENA:
        s->reg[R_UART_INT_ENA] = value;
        break;
    case A_UART_INT_RAW:
    case A_UART_INT_ST:
    case A_UART_STATUS:
    case A_UART_MEM_TX_STATUS:
    case A_UART_MEM_RX_STATUS:
    case A_UART_FSM_STATUS:
    case A_UART_LOWPULSE:
    case A_UART_HIGHPULSE:
    case A_UART_POSPULSE:
    case A_UART_NEGPULSE:
    case A_UART_DATE:
        break;
    case A_UART_CONF0:
        s->reg[R_UART_CONF0] = value;
        if (FIELD_EX32(value, UART_CONF0, RXFIFO_RST)) {
            rx_fifo_reset(s);
        }
        break;
    case A_UART_CONF1:
    case A_UART_MEM_CONF:
        s->reg[addr / 4] = value;
        if (!FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, RX_TOUT_EN)) {
            timer_del(&s->rx_timeout_timer);
        }
        break;
    case A_UART_ID:
        /* A register update completes at once */
        s->reg[R_UART_ID] = FIELD_DP32(value, UART_ID, REG_UPDATE, 0);
        break;
    default:
        s->reg[addr / 4] = value;
        break;
    }
    update_irq(s);
}

static const MemoryRegionOps esp32s3_uart_ops = {
    .read = esp32s3_uart_read,
    .write = esp32s3_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static int chr_can_receive(void *opaque)
{
    ESP32S3UARTState *s = ESP32S3_UART(opaque);
    return fifo8_num_free(&s->rx_fifo);
}

static void chr_receive(void *opaque, const uint8_t *buf, int size)
{
    ESP32S3UARTState *s = ESP32S3_UART(opaque);

    for (int i = 0; i < size; i++) {
        esp32s3_uart_receive_byte(s, buf[i]);
    }
}

static void esp32s3_uart_reset_hold(Object *obj, ResetType type)
{
    ESP32S3UARTState *s = ESP32S3_UART(obj);

    memset(s->reg, 0, sizeof(s->reg));
    s->reg[R_UART_CLKDIV] = 694;
    s->reg[R_UART_CONF0] = (3 << R_UART_CONF0_BIT_NUM_SHIFT) | (1 << R_UART_CONF0_STOP_BIT_NUM_SHIFT) |
                           R_UART_CONF0_MEM_CLK_EN_MASK;
    s->reg[R_UART_CONF1] = (96 << R_UART_CONF1_RXFIFO_FULL_THRHD_SHIFT) |
                           (96 << R_UART_CONF1_TXFIFO_EMPTY_THRHD_SHIFT);
    s->reg[R_UART_MEM_CONF] = (1 << R_UART_MEM_CONF_RX_SIZE_SHIFT) | (1 << R_UART_MEM_CONF_TX_SIZE_SHIFT) |
                              (10 << R_UART_MEM_CONF_RX_TOUT_THRHD_SHIFT);
    /* SCLK_DIV_NUM 1, SCLK_SEL XTAL, SCLK_EN, TX_SCLK_EN, RX_SCLK_EN */
    s->reg[R_UART_CLK_CONF] = 0x03701000;
    s->reg[R_UART_ID] = UART_ID_RESET;
    fifo8_reset(&s->rx_fifo);
    s->rx_pushed = 0;
    s->rx_popped = 0;
    timer_del(&s->rx_timeout_timer);
    update_irq(s);
}

static void esp32s3_uart_realize(DeviceState *dev, Error **errp)
{
    ESP32S3UARTState *s = ESP32S3_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, chr_can_receive, chr_receive, NULL, NULL, s, NULL, true);
}

static void esp32s3_uart_init(Object *obj)
{
    ESP32S3UARTState *s = ESP32S3_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_uart_ops, s, TYPE_ESP32S3_UART,
                          ESP32S3_UART_REG_COUNT * sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    fifo8_create(&s->rx_fifo, ESP32S3_UART_FIFO_LENGTH);
    timer_init_ns(&s->rx_timeout_timer, QEMU_CLOCK_VIRTUAL, rx_timeout_cb, s);
}

static Property esp32s3_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", ESP32S3UARTState, chr),
    DEFINE_PROP_UINT8("index", ESP32S3UARTState, index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_uart_reset_hold;
    dc->realize = esp32s3_uart_realize;
    device_class_set_props(dc, esp32s3_uart_properties);
}

static const TypeInfo esp32s3_uart_info = {
    .name = TYPE_ESP32S3_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3UARTState),
    .instance_init = esp32s3_uart_init,
    .class_init = esp32s3_uart_class_init,
};

static void esp32s3_uart_register_types(void)
{
    type_register_static(&esp32s3_uart_info);
}

type_init(esp32s3_uart_register_types)
