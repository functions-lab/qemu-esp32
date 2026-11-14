/*
 * ESP32-S3 WiFi MAC
 *
 * The S3 MAC has the ESP32-C3's register layout and RX descriptor header, so
 * this follows lcgamboa's esp32c3_wifi.c: frames move through the RX and TX
 * DMA descriptor lists, and the access points of esp32_wifi_ap.c answer them.
 * Received frames carry the access point's exact signal strength, and the
 * guest network Velxio-GUEST is on the air besides lcgamboa's three.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "exec/address-spaces.h"
#include "hw/misc/esp32s3_wifi.h"
#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"
#include "hw/xtensa/picsimlab.h"

REG32(DMA_IN_STATUS, 0x84)
REG32(DMA_INLINK, 0x88)
REG32(NEXT_RX_DSCR, 0x8C)
REG32(LAST_RX_DSCR, 0x90)
REG32(DMA_INT_STATUS, 0xC3C)
REG32(DMA_INT_CLR, 0xC40)
REG32(STATUS, 0xCB0)
REG32(DMA_OUTLINK, 0xD08)
REG32(DMA_OUT_STATUS, 0xD14)

#define INT_TX_DONE     0x80
#define INT_RX_DONE     0x1004000
#define OUTLINK_START   0xC0000000
/* DMA descriptors and buffers live in internal DRAM */
#define DRAM_WINDOW     0x3FC00000

static void update_irq(Esp32WifiState *s)
{
    qemu_set_irq(s->irq, s->raw_interrupt != 0);
}

static void s3_frame_delivered(Esp32WifiState *s)
{
    s->raw_interrupt |= INT_TX_DONE;
    update_irq(s);
}

static bool address_matches(const uint8_t *frame_addr, const uint8_t *mac)
{
    return !memcmp(frame_addr, mac, 6) || !memcmp(frame_addr, BROADCAST, 6);
}

/* A frame from an access point into the guest's next RX descriptor */
static void s3_send_frame(Esp32WifiState *s, mac80211_frame *frame, int length, int signal_strength)
{
    wifi_pkt_rx_ctrl_c3_t pkt = {
        .rssi = signal_strength,
        .rate = 11,
        .sig_len = length,
        .sig_len_copy = length,
        .legacy_length = length,
        .noise_floor = -97,
        .channel = esp32_wifi_channel,
        .timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000,
    };
    const uint8_t *mac0 = (const uint8_t *)s->mem + A_WIFI_MAC_ADDR_FST_0;
    const uint8_t *mac1 = (const uint8_t *)s->mem + A_WIFI_MAC_ADDR_FST_1;
    int total = sizeof(pkt) + length;
    dma_list_item item;

    if (s->dma_inlink_address == 0) {
        return;
    }
    pkt.damatch0 = address_matches(frame->destination_address, mac0);
    pkt.damatch1 = address_matches(frame->destination_address, mac1);
    pkt.bssidmatch0 = address_matches(frame->bssid_address, mac0);
    pkt.bssidmatch1 = address_matches(frame->bssid_address, mac1);

    address_space_read(&address_space_memory, s->dma_inlink_ptr, MEMTXATTRS_UNSPECIFIED, &item, sizeof(item));
    if (!item.owner || item.address == 0 || item.size < total) {
        /* No descriptor handed to the MAC with room for it: the hardware drops the frame */
        return;
    }
    g_autofree uint8_t *buf = g_malloc(total);
    memcpy(buf, &pkt, sizeof(pkt));
    memcpy(buf + sizeof(pkt), frame, length);
    address_space_write(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, buf, total);
    item.length = total;
    item.eof = 1;
    /* Only the first word (size, length, flags) changes */
    address_space_write(&address_space_memory, s->dma_inlink_ptr, MEMTXATTRS_UNSPECIFIED, &item, 4);
    s->dma_inlink_ptr = item.next ? item.next : s->dma_inlink_address;
    s->raw_interrupt |= INT_RX_DONE;
    update_irq(s);
}

static void transmit(Esp32WifiState *s, uint32_t outlink)
{
    mac80211_frame frame;
    dma_list_item item;

    address_space_read(&address_space_memory, DRAM_WINDOW | (outlink & 0xFFFFF), MEMTXATTRS_UNSPECIFIED,
                       &item, sizeof(item));
    if (item.length > sizeof(frame)) {
        picsimlab_reject("esp32s3_wifi: a TX frame of %u bytes is not modeled", (unsigned)item.length);
        return;
    }
    address_space_read(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, &frame, item.length);
    frame.frame_length = item.length;
    frame.next_frame = NULL;
    Esp32_WLAN_handle_frame(s, &frame);
    s3_frame_delivered(s);
}

static uint64_t esp32s3_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    switch (addr) {
    case A_DMA_INLINK:
        return s->dma_inlink_address;
    case A_DMA_IN_STATUS:
        return s->mem[addr / 4] & ~1u;
    case A_DMA_INT_STATUS:
    case A_DMA_INT_CLR:
        return s->raw_interrupt;
    case A_STATUS:
    case A_DMA_OUT_STATUS:
        return 1;
    default:
        return s->mem[addr / 4];
    }
}

static void esp32s3_wifi_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    s->mem[addr / 4] = value;
    switch (addr) {
    case A_DMA_INLINK:
        s->dma_inlink_address = value;
        s->dma_inlink_ptr = value;
        s->mem[R_NEXT_RX_DSCR] = value;
        s->mem[R_LAST_RX_DSCR] = value;
        break;
    case A_DMA_INT_CLR:
        s->raw_interrupt &= ~value;
        update_irq(s);
        break;
    case A_DMA_OUTLINK:
        if (value & OUTLINK_START) {
            transmit(s, value);
        }
        break;
    }
}

static const MemoryRegionOps esp32s3_wifi_ops = {
    .read = esp32s3_wifi_read,
    .write = esp32s3_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_wifi_reset_hold(Object *obj, ResetType type)
{
    Esp32WifiState *s = ESP32_WIFI(obj);

    s->dma_inlink_address = 0;
    s->dma_inlink_ptr = 0;
    s->raw_interrupt = 0;
    memset(s->mem, 0, sizeof(s->mem));
    Esp32_WLAN_reset_ap(s);
    update_irq(s);
}

static void esp32s3_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->send_frame = s3_send_frame;
    s->frame_delivered = s3_frame_delivered;
    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32s3_wifi_ops, s, TYPE_ESP32S3_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    Esp32_WLAN_enable_guest_ap();
    Esp32_WLAN_setup_ap(dev, s);
}

/* A subclass of lcgamboa's esp32_wifi: its NIC and access-points-off properties carry over */
static void esp32s3_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32s3_wifi_realize;
    dc->desc = "ESP32-S3 WiFi";
    rc->phases.enter = NULL;
    rc->phases.hold = esp32s3_wifi_reset_hold;
    rc->phases.exit = NULL;
}

static const TypeInfo esp32s3_wifi_info = {
    .name = TYPE_ESP32S3_WIFI,
    .parent = TYPE_ESP32_WIFI,
    .instance_size = sizeof(Esp32WifiState),
    .class_init = esp32s3_wifi_class_init,
};

static void esp32s3_wifi_register_types(void)
{
    type_register_static(&esp32s3_wifi_info);
}

type_init(esp32s3_wifi_register_types)
