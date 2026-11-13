/*
 * ESP32-S3 machine for PICSimLab-style hosts (velxio)
 *
 * The esp32s3 machine plus the peripherals a board simulator drives from the
 * outside, wired to the host bridge (hw/xtensa/picsimlab.h).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "qemu/memalign.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "target/xtensa/cpu.h"
#include "qemu/datadir.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "core-esp32s3/core-isa.h"

#include "hw/misc/esp32s3_reg.h"
#include "hw/misc/esp32s3_rtc_cntl.h"
#include "hw/xtensa/esp32s3_intc.h"
#include "hw/ssi/esp32s3_spi.h"
#include "hw/misc/esp32s3_cache.h"
#include "hw/char/esp32s3_uart.h"
#include "hw/misc/esp32s3_rng.h"
#include "hw/nvram/esp32s3_efuse.h"
#include "hw/xtensa/esp32s3_clk.h"
#include "hw/dma/esp32s3_gdma.h"
#include "hw/misc/esp32s3_sha.h"
#include "hw/misc/esp32s3_aes.h"
#include "hw/misc/esp32s3_rsa.h"
#include "hw/misc/esp32s3_hmac.h"
#include "hw/misc/esp32s3_ds.h"
#include "hw/timer/esp32s3_timg.h"
#include "hw/timer/esp32s3_systimer.h"
#include "hw/gpio/esp32s3_gpio.h"
#include "hw/misc/esp32s3_iomux.h"
#include "hw/misc/esp32s3_ana.h"
#include "hw/misc/esp32_fe.h"
#include "hw/misc/esp32_phya.h"
#include "hw/misc/esp32s3_ledc.h"
#include "hw/misc/esp32s3_sens.h"
#include "hw/i2c/esp32s3_i2c.h"
#include "hw/ssi/esp32s3_gpspi.h"
#include "hw/misc/esp32s3_wifi.h"
#include "net/net.h"
#include "hw/misc/esp32s3_xts_aes.h"
#include "hw/misc/esp32s3_pms.h"
#include "hw/misc/esp32c3_jtag.h"
#include "hw/xtensa/picsimlab.h"
#include "cpu_esp32s3.h"

#define TYPE_ESP32S3_PICSIMLAB_SOC "xtensa.esp32s3.picsimlab"
#define ESP32S3_PICSIMLAB_SOC(obj) \
    OBJECT_CHECK(Esp32s3PicsimlabSocState, (obj), TYPE_ESP32S3_PICSIMLAB_SOC)

#define TYPE_ESP32S3_CPU XTENSA_CPU_TYPE_NAME("esp32s3")

enum {
    MEMREGION_IROM,
    MEMREGION_DROM,
    MEMREGION_DRAM,
    MEMREGION_IRAM,
    MEMREGION_ICACHE,
    MEMREGION_DCACHE,
    MEMREGION_RTCSLOW,
    MEMREGION_RTCFAST,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} memmap[] = {
    [MEMREGION_DROM] = { 0x3ff00000, 0x20000 },
    [MEMREGION_IROM] = { 0x40000000, 0x60000 },
    [MEMREGION_DRAM] = { 0x3FC80000, 0x170000 },
    [MEMREGION_IRAM] = { 0x40370000, 0x80000 },
    [MEMREGION_DCACHE] = { 0x3c000000, ESP32S3_EXTMEM_REGION_SIZE },
    [MEMREGION_ICACHE] = { 0x42000000, ESP32S3_EXTMEM_REGION_SIZE },
    [MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [MEMREGION_RTCFAST] = { 0x600fe000, 0x2000 },
};

#define SOC_RESET_PROCPU    0x1
#define SOC_RESET_APPCPU    0x2
#define SOC_RESET_PERIPH    0x4
#define SOC_RESET_DIG       (SOC_RESET_PROCPU | SOC_RESET_APPCPU | SOC_RESET_PERIPH)
#define SOC_RESET_RTC       0x8
#define SOC_RESET_ALL       (SOC_RESET_RTC | SOC_RESET_DIG)

/* "QEMU" at APB_CTRL + 0x3F8 tells firmware it runs in QEMU */
#define RGB_QEMU_ORIGIN     0x51454d55
#define RGB_QEMU_ORIGIN_REG 0x3F8

typedef struct Esp32s3PicsimlabSocState {
    DeviceState parent_obj;

    XtensaCPU cpu[ESP32S3_CPU_COUNT];
    Esp32s3IntMatrixState intmatrix;
    ESP32S3UARTState uart[ESP32S3_UART_COUNT];
    ESP32S3GPIOState gpio;
    ESP32S3IOMuxState iomux;
    ESP32S3AnaState ana;
    Esp32FeState fe;
    Esp32PhyaState phya;
    ESP32S3LEDCState ledc;
    ESP32S3SensState sens;
    ESP32S3I2CState i2c0;
    ESP32S3GpspiState gpspi2;
    Esp32s3RngState rng;
    Esp32s3RtcCntlState rtc_cntl;

    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem[ESP32S3_CPU_COUNT];
    ESP32S3SpiState spi1;
    ESP32S3CacheState cache;
    ESP32S3EfuseState efuse;
    ESP32S3ClockState clock;
    ESP32S3GdmaState gdma;
    ESP32S3ShaState sha;
    ESP32S3AesState aes;
    ESP32S3RsaState rsa;
    ESP32S3HmacState hmac;
    ESP32S3DsState ds;
    ESP32S3PmsState pms;
    ESP32S3XtsAesState xts_aes;
    ESP32S3TimgState timg[2];
    ESP32S3SysTimerState systimer;
    ESP32C3UsbJtagState jtag;

    MemoryRegion iomem;
    BlockBackend *flash_blk;

    uint32_t requested_reset;
} Esp32s3PicsimlabSocState;

typedef struct Esp32s3PicsimlabMachineState {
    MachineState parent;
    Esp32s3PicsimlabSocState soc;
} Esp32s3PicsimlabMachineState;

#define TYPE_ESP32S3_PICSIMLAB_MACHINE MACHINE_TYPE_NAME("esp32s3-picsimlab")
OBJECT_DECLARE_SIMPLE_TYPE(Esp32s3PicsimlabMachineState, ESP32S3_PICSIMLAB_MACHINE)

static Esp32s3PicsimlabSocState *board_soc;

/* ---------------------------------------------------------------------------
 * Host bridge
 */

/* Host pin slot -> the GPIO's input line */
static qemu_irq *pin_in;

#define QEMU_INTERNAL_GPIO_IN_SEL       1
#define QEMU_INTERNAL_GPIO_OUT_SEL      2
#define QEMU_INTERNAL_IOMUX_GPIOS       3
#define QEMU_INTERNAL_LEDC_CHANNEL_CONF 4
#define QEMU_INTERNAL_LEDC_TIMER_FREQ   5
#define QEMU_INTERNAL_LEDC_CHANNEL_DUTY 6
#define QEMU_INTERNAL_RTCIO             10

static void s3_set_pin(int pin, int value)
{
    /* The host calls this from its own threads and from inside callbacks */
    const bool locked = bql_locked();
    if (!locked) {
        bql_lock();
    }
    qemu_set_irq(pin_in[pin], value);
    if (!locked) {
        bql_unlock();
    }
}

static void s3_set_apin(int chn, int value)
{
    const bool locked = bql_locked();
    if (!locked) {
        bql_lock();
    }
    esp32s3_sens_set_input(&board_soc->sens, chn, value);
    if (!locked) {
        bql_unlock();
    }
}

static void s3_set_apin_waveform(int chn, const uint16_t *samples, int count, uint64_t period_ns)
{
    const bool locked = bql_locked();
    if (!locked) {
        bql_lock();
    }
    esp32s3_sens_set_waveform(&board_soc->sens, chn, samples, count, period_ns);
    if (!locked) {
        bql_unlock();
    }
}

static uint32_t *s3_get_internals(int cfg)
{
    switch (cfg) {
    case QEMU_INTERNAL_GPIO_IN_SEL:
        return board_soc->gpio.in_sel;
    case QEMU_INTERNAL_GPIO_OUT_SEL:
        return board_soc->gpio.out_sel;
    case QEMU_INTERNAL_IOMUX_GPIOS:
        return board_soc->iomux.pads;
    case QEMU_INTERNAL_LEDC_CHANNEL_CONF:
        return board_soc->ledc.ch_conf0;
    case QEMU_INTERNAL_LEDC_TIMER_FREQ:
        /* timer_freq[4] followed by timer_conf[4] */
        return board_soc->ledc.timer_freq;
    case QEMU_INTERNAL_LEDC_CHANNEL_DUTY:
        return (uint32_t *)board_soc->ledc.duty_pct;
    case QEMU_INTERNAL_RTCIO:
        /* The S3 configures every pad's pulls in IO_MUX, RTC pads included */
        return NULL;
    default:
        error_report("esp32s3-picsimlab: no internals table %d", cfg);
        abort();
    }
}

static void s3_uart_receive(int id, const uint8_t *buf, int size)
{
    if (id < 0 || id >= ESP32S3_UART_COUNT) {
        error_report("esp32s3-picsimlab: no UART%d", id);
        abort();
    }
    const bool locked = bql_locked();
    if (!locked) {
        bql_lock();
    }
    for (int i = 0; i < size; i++) {
        esp32s3_uart_receive_byte(&board_soc->uart[id], buf[i]);
    }
    if (!locked) {
        bql_unlock();
    }
}

static uint32_t s3_get_tiocm(void)
{
    uint32_t state = 0;
    qemu_chr_fe_ioctl(&board_soc->uart[0].chr, CHR_IOCTL_SERIAL_GET_TIOCM, &state);
    return state;
}

static int s3_flash_dump(int64_t offset, void *buf, int bytes)
{
    return blk_pread(board_soc->flash_blk, offset, bytes, buf, 0);
}

/* Whether GPSPI2 chip-select changes reach the host */
static bool spi_cs_events = true;

static void s3_enable_spi_cs_events(int enable)
{
    spi_cs_events = enable != 0;
}

static const PicsimlabBoard s3_board = {
    .set_pin = s3_set_pin,
    .set_apin = s3_set_apin,
    .set_apin_waveform = s3_set_apin_waveform,
    .get_internals = s3_get_internals,
    .get_tiocm = s3_get_tiocm,
    .uart_receive = s3_uart_receive,
    .flash_dump = s3_flash_dump,
    .enable_spi_cs_events = s3_enable_spi_cs_events,
};

static void pin_level_handler(void *opaque, int slot, int level)
{
    picsimlab_write_pin(slot, level);
}

static void pin_dir_handler(void *opaque, int slot, int dir)
{
    picsimlab_dir_pin(slot, dir);
}

static void gpio_in_sync_handler(void *opaque, int n, int level)
{
    picsimlab_dir_pin(-1, PICSIMLAB_SYNC_GPIO_IN);
}

static void gpio_matrix_handler(void *opaque, int gpio, int signal)
{
    picsimlab_gpio_matrix_cb(gpio, signal);
}

static void ledc_update_handler(void *opaque, int channel, int level)
{
    picsimlab_dir_pin(-1, PICSIMLAB_SYNC_LEDC | (channel << 8));
}

static void spi_cs_handler(void *opaque, int lane, int level)
{
    if (spi_cs_events) {
        picsimlab_spi_event(0, ((((lane & 3) << 1) | level) << 8) | 0x01);
    }
}

static void connect_host(Esp32s3PicsimlabSocState *ss)
{
    const short int *pinmap = picsimlab_pinmap;
    DeviceState *gpio = DEVICE(&ss->gpio);

    picsimlab_set_board(&s3_board);
    if (!pinmap) {
        return;
    }

    qemu_irq *sync = qemu_allocate_irqs(gpio_in_sync_handler, NULL, 1);
    qdev_connect_gpio_out_named(gpio, ESP32S3_GPIOS_SYNC, 0, sync[0]);
    qemu_irq *ledc = qemu_allocate_irqs(ledc_update_handler, NULL, ESP32S3_LEDC_CHANNELS);
    for (int ch = 0; ch < ESP32S3_LEDC_CHANNELS; ch++) {
        qdev_connect_gpio_out_named(DEVICE(&ss->ledc), ESP32S3_LEDC_UPDATE, ch, ledc[ch]);
    }
    qemu_irq *cs = qemu_allocate_irqs(spi_cs_handler, NULL, ESP32S3_GPSPI_CS_COUNT);
    for (int lane = 0; lane < ESP32S3_GPSPI_CS_COUNT; lane++) {
        qdev_connect_gpio_out_named(DEVICE(&ss->gpspi2), ESP32S3_GPSPI_CS, lane, cs[lane]);
    }
    qemu_irq *matrix = qemu_allocate_irqs(gpio_matrix_handler, NULL, ESP32S3_GPIO_COUNT);
    for (int n = 0; n < ESP32S3_GPIO_COUNT; n++) {
        qdev_connect_gpio_out_named(gpio, ESP32S3_GPIOS_MATRIX, n, matrix[n]);
    }

    const int slots = pinmap[0] + 1;
    qemu_irq *level = qemu_allocate_irqs(pin_level_handler, NULL, slots);
    qemu_irq *dir = qemu_allocate_irqs(pin_dir_handler, NULL, slots);
    pin_in = g_new0(qemu_irq, slots);
    for (int slot = 1; slot < slots; slot++) {
        const int n = pinmap[slot];
        if (n < 0) {
            continue;
        }
        qdev_connect_gpio_out_named(gpio, ESP32S3_GPIOS, n, level[slot]);
        qdev_connect_gpio_out_named(gpio, ESP32S3_GPIOS_DIR, n, dir[slot]);
        pin_in[slot] = qdev_get_gpio_in_named(gpio, ESP32S3_GPIOS_IN, n);
    }
}

/* ---------------------------------------------------------------------------
 * SoC
 */

static void remove_cpu_watchpoints(XtensaCPU *xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void soc_dig_reset(void *opaque, int n, int level)
{
    Esp32s3PicsimlabSocState *s = ESP32S3_PICSIMLAB_SOC(opaque);
    if (level) {
        s->requested_reset = SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void soc_cpu_reset(void *opaque, int n, int level)
{
    Esp32s3PicsimlabSocState *s = ESP32S3_PICSIMLAB_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? SOC_RESET_PROCPU : SOC_RESET_APPCPU;
        /* An APP CPU reset must not end QEMU under -no-reboot */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        qemu_system_reset_request(cause);
    }
}

static void soc_reset(DeviceState *dev)
{
    Esp32s3PicsimlabSocState *s = ESP32S3_PICSIMLAB_SOC(dev);
    if (s->requested_reset == 0) {
        s->requested_reset = SOC_RESET_ALL;
    }
    if (s->requested_reset & SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->intmatrix));
        for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart[i]));
        }
    }
    if (s->requested_reset & SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    if (s->requested_reset & SOC_RESET_APPCPU) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[1]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    s->requested_reset = 0;
}

static void soc_cpu_stall(void *opaque, int n, int level)
{
}

static void soc_clk_update(void *opaque, int n, int level)
{
}

static void add_periph_device(MemoryRegion *dest, void *dev, hwaddr base)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, base, mr, 0);
}

static void soc_realize(DeviceState *dev, Error **errp)
{
    Esp32s3PicsimlabSocState *s = ESP32S3_PICSIMLAB_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());
    DeviceState *intmatrix_dev = DEVICE(&s->intmatrix);
    MemoryRegion *sys_mem = get_system_memory();

    for (int i = 0; i < ms->smp.cpus; ++i) {
        MemoryRegion *drom = g_new(MemoryRegion, 1);
        MemoryRegion *irom = g_new(MemoryRegion, 1);
        char name[20];

        snprintf(name, sizeof(name), "esp32s3.irom.cpu%d", i);
        memory_region_init_rom(irom, NULL, name, memmap[MEMREGION_IROM].size, &error_fatal);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[MEMREGION_IROM].base, irom);

        snprintf(name, sizeof(name), "esp32s3.drom.cpu%d", i);
        memory_region_init_alias(drom, NULL, name, irom, 0x40000, memmap[MEMREGION_DROM].size);
        memory_region_add_subregion(sys_mem, memmap[MEMREGION_DROM].base, drom);
    }

    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast = g_new(MemoryRegion, 1);
    memory_region_init_ram(iram, NULL, "esp32s3.iram", memmap[MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[MEMREGION_IRAM].base, iram);
    memory_region_init_ram(rtcslow, NULL, "esp32s3.rtcslow", memmap[MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[MEMREGION_RTCSLOW].base, rtcslow);
    memory_region_init_ram(rtcfast, NULL, "esp32s3.rtcfast", memmap[MEMREGION_RTCFAST].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[MEMREGION_RTCFAST].base, rtcfast);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }

    for (int i = 0; i < ESP32S3_CPU_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), name, OBJECT(qemu_get_cpu(i)), &error_abort);
    }
    qdev_realize(DEVICE(&s->intmatrix), &s->periph_bus, &error_fatal);

    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTCCNTL_BASE);
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32S3_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32S3_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32S3_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32S3_RTC_CPU_STALL_GPIO, i));
    }

    for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
        const hwaddr uart_base[] = { DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE };
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    /* APB_CTRL_DATE_REG: ECO3 silicon revision */
    MemoryRegion *apbctrl_mem = g_new(MemoryRegion, 1);
    memory_region_init_ram(apbctrl_mem, NULL, "esp32s3.apbctrl", 0x400, &error_fatal);
    memory_region_add_subregion(sys_mem, DR_REG_APB_CTRL_BASE, apbctrl_mem);
    uint32_t apb_ctrl_date_reg_val = 0x16042000 | 0x80000000;
    uint32_t qemu_sig = RGB_QEMU_ORIGIN;
    cpu_physical_memory_write(DR_REG_APB_CTRL_BASE + 0x7c, &apb_ctrl_date_reg_val, 4);
    cpu_physical_memory_write(DR_REG_APB_CTRL_BASE + RGB_QEMU_ORIGIN_REG, &qemu_sig, 4);

    qemu_register_reset((QEMUResetHandler *) soc_reset, dev);
}

static void soc_init(Object *obj)
{
    Esp32s3PicsimlabSocState *s = ESP32S3_PICSIMLAB_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    MemoryRegion *system_memory = get_system_memory();
    char name[16];

    qbus_init(&s->periph_bus, sizeof(s->periph_bus), TYPE_SYSTEM_BUS, DEVICE(s), "esp32-periph-bus");
    qbus_init(&s->rtc_bus, sizeof(s->rtc_bus), TYPE_SYSTEM_BUS, DEVICE(s), "esp32-rtc-bus");

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);
        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32S3_CPU);
        s->cpu[i].env.ext = qemu_memalign(16, sizeof(CPUXtensaEsp32s3State));
        s->cpu[i].env.sregs[PRID] = (i == 0) ? 0xcdcd : 0xabab;

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState *cs = CPU(&s->cpu[i]);
        cs->num_ases = 1;
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32S3_UART);
        qdev_prop_set_uint8(DEVICE(&s->uart[i]), "index", i);
    }
    qdev_prop_set_chr(DEVICE(&s->uart[0]), "chardev", serial_hd(0));

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32S3_INTMATRIX);
    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32S3_RTC_CNTL);

    qdev_init_gpio_in_named(DEVICE(s), soc_dig_reset, ESP32S3_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), soc_cpu_reset, ESP32S3_RTC_CPU_RESET_GPIO, ESP32S3_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), soc_cpu_stall, ESP32S3_RTC_CPU_STALL_GPIO, ESP32S3_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), soc_clk_update, ESP32S3_RTC_CLK_UPDATE_GPIO, 1);
}

static void soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = soc_realize;
}

static const TypeInfo soc_info = {
    .name = TYPE_ESP32S3_PICSIMLAB_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32s3PicsimlabSocState),
    .instance_init = soc_init,
    .class_init = soc_class_init,
};

/* ---------------------------------------------------------------------------
 * Machine
 */

static uint64_t io_read(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static void io_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
}

/* Peripherals without a model read as zero and ignore writes */
static const MemoryRegionOps io_ops = {
    .read = io_read,
    .write = io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#define MB (1024 * 1024)

static void init_spi_flash(Esp32s3PicsimlabSocState *ss, BlockBackend *blk)
{
    DeviceState *spi_master = DEVICE(&ss->spi1);
    BusState *spi_bus = qdev_get_child_bus(spi_master, "spi");
    const char *flash_model;

    switch (blk_getlength(blk)) {
    case 2 * MB:
        flash_model = "w25x16";
        break;
    case 4 * MB:
        flash_model = "gd25q32";
        break;
    case 8 * MB:
        flash_model = "gd25q64";
        break;
    case 16 * MB:
        flash_model = "is25lp128";
        break;
    default:
        error_report("Drive size error: only 2, 4, 8, and 16MB images are supported");
        exit(1);
    }

    DeviceState *flash_dev = qdev_new(flash_model);
    qdev_prop_set_drive(flash_dev, "drive", blk);
    qdev_realize(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void realize_mapped(void *dev, hwaddr base)
{
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion_overlap(get_system_memory(), base,
                                        sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0), 0);
}

static void connect_irq(void *dev, int n, DeviceState *intmatrix, int source)
{
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), n, qdev_get_gpio_in(intmatrix, source));
}

static void load_rom(XtensaCPU *cpu)
{
    char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32s3_rev0_rom.bin");
    if (rom_binary == NULL) {
        error_report("Error: ROM code binary esp32s3_rev0_rom.bin not found");
        exit(1);
    }
    if (load_image_targphys_as(rom_binary, memmap[MEMREGION_IROM].base,
                               memmap[MEMREGION_IROM].size, CPU(cpu)->as) < 0) {
        error_report("Error: could not load ROM binary '%s'", rom_binary);
        exit(1);
    }
    g_free(rom_binary);
}

static void machine_init(MachineState *machine)
{
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    BlockBackend *blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
    MemoryRegion *sys_mem = get_system_memory();
    Esp32s3PicsimlabMachineState *ms = ESP32S3_PICSIMLAB_MACHINE(machine);

    object_initialize_child(OBJECT(ms), "soc", &ms->soc, TYPE_ESP32S3_PICSIMLAB_SOC);
    Esp32s3PicsimlabSocState *ss = &ms->soc;
    board_soc = ss;
    ss->flash_blk = blk;

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    memory_region_init_ram(dram, NULL, "esp32s3.dram", memmap[MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[MEMREGION_DRAM].base, dram);

    memory_region_init_io(&ss->iomem, OBJECT(&ss->cpu[0]), &io_ops, NULL, "esp32s3.iomem", 0xd1000);
    memory_region_add_subregion(sys_mem, ESP32S3_IO_START_ADDR, &ss->iomem);

    qdev_realize(DEVICE(ss), NULL, &error_fatal);

    object_initialize_child(OBJECT(ss), "extmem", &ss->cache, TYPE_ESP32S3_CACHE);
    object_initialize_child(OBJECT(ss), "spi1", &ss->spi1, TYPE_ESP32S3_SPI);
    object_initialize_child(OBJECT(ss), "efuse", &ss->efuse, TYPE_ESP32S3_EFUSE);
    object_initialize_child(OBJECT(ss), "jtag", &ss->jtag, TYPE_ESP32C3_JTAG);
    object_initialize_child(OBJECT(ss), "gpio", &ss->gpio, TYPE_ESP32S3_GPIO);
    object_initialize_child(OBJECT(ss), "iomux", &ss->iomux, TYPE_ESP32S3_IOMUX);
    object_initialize_child(OBJECT(ss), "ana", &ss->ana, TYPE_ESP32S3_ANA);
    object_initialize_child(OBJECT(ss), "fe", &ss->fe, TYPE_ESP32_FE);
    object_initialize_child(OBJECT(ss), "phya", &ss->phya, TYPE_ESP32_PHYA);
    object_initialize_child(OBJECT(ss), "ledc", &ss->ledc, TYPE_ESP32S3_LEDC);
    object_initialize_child(OBJECT(ss), "sens", &ss->sens, TYPE_ESP32S3_SENS);
    object_initialize_child(OBJECT(ss), "i2c0", &ss->i2c0, TYPE_ESP32S3_I2C);
    object_initialize_child(OBJECT(ss), "gpspi2", &ss->gpspi2, TYPE_ESP32S3_GPSPI);
    object_initialize_child(OBJECT(ss), "rng", &ss->rng, TYPE_ESP32S3_RNG);
    object_initialize_child(OBJECT(ss), "clock", &ss->clock, TYPE_ESP32S3_CLOCK);
    object_initialize_child(OBJECT(ss), "gdma", &ss->gdma, TYPE_ESP32S3_GDMA);
    object_initialize_child(OBJECT(ss), "sha", &ss->sha, TYPE_ESP32S3_SHA);
    object_initialize_child(OBJECT(ss), "aes", &ss->aes, TYPE_ESP32S3_AES);
    object_initialize_child(OBJECT(ss), "rsa", &ss->rsa, TYPE_ESP32S3_RSA);
    object_initialize_child(OBJECT(ss), "hmac", &ss->hmac, TYPE_ESP32S3_HMAC);
    object_initialize_child(OBJECT(ss), "ds", &ss->ds, TYPE_ESP32S3_DS);
    object_initialize_child(OBJECT(ss), "pms", &ss->pms, TYPE_ESP32S3_PMS);
    object_initialize_child(OBJECT(ss), "xts_aes", &ss->xts_aes, TYPE_ESP32S3_XTS_AES);
    object_initialize_child(OBJECT(ss), "timg0", &ss->timg[0], TYPE_ESP32S3_TIMG);
    object_initialize_child(OBJECT(ss), "timg1", &ss->timg[1], TYPE_ESP32S3_TIMG);
    object_initialize_child(OBJECT(ss), "systimer", &ss->systimer, TYPE_ESP32S3_SYSTIMER);

    DeviceState *intmatrix_dev = DEVICE(&ss->intmatrix);
    memory_region_add_subregion_overlap(sys_mem, DR_REG_INTERRUPT_BASE,
                                        sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->intmatrix), 0), 0);

    realize_mapped(&ss->jtag, DR_REG_USB_SERIAL_JTAG_BASE);

    ss->spi1.xts_aes = &ss->xts_aes;
    realize_mapped(&ss->spi1, DR_REG_SPI1_BASE);
    if (blk) {
        init_spi_flash(ss, blk);
    }

    if (blk) {
        ss->cache.flash_blk = blk;
    }
    ss->cache.xts_aes = &ss->xts_aes;
    realize_mapped(&ss->cache, DR_REG_EXTMEM_BASE);
    memory_region_add_subregion(sys_mem, memmap[MEMREGION_DCACHE].base, &ss->cache.dcache);
    memory_region_add_subregion(sys_mem, memmap[MEMREGION_ICACHE].base, &ss->cache.icache);

    realize_mapped(&ss->efuse, DR_REG_EFUSE_BASE);
    connect_irq(&ss->efuse, 0, intmatrix_dev, ETS_EFUSE_INTR_SOURCE);

    realize_mapped(&ss->clock, DR_REG_SYSTEM_BASE);
    for (int i = 0; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
        connect_irq(&ss->clock, i, intmatrix_dev, ETS_FROM_CPU_INTR0_SOURCE + i);
    }

    const hwaddr timg_base[] = { DR_REG_TIMERGROUP0_BASE, DR_REG_TIMERGROUP1_BASE };
    const int timg_source[][3] = {
        { ETS_TG0_T0_LEVEL_INTR_SOURCE, ETS_TG0_T1_LEVEL_INTR_SOURCE, ETS_TG0_WDT_LEVEL_INTR_SOURCE },
        { ETS_TG1_T0_LEVEL_INTR_SOURCE, ETS_TG1_T1_LEVEL_INTR_SOURCE, ETS_TG1_WDT_LEVEL_INTR_SOURCE },
    };
    for (int i = 0; i < 2; i++) {
        realize_mapped(&ss->timg[i], timg_base[i]);
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[i]), ESP32S3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, timg_source[i][0]));
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[i]), ESP32S3_T1_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, timg_source[i][1]));
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[i]), ESP32S3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, timg_source[i][2]));
    }

    realize_mapped(&ss->systimer, DR_REG_SYSTIMER_BASE);
    for (int i = 0; i < ESP_SYSTIMER_IRQ_COUNT; i++) {
        connect_irq(&ss->systimer, i, intmatrix_dev, ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE + i);
    }

    realize_mapped(&ss->gpio, DR_REG_GPIO_BASE);
    connect_irq(&ss->gpio, 0, intmatrix_dev, ETS_GPIO_INTR_SOURCE);
    connect_irq(&ss->gpio, 1, intmatrix_dev, ETS_GPIO_NMI_SOURCE);

    realize_mapped(&ss->iomux, DR_REG_IO_MUX_BASE);
    realize_mapped(&ss->ana, DR_REG_I2C_MST_BASE);
    realize_mapped(&ss->fe, DR_REG_FE_BASE);
    realize_mapped(&ss->phya, DR_REG_WIFI_PHYA_BASE);

    realize_mapped(&ss->ledc, DR_REG_LEDC_BASE);
    connect_irq(&ss->ledc, 0, intmatrix_dev, ETS_LEDC_INTR_SOURCE);

    realize_mapped(&ss->sens, DR_REG_SENS_BASE);

    realize_mapped(&ss->i2c0, DR_REG_I2C_EXT_BASE);
    connect_irq(&ss->i2c0, 0, intmatrix_dev, ETS_I2C_EXT0_INTR_SOURCE);

    qdev_realize(DEVICE(&ss->rng), &ss->periph_bus, &error_fatal);
    add_periph_device(sys_mem, &ss->rng, ESP32S3_RNG_BASE);

    object_property_set_link(OBJECT(&ss->gdma), "soc_mr", OBJECT(dram), &error_abort);
    realize_mapped(&ss->gdma, DR_REG_GDMA_BASE);
    for (int i = 0; i < ESP32S3_GDMA_CHANNEL_COUNT; i++) {
        qdev_connect_gpio_out_named(DEVICE(&ss->gdma), ESP_GDMA_IRQ_IN_NAME, i,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_DMA_IN_CH0_INTR_SOURCE + i));
        qdev_connect_gpio_out_named(DEVICE(&ss->gdma), ESP_GDMA_IRQ_OUT_NAME, i,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_DMA_OUT_CH0_INTR_SOURCE + i));
    }

    ss->sha.parent.gdma = ESP_GDMA(&ss->gdma);
    realize_mapped(&ss->sha, DR_REG_SHA_BASE);
    connect_irq(&ss->sha, 0, intmatrix_dev, ETS_SHA_INTR_SOURCE);

    ss->aes.parent.gdma = ESP_GDMA(&ss->gdma);
    realize_mapped(&ss->aes, DR_REG_AES_BASE);

    ss->gpspi2.gdma = ESP_GDMA(&ss->gdma);
    realize_mapped(&ss->gpspi2, DR_REG_SPI2_BASE);
    connect_irq(&ss->gpspi2, 0, intmatrix_dev, ETS_SPI2_INTR_SOURCE);

    /* The WiFi MAC exists when the command line gives it a NIC */
    NICInfo *nd = qemu_find_nic_info(TYPE_ESP32S3_WIFI, false, NULL);
    if (nd) {
        Esp32WifiState *wifi = ESP32_WIFI(qdev_new(TYPE_ESP32S3_WIFI));
        const uint8_t *efuse_mac = (const uint8_t *)&ss->efuse.parent.efuses.blocks.rd_mac_spi_sys_0;
        for (int i = 0; i < 6; i++) {
            wifi->macaddr[i] = efuse_mac[5 - i];
        }
        qdev_set_nic_properties(DEVICE(wifi), nd);
        realize_mapped(wifi, DR_REG_WIFI_MAC_BASE);
        connect_irq(wifi, 0, intmatrix_dev, ETS_WIFI_MAC_INTR_SOURCE);
    }
    connect_irq(&ss->aes, 0, intmatrix_dev, ETS_AES_INTR_SOURCE);

    realize_mapped(&ss->rsa, DR_REG_RSA_BASE);
    connect_irq(&ss->rsa, 0, intmatrix_dev, ETS_RSA_INTR_SOURCE);

    realize_mapped(&ss->pms, DR_REG_SENSITIVE_BASE);

    ss->hmac.parent.efuse = ESP_EFUSE(&ss->efuse);
    qdev_realize(DEVICE(&ss->hmac), &ss->periph_bus, &error_fatal);
    add_periph_device(sys_mem, &ss->hmac, DR_REG_HMAC_BASE);

    ss->ds.parent.hmac = ESP_HMAC(&ss->hmac);
    ss->ds.parent.aes = ESP_AES(&ss->aes);
    ss->ds.parent.rsa = ESP_RSA(&ss->rsa);
    ss->ds.parent.sha = ESP_SHA(&ss->sha);
    qdev_realize(DEVICE(&ss->ds), &ss->periph_bus, &error_fatal);
    add_periph_device(sys_mem, &ss->ds, DR_REG_DIGITAL_SIGNATURE_BASE);

    ss->xts_aes.efuse = ESP_EFUSE(&ss->efuse);
    ss->xts_aes.clock = &ss->clock;
    qdev_realize(DEVICE(&ss->xts_aes), &ss->periph_bus, &error_fatal);
    add_periph_device(sys_mem, &ss->xts_aes, DR_REG_AES_XTS_BASE);

    connect_host(ss);

    /* The MMU must be set up before the ROM is loaded through the CPU's view */
    cpu_reset(CPU(&ss->cpu[0]));
    for (int i = 0; i < machine->smp.cpus; i++) {
        load_rom(&ss->cpu[i]);
    }
}

static void machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32-S3 machine for PICSimLab-style hosts";
    mc->init = machine_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_ram_size = 0;
}

static const TypeInfo machine_info = {
    .name = TYPE_ESP32S3_PICSIMLAB_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32s3PicsimlabMachineState),
    .class_init = machine_class_init,
};

static void esp32s3_picsimlab_register_types(void)
{
    type_register_static(&soc_info);
    type_register_static(&machine_info);
}

type_init(esp32s3_picsimlab_register_types)
