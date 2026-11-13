/*
 * PICSimLab / velxio host bridge shared by the xtensa picsimlab machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "hw/xtensa/picsimlab.h"

/* The host's callback table, field for field. */
typedef struct {
    void (*write_pin)(int pin, int value);
    void (*dir_pin)(int pin, int value);
    int (*i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event);
    uint8_t (*spi_event)(const uint8_t id, const uint16_t event);
    void (*uart_tx_event)(const uint8_t id, const uint8_t value);
    const short int *pinmap;
    void (*gpio_matrix_cb)(int gpio, int signal);
    void (*rmt_event)(const uint8_t channel, const uint32_t config0, const uint32_t value);
    void (*spi_event_batch)(const uint8_t id, const uint8_t *mosi, int len);
} callbacks_t;

static void no_pin(int pin, int value) {}
static int no_i2c(const uint8_t id, const uint8_t addr, const uint16_t event) { return 0; }
static uint8_t no_spi(const uint8_t id, const uint16_t event) { return 0; }
static void no_uart(const uint8_t id, const uint8_t value) {}
static void no_matrix(int gpio, int signal) {}
static void no_rmt(const uint8_t channel, const uint32_t config0, const uint32_t value) {}
static void no_spi_batch(const uint8_t id, const uint8_t *mosi, int len) {}

void (*picsimlab_write_pin)(int pin, int value) = no_pin;
void (*picsimlab_dir_pin)(int pin, int value) = no_pin;
int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event) = no_i2c;
uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event) = no_spi;
void (*picsimlab_uart_tx_event)(const uint8_t id, const uint8_t value) = no_uart;
const short int *picsimlab_pinmap;
void (*picsimlab_gpio_matrix_cb)(int gpio, int signal) = no_matrix;
void (*picsimlab_rmt_event)(const uint8_t channel, const uint32_t config0, const uint32_t value) = no_rmt;
void (*picsimlab_spi_event_batch)(const uint8_t id, const uint8_t *mosi, int len) = no_spi_batch;

static const PicsimlabBoard *board;

void picsimlab_set_board(const PicsimlabBoard *b)
{
    board = b;
}

void qemu_picsimlab_register_callbacks(void *arg);
void qemu_picsimlab_set_pin(int pin, int value);
void qemu_picsimlab_set_apin(int chn, int value);
void qemu_picsimlab_set_apin_waveform(int chn, const uint16_t *samples, int count, uint64_t period_ns);
uint32_t *qemu_picsimlab_get_internals(int cfg);
uint32_t qemu_picsimlab_get_TIOCM(void);
void qemu_picsimlab_uart_receive(const int id, const uint8_t *buf, int size);
int qemu_picsimlab_flash_dump(int64_t offset, void *buf, int bytes);
void qemu_picsimlab_enable_spi_cs_events(int enable);

void qemu_picsimlab_register_callbacks(void *arg)
{
    const callbacks_t *callbacks = arg;

    picsimlab_write_pin = callbacks->write_pin;
    picsimlab_dir_pin = callbacks->dir_pin;
    picsimlab_i2c_event = callbacks->i2c_event;
    picsimlab_spi_event = callbacks->spi_event;
    picsimlab_uart_tx_event = callbacks->uart_tx_event;
    picsimlab_pinmap = callbacks->pinmap;
    picsimlab_gpio_matrix_cb = callbacks->gpio_matrix_cb;
    picsimlab_rmt_event = callbacks->rmt_event;
    picsimlab_spi_event_batch = callbacks->spi_event_batch;
}

void qemu_picsimlab_set_pin(int pin, int value)
{
    board->set_pin(pin, value);
}

void qemu_picsimlab_set_apin(int chn, int value)
{
    board->set_apin(chn, value);
}

void qemu_picsimlab_set_apin_waveform(int chn, const uint16_t *samples, int count, uint64_t period_ns)
{
    if (!board->set_apin_waveform) {
        error_report("picsimlab: this machine has no ADC waveform input");
        abort();
    }
    board->set_apin_waveform(chn, samples, count, period_ns);
}

uint32_t *qemu_picsimlab_get_internals(int cfg)
{
    return board->get_internals(cfg);
}

uint32_t qemu_picsimlab_get_TIOCM(void)
{
    return board->get_tiocm();
}

void qemu_picsimlab_uart_receive(const int id, const uint8_t *buf, int size)
{
    board->uart_receive(id, buf, size);
}

int qemu_picsimlab_flash_dump(int64_t offset, void *buf, int bytes)
{
    return board->flash_dump(offset, buf, bytes);
}

void qemu_picsimlab_enable_spi_cs_events(int enable)
{
    if (!board->enable_spi_cs_events) {
        error_report("picsimlab: this machine has no SPI chip-select events");
        abort();
    }
    board->enable_spi_cs_events(enable);
}
