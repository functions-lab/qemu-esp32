/*
 * PICSimLab / velxio host bridge shared by the xtensa picsimlab machines.
 *
 * The host registers its callbacks before qemu_init(); the machine selected
 * on the command line installs its board functions while it is created, and
 * the exported qemu_picsimlab_* entry points forward to them.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

typedef struct PicsimlabBoard {
    void (*set_pin)(int pin, int value);
    void (*set_apin)(int chn, int value);
    void (*set_apin_waveform)(int chn, const uint16_t *samples, int count, uint64_t period_ns);
    uint32_t *(*get_internals)(int cfg);
    uint32_t (*get_tiocm)(void);
    void (*uart_receive)(int id, const uint8_t *buf, int size);
    int (*flash_dump)(int64_t offset, void *buf, int bytes);
    void (*enable_spi_cs_events)(int enable);
} PicsimlabBoard;

void picsimlab_set_board(const PicsimlabBoard *board);

extern void (*picsimlab_write_pin)(int pin, int value);
extern void (*picsimlab_dir_pin)(int pin, int value);
extern int (*picsimlab_i2c_event)(const uint8_t id, const uint8_t addr, const uint16_t event);
extern uint8_t (*picsimlab_spi_event)(const uint8_t id, const uint16_t event);
extern void (*picsimlab_uart_tx_event)(const uint8_t id, const uint8_t value);
extern const short int *picsimlab_pinmap;
extern void (*picsimlab_gpio_matrix_cb)(int gpio, int signal);
extern void (*picsimlab_rmt_event)(const uint8_t channel, const uint32_t config0, const uint32_t value);
extern void (*picsimlab_spi_event_batch)(const uint8_t id, const uint8_t *mosi, int len);

/*
 * The firmware asked the hardware for an operation no model covers: the host
 * fails the run with this message and the guest stops. The caller leaves its
 * device unchanged and returns. Only the first rejection is reported.
 */
void picsimlab_reject(const char *fmt, ...) G_GNUC_PRINTF(1, 2);

/* picsimlab_dir_pin(-1, value) codes */
#define PICSIMLAB_SYNC_GPIO_IN  (-1)
#define PICSIMLAB_SYNC_LEDC     0x5000

/* picsimlab_spi_event() low byte */
#define PICSIMLAB_SPI_BYTE      0x00
#define PICSIMLAB_SPI_CS        0x01

/* picsimlab_i2c_event() low byte; WRITE carries its byte in the high byte */
#define PICSIMLAB_I2C_START_RECV  0x00
#define PICSIMLAB_I2C_START_SEND  0x01
#define PICSIMLAB_I2C_FINISH      0x03
#define PICSIMLAB_I2C_NACK        0x04
#define PICSIMLAB_I2C_WRITE       0x05
#define PICSIMLAB_I2C_READ        0x06
