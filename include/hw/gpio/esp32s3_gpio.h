#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_GPIO "esp32s3.gpio"
#define ESP32S3_GPIO(obj) OBJECT_CHECK(ESP32S3GPIOState, (obj), TYPE_ESP32S3_GPIO)

/* Bootstrap options for ESP32-S3 (4-bit) */
#define ESP32S3_STRAP_MODE_FLASH_BOOT 0x4   /* SPI Boot */

#define ESP32S3_GPIO_COUNT          49
#define ESP32S3_GPIO_IN_SIGNALS     256
#define ESP32S3_GPIO_SIG_GPIO_OUT   0x100

/* Output: level the GPIO_OUT register drives onto a pin routed to it */
#define ESP32S3_GPIOS        "esp32s3_gpios"
/* Output: output enable of the pin */
#define ESP32S3_GPIOS_DIR    "esp32s3_gpios_dir"
/* Output: the guest is about to read GPIO_IN / GPIO_IN1 */
#define ESP32S3_GPIOS_SYNC   "esp32s3_gpios_sync"
/* Output: FUNCn_OUT_SEL was written; the level is the 9-bit signal index */
#define ESP32S3_GPIOS_MATRIX "esp32s3_gpios_matrix"
/* Input: level the outside world puts on the pin */
#define ESP32S3_GPIOS_IN     "esp32s3_gpios_in"

REG32(GPIO_OUT, 0x04)
REG32(GPIO_OUT_W1TS, 0x08)
REG32(GPIO_OUT_W1TC, 0x0C)
REG32(GPIO_OUT1, 0x10)
REG32(GPIO_OUT1_W1TS, 0x14)
REG32(GPIO_OUT1_W1TC, 0x18)
REG32(GPIO_ENABLE, 0x20)
REG32(GPIO_ENABLE_W1TS, 0x24)
REG32(GPIO_ENABLE_W1TC, 0x28)
REG32(GPIO_ENABLE1, 0x2C)
REG32(GPIO_ENABLE1_W1TS, 0x30)
REG32(GPIO_ENABLE1_W1TC, 0x34)
REG32(GPIO_STRAP, 0x38)
REG32(GPIO_IN, 0x3C)
REG32(GPIO_IN1, 0x40)
REG32(GPIO_STATUS, 0x44)
REG32(GPIO_STATUS_W1TS, 0x48)
REG32(GPIO_STATUS_W1TC, 0x4C)
REG32(GPIO_STATUS1, 0x50)
REG32(GPIO_STATUS1_W1TS, 0x54)
REG32(GPIO_STATUS1_W1TC, 0x58)
REG32(GPIO_PCPU_INT, 0x5C)
REG32(GPIO_PCPU_NMI_INT, 0x60)
REG32(GPIO_CPUSDIO_INT, 0x64)
REG32(GPIO_PCPU_INT1, 0x68)
REG32(GPIO_PCPU_NMI_INT1, 0x6C)
REG32(GPIO_CPUSDIO_INT1, 0x70)
REG32(GPIO_PIN0, 0x74)
    FIELD(GPIO_PIN0, INT_TYPE, 7, 3)
    FIELD(GPIO_PIN0, INT_ENA, 13, 5)
REG32(GPIO_FUNC0_IN_SEL_CFG, 0x154)
REG32(GPIO_FUNC0_OUT_SEL_CFG, 0x554)
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, OUT_SEL, 0, 9)
REG32(GPIO_CLOCK_GATE, 0x62C)
REG32(GPIO_DATE, 0x6FC)

typedef struct ESP32S3GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq nmi_irq;
    qemu_irq level_out[ESP32S3_GPIO_COUNT];
    qemu_irq dir_out[ESP32S3_GPIO_COUNT];
    qemu_irq matrix_out[ESP32S3_GPIO_COUNT];
    qemu_irq sync;

    uint32_t strap_mode;
    /* Index 0: GPIO0-31, index 1: GPIO32-48 */
    uint32_t out[2];
    uint32_t enable[2];
    uint32_t in[2];
    uint32_t status[2];
    uint32_t pin[ESP32S3_GPIO_COUNT];
    uint32_t in_sel[ESP32S3_GPIO_IN_SIGNALS];
    uint32_t out_sel[ESP32S3_GPIO_COUNT];
    uint32_t clock_gate;
} ESP32S3GPIOState;
