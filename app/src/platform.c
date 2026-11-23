#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "platform.h"

/* the I2C bus the sensor is on: arduino_i2c = D14 (SDA) / D15 (SCL) on the Nucleo */
static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(arduino_i2c));

/* write len bytes to a 16-bit register: send [reg high][reg low][data...] */
static uint8_t wr(Dev_t dev, uint16_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[6];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        buf[2 + i] = data[i];
    }
    return i2c_write(i2c_bus, buf, len + 2, dev) != 0;
}

/* read len bytes from a 16-bit register: send [reg high][reg low], then read */
static uint8_t rd(Dev_t dev, uint16_t reg, uint8_t *data, uint8_t len)
{
    uint8_t r[2] = { reg >> 8, reg & 0xFF };

    return i2c_write_read(i2c_bus, dev, r, 2, data, len) != 0;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t reg, uint8_t value)
{
    return wr(dev, reg, &value, 1);
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t reg, uint16_t value)
{
    uint8_t b[2] = { value >> 8, value & 0xFF };   /* high byte first */

    return wr(dev, reg, b, 2);
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t reg, uint32_t value)
{
    uint8_t b[4] = { value >> 24, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF };

    return wr(dev, reg, b, 4);
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t reg, uint8_t *value)
{
    return rd(dev, reg, value, 1);
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t reg, uint16_t *value)
{
    uint8_t b[2];
    uint8_t s = rd(dev, reg, b, 2);

    *value = ((uint16_t)b[0] << 8) | b[1];   /* glue the 2 bytes back together */
    return s;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t reg, uint32_t *value)
{
    uint8_t b[4];
    uint8_t s = rd(dev, reg, b, 4);

    *value = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    return s;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
    k_msleep(TimeMs);   /* lets other threads run while waiting */
    return 0;
}
