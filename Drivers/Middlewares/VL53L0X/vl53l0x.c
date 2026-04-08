#include "vl53l0x.h"

/* --- Private I2C Wrappers --- */
static void writeReg(VL53L0X_Dev *dev, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(dev->i2c, dev->address, reg, 1, &value, 1, dev->timeout_ms);
}
static void writeReg16Bit(VL53L0X_Dev *dev, uint8_t reg, uint16_t value) {
    uint8_t buf[2] = { (value >> 8) & 0xFF, value & 0xFF };
    HAL_I2C_Mem_Write(dev->i2c, dev->address, reg, 1, buf, 2, dev->timeout_ms);
}
static uint8_t readReg(VL53L0X_Dev *dev, uint8_t reg) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, 1, &value, 1, dev->timeout_ms);
    return value;
}
static uint16_t readReg16Bit(VL53L0X_Dev *dev, uint8_t reg) {
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, 1, buf, 2, dev->timeout_ms);
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

/* --- Public API --- */
void VL53L0X_SetAddress(VL53L0X_Dev *dev, uint8_t new_addr) {
    writeReg(dev, 0x8A, (new_addr >> 1) & 0x7F); /* 0x8A is I2C_SLAVE_DEVICE_ADDRESS */
    dev->address = new_addr;
}

uint8_t VL53L0X_Init(VL53L0X_Dev *dev, I2C_HandleTypeDef *hi2c, uint8_t initial_addr) {
    dev->i2c = hi2c;
    dev->address = initial_addr;
    dev->timeout_ms = 100;

    /* Check if sensor is alive by reading a known register */
    if (readReg(dev, 0xC0) != 0xEE) return 0; /* Model ID should be 0xEE */

    writeReg(dev, 0x88, 0x00);
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    dev->stop_variable = readReg(dev, 0x91);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);

    /* Disable MSRC and PRE_RANGE limit checks */
    writeReg(dev, 0x60, readReg(dev, 0x60) | 0x12);

    /* --- ST Microelectronics Magic Tuning Settings --- */
    writeReg(dev, 0xFF, 0x01); writeReg(dev, 0x00, 0x00); writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x09, 0x00); writeReg(dev, 0x10, 0x00); writeReg(dev, 0x11, 0x00);
    writeReg(dev, 0x24, 0x01); writeReg(dev, 0x25, 0xFF); writeReg(dev, 0x75, 0x00);
    writeReg(dev, 0xFF, 0x01); writeReg(dev, 0x4E, 0x2C); writeReg(dev, 0x48, 0x00);
    writeReg(dev, 0x30, 0x20); writeReg(dev, 0xFF, 0x00); writeReg(dev, 0x30, 0x09);
    writeReg(dev, 0x54, 0x00); writeReg(dev, 0x31, 0x04); writeReg(dev, 0x32, 0x03);
    writeReg(dev, 0x40, 0x83); writeReg(dev, 0x46, 0x25); writeReg(dev, 0x60, 0x00);
    writeReg(dev, 0x27, 0x00); writeReg(dev, 0x50, 0x06); writeReg(dev, 0x51, 0x00);
    writeReg(dev, 0x52, 0x96); writeReg(dev, 0x56, 0x08); writeReg(dev, 0x57, 0x30);
    writeReg(dev, 0x61, 0x00); writeReg(dev, 0x62, 0x00); writeReg(dev, 0x64, 0x00);
    writeReg(dev, 0x65, 0x00); writeReg(dev, 0x66, 0xA0); writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x22, 0x32); writeReg(dev, 0x47, 0x14); writeReg(dev, 0x49, 0xFF);
    writeReg(dev, 0x4A, 0x00); writeReg(dev, 0xFF, 0x00); writeReg(dev, 0x7A, 0x0A);
    writeReg(dev, 0x7B, 0x00); writeReg(dev, 0x78, 0x21); writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x23, 0x34); writeReg(dev, 0x42, 0x00); writeReg(dev, 0x44, 0xFF);
    writeReg(dev, 0x45, 0x26); writeReg(dev, 0x46, 0x05); writeReg(dev, 0x40, 0x40);
    writeReg(dev, 0x0E, 0x06); writeReg(dev, 0x20, 0x1A); writeReg(dev, 0x43, 0x40);
    writeReg(dev, 0xFF, 0x00); writeReg(dev, 0x34, 0x03); writeReg(dev, 0x35, 0x44);
    writeReg(dev, 0xFF, 0x01); writeReg(dev, 0x31, 0x04); writeReg(dev, 0x4B, 0x09);
    writeReg(dev, 0x4C, 0x05); writeReg(dev, 0x4D, 0x04); writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x44, 0x00); writeReg(dev, 0x45, 0x20); writeReg(dev, 0x47, 0x08);
    writeReg(dev, 0x48, 0x28); writeReg(dev, 0x67, 0x00); writeReg(dev, 0x70, 0x04);
    writeReg(dev, 0x71, 0x01); writeReg(dev, 0x72, 0xFE); writeReg(dev, 0x76, 0x00);
    writeReg(dev, 0x77, 0x00); writeReg(dev, 0xFF, 0x01); writeReg(dev, 0x0D, 0x01);
    writeReg(dev, 0xFF, 0x00); writeReg(dev, 0x80, 0x01); writeReg(dev, 0x01, 0xF8);
    writeReg(dev, 0xFF, 0x01); writeReg(dev, 0x8E, 0x01); writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00); writeReg(dev, 0x80, 0x00);
    /* ---------------------------------------------------- */

    writeReg(dev, 0x0A, 0x04);
    writeReg(dev, 0x84, readReg(dev, 0x84) & ~0x10);
    writeReg(dev, 0x0B, 0x01);

    return 1; /* Success */
}

uint16_t VL53L0X_ReadDistance(VL53L0X_Dev *dev) {
    writeReg(dev, 0x80, 0x01);
    writeReg(dev, 0xFF, 0x01);
    writeReg(dev, 0x00, 0x00);
    writeReg(dev, 0x91, dev->stop_variable);
    writeReg(dev, 0x00, 0x01);
    writeReg(dev, 0xFF, 0x00);
    writeReg(dev, 0x80, 0x00);
    writeReg(dev, 0x00, 0x01); /* SYSRANGE_START */

    uint32_t tick = HAL_GetTick();
    while (readReg(dev, 0x00) & 0x01) {
        if (HAL_GetTick() - tick > dev->timeout_ms) return 65535;
    }

    tick = HAL_GetTick();
    while ((readReg(dev, 0x13) & 0x07) == 0) {
        if (HAL_GetTick() - tick > dev->timeout_ms) return 65535;
    }

    uint16_t range = readReg16Bit(dev, 0x14 + 10);
    writeReg(dev, 0x0B, 0x01); /* SYSTEM_INTERRUPT_CLEAR */

    return range;
}
