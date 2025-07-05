#include "MPU6050.h"

#define MPU6050_I2C_ADDR         (0x68 << 1)
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

// --- Inicialización avanzada para micromouse ---
int MPU6050_Init(MPU6050_t *dev) {
    if (!dev || !dev->iface.i2c_write) return -1;
    uint8_t data;
    int res = 0;

    // Wake up (PWR_MGMT_1 = 0)
    data = 0x00;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, &data, 1, dev->iface.user_ctx);

    // Sample Rate Divider (SMPLRT_DIV = 9) -> 100Hz
    data = 0x09;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, &data, 1, dev->iface.user_ctx);

    // DLPF Config (CONFIG = 0x04) -> 20Hz
    data = 0x04;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, &data, 1, dev->iface.user_ctx);

    // Gyro config (GYRO_CONFIG = 0x08) -> ±500 °/s
    data = 0x08;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, &data, 1, dev->iface.user_ctx);

    // Accel config (ACCEL_CONFIG = 0x08) -> ±4g
    data = 0x08;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, &data, 1, dev->iface.user_ctx);

    dev->ready = 1;
    return res;
}

// Procesamiento centralizado de datos RAW
static void MPU6050_ProcessRaw(MPU6050_t *dev) {
    dev->accel_x = (int16_t)(dev->raw[0] << 8 | dev->raw[1]);
    dev->accel_y = (int16_t)(dev->raw[2] << 8 | dev->raw[3]);
    dev->accel_z = (int16_t)(dev->raw[4] << 8 | dev->raw[5]);
    dev->gyro_x  = (int16_t)(dev->raw[8] << 8 | dev->raw[9]);
    dev->gyro_y  = (int16_t)(dev->raw[10] << 8 | dev->raw[11]);
    dev->gyro_z  = (int16_t)(dev->raw[12] << 8 | dev->raw[13]);
}

// --- Lectura síncrona (bloqueante) ---
int MPU6050_ReadAll(MPU6050_t *dev) {
    if (!dev || !dev->iface.i2c_read) return -1;
    if (dev->iface.i2c_read(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, dev->raw, 14, dev->iface.user_ctx) != 0)
        return -1;
    MPU6050_ProcessRaw(dev);
    dev->ready = 1;
    return 0;
}

// --- Lectura asíncrona (DMA/interrupción) ---
int MPU6050_ReadAll_Async(MPU6050_t *dev, void (*cb)(MPU6050_t *dev)) {
    if (!dev || !dev->iface.i2c_read_async) return -1;
    dev->ready = 0;
    dev->user_cb = cb;
    return dev->iface.i2c_read_async(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, dev->raw, 14, dev->iface.user_ctx, (void (*)(void *))MPU6050_OnDataReady, dev);
}

// --- Procesa los datos RAW y notifica al usuario ---
void MPU6050_OnDataReady(MPU6050_t *dev) {
    if (!dev) return;
    MPU6050_ProcessRaw(dev);
    dev->ready = 1;
    if (dev->user_cb) dev->user_cb(dev);
} 