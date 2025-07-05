#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/**
 * @file MPU6050.h
 * @brief Driver portable y agnóstico para MPU6050 (solo C estándar, sin dependencias de hardware).
 */

// Interfaz de bajo nivel para acceso I2C (el usuario debe implementarla)
typedef struct {
    // Escritura síncrona: retorna 0 en éxito, <0 en error
    int (*i2c_write)(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, void *user_ctx);
    // Lectura síncrona: retorna 0 en éxito, <0 en error
    int (*i2c_read)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx);
    // Lectura asíncrona: retorna 0 en éxito, <0 en error. cb se llama al finalizar, cb_ctx es el contexto del callback.
    int (*i2c_read_async)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx, void (*cb)(void *cb_ctx), void *cb_ctx);
    void *user_ctx; // Contexto de usuario (ej: handler, struct, etc.)
} MPU6050_I2C_Interface_t;

// Estructura principal del driver
typedef struct MPU6050_t{
    MPU6050_I2C_Interface_t iface;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    uint8_t raw[14];
    volatile uint8_t ready;
    void (*user_cb)(struct MPU6050_t *dev); // Callback de usuario para async
} MPU6050_t;

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el sensor (wake up, configuración básica)
int MPU6050_Init(MPU6050_t *dev);

// Lee acelerómetro y giroscopio (bloqueante)
int MPU6050_ReadAll(MPU6050_t *dev);

// Inicia la lectura asíncrona (DMA/interrupción)
int MPU6050_ReadAll_Async(MPU6050_t *dev, void (*cb)(MPU6050_t *dev));

// Procesa los datos RAW y notifica al usuario (llamar desde el callback de bajo nivel)
void MPU6050_OnDataReady(MPU6050_t *dev);

#ifdef __cplusplus
}
#endif

/**
 * Ejemplo de integración:
 *
 * // Implementación de funciones de bajo nivel (en tu código, no en el driver)
 * int my_i2c_write(...);
 * int my_i2c_read(...);
 * int my_i2c_read_async(...);
 *
 * // Inicialización:
 * MPU6050_t mpu;
 * mpu.iface.i2c_write = my_i2c_write;
 * mpu.iface.i2c_read = my_i2c_read;
 * mpu.iface.i2c_read_async = my_i2c_read_async;
 * mpu.iface.user_ctx = ...;
 * MPU6050_Init(&mpu);
 *
 * // Lectura síncrona:
 * MPU6050_ReadAll(&mpu);
 * // Los datos quedan en mpu.accel_x, etc.
 *
 * // Lectura asíncrona:
 * void mpu_user_callback(MPU6050_t *dev) { ... }
 * MPU6050_ReadAll_Async(&mpu, mpu_user_callback);
 */

#endif // MPU6050_H 