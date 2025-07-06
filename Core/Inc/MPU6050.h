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
    int8_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len, void *user_ctx);
    // Lectura síncrona: retorna 0 en éxito, <0 en error
    int8_t (*i2c_read)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx);
    // Lectura asíncrona: retorna 0 en éxito, <0 en error. cb se llama al finalizar, cb_ctx es el contexto del callback.
    int8_t (*i2c_read_async)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len, void *user_ctx, void (*cb)(void *cb_ctx), void *cb_ctx);
    void *user_ctx; // Contexto de usuario (ej: handler, struct, etc.)
} MPU6050_I2C_Interface_t;

typedef enum {
    MPU6050_ACCEL_RANGE_2G,
    MPU6050_ACCEL_RANGE_4G,
    MPU6050_ACCEL_RANGE_8G,
    MPU6050_ACCEL_RANGE_16G
} MPU6050_AccelRange_t;

typedef enum {
    MPU6050_GYRO_RANGE_250_DPS,
    MPU6050_GYRO_RANGE_500_DPS,
    MPU6050_GYRO_RANGE_1000_DPS,
    MPU6050_GYRO_RANGE_2000_DPS
} MPU6050_GyroRange_t;

typedef enum {
    MPU6050_DLPF_BW_5HZ   = 6,  // Accel BW 5Hz,   Gyro BW 5Hz
    MPU6050_DLPF_BW_10HZ  = 5,  // Accel BW 10Hz,  Gyro BW 10Hz
    MPU6050_DLPF_BW_20HZ  = 4,  // Accel BW 20Hz,  Gyro BW 20Hz
    MPU6050_DLPF_BW_41HZ  = 3,  // Accel BW 41Hz,  Gyro BW 41Hz
    MPU6050_DLPF_BW_89HZ  = 2,  // Accel BW 89Hz,  Gyro BW 89Hz
    MPU6050_DLPF_BW_178HZ = 1,  // Accel BW 178Hz, Gyro BW 178Hz
    MPU6050_DLPF_BW_250HZ = 0   // Accel BW 250Hz, Gyro BW 250Hz
} MPU6050_DLPF_Bandwidth_t;

// Estructura de datos de calibración 
typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y; 
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    uint8_t is_calibrated;
} MPU6050_Calibration_t;

// Estructura de configuración para la inicialización
typedef struct {
    MPU6050_AccelRange_t accel_range;
    MPU6050_GyroRange_t gyro_range;
    MPU6050_DLPF_Bandwidth_t dlpf_bandwidth;
    uint8_t sample_rate_divider; // Divisor de frecuencia: 0-255
    uint8_t enable_calibration;     // 1 = calibrar durante init, 0 = omitir
    uint16_t calibration_samples;   // Número de muestras para calibración (ej: 1000)
} MPU6050_Config_t;

// Estructura principal del driver
typedef struct MPU6050_t{
    MPU6050_I2C_Interface_t iface;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    uint8_t raw[14];
    volatile uint8_t ready;
    void (*user_cb)(struct MPU6050_t *dev); // Callback de usuario para async
    MPU6050_Calibration_t calibration;  // Datos de calibración
} MPU6050_t;

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el sensor con la configuración proporcionada
int8_t MPU6050_Init(MPU6050_t *dev, const MPU6050_Config_t *config);

// Lee acelerómetro y giroscopio (bloqueante)
int8_t MPU6050_ReadAll(MPU6050_t *dev);

// Inicia la lectura asíncrona (DMA/interrupción)
int8_t MPU6050_ReadAll_Async(MPU6050_t *dev, void (*cb)(MPU6050_t *dev));

// Procesa los datos RAW y notifica al usuario (llamar desde el callback de bajo nivel)
void MPU6050_OnDataReady(MPU6050_t *dev);

// Nueva función para calibración manual (opcional)
int8_t MPU6050_Calibrate(MPU6050_t *dev, uint16_t samples);

// Función para aplicar/quitar calibración
void MPU6050_SetCalibrationEnabled(MPU6050_t *dev, uint8_t enabled);

// Función para obtener datos sin calibrar (para debugging)
void MPU6050_GetRawData(MPU6050_t *dev, int16_t *accel_raw, int16_t *gyro_raw);

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
 * // Configuración
 * MPU6050_Config_t mpu_config = {
 *     .accel_range = MPU6050_ACCEL_RANGE_4G,
 *     .gyro_range = MPU6050_GYRO_RANGE_500_DPS,
 *     .dlpf_bandwidth = MPU6050_DLPF_BW_21HZ,
 *     .sample_rate_divider = 9 // Para 100Hz: 1kHz / (1 + 9)
 * };
 *
 * // Inicialización:
 * MPU6050_t mpu;
 * mpu.iface.i2c_write = my_i2c_write;
 * mpu.iface.i2c_read = my_i2c_read;
 * mpu.iface.i2c_read_async = my_i2c_read_async;
 * mpu.iface.user_ctx = ...;
 * if (MPU6050_Init(&mpu, &mpu_config) != 0) {
 *     // Error
 * }
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