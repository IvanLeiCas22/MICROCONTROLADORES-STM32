#include "MPU6050.h"

#define MPU6050_I2C_ADDR         (0x68 << 1)
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

// Función interna para obtener el factor de escala del acelerómetro (para calibración Z)
static int16_t MPU6050_GetAccelGravityReference(MPU6050_AccelRange_t range) {
    switch(range) {
        case MPU6050_ACCEL_RANGE_2G:  return 16384;  // ±2g  -> 16384 LSB/g
        case MPU6050_ACCEL_RANGE_4G:  return 8192;   // ±4g  -> 8192 LSB/g  
        case MPU6050_ACCEL_RANGE_8G:  return 4096;   // ±8g  -> 4096 LSB/g
        case MPU6050_ACCEL_RANGE_16G: return 2048;   // ±16g -> 2048 LSB/g
        default: return 16384;
    }
}

// Función interna de calibración
static int8_t MPU6050_PerformCalibration(MPU6050_t *dev, uint16_t samples, MPU6050_AccelRange_t accel_range) {
    if (!dev || samples == 0) return -1;
    
    // Acumuladores para promedios (usar int32_t para evitar overflow)
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    uint16_t valid_samples = 0;
    
    // Obtener referencia de gravedad según el rango configurado
    int16_t gravity_ref = MPU6050_GetAccelGravityReference(accel_range);
    
    // Tomar muestras para calibración
    for (uint16_t i = 0; i < samples; i++) {
        if (dev->iface.i2c_read(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, dev->raw, 14, dev->iface.user_ctx) == 0) {
            // Acumular valores RAW (sin calibración previa)
            int16_t raw_accel_x = (int16_t)(dev->raw[0] << 8 | dev->raw[1]);
            int16_t raw_accel_y = (int16_t)(dev->raw[2] << 8 | dev->raw[3]);
            int16_t raw_accel_z = (int16_t)(dev->raw[4] << 8 | dev->raw[5]);
            int16_t raw_gyro_x  = (int16_t)(dev->raw[8] << 8 | dev->raw[9]);
            int16_t raw_gyro_y  = (int16_t)(dev->raw[10] << 8 | dev->raw[11]);
            int16_t raw_gyro_z  = (int16_t)(dev->raw[12] << 8 | dev->raw[13]);
            
            accel_sum_x += raw_accel_x;
            accel_sum_y += raw_accel_y;
            accel_sum_z += raw_accel_z;
            gyro_sum_x += raw_gyro_x;
            gyro_sum_y += raw_gyro_y;
            gyro_sum_z += raw_gyro_z;
            valid_samples++;
        }
        
        // Pequeño delay para estabilidad (solo durante calibración en init)
        // Implementación depende del hardware - aquí un loop simple
        for (volatile uint32_t delay = 0; delay < 10000; delay++);
    }
    
    if (valid_samples < (samples / 2)) return -1; // Falló si < 50% de muestras válidas
    
    // Calcular offsets promedio
    dev->calibration.accel_offset_x = -(accel_sum_x / valid_samples);
    dev->calibration.accel_offset_y = -(accel_sum_y / valid_samples);
    dev->calibration.accel_offset_z = -(accel_sum_z / valid_samples) + gravity_ref; // Compensar gravedad
    dev->calibration.gyro_offset_x = -(gyro_sum_x / valid_samples);
    dev->calibration.gyro_offset_y = -(gyro_sum_y / valid_samples);
    dev->calibration.gyro_offset_z = -(gyro_sum_z / valid_samples);
    dev->calibration.is_calibrated = 1;
    
    return 0;
}

// --- Inicialización configurable ---
int8_t MPU6050_Init(MPU6050_t *dev, const MPU6050_Config_t *config) {
    if (!dev || !dev->iface.i2c_write || !config) return -1;
    uint8_t data;
    int8_t res = 0;

    // Inicializar estructura de calibración
    dev->calibration.accel_offset_x = 0;
    dev->calibration.accel_offset_y = 0;
    dev->calibration.accel_offset_z = 0;
    dev->calibration.gyro_offset_x = 0;
    dev->calibration.gyro_offset_y = 0;
    dev->calibration.gyro_offset_z = 0;
    dev->calibration.is_calibrated = 0;

    // Wake up (PWR_MGMT_1 = 0)
    data = 0x00;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, &data, 1, dev->iface.user_ctx);

    // Sample Rate Divider (SMPLRT_DIV). Tasa = Tasa_Giroscopio / (1 + divisor)
    // Tasa_Giroscopio es 1kHz si DLPF está habilitado.
    data = config->sample_rate_divider;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, &data, 1, dev->iface.user_ctx);

    // DLPF Config (CONFIG). El valor del enum se corresponde con el valor del registro.
    data = config->dlpf_bandwidth;
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, &data, 1, dev->iface.user_ctx);

    // Gyro config (GYRO_CONFIG). El valor del enum se mapea a los bits [4:3].
    data = (config->gyro_range << 3);
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, &data, 1, dev->iface.user_ctx);

    // Accel config (ACCEL_CONFIG). El valor del enum se mapea a los bits [4:3].
    data = (config->accel_range << 3);
    res |= dev->iface.i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, &data, 1, dev->iface.user_ctx);

    // Realizar calibración si está habilitada
    if (config->enable_calibration) {
        uint16_t cal_samples = config->calibration_samples > 0 ? config->calibration_samples : 1000;
        res |= MPU6050_PerformCalibration(dev, cal_samples, config->accel_range);
    }

    dev->ready = 1;
    return res;
}

// Procesamiento centralizado de datos RAW
static void MPU6050_ProcessRaw(MPU6050_t *dev) {
    // Procesar datos RAW
    int16_t raw_accel_x = (int16_t)(dev->raw[0] << 8 | dev->raw[1]);
    int16_t raw_accel_y = (int16_t)(dev->raw[2] << 8 | dev->raw[3]);
    int16_t raw_accel_z = (int16_t)(dev->raw[4] << 8 | dev->raw[5]);
    int16_t raw_gyro_x  = (int16_t)(dev->raw[8] << 8 | dev->raw[9]);
    int16_t raw_gyro_y  = (int16_t)(dev->raw[10] << 8 | dev->raw[11]);
    int16_t raw_gyro_z  = (int16_t)(dev->raw[12] << 8 | dev->raw[13]);
    
    // Aplicar calibración si está disponible
    if (dev->calibration.is_calibrated) {
        dev->accel_x = raw_accel_x + dev->calibration.accel_offset_x;
        dev->accel_y = raw_accel_y + dev->calibration.accel_offset_y;
        dev->accel_z = raw_accel_z + dev->calibration.accel_offset_z;
        dev->gyro_x  = raw_gyro_x + dev->calibration.gyro_offset_x;
        dev->gyro_y  = raw_gyro_y + dev->calibration.gyro_offset_y;
        dev->gyro_z  = raw_gyro_z + dev->calibration.gyro_offset_z;
    } else {
        // Sin calibración, usar valores RAW
        dev->accel_x = raw_accel_x;
        dev->accel_y = raw_accel_y;
        dev->accel_z = raw_accel_z;
        dev->gyro_x  = raw_gyro_x;
        dev->gyro_y  = raw_gyro_y;
        dev->gyro_z  = raw_gyro_z;
    }
}

// --- Lectura síncrona (bloqueante) ---
int8_t MPU6050_ReadAll(MPU6050_t *dev) {
    if (!dev || !dev->iface.i2c_read) return -1;
    if (dev->iface.i2c_read(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, dev->raw, 14, dev->iface.user_ctx) != 0)
        return -1;
    MPU6050_ProcessRaw(dev);
    dev->ready = 1;
    return 0;
}

// --- Lectura asíncrona (DMA/interrupción) ---
int8_t MPU6050_ReadAll_Async(MPU6050_t *dev, void (*cb)(MPU6050_t *dev)) {
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

// --- Nuevas funciones públicas ---

// Calibración manual (post-inicialización)
int8_t MPU6050_Calibrate(MPU6050_t *dev, uint16_t samples) {
    if (!dev) return -1;
    // Para calibración manual, necesitamos conocer el rango actual.
    // Como es agnóstico, asumimos rango por defecto o leemos del registro
    return MPU6050_PerformCalibration(dev, samples, MPU6050_ACCEL_RANGE_4G);
}

// Habilitar/deshabilitar aplicación de calibración
void MPU6050_SetCalibrationEnabled(MPU6050_t *dev, uint8_t enabled) {
    if (dev) {
        dev->calibration.is_calibrated = enabled;
    }
}

// Obtener datos RAW sin calibrar
void MPU6050_GetRawData(MPU6050_t *dev, int16_t *accel_raw, int16_t *gyro_raw) {
    if (!dev || !accel_raw || !gyro_raw) return;
    
    accel_raw[0] = (int16_t)(dev->raw[0] << 8 | dev->raw[1]);   // X
    accel_raw[1] = (int16_t)(dev->raw[2] << 8 | dev->raw[3]);   // Y  
    accel_raw[2] = (int16_t)(dev->raw[4] << 8 | dev->raw[5]);   // Z
    gyro_raw[0]  = (int16_t)(dev->raw[8] << 8 | dev->raw[9]);   // X
    gyro_raw[1]  = (int16_t)(dev->raw[10] << 8 | dev->raw[11]); // Y
    gyro_raw[2]  = (int16_t)(dev->raw[12] << 8 | dev->raw[13]); // Z
} 