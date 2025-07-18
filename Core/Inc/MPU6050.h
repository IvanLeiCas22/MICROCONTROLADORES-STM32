// Direcciones de registros
#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_WHO_AM_I_ADDR 0x75
#define MPU6050_PWR_MGMT_1_ADDR 0x6B
#define MPU6050_ACCEL_CONFIG_ADDR 0x1C
#define MPU6050_GYRO_CONFIG_ADDR 0x1B
#define MPU6050_CONFIG_ADDR 0x1A
#define MPU6050_ACCEL_XOUT_H_ADDR 0x3B

// Valores esperados y de configuración
#define MPU6050_EXPECTED_WHO_AM_I_VALUE 0x68
#define MPU6050_PWR_MGMT_1_RESET_BIT 0x80    // Bit 7 para DEVICE_RESET
#define MPU6050_PWR_MGMT_1_WAKEUP_VALUE 0x00 // Valor para salir del modo sleep

// Longitudes de datos
#define MPU6050_SINGLE_BYTE_LEN 1
#define MPU6050_RAW_DATA_LEN 14 // 14 bytes para Accel X,Y,Z, Temp, Gyro X,Y,Z

// Tiempos de retardo
#define MPU6050_INIT_DELAY_MS 100
#define MPU6050_RESET_DELAY_MS 1000

// Códigos de retorno
#define MPU6050_OK 1
#define MPU6050_I2C_ERROR -1
#define MPU6050_ERROR_WRONG_DEVICE_ID -2
#define MPU6050_ERROR_WAKEUP_FAILED -3
#define MPU6050_ERROR_ACCEL_CONFIG_FAILED -4
#define MPU6050_ERROR_GYRO_CONFIG_FAILED -5
#define MPU6050_ERROR_DLPF_CONFIG_FAILED -6
#define MPU6050_ERROR_I2C_FUNCS_NOT_ASSIGNED -7
#define MPU6050_ERROR_RESET_FAILED -8
#define MPU6050_ERROR_WHO_AM_I_READ_FAILED -9

typedef enum
{
    MPU6050_ACCEL_RANGE_2G = 0x00, // ±2g
    MPU6050_ACCEL_RANGE_4G = 0x08, // ±4g
    MPU6050_ACCEL_RANGE_8G = 0x10, // ±8g
    MPU6050_ACCEL_RANGE_16G = 0x18 // ±16g
} MPU6050_AccelRangeTypeDef;

typedef enum
{
    MPU6050_GYRO_RANGE_250DPS = 0x00,  // ±250°/s
    MPU6050_GYRO_RANGE_500DPS = 0x08,  // ±500°/s
    MPU6050_GYRO_RANGE_1000DPS = 0x10, // ±1000°/s
    MPU6050_GYRO_RANGE_2000DPS = 0x18  // ±2000°/s
} MPU6050_GyroRangeTypeDef;

typedef enum
{
    MPU6050_DLPF_260HZ = 0x00, // 260Hz, 256Hz
    MPU6050_DLPF_184HZ = 0x01, // 184Hz, 188Hz
    MPU6050_DLPF_94HZ = 0x02,  // 94Hz, 98Hz
    MPU6050_DLPF_44HZ = 0x03,  // 44Hz, 42Hz
    MPU6050_DLPF_21HZ = 0x04,  // 21Hz, 20Hz
    MPU6050_DLPF_10HZ = 0x05,  // 10Hz, 10Hz
    MPU6050_DLPF_5HZ = 0x06    // 5Hz, 5Hz
} MPU6050_DLPFTypeDef;

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;
} MPU6050_RawDataTypeDef;

typedef struct
{
    // Configuraciones
    MPU6050_AccelRangeTypeDef accel_range;
    MPU6050_GyroRangeTypeDef gyro_range;
    MPU6050_DLPFTypeDef dlpf_config;

    // Datos raw
    MPU6050_RawDataTypeDef raw_data;
    uint8_t dma_buffer[14];

    // Estado
    uint8_t is_initialized;
    uint8_t is_connected;

    // Offsets de calibración
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;

    // Punteros a funciones I2C (desacoplamiento)
    int8_t (*i2c_write_blocking)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
    int8_t (*i2c_read_blocking)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
    int8_t (*i2c_write_dma)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);
    int8_t (*i2c_read_dma)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t data_len, void *context);

    // Contexto I2C
    void *i2c_context;

    // Puntero a función de retardo
    void (*delay_ms)(uint32_t ms);

    // Dirección del dispositivo
    uint8_t device_address;

} MPU6050_HandleTypeDef;

int8_t MPU6050_Init(MPU6050_HandleTypeDef *hmpu);
int8_t MPU6050_ReadRawDataDMA(MPU6050_HandleTypeDef *hmpu);

// Calibración y lectura calibrada
void MPU6050_Calibrate(MPU6050_HandleTypeDef *hmpu, uint16_t samples);
void MPU6050_GetCalibratedData(MPU6050_HandleTypeDef *hmpu, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);