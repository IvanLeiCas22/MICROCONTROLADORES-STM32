/* STM32/Test2024-master/Core/Inc/app_config.h */
#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Este archivo centraliza todas las definiciones, enumeraciones, y tipos
 * de datos específicos de la aplicación para limpiar main.c y facilitar
 * el mantenimiento.
 */

//==============================================================================
// TIPOS DE DATOS Y ENUMERACIONES
//==============================================================================
typedef enum
{
    MOTOR_REAR_RIGHT_IDX = 0,
    MOTOR_FRONT_RIGHT_IDX,
    MOTOR_REAR_LEFT_IDX,
    MOTOR_FRONT_LEFT_IDX
} MotorIndexTypeDef;

typedef enum
{
    I2C_BUS_IDLE,
    I2C_BUS_BUSY_MPU,
    I2C_BUS_BUSY_SSD
} I2C_BusStateTypeDef;

typedef enum
{
    TURN_STATE_IDLE,
    TURN_STATE_TURNING
} TurnStateTypeDef;

typedef union
{
    struct
    {
        uint8_t b0 : 1;
        uint8_t b1 : 1;
        uint8_t b2 : 1;
        uint8_t b3 : 1;
        uint8_t b4 : 1;
        uint8_t b5 : 1;
        uint8_t b6 : 1;
        uint8_t b7 : 1;
    } bit;
    uint8_t byte;
} SystemFlagTypeDef;

typedef enum
{
    CMD_ACK = 0x0D,
    CMD_GET_ALIVE = 0xF0,
    CMD_START_CONFIG = 0xEE,
    CMD_FIRMWARE = 0xF1,
    CMD_GET_BUTTON_STATE = 0x12,
    CMD_GET_MPU_DATA = 0xA2,
    CMD_GET_LAST_ADC_VALUES = 0xA0,
    CMD_TEST_MOTORS = 0xA1,
    CMD_GET_MOTOR_SPEEDS = 0xA4,
    CMD_SET_MOTOR_PWM = 0xA5,
    CMD_GET_MOTOR_PWM = 0xA6,
    CMD_GET_LOCAL_IP_ADDRESS = 0xE0,
    CMD_SET_UART_BYPASS_CONTROL = 0xDD,
    CMD_CALIBRATE_MPU = 0xA3,
    CMD_SET_PID_GAINS = 0x40,          // Para configurar Kp, Ki, Kd
    CMD_GET_PID_GAINS = 0x41,          // Para leer Kp, Ki, Kd
    CMD_SET_CONTROL_PARAMETERS = 0x42, // Para configurar Setpoint, Base Speed, etc.
    CMD_GET_CONTROL_PARAMETERS = 0x43, // Para leer Setpoint, Base Speed, etc.
    CMD_SET_MOTOR_BASE_SPEEDS = 0x44,  // Para configurar velocidades base independientes
    CMD_GET_MOTOR_BASE_SPEEDS = 0x45,  // Para leer velocidades base independientes
    CMD_CALIBRATE_MOTORS = 0x46,       // Para calibrar motores automáticamente
    CMD_TURN_DEGREES = 0x47,           // Para girar un número de grados
    CMD_SET_TURN_PID_GAINS = 0x48,     // Para configurar Kp, Ki, Kd del PID de giro
    CMD_GET_TURN_PID_GAINS = 0x49,     // Para leer Kp, Ki, Kd del PID de giro
    CMD_SET_PWM_PERIOD = 0x50,
    CMD_GET_PWM_PERIOD = 0x51,
    CMD_OTHERS
} CommandIdTypeDef;

typedef union
{
    uint8_t u8[4];
    int8_t i8[4];
    uint16_t u16[2];
    int16_t i16[2];
    uint32_t u32;
    int32_t i32;
} DataUnionTypeDef;

//==============================================================================
// DEFINICIONES Y MACROS
//==============================================================================

/* Flags del sistema (definidos en app_core.c) */
extern SystemFlagTypeDef flags0;
#define ON10MS flags0.bit.b0
#define UART_BYPASS flags0.bit.b1
#define MPU_READ_REQUEST flags0.bit.b2
#define SSD_UPDATE_REQUEST flags0.bit.b3
#define ACTIVATE_PID flags0.bit.b4

/* MPU6050 */
#define MPU_DMA_BUFFER_SIZE 14
#define MPU_RAW_DATA_SIZE 14
#define MPU_READ_ERROR_BLINK_DELAY_MS 200
#define MPU_READ_ERROR_BLINKS 5
#define MPU_DMA_BUF_ACCEL_X_H 0
#define MPU_DMA_BUF_ACCEL_X_L 1
#define MPU_DMA_BUF_ACCEL_Y_H 2
#define MPU_DMA_BUF_ACCEL_Y_L 3
#define MPU_DMA_BUF_ACCEL_Z_H 4
#define MPU_DMA_BUF_ACCEL_Z_L 5
#define MPU_DMA_BUF_TEMP_H 6
#define MPU_DMA_BUF_TEMP_L 7
#define MPU_DMA_BUF_GYRO_X_H 8
#define MPU_DMA_BUF_GYRO_X_L 9
#define MPU_DMA_BUF_GYRO_Y_H 10
#define MPU_DMA_BUF_GYRO_Y_L 11
#define MPU_DMA_BUF_GYRO_Z_H 12
#define MPU_DMA_BUF_GYRO_Z_L 13

/* ADC */
#define ADC_BUFFER_SIZE 48
#define ADC_CHANNELS 8
#define ADC_DATA_BYTES (ADC_CHANNELS * 2)

/* PWM */
#define PWM_CHANNELS 4
extern uint16_t pwm_max_value;
#define PWM_DATA_BYTES (PWM_CHANNELS * 2)

/* Timers */
#define TIME_10MS_PERIOD_COUNT 40
#define TIME_100MS_PEDIOD_COUNT 10
#define ALIVE_UDP_PERIOD_COUNT 50

/* Communication */
#define IP_ADDRESS_STRING_LENGTH 16
#define UNERBUS_CMD_ID_SIZE 1
#define UNERBUS_ACK_SIZE 1
#define UNERBUS_BYPASS_STATUS_SIZE 1
#define UNERBUS_BUTTON_EVENT_SIZE 1
#define UNERBUS_PWM_RESPONSE_STATUS_SIZE 1
#define UNERBUS_PID_GAINS_SIZE (sizeof(uint16_t) * 3)         // Kp, Ki, Kd como uint16_t
#define UNERBUS_CONTROL_PARAMS_SIZE (sizeof(uint16_t) * 3)    // Setpoint, Speed, Correction como uint16_t
#define UNERBUS_MOTOR_BASE_SPEEDS_SIZE (sizeof(uint16_t) * 2) // Right, Left motor base speeds como uint16_t
#define UNERBUS_TURN_DEGREES_SIZE 2                           // int16_t
#define UNERBUS_TURN_PID_GAINS_SIZE (sizeof(uint16_t) * 3)    // Kp, Ki, Kd para el giro como uint16_t
#define UNERBUS_PWM_PERIOD_SIZE (sizeof(uint16_t))

/* USB CDC Buffer Sizes */
#define USB_CDC_RX_BUFFER_SIZE 128
#define USB_CDC_TX_BUFFER_SIZE 256

/* WiFi ESP01 Buffer Sizes */
#define WIFI_RX_BUFFER_SIZE 128
#define WIFI_TX_BUFFER_SIZE 128

/* Heartbeat Masks*/
#define HEARTBEAT_IDLE 0xF4000000
#define HEARTBEAT_WIFI_READY 0xF5000000
#define HEARTBEAT_UDP_READY 0xF5400000

/* Wifi Settings */
#define WIFI_SSID "InternetPlus_8e2fbb"
#define WIFI_PASSWORD "Akhantos2340"
#define WIFI_UDP_REMOTE_IP "192.168.1.3"
#define WIFI_UDP_REMOTE_PORT 30010
#define WIFI_UDP_LOCAL_PORT 30000

/* I2C */
#define I2C_DEFAULT_TIMEOUT_MS 1000
#define I2C_INIT_ERROR_BLINK_DELAY_MS 800

/* Initialization */
#define DEVICE_INIT_DELAY_MS 1000

/* --- Yaw Calculation --- */
// Scaling factor to convert raw gyro Z value to Q16.16 fixed-point degrees
// Formula: (dt_s * (1 << 16)) / LSB_per_dps = (0.01s * 65536) / 131 = 4.995...
#define GYRO_Z_SCALER 5

/* --- Turn PID Controller --- */
#define TURN_PID_KP_DEFAULT 80.0f   // Ganancia Proporcional inicial
#define TURN_PID_KI_DEFAULT 0.0f    // Ganancia Integral (iniciamos en 0)
#define TURN_PID_KD_DEFAULT 150.0f  // Ganancia Derivativa inicial (NOTA: estos valores probablemente necesiten reajuste)
#define TURN_PID_MAX_EFFORT 1000    // Esfuerzo máximo de giro (rango de -1000 a 1000)
#define TURN_COMPLETION_DEAD_ZONE 1 // Zona muerta en grados para considerar el giro completo

#endif /* INC_APP_CONFIG_H_ */