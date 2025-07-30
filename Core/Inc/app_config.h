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

typedef enum
{
    APP_STATE_MENU,
    APP_STATE_RUNNING
} AppStateTypeDef;

typedef enum
{
    MENU_MODE_IDLE,
    MENU_MODE_FIND_CELLS,
    MENU_MODE_GO_TO_B,
    MENU_MODE_COUNT // Mantener este último para ciclar fácilmente
} MenuModeTypeDef;

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
    CMD_SET_MAX_PWM_CORRECTION = 0x42, // Para configurar la corrección máxima del PWM
    CMD_GET_MAX_PWM_CORRECTION = 0x43, // Para leer la corrección máxima del PWM
    CMD_SET_MOTOR_BASE_SPEEDS = 0x44,  // Para configurar velocidades base independientes
    CMD_GET_MOTOR_BASE_SPEEDS = 0x45,  // Para leer velocidades base independientes
    CMD_CALIBRATE_MOTORS = 0x46,       // Para calibrar motores automáticamente
    CMD_TURN_DEGREES = 0x47,           // Para girar un número de grados
    CMD_SET_TURN_PID_GAINS = 0x48,     // Para configurar Kp, Ki, Kd del PID de giro
    CMD_GET_TURN_PID_GAINS = 0x49,     // Para leer Kp, Ki, Kd del PID de giro
    CMD_SET_TURN_MAX_SPEED = 0x4A,
    CMD_GET_TURN_MAX_SPEED = 0x4B,
    CMD_SET_PWM_PERIOD = 0x50,
    CMD_GET_PWM_PERIOD = 0x51,
    CMD_SET_MPU_CONFIG = 0xA7,
    CMD_GET_MPU_CONFIG = 0xA8,
    CMD_SET_TURN_MIN_SPEED = 0x4C,  // Para configurar la velocidad mínima de giro
    CMD_GET_TURN_MIN_SPEED = 0x4D,  // Para leer la velocidad mínima de giro
    CMD_SET_WALL_THRESHOLDS = 0x60, // Configurar el umbral de pared
    CMD_GET_WALL_THRESHOLDS = 0x61, // Leer el umbral de pared
    CMD_SET_WALL_TARGET_ADC = 0x62, // Configurar el valor ADC objetivo para seguimiento de pared
    CMD_GET_WALL_TARGET_ADC = 0x63, // Leer el valor ADC objetivo
    CMD_SET_APP_STATE = 0x70,       // Para cambiar entre MENU y RUNNING
    CMD_GET_APP_STATE = 0x71,       // Para leer el estado de la app
    CMD_SET_MENU_MODE = 0x72,       // Para seleccionar un modo de operación
    CMD_GET_MENU_MODE = 0x73,       // Para leer el modo de operación actual
    CMD_GET_ROBOT_STATUS = 0x74,    // Para leer el estado completo (AppState y MenuMode)
    CMD_SET_CRUISE_PARAMS = 0x4E,   // Configurar velocidad crucero y umbral de aceleración
    CMD_GET_CRUISE_PARAMS = 0x4F,   // Leer velocidad crucero y umbral de aceleración
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

typedef enum
{
    STATE_IDLE,
    STATE_CENTERING,
    STATE_DECIDING,
    STATE_TURNING_LEFT,
    STATE_TURNING_RIGHT,
    STATE_TURN_AROUND
} RobotStateTypeDef;

//==============================================================================
// DEFINICIONES Y MACROS
//==============================================================================

/* Flags del sistema (definidos en app_core.c) */
extern SystemFlagTypeDef flags0;
#define ON10MS flags0.bit.b0
#define UART_BYPASS flags0.bit.b1
#define MPU_READ_REQUEST flags0.bit.b2
#define SSD_UPDATE_REQUEST flags0.bit.b3

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
#define ADC_MOVING_AVERAGE_SAMPLES 40 // Número de muestras a promediar (10ms / 250us)

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
#define UNERBUS_CONTROL_PARAMS_SIZE (sizeof(uint16_t))        // Setpoint, Speed, Correction como uint16_t
#define UNERBUS_MOTOR_BASE_SPEEDS_SIZE (sizeof(uint16_t) * 2) // Right, Left motor base speeds como uint16_t
#define UNERBUS_TURN_DEGREES_SIZE 2                           // int16_t
#define UNERBUS_TURN_PID_GAINS_SIZE (sizeof(uint16_t) * 3)    // Kp, Ki, Kd para el giro como uint16_t
#define UNERBUS_TURN_MAX_SPEED_SIZE (sizeof(uint16_t))
#define UNERBUS_TURN_MIN_SPEED_SIZE (sizeof(uint16_t))
#define UNERBUS_PWM_PERIOD_SIZE (sizeof(uint16_t))
#define UNERBUS_MPU_CONFIG_SIZE (sizeof(uint8_t) * 3) // Accel, Gyro, DLPF
#define UNERBUS_WALL_THRESHOLDS_SIZE (sizeof(uint16_t) * 2)
#define UNERBUS_WALL_TARGET_ADC_SIZE (sizeof(uint16_t))
#define UNERBUS_APP_STATE_SIZE (sizeof(uint8_t))
#define UNERBUS_MENU_MODE_SIZE (sizeof(uint8_t))
#define UNERBUS_ROBOT_STATUS_SIZE (sizeof(uint8_t) * 2)
#define UNERBUS_CRUISE_PARAMS_SIZE (sizeof(uint16_t) * 3) // cruise_speed, accel_threshold, confirm_ticks

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

/* --- Máscaras de Heartbeat para Menú y Estados --- */
// Eventos de Botón (temporales)
#define HEARTBEAT_BTN_SHORT_PRESS 0xFFF00000 // Blink rápido
#define HEARTBEAT_BTN_LONG_PRESS 0xFFFFFFFF  // Sólido ON

// Modos de Menú (en APP_STATE_MENU)
#define HEARTBEAT_MENU_IDLE 0x88880000       // Doble blink lento
#define HEARTBEAT_MENU_FIND_CELLS 0xCCCCCCCC // Triple blink rápido
#define HEARTBEAT_MENU_GO_TO_B 0xF0F00000    // Blink alternado

// Modos en Ejecución (en APP_STATE_RUNNING)
#define HEARTBEAT_RUNNING_IDLE 0x80808080       // Blink regular lento
#define HEARTBEAT_RUNNING_FIND_CELLS 0xFEFE0000 // Blink "corriendo" rápido
#define HEARTBEAT_RUNNING_GO_TO_B 0xFAFA0000    // Blink "corriendo" diferente

/* Wifi Settings */
#define WIFI_SSID "InternetPlus_403ea8"
#define WIFI_PASSWORD "Fenofinalform01"
#define WIFI_UDP_REMOTE_IP "192.168.1.100"
#define WIFI_UDP_REMOTE_PORT 30010
#define WIFI_UDP_LOCAL_PORT 30000

/* I2C */
#define I2C_DEFAULT_TIMEOUT_MS 1000
#define I2C_INIT_ERROR_BLINK_DELAY_MS 800

/* Initialization */
#define DEVICE_INIT_DELAY_MS 1000

/* --- Turn PID Controller --- */
#define TURN_PID_KP_DEFAULT 80.0f   // Ganancia Proporcional inicial
#define TURN_PID_KI_DEFAULT 0.0f    // Ganancia Integral (iniciamos en 0)
#define TURN_PID_KD_DEFAULT 150.0f  // Ganancia Derivativa inicial (NOTA: estos valores probablemente necesiten reajuste)
#define TURN_PID_MAX_EFFORT 1000    // Esfuerzo máximo de giro (rango de -1000 a 1000)
#define TURN_COMPLETION_DEAD_ZONE 1 // Zona muerta en grados para considerar el giro completo
#define TURN_MAX_SPEED_DEFAULT 6500 // Velocidad máxima de giro en PWM
#define TURN_MIN_SPEED_DEFAULT 2600 // Velocidad mínima de giro para vencer la inercia

/* --- Sensores --- */
#define SENSOR_RIGHT_LAT_CH 0
#define SENSOR_FRONT_RIGHT_CH 2
#define SENSOR_FRONT_LEFT_CH 4
#define SENSOR_LEFT_LAT_CH 6
#define WALL_THRESHOLD_FRONT_DEFAULT 1500 // Umbral ADC para detectar pared frontal
#define WALL_THRESHOLD_SIDE_DEFAULT 1000  // Umbral ADC para detectar pared lateral
#define WALL_TARGET_ADC_DEFAULT 2000      // Valor ADC objetivo al seguir una sola pared

/* --- Cruise Control --- */
#define MOTOR_CRUISE_SPEED_DEFAULT 2600      // Velocidad PWM para navegación en rectas
#define ACCEL_MOTION_THRESHOLD_DEFAULT 1000  // Umbral de aceleración (raw) para detectar movimiento
#define ACCEL_MOTION_CONFIRM_TICKS_DEFAULT 2 // Nº de ciclos de 10ms para confirmar movimiento

#endif /* INC_APP_CONFIG_H_ */