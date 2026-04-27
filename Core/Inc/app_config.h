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
    MENU_MODE_MANUAL_CONTROL,
    MENU_MODE_DRIVE_STRAIGHT,
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
    CMD_GET_IR_SENSOR_SNAPSHOT = 0xA0,
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
    CMD_SET_PIVOT_TURN_DPS = 0x4C,  // Para configurar la velocidad de giro en pivote
    CMD_GET_PIVOT_TURN_DPS = 0x4D,  // Para leer la velocidad de giro en pivote
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
    CMD_SET_BRAKING_PID_GAINS = 0x64,
    CMD_GET_BRAKING_PID_GAINS = 0x65,
    CMD_SET_BRAKING_PARAMS = 0x66,
    CMD_GET_BRAKING_PARAMS = 0x67,
    CMD_SET_BRAKING_MAX_SPEED = 0x68,
    CMD_GET_BRAKING_MAX_SPEED = 0x69,
    CMD_SET_BRAKING_MIN_SPEED = 0x6A,       // Para configurar la velocidad mínima de frenado
    CMD_GET_BRAKING_MIN_SPEED = 0x6B,       // Para leer la velocidad mínima de frenado
    CMD_SET_BRAKING_DEAD_ZONE = 0x6C,       // Para configurar la zona muerta de frenado
    CMD_GET_BRAKING_DEAD_ZONE = 0x6D,       // Para leer la zona muerta de frenado
    CMD_GET_YAW_ANGLE = 0x75,               // Para leer el ángulo de guiñada actual
    CMD_GET_SMOOTH_TURN_CONFIG = 0x80,      // Para leer la configuración de giro suave
    CMD_SET_SMOOTH_TURN_CONFIG = 0x81,      // Para configurar el giro suave
    CMD_SET_TURN_VELOCITY_PID_GAINS = 0x82, // Configurar Kp, Ki, Kd del PID de velocidad de giro
    CMD_GET_TURN_VELOCITY_PID_GAINS = 0x83, // Leer Kp, Ki, Kd del PID de velocidad de giro
    CMD_SET_TURN_TARGET_DPS = 0x84,         // Configurar la velocidad angular objetivo para giros
    CMD_GET_TURN_TARGET_DPS = 0x85,         // Leer la velocidad angular objetivo
    CMD_GET_DELAY_TICKS = 0X90,             // Leer el número de ticks de retardo
    CMD_SET_DELAY_TICKS = 0X91,             // Configurar el número de ticks de retardo
    CMD_UPDATE_MAZE_CELL = 0x92,            // (STM32 -> Qt) Enviar actualización de info de celda
    CMD_SYNC_MAZE_COLUMN = 0x93,            // Sincronizar 1 columna entera del laberinto
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
    STATE_NAVIGATING,
    STATE_BRAKING,
    STATE_DECIDING,
    STATE_TURNING_LEFT,
    STATE_TURNING_RIGHT,
    STATE_SMOOTH_TURN_LEFT,
    STATE_SMOOTH_TURN_RIGHT,
    STATE_STRAIGHT_DRIVE,
    STATE_STRAIGHT_DRIVE_DESIDING,
    STATE_LEFT_WALL_FADE,
    STATE_RIGHT_WALL_FADE,
    STATE_TURN_AROUND_RIGHT,
    STATE_TURN_AROUND_LEFT
} RobotStateTypeDef;

typedef enum
{
    LEFT_WALL_FADED = 1,
    RIGHT_WALL_FADED = 2,
    NO_WALL_FADED = 0
} WallFadeTypesTypeDef;

//--- Laberinto ---
typedef enum
{
    HEADING_NORTH = 0,
    HEADING_EAST = 1,
    HEADING_SOUTH = 2,
    HEADING_WEST = 3
} HeadingTypeDef;

typedef enum
{
    TURN_RIGHT = 1,
    TURN_LEFT = -1,
    TURN_AROUND = 2
} TurnTypeDef;

//==============================================================================
// DEFINICIONES Y MACROS
//==============================================================================

/* Flags del sistema (definidos en modulos de la aplicacion) */
extern SystemFlagTypeDef flags0;
extern volatile bool app_uart_bypass;
extern volatile bool app_mpu_read_request;
extern volatile bool app_ssd_update_request;
#define UART_BYPASS app_uart_bypass
#define MPU_READ_REQUEST app_mpu_read_request
#define SSD_UPDATE_REQUEST app_ssd_update_request

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
#define ADC_BUFFER_SIZE 32
#define ADC_CHANNELS 8
#define ADC_DATA_BYTES (ADC_CHANNELS * 2)
#define IR_SENSOR_SNAPSHOT_BYTES (ADC_DATA_BYTES + 1U)
#define ADC_MOVING_AVERAGE_SAMPLES 16 // Ventana de 16 muestras (aprox. 4ms a 250us)
#define ADC_FILTER_SHIFT 4            // Corresponde a log2(16)
#define ADC_BUF_MASK (ADC_BUFFER_SIZE - 1)

/* PWM */
#define PWM_CHANNELS 4
extern uint16_t pwm_max_value;
#define PWM_DATA_BYTES (PWM_CHANNELS * 2)

/* Timers */
#define ALIVE_UDP_PERIOD_COUNT 50
#define TIM1_TICK_US 250U
#define APP_TIMEBASE_1MS_TICKS (1000U / TIM1_TICK_US)
#define APP_TIMEBASE_10MS_TICKS 40U
#define APP_TIMEBASE_100MS_TICKS 400U
#define APP_TIMEBASE_IR_SAMPLE_TICKS 1U
#define APP_TIMEBASE_MPU_SAMPLE_TICKS 12U
#define APP_TIMEBASE_CONTROL_TICKS APP_TIMEBASE_10MS_TICKS
#define APP_TIMEBASE_MAX_PENDING_EVENTS 10U
#define CONTROL_PERIOD_MS 10U
#define MPU_SAMPLE_PERIOD_US (APP_TIMEBASE_MPU_SAMPLE_TICKS * TIM1_TICK_US)

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
#define UNERBUS_WALL_THRESHOLDS_SIZE (sizeof(uint16_t) * 4)
#define UNERBUS_WALL_TARGET_ADC_SIZE (sizeof(uint16_t) * 2)
#define UNERBUS_APP_STATE_SIZE (sizeof(uint8_t))
#define UNERBUS_MENU_MODE_SIZE (sizeof(uint8_t))
#define UNERBUS_ROBOT_STATUS_SIZE (sizeof(uint8_t) * 2)
#define UNERBUS_CRUISE_PARAMS_SIZE (sizeof(uint16_t) * 3) // cruise_speed, accel_threshold, confirm_ticks
#define UNERBUS_BRAKING_PID_GAINS_SIZE (sizeof(uint16_t) * 3)
#define UNERBUS_BRAKING_PARAMS_SIZE (sizeof(uint16_t) * 2)
#define UNERBUS_BRAKING_MAX_SPEED_SIZE (sizeof(uint16_t))
#define UNERBUS_BRAKING_MIN_SPEED_SIZE (sizeof(uint16_t))
#define UNERBUS_BRAKING_DEAD_ZONE_SIZE (sizeof(uint16_t))
#define UNERBUS_YAW_ANGLE_SIZE (sizeof(int32_t)) // Yaw angle como int32_t
#define UNERBUS_SMOOTH_TURN_CONFIG_SIZE (sizeof(uint16_t) * 2)
#define UNERBUS_TURN_VELOCITY_PID_GAINS_SIZE (sizeof(uint16_t) * 3)
#define UNERBUS_TURN_TARGET_DPS_SIZE (sizeof(uint16_t))
#define UNERBUS_DELAY_TICKS_SIZE (sizeof(uint8_t))          // Número de ticks de retardo como uint8_t
#define UNERBUS_MAZE_CELL_UPDATE_SIZE (sizeof(uint8_t) * 4) // x, y, walls, heading

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
#define HEARTBEAT_MENU_IDLE 0x88880000           // Doble blink lento
#define HEARTBEAT_MENU_FIND_CELLS 0xCCCCCCCC     // Triple blink rápido
#define HEARTBEAT_MENU_GO_TO_B 0xF0F00000        // Blink alternado
#define HEARTBEAT_MENU_MANUAL_CONTROL 0xAAAA0000 // Blink corto y rápido
#define HEARTBEAT_MENU_DRIVE_STRAIGHT 0x0F0F0F0F // Blink alternado rápido

// Modos en Ejecución (en APP_STATE_RUNNING)
#define HEARTBEAT_RUNNING_IDLE 0x80808080           // Blink regular lento
#define HEARTBEAT_RUNNING_FIND_CELLS 0xFEFE0000     // Blink "corriendo" rápido
#define HEARTBEAT_RUNNING_GO_TO_B 0xFAFA0000        // Blink "corriendo" diferente
#define HEARTBEAT_RUNNING_MANUAL_CONTROL 0xF0F0F0F0 // <--- AÑADIR ESTA LÍNEA (Blink constante y rápido)

/* Wifi Settings */
#define WIFI_SSID "InternetPlus_8e2fbb"//"FCAL"
#define WIFI_PASSWORD "Akhantos2340"//"fcalconcordia.06-2019"
#define WIFI_UDP_REMOTE_IP "192.168.1.120"//"172.23.225.120"
#define WIFI_UDP_REMOTE_PORT 30010
#define WIFI_UDP_LOCAL_PORT 30000

/* I2C */
#define I2C_DEFAULT_TIMEOUT_MS 1000
#define I2C_INIT_ERROR_BLINK_DELAY_MS 800

/* Initialization */
#define DEVICE_INIT_DELAY_MS 1000

/* --- Turn PID Controller --- */
#define TURN_PID_KP_DEFAULT_X100 8000     // Ganancia Proporcional inicial
#define TURN_PID_KI_DEFAULT_X100 0        // Ganancia Integral (iniciamos en 0)
#define TURN_PID_KD_DEFAULT_X100 15000    // Ganancia Derivativa inicial (NOTA: estos valores probablemente necesiten reajuste)
#define TURN_COMPLETION_DEAD_ZONE 1       // Zona muerta en grados para considerar el giro completo
#define TURN_MAX_SPEED_DEFAULT 6500       // Velocidad máxima de giro en PWM
#define PIVOT_TURN_TARGET_DPS_DEFAULT 360 // Velocidad mínima de giro para vencer la inercia

#define TURN_VELOCITY_PID_KP_DEFAULT_X100 2000 // Kp para el control de velocidad angular
#define TURN_VELOCITY_PID_KI_DEFAULT_X100 500  // Ki para el control de velocidad angular
#define TURN_VELOCITY_PID_KD_DEFAULT_X100 200  // Kd para el control de velocidad angular
#define TURN_TARGET_DPS_DEFAULT 360        // Velocidad angular objetivo en grados/segundo

/* --- Sensores --- */
#define SENSOR_RIGHT_LAT_CH 0
#define SENSOR_DIAGONAL_RIGHT_CH 1
#define SENSOR_FRONT_RIGHT_CH 2
#define SENSOR_FLOOR_FRONT_CH 3
#define SENSOR_FRONT_LEFT_CH 4
#define SENSOR_DIAGONAL_LEFT_CH 5
#define SENSOR_LEFT_LAT_CH 6
#define SENSOR_FLOOR_REAR_CH 7

#define SENSOR_DET_WALL_FRONT 0x01
#define SENSOR_DET_WALL_LEFT 0x02
#define SENSOR_DET_WALL_RIGHT 0x04
#define SENSOR_DET_WALL_DIAG_LEFT 0x08
#define SENSOR_DET_WALL_DIAG_RIGHT 0x10
#define SENSOR_DET_FLOOR_FRONT 0x20
#define SENSOR_DET_FLOOR_REAR 0x40

#define WALL_HYSTERESIS_MM 15
#define TAPE_HYSTERESIS_ADC 200

#define WALL_PRESENCE_THRESHOLD_MM_SIDE 100         // Distancia (mm) para detectar una pared lateral.
#define WALL_PRESENCE_THRESHOLD_MM_DIAGONAL 140     // Distancia (mm) para detectar una pared diagonal derecha.
#define WALL_PRESENCE_THRESHOLD_MM_FRONT 70         // Distancia (mm) para detectar una pared frontal.
#define WALL_PRESENCE_THRESHOLD_MM_BRAKING_START 40 // Distancia (mm) para detectar una pared y comenzar el frenado.
#define WALL_FOLLOW_TARGET_MM 50                    // Distancia (mm) objetivo para el seguimiento de pared.
#define WALL_BRAKING_TARGET_MM 20                   // Distancia (mm) objetivo para terminar el frenado.
#define WALL_FADE_TICKS_DEFAULT 2                   // Ticks para desvanecer la detección de pared

/* --- Cruise Control --- */
#define MOTOR_KICK_START_SPEED_DEFAULT 2600  // Velocidad PWM para navegación en rectas
#define ACCEL_MOTION_THRESHOLD_DEFAULT 1000  // Umbral de aceleración (raw) para detectar movimiento
#define ACCEL_MOTION_CONFIRM_TICKS_DEFAULT 2 // Nº de ciclos de 10ms para confirmar movimiento

/* --- Braking PID Controller --- */
#define BRAKING_PID_KP_DEFAULT_X100 1000
#define BRAKING_PID_KI_DEFAULT_X100 10
#define BRAKING_PID_KD_DEFAULT_X100 500
#define BRAKING_DEAD_ZONE_DEFAULT 5              // Tolerancia en ADC para considerar la parada
#define BRAKING_ACCEL_STOP_THRESHOLD_DEFAULT 400 // Umbral de acelerómetro para confirmar detención
#define BRAKING_MAX_SPEED_DEFAULT 4000           // Velocidad máxima de frenado en PWM
#define BRAKING_MIN_SPEED_DEFAULT 2200           // Velocidad mínima de frenado para vencer inercia

/* --- Go straight --- */
#define FRONT_OBSTACLE_STOP_DISTANCE_MM 50

/* --- Laberinto --- */
// Definición de bits para cada celda
#define WALL_NORTH 0x01   // 0000 0001
#define WALL_SOUTH 0x02   // 0000 0010
#define WALL_EAST 0x04    // 0000 0100
#define WALL_WEST 0x08    // 0000 1000
#define CELL_VISITED 0x10 // 0001 0000
#define CELL_SPECIAL 0x20 // 0010 0000
// Tamaño del arreglo lógico (15x15) para laberinto físico de 8x8
#define MAZE_WIDTH 15
#define MAZE_HEIGHT 15

#endif /* INC_APP_CONFIG_H_ */
