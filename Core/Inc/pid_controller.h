#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

// --- Aritmética de Punto Fijo (Formato Q16.16) ---
// Define la cantidad de bits para la parte fraccional.
#define FIXED_POINT_SHIFT 16

// Macros para conversiones y operaciones en punto fijo.
#define INT_TO_FIXED(x) ((int32_t)((x) << FIXED_POINT_SHIFT))
#define FIXED_TO_INT(x) ((int32_t)((x) >> FIXED_POINT_SHIFT))
#define HUNDREDTHS_TO_FIXED(x100) ((int32_t)(((int64_t)(x100) << FIXED_POINT_SHIFT) / 100))
#define FIXED_MUL(a, b) ((int32_t)(((int64_t)(a) * (b)) >> FIXED_POINT_SHIFT))
#define FIXED_DIV(a, b) ((int32_t)((((int64_t)(a)) << FIXED_POINT_SHIFT) / (b)))

/**
 * @brief Estructura para almacenar el estado y configuración de un controlador PID.
 */
typedef struct
{
    // Ganancias del PID en formato de punto fijo.
    int32_t kp;
    int32_t ki;
    int32_t kd;

    // Valor de consigna (setpoint) en formato de punto fijo.
    int32_t setpoint;

    // Términos internos del PID.
    int32_t integral;
    int32_t prev_error;

    // Límites de la salida para evitar saturación (anti-windup).
    int32_t out_min;
    int32_t out_max;

} PID_Controller_t;

/**
 * @brief Configuración reusable de un controlador PID.
 *        Las ganancias y los límites se almacenan en el mismo formato interno
 *        usado por el controlador (Q16.16 para ganancias y límites).
 */
typedef struct
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t out_min;
    int32_t out_max;
} PID_Config_t;

// --- Prototipos de Funciones Públicas ---

/**
 * @brief Inicializa el controlador PID con sus ganancias.
 * @param pid Puntero a la estructura del controlador.
 * @param kp Ganancia proporcional (en punto fijo).
 * @param ki Ganancia integral (en punto fijo).
 * @param kd Ganancia derivativa (en punto fijo).
 */
void PID_Init(PID_Controller_t *pid, int32_t kp, int32_t ki, int32_t kd);

/**
 * @brief Aplica una configuración completa al controlador PID.
 * @param pid Puntero a la estructura del controlador.
 * @param cfg Configuración a aplicar.
 * @param reset_state Si es true, reinicia integral y error previo.
 */
void PID_ApplyConfig(PID_Controller_t *pid, const PID_Config_t *cfg, bool reset_state);

/**
 * @brief Establece el valor de consigna (setpoint) del PID.
 * @param pid Puntero a la estructura del controlador.
 * @param setpoint El valor deseado (se convertirá internamente a punto fijo).
 */
void PID_Set_Setpoint(PID_Controller_t *pid, int32_t setpoint);

/**
 * @brief Establece el setpoint usando un valor Q16.16 directo.
 * @param pid Puntero a la estructura del controlador.
 * @param setpoint_q16 Valor deseado ya expresado en Q16.16.
 */
void PID_Set_Setpoint_Fixed(PID_Controller_t *pid, int32_t setpoint_q16);

/**
 * @brief Calcula la salida del controlador PID basándose en la medición actual.
 * @param pid Puntero a la estructura del controlador.
 * @param current_value El valor actual medido por el sensor.
 * @param dt_ms El intervalo de tiempo en milisegundos desde la última llamada.
 * @return La salida del controlador (corrección) en formato de punto fijo.
 */
int32_t PID_Update(PID_Controller_t *pid, int32_t current_value, uint32_t dt_ms);

/**
 * @brief Calcula la salida usando una medicion Q16.16 directa.
 * @param pid Puntero a la estructura del controlador.
 * @param current_value_q16 Valor actual ya expresado en Q16.16.
 * @param dt_ms El intervalo de tiempo en milisegundos desde la ultima llamada.
 * @return La salida del controlador en formato Q16.16.
 */
int32_t PID_Update_Fixed(PID_Controller_t *pid, int32_t current_value_q16, uint32_t dt_ms);

/**
 * @brief Establece los límites mínimo y máximo de la salida del PID.
 * @param pid Puntero a la estructura del controlador.
 * @param min Límite inferior de la salida (en punto fijo).
 * @param max Límite superior de la salida (en punto fijo).
 */
void PID_Set_Output_Limits(PID_Controller_t *pid, int32_t min, int32_t max);

/**
 * @brief Resetea los términos internos del controlador (integral y error previo).
 * @param pid Puntero a la estructura del controlador.
 */
void PID_Reset(PID_Controller_t *pid);

#endif // PID_CONTROLLER_H
