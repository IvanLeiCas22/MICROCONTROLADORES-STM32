#include "pid_controller.h"

void PID_Init(PID_Controller_t *pid, int32_t kp, int32_t ki, int32_t kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_min = INT32_MIN; // Valor por defecto sin límites
    pid->out_max = INT32_MAX; // Valor por defecto sin límites
}

void PID_Set_Setpoint(PID_Controller_t *pid, int32_t setpoint)
{
    // El setpoint se almacena en punto fijo para compararlo directamente con el error.
    pid->setpoint = INT_TO_FIXED(setpoint);
}

void PID_Set_Output_Limits(PID_Controller_t *pid, int32_t min, int32_t max)
{
    pid->out_min = min;
    pid->out_max = max;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}

int32_t PID_Update(PID_Controller_t *pid, int32_t current_value, uint32_t dt_ms)
{
    // Convertir el valor actual a punto fijo para los cálculos
    int32_t current_fixed = INT_TO_FIXED(current_value);

    // 1. Calcular el error
    int32_t error = pid->setpoint - current_fixed;

    // 2. Calcular el término Proporcional
    int32_t p_term = FIXED_MUL(pid->kp, error);

    // 3. Calcular el término Integral
    // Convertir dt de ms a segundos en punto fijo para escalar la integral correctamente
    int32_t dt_fixed = FIXED_DIV(INT_TO_FIXED(dt_ms), INT_TO_FIXED(1000));
    pid->integral += FIXED_MUL(error, dt_fixed);

    // Anti-windup: Limitar el término integral para evitar que crezca indefinidamente
    if (pid->integral > pid->out_max)
    {
        pid->integral = pid->out_max;
    }
    else if (pid->integral < pid->out_min)
    {
        pid->integral = pid->out_min;
    }
    int32_t i_term = FIXED_MUL(pid->ki, pid->integral);

    // 4. Calcular el término Derivativo
    int32_t d_term = 0;
    if (dt_ms > 0)
    {
        // La derivada es el cambio en el error sobre el cambio en el tiempo
        int32_t error_diff = error - pid->prev_error;
        int32_t derivative = FIXED_DIV(error_diff, dt_fixed);
        d_term = FIXED_MUL(pid->kd, derivative);
    }

    // 5. Calcular la salida total sumando los términos
    int32_t output = p_term + i_term + d_term;

    // 6. Limitar la salida final
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    // 7. Guardar el error actual para la siguiente iteración del derivativo
    pid->prev_error = error;

    return output;
}