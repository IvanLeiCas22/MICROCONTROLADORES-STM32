#include "pid_controller.h"

void PID_Init(PID_Controller_t *pid, int32_t kp, int32_t ki, int32_t kd)
{
    PID_Config_t cfg = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .out_min = INT32_MIN,
        .out_max = INT32_MAX,
    };

    PID_ApplyConfig(pid, &cfg, true);
    pid->setpoint = 0;
}

void PID_ApplyConfig(PID_Controller_t *pid, const PID_Config_t *cfg, bool reset_state)
{
    pid->kp = cfg->kp;
    pid->ki = cfg->ki;
    pid->kd = cfg->kd;
    pid->out_min = cfg->out_min;
    pid->out_max = cfg->out_max;

    if (reset_state)
    {
        PID_Reset(pid);
    }
}

void PID_Set_Setpoint(PID_Controller_t *pid, int32_t setpoint)
{
    PID_Set_Setpoint_Fixed(pid, INT_TO_FIXED(setpoint));
}

void PID_Set_Setpoint_Fixed(PID_Controller_t *pid, int32_t setpoint_q16)
{
    pid->setpoint = setpoint_q16;
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
    return PID_Update_Fixed(pid, INT_TO_FIXED(current_value), dt_ms);
}

int32_t PID_Update_Fixed(PID_Controller_t *pid, int32_t current_value_q16, uint32_t dt_ms)
{
    // 1. Calcular el error.
    int32_t error = pid->setpoint - current_value_q16;

    // 2. Calcular el termino proporcional.
    int32_t p_term = FIXED_MUL(pid->kp, error);

    // 3. Calcular el termino integral.
    // Convertir dt de ms a segundos en punto fijo para escalar la integral correctamente.
    int32_t dt_fixed = FIXED_DIV(INT_TO_FIXED(dt_ms), INT_TO_FIXED(1000));
    pid->integral += FIXED_MUL(error, dt_fixed);

    // Anti-windup: limitar el termino integral para evitar que crezca indefinidamente.
    if (pid->integral > pid->out_max)
    {
        pid->integral = pid->out_max;
    }
    else if (pid->integral < pid->out_min)
    {
        pid->integral = pid->out_min;
    }
    int32_t i_term = FIXED_MUL(pid->ki, pid->integral);

    // 4. Calcular el termino derivativo.
    int32_t d_term = 0;
    if (dt_ms > 0)
    {
        // La derivada es el cambio en el error sobre el cambio en el tiempo.
        int32_t error_diff = error - pid->prev_error;
        int32_t derivative = FIXED_DIV(error_diff, dt_fixed);
        d_term = FIXED_MUL(pid->kd, derivative);
    }

    // 5. Calcular la salida total sumando los terminos.
    int32_t output = p_term + i_term + d_term;

    // 6. Limitar la salida final.
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    // 7. Guardar el error actual para la siguiente iteracion del derivativo.
    pid->prev_error = error;

    return output;
}
