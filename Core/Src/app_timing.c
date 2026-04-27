/* STM32/Test2024-master/Core/Src/app_timing.c */
#include "app_timing.h"

#define APP_TIMING_US_PER_SECOND 1000000U

void App_Timing_Init(AppTimingClock *clock, uint32_t ticks_per_second)
{
    if (clock == 0)
    {
        return;
    }

    clock->ticks_per_us = ticks_per_second / APP_TIMING_US_PER_SECOND;
    if (clock->ticks_per_us == 0U)
    {
        clock->ticks_per_us = 1U;
    }
}

uint32_t App_Timing_ElapsedUs(const AppTimingClock *clock, uint32_t start_ticks, uint32_t end_ticks)
{
    uint32_t ticks_per_us = 1U;

    if ((clock != 0) && (clock->ticks_per_us != 0U))
    {
        ticks_per_us = clock->ticks_per_us;
    }

    return (end_ticks - start_ticks) / ticks_per_us;
}
