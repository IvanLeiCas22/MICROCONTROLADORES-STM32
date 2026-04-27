/* STM32/Test2024-master/Core/Inc/app_timing.h */
#ifndef INC_APP_TIMING_H_
#define INC_APP_TIMING_H_

#include <stdint.h>

typedef struct
{
    uint32_t ticks_per_us;
} AppTimingClock;

void App_Timing_Init(AppTimingClock *clock, uint32_t ticks_per_second);
uint32_t App_Timing_ElapsedUs(const AppTimingClock *clock, uint32_t start_ticks, uint32_t end_ticks);

#endif /* INC_APP_TIMING_H_ */
