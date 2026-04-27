/* STM32/Test2024-master/Core/Inc/app_timebase.h */
#ifndef INC_APP_TIMEBASE_H_
#define INC_APP_TIMEBASE_H_

#include <stdint.h>

typedef enum
{
    APP_TIMEBASE_EVENT_1MS = 0,
    APP_TIMEBASE_EVENT_10MS,
    APP_TIMEBASE_EVENT_100MS,
    APP_TIMEBASE_EVENT_IR_SAMPLE,
    APP_TIMEBASE_EVENT_MPU_SAMPLE,
    APP_TIMEBASE_EVENT_CONTROL,
    APP_TIMEBASE_EVENT_COUNT
} AppTimebaseEvent;

typedef struct
{
    uint32_t period_ticks[APP_TIMEBASE_EVENT_COUNT];
    uint8_t max_pending_events;
} AppTimebaseConfig;

typedef struct
{
    uint32_t period_ticks;
    uint32_t elapsed_ticks;
    volatile uint8_t pending_events;
} AppTimebaseSlot;

typedef struct
{
    AppTimebaseSlot slots[APP_TIMEBASE_EVENT_COUNT];
    uint8_t max_pending_events;
} AppTimebase;

void App_Timebase_Init(AppTimebase *timebase, const AppTimebaseConfig *config);
void App_Timebase_Reset(AppTimebase *timebase);
void App_Timebase_OnTick(AppTimebase *timebase);

/* Protect this call externally if App_Timebase_OnTick runs from an ISR. */
uint8_t App_Timebase_Consume(AppTimebase *timebase, AppTimebaseEvent event);

#endif /* INC_APP_TIMEBASE_H_ */
