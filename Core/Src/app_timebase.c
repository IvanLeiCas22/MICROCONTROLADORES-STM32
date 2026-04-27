/* STM32/Test2024-master/Core/Src/app_timebase.c */
#include "app_timebase.h"

static void App_Timebase_UpdateSlot(AppTimebaseSlot *slot, uint8_t max_pending_events)
{
    if (slot->period_ticks == 0U)
    {
        return;
    }

    slot->elapsed_ticks++;
    if (slot->elapsed_ticks < slot->period_ticks)
    {
        return;
    }

    slot->elapsed_ticks = 0;
    if (slot->pending_events < max_pending_events)
    {
        slot->pending_events++;
    }
}

void App_Timebase_Init(AppTimebase *timebase, const AppTimebaseConfig *config)
{
    if ((timebase == 0) || (config == 0))
    {
        return;
    }

    timebase->max_pending_events = config->max_pending_events;
    if (timebase->max_pending_events == 0U)
    {
        timebase->max_pending_events = 1U;
    }

    for (uint8_t i = 0; i < APP_TIMEBASE_EVENT_COUNT; i++)
    {
        timebase->slots[i].period_ticks = config->period_ticks[i];
        timebase->slots[i].elapsed_ticks = 0;
        timebase->slots[i].pending_events = 0;
    }
}

void App_Timebase_Reset(AppTimebase *timebase)
{
    if (timebase == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < APP_TIMEBASE_EVENT_COUNT; i++)
    {
        timebase->slots[i].elapsed_ticks = 0;
        timebase->slots[i].pending_events = 0;
    }
}

void App_Timebase_OnTick(AppTimebase *timebase)
{
    if (timebase == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < APP_TIMEBASE_EVENT_COUNT; i++)
    {
        App_Timebase_UpdateSlot(&timebase->slots[i], timebase->max_pending_events);
    }
}

uint8_t App_Timebase_Consume(AppTimebase *timebase, AppTimebaseEvent event)
{
    if ((timebase == 0) || (event >= APP_TIMEBASE_EVENT_COUNT))
    {
        return 0;
    }

    uint8_t pending_events = timebase->slots[event].pending_events;
    timebase->slots[event].pending_events = 0;
    return pending_events;
}
