#include <stdint.h>
#include <stddef.h> // For NULL

#include "BUTTONS.h"

#define PRESSED 0
#define RELEASED 1

uint8_t Button_Init(Button_HandleTypeDef *hbutton, ButtonReadPinCallback read_pin_callback, void *context)
{
    if (hbutton == NULL || read_pin_callback == NULL)
    {
        return 0; // Error: Invalid parameters
    }

    hbutton->state = BUTTON_STATE_RELEASED;
    hbutton->event = EVENT_NONE;
    hbutton->read_pin = read_pin_callback;
    hbutton->context = context;
    hbutton->press_duration_ticks = 0;
    hbutton->debounce_counter_ticks = 0;
    hbutton->debounce_threshold_ticks = 5;     // 5 ticks = 50 ms
    hbutton->long_press_threshold_ticks = 100; // 100 ticks = 1000 ms

    return 1; // Success
}

void Button_Tick(Button_HandleTypeDef *hbutton)
{
    if (hbutton == NULL || hbutton->read_pin == NULL)
    {
        return;
    }

    uint8_t pin_state = hbutton->read_pin(hbutton->context);

    // --- Button State Machine ---
    switch (hbutton->state)
    {
    case BUTTON_STATE_RELEASED:
        if (pin_state == PRESSED)
        {
            hbutton->state = BUTTON_STATE_FALLING;
            hbutton->debounce_counter_ticks = 0;
        }
        break;

    case BUTTON_STATE_FALLING:
        if (pin_state == PRESSED)
        {
            hbutton->debounce_counter_ticks++;
            if (hbutton->debounce_counter_ticks >= hbutton->debounce_threshold_ticks)
            {
                hbutton->state = BUTTON_STATE_PRESSED;
                hbutton->event = EVENT_PRESSED;
                hbutton->press_duration_ticks = 0;
            }
        }
        else
        {
            hbutton->state = BUTTON_STATE_RELEASED; // Glitch, go back
        }
        break;

    case BUTTON_STATE_PRESSED:
        if (pin_state == PRESSED)
        {
            hbutton->press_duration_ticks++;
            if (hbutton->press_duration_ticks >= hbutton->long_press_threshold_ticks)
            {
                hbutton->state = BUTTON_STATE_LONG_PRESS;
                hbutton->event = EVENT_LONG_PRESS;
            }
        }
        else // Pin is high (released)
        {
            hbutton->state = BUTTON_STATE_RISING;
            hbutton->debounce_counter_ticks = 0;
        }
        break;

    case BUTTON_STATE_RISING:
        if (pin_state == RELEASED)
        {
            hbutton->debounce_counter_ticks++;
            if (hbutton->debounce_counter_ticks >= hbutton->debounce_threshold_ticks)
            {
                // Debounce complete, now decide which release event to fire.
                if (hbutton->press_duration_ticks >= hbutton->long_press_threshold_ticks)
                {
                    hbutton->event = EVENT_LONG_PRESS_RELEASED;
                }
                else
                {
                    hbutton->event = EVENT_PRESS_RELEASED;
                }
                hbutton->state = BUTTON_STATE_RELEASED;
                hbutton->press_duration_ticks = 0; // Reset for next press
            }
        }
        else
        {
            // Glitch, went back to pressed. Return to the correct pressed state.
            if (hbutton->press_duration_ticks >= hbutton->long_press_threshold_ticks)
            {
                hbutton->state = BUTTON_STATE_LONG_PRESS;
            }
            else
            {
                hbutton->state = BUTTON_STATE_PRESSED;
            }
        }
        break;

    case BUTTON_STATE_LONG_PRESS:
        if (pin_state == PRESSED) // Still long-pressed
        {
            if (hbutton->press_duration_ticks < 0xFFFF) // Prevent overflow
            {
                hbutton->press_duration_ticks++;
            }
        }
        else // Pin is high (released)
        {
            hbutton->state = BUTTON_STATE_RISING;
            hbutton->debounce_counter_ticks = 0;
        }
        break;
    }
}

Button_EventsTypeDef Button_GetEvent(Button_HandleTypeDef *hbutton)
{
    if (hbutton == NULL)
    {
        return EVENT_NONE;
    }

    Button_EventsTypeDef event = hbutton->event;
    hbutton->event = EVENT_NONE; // Consume the event
    return event;
}