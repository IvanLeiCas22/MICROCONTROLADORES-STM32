#include <stdint.h>

typedef enum
{
    BUTTON_STATE_RELEASED = 0,
    BUTTON_STATE_FALLING,
    BUTTON_STATE_PRESSED,
    BUTTON_STATE_RISING,
    BUTTON_STATE_LONG_PRESS
} Button_StateTypeDef;

typedef enum
{
    EVENT_NONE = 0,
    EVENT_PRESSED,
    EVENT_PRESS_RELEASED,
    EVENT_LONG_PRESS,
    EVENT_LONG_PRESS_RELEASED
} Button_EventsTypeDef;

typedef uint8_t (*ButtonReadPinCallback)(void *context);

typedef struct
{
    Button_StateTypeDef state;
    Button_EventsTypeDef event;

    void *context;                  // Context for read pin callback
    ButtonReadPinCallback read_pin; // Callback to read the button pin state
    uint16_t press_duration_ticks;
    uint16_t debounce_counter_ticks;
    uint16_t debounce_threshold_ticks;
    uint16_t long_press_threshold_ticks;

} Button_HandleTypeDef;

uint8_t Button_Init(Button_HandleTypeDef *hbutton, ButtonReadPinCallback read_pin_callback, void *context);
void Button_Tick(Button_HandleTypeDef *hbutton);
Button_EventsTypeDef Button_GetEvent(Button_HandleTypeDef *hbutton);