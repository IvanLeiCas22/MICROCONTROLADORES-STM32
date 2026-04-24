#ifndef INC_APP_SENSORS_H_
#define INC_APP_SENSORS_H_

#include <stdint.h>

void App_Sensors_Init(void);
uint16_t *App_Sensors_GetAdcDmaWriteBuffer(void);
void App_Sensors_OnAdcDmaComplete(void);
void App_Sensors_ProcessAdcSamples(void);
uint16_t App_Sensors_GetFilteredAdcValue(uint8_t channel);
uint16_t App_Sensors_ConvertAdcToDistanceMm(uint16_t adc_value);

#endif /* INC_APP_SENSORS_H_ */
