#include "app_sensors.h"

#include <string.h>

#include "app_config.h"
#include "pid_controller.h"

typedef struct
{
    uint16_t adc;
    uint16_t dist_mm;
} SensorLutEntry;

typedef struct
{
    uint16_t x0;
    uint16_t x1;
    uint16_t y0;
    int32_t slope_q16;
} ADC_LutSeg_t;

static const SensorLutEntry sensor_lut[] = {
    {30, 150}, {65, 140}, {106, 130}, {135, 120}, {169, 110}, {208, 100}, {260, 90}, {337, 80}, {441, 70}, {511, 65}, {590, 60}, {711, 55}, {827, 50}, {1020, 45}, {1305, 40}, {1613, 35}, {2130, 30}, {2870, 25}, {3760, 20}};

enum
{
    ADC_LUT_SIZE = (int)(sizeof(sensor_lut) / sizeof(sensor_lut[0]))
};

enum
{
    ADC_SEG_COUNT = ADC_LUT_SIZE - 1
};

static uint16_t adc_dma_buffer[ADC_BUFFER_SIZE][ADC_CHANNELS];
static volatile uint8_t adc_buf_write_idx = 0;
static uint32_t adc_running_sum[ADC_CHANNELS] = {0};
static uint16_t adc_filtered_avg[ADC_CHANNELS] = {0};
static uint8_t adc_samples_accumulated = 0;
static uint8_t adc_processed_idx = 0;
static ADC_LutSeg_t adc_lut_segs[ADC_SEG_COUNT];

static void App_Sensors_PrecomputeLut(void);

void App_Sensors_Init(void)
{
    memset(adc_dma_buffer, 0, sizeof(adc_dma_buffer));
    memset(adc_running_sum, 0, sizeof(adc_running_sum));
    memset(adc_filtered_avg, 0, sizeof(adc_filtered_avg));
    adc_buf_write_idx = 0;
    adc_samples_accumulated = 0;
    adc_processed_idx = 0;

    App_Sensors_PrecomputeLut();
}

uint16_t *App_Sensors_GetAdcDmaWriteBuffer(void)
{
    return adc_dma_buffer[adc_buf_write_idx];
}

void App_Sensors_OnAdcDmaComplete(void)
{
    adc_buf_write_idx = (uint8_t)((adc_buf_write_idx + 1U) & ADC_BUF_MASK);
}

void App_Sensors_ProcessAdcSamples(void)
{
    while (adc_processed_idx != adc_buf_write_idx)
    {
        uint8_t idx_new = adc_processed_idx;
        bool window_full = (adc_samples_accumulated >= ADC_MOVING_AVERAGE_SAMPLES);

        if (!window_full)
        {
            adc_samples_accumulated++;
            for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
            {
                adc_running_sum[ch] += adc_dma_buffer[idx_new][ch];
                adc_filtered_avg[ch] = (uint16_t)(adc_running_sum[ch] / adc_samples_accumulated);
            }
        }
        else
        {
            uint8_t idx_old = (uint8_t)((idx_new - ADC_MOVING_AVERAGE_SAMPLES) & ADC_BUF_MASK);
            for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
            {
                uint16_t new_val = adc_dma_buffer[idx_new][ch];
                uint16_t old_val = adc_dma_buffer[idx_old][ch];

                if (old_val > adc_running_sum[ch])
                {
                    adc_running_sum[ch] = 0;
                }
                else
                {
                    adc_running_sum[ch] -= old_val;
                }

                adc_running_sum[ch] += new_val;
                adc_filtered_avg[ch] = (uint16_t)(adc_running_sum[ch] >> ADC_FILTER_SHIFT);
            }
        }

        adc_processed_idx = (uint8_t)((adc_processed_idx + 1U) & ADC_BUF_MASK);
    }
}

uint16_t App_Sensors_GetFilteredAdcValue(uint8_t channel)
{
    if (channel >= ADC_CHANNELS)
    {
        return 0;
    }

    return adc_filtered_avg[channel];
}

uint16_t App_Sensors_ConvertAdcToDistanceMm(uint16_t adc_value)
{
    if (adc_value <= sensor_lut[0].adc)
    {
        return sensor_lut[0].dist_mm;
    }

    if (adc_value >= sensor_lut[ADC_LUT_SIZE - 1].adc)
    {
        return sensor_lut[ADC_LUT_SIZE - 1].dist_mm;
    }

    uint8_t lo = 0;
    uint8_t hi = (uint8_t)(ADC_LUT_SIZE - 1);
    while ((uint8_t)(hi - lo) > 1U)
    {
        uint8_t mid = (uint8_t)((lo + hi) >> 1);
        if (adc_value < sensor_lut[mid].adc)
        {
            hi = mid;
        }
        else
        {
            lo = mid;
        }
    }

    const ADC_LutSeg_t *segment = &adc_lut_segs[lo];
    int32_t dx = (int32_t)adc_value - (int32_t)segment->x0;
    int64_t mul = (int64_t)segment->slope_q16 * (int64_t)dx;
    int32_t increment = (int32_t)((mul + (1LL << (FIXED_POINT_SHIFT - 1))) >> FIXED_POINT_SHIFT);
    int32_t distance = (int32_t)segment->y0 + increment;

    if (distance < 0)
    {
        return 0;
    }

    return (uint16_t)distance;
}

static void App_Sensors_PrecomputeLut(void)
{
    for (uint8_t i = 0; i < ADC_SEG_COUNT; ++i)
    {
        const uint16_t x0 = sensor_lut[i].adc;
        const uint16_t x1 = sensor_lut[i + 1].adc;
        const int32_t y0 = (int32_t)sensor_lut[i].dist_mm;
        const int32_t y1 = (int32_t)sensor_lut[i + 1].dist_mm;
        const int32_t dx = (int32_t)x1 - (int32_t)x0;
        const int32_t dy = y1 - y0;

        adc_lut_segs[i].x0 = x0;
        adc_lut_segs[i].x1 = x1;
        adc_lut_segs[i].y0 = (uint16_t)y0;

        if (dx != 0)
        {
            adc_lut_segs[i].slope_q16 = (dy << FIXED_POINT_SHIFT) / dx;
        }
        else
        {
            adc_lut_segs[i].slope_q16 = 0;
        }
    }
}
