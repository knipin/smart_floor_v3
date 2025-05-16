#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

extern adc_oneshot_unit_handle_t adc_handle;

esp_err_t app_temp_sensor_init(void);
esp_err_t app_temp_sensor_read(float *temperature_celsius);

#ifdef __cplusplus
}
#endif
