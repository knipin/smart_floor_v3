#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_heater_init(void);
esp_err_t app_heater_set_power(float percent); // 0.0–100.0 %

#ifdef __cplusplus
}
#endif
