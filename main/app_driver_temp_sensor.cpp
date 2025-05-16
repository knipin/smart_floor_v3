#include "app_driver_temp_sensor.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "temp_sensor"

#define TEMP_ADC_UNIT        ADC_UNIT_1
#define TEMP_ADC_CHANNEL     ADC_CHANNEL_0   // GPIO0
#define TEMP_ADC_ATTEN       ADC_ATTEN_DB_12 // до 3.3В
#define TEMP_ADC_BITWIDTH    ADC_BITWIDTH_DEFAULT

adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;
static bool adc_calibrated = false;

esp_err_t app_temp_sensor_init(void) {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = TEMP_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = TEMP_ADC_ATTEN,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TEMP_ADC_CHANNEL, &chan_config));

    // Калибровка
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = TEMP_ADC_UNIT,
        .atten = TEMP_ADC_ATTEN,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available");
    }

    return ESP_OK;
}

esp_err_t app_temp_sensor_read(float *temperature_celsius) {
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, TEMP_ADC_CHANNEL, &raw));

    float voltage = 0.0f;
    if (adc_calibrated) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv) == ESP_OK) {
            voltage = (float)mv / 1000.0f;
        }
    } else {
        voltage = (float)raw * 3.3f / 4095.0f;
    }

    *temperature_celsius = (voltage - 0.424f) / 0.0195f;

    return ESP_OK;
}
