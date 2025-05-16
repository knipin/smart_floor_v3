#include "app_driver_heater.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define TAG "heater"

#define HEATER_GPIO         1                    // Используй нужный GPIO для нагревателя
#define HEATER_PWM_FREQ     5000                 // Частота ШИМ в Гц
#define HEATER_PWM_RES      LEDC_TIMER_10_BIT    // Разрешение ШИМ (1024 уровней)
#define HEATER_PWM_CHANNEL  LEDC_CHANNEL_0
#define HEATER_PWM_TIMER    LEDC_TIMER_0
#define HEATER_PWM_SPEED    LEDC_LOW_SPEED_MODE

esp_err_t app_heater_init(void) {
    // Инициализация таймера ШИМ
    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode = HEATER_PWM_SPEED;
    timer_conf.timer_num = HEATER_PWM_TIMER;
    timer_conf.duty_resolution = HEATER_PWM_RES;
    timer_conf.freq_hz = HEATER_PWM_FREQ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Инициализация канала
    ledc_channel_config_t channel_conf = {
        .gpio_num   = HEATER_GPIO,
        .speed_mode = HEATER_PWM_SPEED,
        .channel    = HEATER_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = HEATER_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    return ESP_OK;
}

esp_err_t app_heater_set_power(float percent) {
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t max_duty = (1 << HEATER_PWM_RES) - 1;
    uint32_t duty = (uint32_t)(percent / 100.0f * max_duty);

    ESP_ERROR_CHECK(ledc_set_duty(HEATER_PWM_SPEED, HEATER_PWM_CHANNEL, duty));
    return ledc_update_duty(HEATER_PWM_SPEED, HEATER_PWM_CHANNEL);
}
