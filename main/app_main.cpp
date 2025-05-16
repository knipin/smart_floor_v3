#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
#include <platform/ESP32/ESP32SecureCertDACProvider.h>
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
#include <platform/ESP32/ESP32FactoryDataProvider.h>
#endif
using namespace chip::DeviceLayer;
#endif

#include "app_driver_temp_sensor.h"
#include "app_driver_heater.h"
#include "pid_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include <cmath>

static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

static float g_target_temperature = 40.0f;
static pid_controller_t g_pid;
#define TEMP_CONTROL_TASK_PERIOD_MS 1000

// Функция для чтения температуры с термистора по формуле Стейнхарта-Харта
float Temp_READ(float R, float Ro, float Bo, float To, float A, adc_channel_t channel, adc_oneshot_unit_handle_t adc_handle) {
    int Val_ADC = 0;
    // Чтение значения с АЦП
    if (adc_oneshot_read(adc_handle, channel, &Val_ADC) != ESP_OK) {
        return -273.15f; // Ошибка, возвращаем невозможную температуру
    }
    float Rt = (float)R / ((float)A/float(Val_ADC)-1);
    float Temp = 1.0 / (1.0 / To + 1.0 / Bo * log((float) Rt / Ro)) - 283.15;
    return Temp;
}

static void temp_control_task(void *arg)
{
    while (true) {
        float current_temp = Temp_READ(10000, 10000, 3950, 298, 4095, ADC_CHANNEL_0 , adc_handle);
        float power = pid_compute(&g_pid, g_target_temperature, current_temp);
        app_heater_set_power(power);
        ESP_LOGI("temp_ctrl", "T=%.2f°C → power=%.1f%% (target=%.1f)", current_temp, power, g_target_temperature);
        vTaskDelay(pdMS_TO_TICKS(TEMP_CONTROL_TASK_PERIOD_MS));
    }
}

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");
const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");
static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif

#ifdef CONFIG_ENABLE_MEMORY_PROFILING
static void memory_profiler_dump_heap_stat(const char *state)
{
    ESP_LOGI(TAG,"========== HEAP-DUMP-START ==========\n");
    ESP_LOGI(TAG,"state: %s\n", state);
    ESP_LOGI(TAG,"\tDescription\tInternal\n");
    ESP_LOGI(TAG,"Current Free Memory\t%d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG,"Largest Free Block\t%d\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG,"Min. Ever Free Size\t%d\n", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG,"========== HEAP-DUMP-END ==========\n");
}
#endif

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
            ESP_LOGI(TAG, "Interface IP Address changed");
            break;
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "Commissioning complete");
#ifdef CONFIG_ENABLE_MEMORY_PROFILING
            memory_profiler_dump_heap_stat("commissioning complete");
#endif
            break;
        case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
            ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
            break;
        case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
            ESP_LOGI(TAG, "Commissioning session started");
            break;
        case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
            ESP_LOGI(TAG, "Commissioning session stopped");
            break;
        case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
            ESP_LOGI(TAG, "Commissioning window opened");
#ifdef CONFIG_ENABLE_MEMORY_PROFILING
            memory_profiler_dump_heap_stat("commissioning window opened");
#endif
            break;
        case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
            ESP_LOGI(TAG, "Commissioning window closed");
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen()) {
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR) {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
            break;
        }
        case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
            ESP_LOGI(TAG, "Fabric will be removed");
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
            ESP_LOGI(TAG, "Fabric is updated");
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
            ESP_LOGI(TAG, "Fabric is committed");
            break;
        case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
            ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
#ifdef CONFIG_ENABLE_MEMORY_PROFILING
            memory_profiler_dump_heap_stat("BLE deinitialized");
#endif
            break;
        default:
            break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        if (cluster_id == LevelControl::Id &&
            attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
            g_target_temperature = 20.0f + (val->val.u8 / 254.0f) * 20.0f;
            ESP_LOGI(TAG, "Target temperature set to %.2f°C", g_target_temperature);
        }

        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    nvs_flash_init();
    app_temp_sensor_init();
    app_heater_init();
    pid_init(&g_pid, 5.0f, 0.5f, 1.0f, 0.0f, 100.0f);
    xTaskCreate(temp_control_task, "temp_control_task", 4096, NULL, 5, NULL);

    app_driver_handle_t light_handle = app_driver_light_init();
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle);

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    extended_color_light::config_t light_config;
    light_config.on_off.on_off = DEFAULT_POWER;
    light_config.on_off.lighting.start_up_on_off = nullptr;
    light_config.level_control.current_level = DEFAULT_BRIGHTNESS;
    light_config.level_control.on_level = DEFAULT_BRIGHTNESS;
    light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
    light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;

    endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create extended color light endpoint"));

    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);

    attribute_t *current_level_attribute = attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::set_deferred_persistence(current_level_attribute);

    attribute_t *current_x_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id);
    attribute::set_deferred_persistence(current_x_attribute);
    attribute_t *current_y_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id);
    attribute::set_deferred_persistence(current_y_attribute);
    attribute_t *color_temp_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
    attribute::set_deferred_persistence(color_temp_attribute);

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif
}
