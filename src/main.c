#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// --- АПАРАТНІ НАЛАШТУВАННЯ АЦП ---
#define ADC_UNIT           ADC_UNIT_1
#define ADC_CHAN           ADC_CHANNEL_3    // GPIO 4
#define ADC_ATTEN          ADC_ATTEN_DB_12  // Атенюація 12 дБ (діапазон вимірювання до ~3.3V)
#define ADC_BITWIDTH       ADC_BITWIDTH_12  // Розрядність 12 біт
#define ADC_MAX_RAW        4095.0f          // Максимальне значення для 12 біт (2^12 - 1)
#define MAX_VOLTAGE_MV     3300.0f          // Теоретична максимальна опорна напруга (mV)

static const char *TAG = "POTENTIOMETER";

// Ініціалізація схеми калібрування
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    }
#endif

    *out_handle = handle;
    return calibrated;
}

void app_main(void) {
    // 1. Ініціалізація модулю АЦП
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // 2. Налаштування каналу (GPIO 4)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHAN, &config));

    // 3. Запуск калібрування
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHAN, ADC_ATTEN, &adc1_cali_handle);
    
    if (!do_calibration) {
        ESP_LOGE(TAG, "Калібрування недоступне! Вимірювання можуть бути неточними.");
    }

    // 4. Вивід інформаційної шапки з параметрами АЦП
    printf("\n============================================================\n");
    printf(" ПАРАМЕТРИ АЦП ESP32 (Пін: GPIO 4 / ADC1_CH3)\n");
    printf("============================================================\n");
    printf(" • Розрядність (Resolution) : 12-bit (0 .. 4095)\n");
    printf(" • Атенюація (Attenuation)  : 12 dB (Діапазон 0..3300 mV)\n");
    printf(" • Опорна напруга (V_ref)   : %.0f mV (теоретична)\n", MAX_VOLTAGE_MV);
    printf("============================================================\n\n");

    // Вивід заголовка таблиці
    printf("%-8s  %-14s  %-14s  %-10s\n", "RAW", "U_manual(mV)", "U_cali(mV)", "Error(%)");
    printf("------------------------------------------------------------\n");

    int raw_val;
    int voltage_cali;

    while (1) {
        // Зчитуємо сире значення RAW (0..4095)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHAN, &raw_val));

        // 1. Ручний розрахунок: (RAW / 4095) * 3300 мВ
        float u_manual = ((float)raw_val * MAX_VOLTAGE_MV) / ADC_MAX_RAW;

        // 2. Отримання точного каліброваного значення в мВ
        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw_val, &voltage_cali));
        } else {
            voltage_cali = (int)u_manual; // Якщо калібрування збійнуло, беремо ручне
        }

        // 3. Розрахунок відсотка похибки відносно каліброваного значення
        float error_percent = 0.0f;
        if (voltage_cali > 0) {
            error_percent = (fabsf(u_manual - (float)voltage_cali) / (float)voltage_cali) * 100.0f;
        }

        // Виводимо рядок з даними в консоль
        printf("%-8d  %-14.1f  %-14d  %-10.2f\n", raw_val, u_manual, voltage_cali, error_percent);

        // Затримка ~100 мс (10 вимірювань на секунду)
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Очищення ресурсів (якщо цикл колись завершиться)
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (do_calibration) {
        adc_cali_delete_scheme_line_fitting(adc1_cali_handle);
    }
    #endif
}