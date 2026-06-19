#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "LIGHT_SERVO_CTRL";

// --- Configuration ---
#define PHOTO_SENSOR_ADC_CHANNEL ADC_CHANNEL_3 // GPIO 4 is ADC1 Channel 3 on ESP32-S3
#define SERVO_GPIO               5             // GPIO 5 for Servo PWM
#define LIGHT_THRESHOLD_HIGH     600           // mV threshold to enter BRIGHT state
#define LIGHT_THRESHOLD_LOW      400           // mV threshold to enter DARK state

// Servo PWM Parameters (Standard 50Hz Servo)
#define SERVO_CLK_SRC            LEDC_AUTO_CLK
#define SERVO_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL            LEDC_CHANNEL_0
#define SERVO_TIMER              LEDC_TIMER_0
#define SERVO_DUTY_RES           LEDC_TIMER_14_BIT // 14-bit resolution (0 to 16383)

// Servo Pulse Width Limits (Standard 0.5ms to 2.5ms)
// 50Hz frequency means a period of 20ms.
// 14-bit max value = 16384. 
// 0.5ms = (0.5/20) * 16384 = 410
// 2.5ms = (2.5/20) * 16384 = 2048
#define SERVO_MIN_DUTY           410   // Approx 0 degrees
#define SERVO_MAX_DUTY           2048  // Approx 180 degrees

// State Tracking
typedef enum {
    STATE_DARK,
    STATE_BRIGHT
} LightState_t;

// --- Helper Functions ---
void init_servo(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = SERVO_SPEED_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = SERVO_DUTY_RES,
        .freq_hz          = 50, // Standard servo frequency
        .clk_cfg          = SERVO_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = SERVO_SPEED_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = SERVO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_GPIO,
        .duty           = SERVO_MIN_DUTY, // Start at 0 degrees
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_servo_angle(uint32_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_SPEED_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_SPEED_MODE, SERVO_CHANNEL));
}

LightState_t get_light_state(int voltage_mv, LightState_t current_state)
{
    if (current_state == STATE_BRIGHT) {
        return (voltage_mv > LIGHT_THRESHOLD_LOW) ? STATE_BRIGHT : STATE_DARK;
    }
    return (voltage_mv >= LIGHT_THRESHOLD_HIGH) ? STATE_BRIGHT : STATE_DARK;
}

void app_main(void) {
    // 1. Initialize Servo
    init_servo();

    // 2. Initialize ADC (Oneshot mode) for Pin 4
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Typically 12-bit
        .atten    = ADC_ATTEN_DB_12,      // Allows reading full range up to ~3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHOTO_SENSOR_ADC_CHANNEL, &config));

    // Optional but recommended: Setup calibration to get actual millivolts
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration = false;
    
    // Simple check/init for calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = PHOTO_SENSOR_ADC_CHANNEL,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
        do_calibration = true;
    }

    // 3. Main Loop Logic
    LightState_t current_state = STATE_DARK;
    set_servo_angle(SERVO_MIN_DUTY); // Initial servo position

    int adc_raw;
    int voltage_mv;

    ESP_LOGI(TAG, "System running. Monitoring photo sensor...");

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PHOTO_SENSOR_ADC_CHANNEL, &adc_raw));

        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv));
        } else {
            voltage_mv = (adc_raw * 3300) / 4095;
        }

        LightState_t detected_state = get_light_state(voltage_mv, current_state);
        uint32_t duty = (detected_state == STATE_BRIGHT) ? SERVO_MAX_DUTY : SERVO_MIN_DUTY;

        ESP_LOGI(TAG, "ADC raw=%d, voltage=%d mV, state=%s, duty=%u.",
                 adc_raw,
                 voltage_mv,
                 detected_state == STATE_BRIGHT ? "BRIGHT" : "DARK",
                 duty);

        if (detected_state != current_state) {
            ESP_LOGI(TAG, "State changed to %s. Moving servo to %s position.",
                     detected_state == STATE_BRIGHT ? "BRIGHT" : "DARK",
                     detected_state == STATE_BRIGHT ? "MAX" : "MIN");
            set_servo_angle(duty);
            current_state = detected_state;
        }

        vTaskDelay(pdMS_TO_TICKS(250)); // Sample every 250ms
    }

    // Clean up (never reached in this loop environment, but good practice)
    if (do_calibration) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
    }
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}