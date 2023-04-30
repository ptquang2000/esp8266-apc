#include "VL53L1X_api.h"
#include "driver/i2c.h"
#include "FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include <stdint.h>

#define CFG_I2C_MASTER_SDA_IO   4
#define CFG_I2C_MASTER_SCL_IO   5

const char* TAG = "APC";

void app_main(void)
{
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CFG_I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = CFG_I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));

    uint16_t dev = I2C_NUM_0;
    uint8_t state = 0;
    while (!state)
    {
        ESP_ERROR_CHECK(VL53L1X_BootState(I2C_NUM_0, &state));
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "VL53L1X is ready");

    uint16_t id;
    ESP_ERROR_CHECK(VL53L1X_GetSensorId(dev, &id));
    ESP_LOGI(TAG, "VL53L1X ID: 0x%04x", id);
    uint16_t num = 0xeeac;
    ESP_LOGI(TAG, "VL53L1X ID: 0x%04x", num);


    ESP_ERROR_CHECK(VL53L1X_SensorInit(dev));
    ESP_LOGI(TAG, "VL53L1X init");

    uint16_t distanceMode = 2;
    ESP_ERROR_CHECK(VL53L1X_SetDistanceMode(dev, distanceMode));
    ESP_ERROR_CHECK(VL53L1X_GetDistanceMode(dev, &distanceMode));
    ESP_LOGI(TAG, "VL53L1X mode: %d", distanceMode);

    uint16_t timingBudget = 20;
    ESP_ERROR_CHECK(VL53L1X_SetTimingBudgetInMs(dev, timingBudget));
    ESP_ERROR_CHECK(VL53L1X_GetTimingBudgetInMs(dev, &timingBudget));
    ESP_LOGI(TAG, "VL53L1X Timing Budget: %d ms", timingBudget);

    ESP_ERROR_CHECK(VL53L1X_SetInterMeasurementInMs(dev, timingBudget));
    ESP_ERROR_CHECK(VL53L1X_GetInterMeasurementInMs(dev, &timingBudget));
    ESP_LOGI(TAG, "VL53L1X Inter Measurement: %d ms", timingBudget);

    ESP_ERROR_CHECK(VL53L1X_StartRanging(dev));
    uint8_t isDataReady = 0, rangeStatus;
    uint16_t distance;
    while (1)
    {
        while (!isDataReady)
        {
            ESP_ERROR_CHECK(VL53L1X_CheckForDataReady(dev, &isDataReady));
            if (isDataReady) break;
            vTaskDelay(timingBudget / portTICK_PERIOD_MS);
        }
        isDataReady = 0;
        ESP_ERROR_CHECK(VL53L1X_GetRangeStatus(dev, &rangeStatus));
        ESP_LOGI(TAG, "Status: %d", rangeStatus);
        ESP_ERROR_CHECK(VL53L1X_GetDistance(dev, &distance));
        ESP_LOGI(TAG, "Distance: %d mm", distance);
        ESP_ERROR_CHECK(VL53L1X_ClearInterrupt(dev));
    }
}
