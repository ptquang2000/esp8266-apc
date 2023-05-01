#include "apc.h" 
#include "esp_log.h"
#include "FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    APC_create();

    APC_config conf = {
        .roi_x = 8,
        .roi_y = 16,
        .timing_budget = Ms20,
        .distance_mode = LongDistanceMode,
    };
    APC_initialize(&conf);

    APC_start_count(1600);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI("MAIN", "Count: %d", APC_get_count());
    }
}
