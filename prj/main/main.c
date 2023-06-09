#include "apc.h" 
#include "esp_log.h"
#include "FreeRTOS.h"
#include "freertos/task.h"

#define APC_COUNT_MODE 0


void app_main(void)
{
    APC_create();
    int count = 0;
    uint16_t threshold = 1600;

#if APC_COUNT_MODE == 1
    APC_config conf = {
        .roi_x = 8,
        .roi_y = 16,
        .timing_budget = Ms20,
        .distance_mode = LongDistanceMode,
    };
    APC_initialize(&conf);

    APC_start_count(threshold);
#else
    APC_initialize_4x4();
    APC_start_4x4(threshold);
#endif

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (APC_get_count() != count)
        {
            count = APC_get_count();
            //ESP_LOGI("MAIN", "Count: %d", count);
        }
    }
}
