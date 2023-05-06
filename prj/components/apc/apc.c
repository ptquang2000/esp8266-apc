#include "apc.h"
#include "VL53L1X_api.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"

#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "driver/i2c.h"

#include "FreeRTOS.h"
#include "freertos/task.h"

#define NO_PERSON                   0
#define IN_FIRST_ZONE               1
#define IN_SECOND_ZONE              2
#define IN_OVERLAP_ZONE             3

#define MAX_DISTANCE                2400 // mm
#define MIN_DISTANCE                0   // mm
#define DISTANCES_ARRAY_SIZE        10
#define NOBODY                      0
#define SOMEONE                     1
#define LEFT                        0
#define RIGHT                       1

static const char* TAG = "APC";


typedef struct APC_struct
{
    uint8_t roi_center[2];
    APC_config conf;
    uint16_t dev;

    uint16_t count;
    TaskHandle_t count_task_handle;
} APC;

static APC* s_apc = NULL;

static void set_roi_center()
{
    uint8_t X = s_apc->conf.roi_x > s_apc->conf.roi_y ? s_apc->conf.roi_y: s_apc->conf.roi_x;
    uint8_t Y = s_apc->conf.roi_x > s_apc->conf.roi_y ? s_apc->conf.roi_x : s_apc->conf.roi_y;
    uint8_t x = X  / 2;
    uint8_t y = Y / 2;

    uint8_t spad[16][16];
    for (uint8_t i = 0; i < 8; i++)
    {
        for (uint8_t j = 0; j < 16; j++)
        {
            spad[i][j] = 128 + j * 8 + i;
            spad[i + 8][j] = 255 - spad[i][j];
        }
    } 

    s_apc->roi_center[0] = spad[15 - y][x];
    s_apc->roi_center[1] = spad[15 - y][16 - x];
}

void APC_create()
{
    s_apc = malloc(sizeof(APC));
}

void APC_destroy()
{
    free(s_apc);
}

void APC_initialize(APC_config* apc_conf)
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

    memcpy(&s_apc->conf, apc_conf, sizeof(APC_config));
    ESP_ERROR_CHECK(s_apc->conf.roi_x > 16 || s_apc->conf.roi_x < 4);
    ESP_ERROR_CHECK(s_apc->conf.roi_y > 16 || s_apc->conf.roi_y < 4);
    set_roi_center();

    s_apc->dev = I2C_NUM_0;
    s_apc->count_task_handle = NULL;
    s_apc->count = 0;

    uint8_t state = 0;
    while (!state)
    {
        ESP_ERROR_CHECK(VL53L1X_BootState(I2C_NUM_0, &state));
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "VL53L1X is ready");

    uint16_t value = 0, value2 = 0;

    // s_apc->roi_center[0] = 175;
    // s_apc->roi_center[1] = 231;
    ESP_LOGI(TAG, "ROI center: %d %d", s_apc->roi_center[0], s_apc->roi_center[1]);

    ESP_ERROR_CHECK(VL53L1X_GetSensorId(s_apc->dev, &value));
    ESP_LOGI(TAG, "VL53L1X ID: 0x%04x", value);

    ESP_ERROR_CHECK(VL53L1X_SensorInit(s_apc->dev));

    ESP_ERROR_CHECK(VL53L1X_SetDistanceMode(s_apc->dev, s_apc->conf.distance_mode));
    ESP_ERROR_CHECK(VL53L1X_GetDistanceMode(s_apc->dev, &value));
    ESP_ERROR_CHECK(value != s_apc->conf.distance_mode);
    value = 0;

    ESP_ERROR_CHECK(VL53L1X_SetTimingBudgetInMs(s_apc->dev, s_apc->conf.timing_budget));
    ESP_ERROR_CHECK(VL53L1X_GetTimingBudgetInMs(s_apc->dev, &value));
    ESP_ERROR_CHECK(value != s_apc->conf.timing_budget);
    value = 0;

    ESP_ERROR_CHECK(VL53L1X_SetInterMeasurementInMs(s_apc->dev, s_apc->conf.timing_budget));
    ESP_ERROR_CHECK(VL53L1X_GetInterMeasurementInMs(s_apc->dev, &value));
    ESP_ERROR_CHECK(value != s_apc->conf.timing_budget);
    value = 0;

    ESP_ERROR_CHECK(VL53L1X_SetROI(s_apc->dev, s_apc->conf.roi_x, s_apc->conf.roi_y));
    ESP_ERROR_CHECK(VL53L1X_GetROI_XY(s_apc->dev, &value, &value2));
    ESP_ERROR_CHECK(value != s_apc->conf.roi_x || value2 != s_apc->conf.roi_y);
}

static void count_task(void* p)
{
    ESP_ERROR_CHECK(VL53L1X_StartRanging(s_apc->dev));
    uint8_t is_data_ready = 0, range_status;
    uint16_t threshold = (uint16_t)p;

    uint8_t path_track[] = {0, 0, 0, 0};
    uint8_t path_track_filling_size = 1;
    uint8_t right_prev_state = NO_PERSON;
    uint8_t left_prev_state = NO_PERSON;
    uint16_t distances[2][DISTANCES_ARRAY_SIZE];
    uint8_t distances_table_size[] = {0,0};

    uint8_t zone = RIGHT;
    uint8_t event_counts = 0;
    uint16_t distance;
    uint16_t min_distance;
    uint8_t i;

    uint8_t current_zone_state = NOBODY;
    uint8_t all_zones_current_state = 0;
    uint8_t an_event_occurred = 0;

    while (1)
    {
        while (!is_data_ready)
        {
            ESP_ERROR_CHECK(VL53L1X_CheckForDataReady(s_apc->dev, &is_data_ready));
            vTaskDelay(1 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
        }
        is_data_ready = 0;
        ESP_ERROR_CHECK(VL53L1X_GetRangeStatus(s_apc->dev, &range_status));
        ESP_ERROR_CHECK(VL53L1X_GetDistance(s_apc->dev, &distance));
        ESP_ERROR_CHECK(VL53L1X_ClearInterrupt(s_apc->dev));
        ESP_ERROR_CHECK(VL53L1X_SetROICenter(s_apc->dev, s_apc->roi_center[zone]));

        if ((range_status == 0) || (range_status == 4) || (range_status == 7)) 
        {
            distance = distance <= MIN_DISTANCE ? MAX_DISTANCE + MIN_DISTANCE : distance;
        }
        else
        {
            distance = MAX_DISTANCE;
        }

        current_zone_state = NOBODY;
        all_zones_current_state = 0;
        an_event_occurred = 0;

        if (distances_table_size[zone] < DISTANCES_ARRAY_SIZE) {
            distances[zone][distances_table_size[zone]] = distance;
            distances_table_size[zone]++;
        }
        else {
            for (i = 1; i < DISTANCES_ARRAY_SIZE; i++)
                distances[zone][i-1] = distances[zone][i];
            distances[zone][DISTANCES_ARRAY_SIZE-1] = distance;
        }

        min_distance = MAX_DISTANCE;
        uint16_t sum = 1;
        if (distances_table_size[zone] >= 2) {
            for (i = 0; i < distances_table_size[zone]; i++) {
                if (distances[zone][i] < min_distance)
                {
                    min_distance = distances[zone][i];
                }
                
                // if (distances[zone][i] < threshold)
                // {
                //     min_distance += distances[zone][i];
                //     sum++;
                // }
                // if (i == distances_table_size[zone] - 1)
                // {
                //     min_distance = min_distance / sum;
                // }
            }
        }

        if (min_distance < threshold) {
            current_zone_state = SOMEONE;
        }

        if (distance < threshold)
        {
            event_counts++;
        }

        if (zone == LEFT) {
            if (current_zone_state != left_prev_state) {
                an_event_occurred = 1;
                if (current_zone_state == SOMEONE) {
                    all_zones_current_state += IN_FIRST_ZONE;
                }
                if (right_prev_state == SOMEONE) {
                    all_zones_current_state += IN_SECOND_ZONE;
                }
                left_prev_state = current_zone_state;
            }
        }
        else {
            if (current_zone_state != right_prev_state) {
                an_event_occurred = 1;
                if (current_zone_state == SOMEONE) {
                    all_zones_current_state += IN_SECOND_ZONE;
                }
                if (left_prev_state == SOMEONE) {
                    all_zones_current_state += IN_FIRST_ZONE;
                }
                right_prev_state = current_zone_state;
            }
        }

        if (an_event_occurred) {
            if (path_track_filling_size < 4) {
                path_track_filling_size++;
            }

            if ((left_prev_state == NOBODY) && (right_prev_state == NOBODY)) {
                if (path_track_filling_size == 4) {
                    ESP_LOGI(TAG, "Path track: %d %d %d %d", path_track[0], path_track[1], path_track[2], path_track[3]);
                    if (event_counts >= DISTANCES_ARRAY_SIZE * .5)
                    {
                        if (
                                (path_track[1] == IN_FIRST_ZONE) && 
                                (path_track[2] == IN_OVERLAP_ZONE) &&
                                (path_track[3] == IN_SECOND_ZONE)) {
                            s_apc->count++;
                        } else if (
                                (path_track[1] == IN_SECOND_ZONE) &&
                                (path_track[2] == IN_OVERLAP_ZONE) && 
                                (path_track[3] == IN_FIRST_ZONE)) {
                            s_apc->count--;
                        }
                    } 
                    distances_table_size[0] = 0;
                    distances_table_size[1] = 0;
                }
                event_counts = 0;
                path_track_filling_size = 1;
            }
            else {
                // printf("Zone %d:", zone);
                // for (i = 0; i < DISTANCES_ARRAY_SIZE; i++)
                // {
                //     printf("\t%d", distances[zone][i]); 
                //     if (i == DISTANCES_ARRAY_SIZE - 1) printf("\n");
                // }

                printf("Event:\t%d, Path size:\t%d, Zone %d:", event_counts, path_track_filling_size - 1, zone);
                printf("\t%d", distances[LEFT][distances_table_size[LEFT] - 1]); 
                printf("\t%d\n", distances[RIGHT][distances_table_size[RIGHT] - 1]); 

                // printf("Event:\t%d, Path size:\t%d, Zone %d:\t%d\n", event_counts, path_track_filling_size - 1, zone, min_distance);

                path_track[path_track_filling_size - 1] = all_zones_current_state;
            }
        }
        zone++;
        zone = zone % 2;
    }
}

void APC_start_count(uint16_t threshold)
{
    ESP_ERROR_CHECK(s_apc->count_task_handle != NULL);
    xTaskCreate(count_task, "apc_count", 1024, (void*) threshold, tskIDLE_PRIORITY + 1, &s_apc->count_task_handle); 
}

void APC_stop_count()
{
    vTaskDelete(s_apc->count_task_handle);
    s_apc->count_task_handle = NULL;
}

uint16_t APC_get_count()
{
    return s_apc->count;
}

