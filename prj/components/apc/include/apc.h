#ifndef APC_H
#define APC_H

#include "stdint.h"

#define CFG_I2C_MASTER_SDA_IO   4
#define CFG_I2C_MASTER_SCL_IO   5


typedef struct APC_struct APC;

typedef enum APC_DistanceMode_enum
{
    ShortDistanceMode = 1,
    LongDistanceMode,
} APC_DistanceMode;

typedef enum APC_TimingBudget_enum
{
    Ms15 = 15,
    Ms20 = 20,
    Ms33 = 33,
    Ms50 = 50,
    Ms100 = 100,
    Ms200 = 200,
    Ms500 = 500,
} APC_TimingBudget;

typedef struct APC_config_struct
{
    uint16_t roi_x;
    uint16_t roi_y;
    APC_TimingBudget timing_budget;
    APC_DistanceMode distance_mode;
} APC_config;

void APC_create();
void APC_initialize(APC_config* conf);
void APC_start_count(uint16_t threshold);
void APC_stop_count();
uint16_t APC_get_count();

#endif // APC_H
