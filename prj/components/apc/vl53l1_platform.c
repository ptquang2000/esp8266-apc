
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define CFG_VL53L1X_WRITE_ADDR	0x52
#define CFG_VL53L1X_READ_ADDR	0x53
#define ACK_CHECK_EN			0x01
#define ACK_VAL                 0x00
#define NACK_VAL                0x01
#define LAST_NACK_VAL           0x02

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_write(cmd, pdata, count, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_READ_ADDR, ACK_CHECK_EN);
    i2c_master_read(cmd, pdata, sizeof(uint8_t), LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_write(cmd, &data, sizeof(uint8_t), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    int ret;
    uint16_t temp = ((data & 0xff00) >> 8) | ((data & 0x00ff) << 8);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_write(cmd, &temp, sizeof(uint16_t), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    int ret;
    uint32_t temp =  
        ((data & 0xff000000) >> 24) | ((data & 0x0000ff00) << 8) |
        ((data & 0x000000ff) << 24) | ((data & 0x00ff0000) >> 8);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_write(cmd, &temp, sizeof(uint32_t), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_READ_ADDR, ACK_CHECK_EN);
    i2c_master_read(cmd, data, sizeof(uint8_t), LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_READ_ADDR, ACK_CHECK_EN);
    i2c_master_read(cmd, data, sizeof(uint16_t), LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    uint16_t temp = *data;
    *data = ((temp & 0xff00) >> 8) | ((temp & 0x00ff) << 8);

    return ret;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(index), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CFG_VL53L1X_READ_ADDR, ACK_CHECK_EN);
    i2c_master_read(cmd, data, sizeof(uint32_t), LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    uint32_t temp = *data;
    *data = 
        ((temp & 0xff000000) >> 24) | ((temp & 0x0000ff00) << 8) |
        ((temp & 0x000000ff) << 24) | ((temp & 0x00ff0000) >> 8);

    return ret;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	vTaskDelay((TickType_t)wait_ms / portTICK_PERIOD_MS);
    return 1;
}
