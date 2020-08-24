#ifndef _BMHE280HAND_H
#define _BMHE280HAND_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_log.h>

#include "bme280.h"
#include "bme280_defs.h"

#define IERROR_CHK(C) (((C) == ESP_OK) ? 0 : -1)
#define BME280_HANDLER_LOG_LEVEL CONFIG_lOG_LEVEL_BME280_HANDLER

typedef struct {
    uint8_t dev_addr; 
    int8_t bmestatus;
    uint8_t settings_sel;
    struct bme280_data comp_data;
    struct bme280_dev bme_dev;
} bme280_handler_t;

esp_err_t bme280handler_init(bme280_handler_t *ptr);

void bme280handler_set_normal_mode(bme280_handler_t *ptr);
void bme280handler_forced_mode(bme280_handler_t *ptr);

void bme280handler_print_rslt(const char api_name[], int8_t rslt); 
void bme280handler_print_sensor_data(bme280_handler_t *ptr);
void bme280handler_delay_ms(uint32_t period_ms, void *intf_ptr); 

int8_t bme280handler_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);  
int8_t bme280handler_i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr); 

#endif //_BMHE280HAND_H