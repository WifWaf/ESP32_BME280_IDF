/* Aus BOSCH-Anleitung unter https://github.com/BoschSensortec/BME280_driver */

#include <stdio.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
 
#include "BME280Handler.h"

#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY
#define RTOS_DELAY_1SEC   ( 1 * 1000 / portTICK_PERIOD_MS)

static const char *TAG_BME_H = CONFIG_TAG_BME280_HANDLER;

esp_err_t bme280handler_init(bme280_handler_t *ptr) 
{
  esp_err_t f_retval = ESP_OK;                                                   // changed 
  i2c_cmd_handle_t cmd;

  esp_log_level_set(TAG_BME_H, BME280_HANDLER_LOG_LEVEL);   

  ptr->bmestatus = bme280_init(&ptr->bme_dev);
  bme280handler_print_rslt("bme280_init status      ", ptr->bmestatus); 
  bme280handler_set_normal_mode(ptr);                                            // set normal mode as the default setup
 
  cmd = i2c_cmd_link_create();                                                   // Verify that the I2C slave is working properly

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BME280_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);

  f_retval = i2c_master_cmd_begin(I2C_NUM_0, cmd, RTOS_DELAY_1SEC);
  if (f_retval != ESP_OK) 
    ESP_LOGE(TAG_BME_H, "I2C slave NOT working or wrong I2C slave address - error (%i)", f_retval);

  i2c_cmd_link_delete(cmd);
  
  return f_retval;
}

int8_t bme280handler_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) 
{
  int8_t i2c_addr = *((int *)intf_ptr);
  esp_err_t esp_err = ESP_OK;
  i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

  i2c_master_start(cmd_handle);
  i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd_handle, reg_addr, true);
  i2c_master_start(cmd_handle);
  i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_READ, true);

  if (length > 1)
      i2c_master_read(cmd_handle, reg_data, length - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd_handle, reg_data + length - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd_handle);

  esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd_handle);

  return IERROR_CHK(esp_err);
}

int8_t bme280handler_i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) 
{
  int8_t i2c_addr = *((int *)intf_ptr);
  esp_err_t esp_err = ESP_OK;
  i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

  i2c_master_start(cmd_handle);
  i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd_handle, reg_addr, true);
  i2c_master_write(cmd_handle, reg_data, length,true);
  i2c_master_stop(cmd_handle);

  esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);    
  i2c_cmd_link_delete(cmd_handle);

  return IERROR_CHK(esp_err);
}

void bme280handler_delay_ms(uint32_t period_ms, void *intf_ptr) 
{
  ets_delay_us(period_ms * 1000);  //vTaskDelay(period_ms / portTICK_PERIOD_MS);
}

void bme280handler_print_rslt(const char api_name[], int8_t rslt) 
{
  if (rslt != BME280_OK) 
  {
    ESP_LOGV(TAG_BME_H, "%s", api_name);
    switch(rslt)
    {
      case BME280_E_NULL_PTR:
        ESP_LOGE(TAG_BME_H, "Error [%d] : Null pointer error", rslt);
        break;
      case BME280_E_DEV_NOT_FOUND:
        ESP_LOGW(TAG_BME_H, "Error [%d] : Device not found", rslt);
        break;
      case BME280_E_INVALID_LEN:
        ESP_LOGE(TAG_BME_H, "Error [%d] : Invalid Lenght", rslt);
        break;
      case BME280_E_COMM_FAIL:
        ESP_LOGW(TAG_BME_H, "Error [%d] : Bus communication failed", rslt);
        break;
      case BME280_E_SLEEP_MODE_FAIL:
        ESP_LOGE(TAG_BME_H, "Error [%d] : SLEEP_MODE_FAIL", rslt);
        break;
      default: 
        ESP_LOGE(TAG_BME_H, "Error [%d] : Unknown error code", rslt);  /* For more error codes refer "*_defs.h" */     
        break;
    }        
  }
  else
  {
    ESP_LOGV(TAG_BME_H, "%s", api_name);
    ESP_LOGV(TAG_BME_H, " BME280 status [%d]",rslt);
  }
}

void bme280handler_print_sensor_data(bme280_handler_t *ptr)
{
  #ifdef BME280_FLOAT_ENABLE
    ESP_LOGI(TAG_BME_H, "Temperature in °C: %0.2f, Pressure: %0.2f, Humidity: %0.2f", ptr->comp_data.temperature, (ptr->comp_data.pressure / 100), ptr->comp_data.humidity);
  #else
    ESP_LOGI(TAG_BME_H, "%ld, %ld, %ld",ptr->comp_data.temperature, ptr->comp_data.pressure, ptr->comp_data.humidity);
  #endif
}

void bme280handler_set_normal_mode(bme280_handler_t *ptr)
{
  //*******// configure the sampling according to data spec sheet recommendation
  ptr->bme_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  ptr->bme_dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  ptr->bme_dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  ptr->bme_dev.settings.filter = BME280_FILTER_COEFF_16;
  ptr->bme_dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  ptr->settings_sel = BME280_OSR_PRESS_SEL;
  ptr->settings_sel |= BME280_OSR_TEMP_SEL;
  ptr->settings_sel |= BME280_OSR_HUM_SEL;
  ptr->settings_sel |= BME280_STANDBY_SEL;
  ptr->settings_sel |= BME280_FILTER_SEL;

  ptr->bmestatus = bme280_set_sensor_settings(ptr->settings_sel, &ptr->bme_dev);
  ptr->bmestatus = bme280_set_sensor_mode(BME280_NORMAL_MODE, &ptr->bme_dev);  

  bme280handler_print_rslt("bme280_set_sensor_settings status", ptr->bmestatus);
}

void bme280handler_forced_mode(bme280_handler_t *ptr)
{
  /* Recommended mode of operation: Indoor navigation */
  ptr->bme_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  ptr->bme_dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  ptr->bme_dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  ptr->bme_dev.settings.filter = BME280_FILTER_COEFF_16;
  ptr->settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  ptr->bmestatus = bme280_set_sensor_settings(ptr->settings_sel, &ptr->bme_dev);
  bme280handler_print_rslt("bme280_set_sensor_settings status", ptr->bmestatus);

  /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
    *  and the oversampling configuration. */
  ptr->bme_dev.delay_us(bme280_cal_meas_delay(&ptr->bme_dev.settings), ptr->bme_dev.intf_ptr);
  /*
  ESP_LOGI(TAG_BME_H, "Temperature, Pressure, Humidity\r\n");
  // Continuously stream sensor data
  while (1) 
  {
    ptr->bmestatus = bme280_set_sensor_mode(BME280_FORCED_MODE, ptr);
    bme280handler_print_rslt("bme280_set_sensor_mode status", ptr->bmestatus);
    // Wait for the measurement to complete and print data @25Hz
    ptr->bme_dev.delay_us(req_delay, ptr->bme_dev.intf_ptr);
    ptr->bmestatus = bme280_get_sensor_data(BME280_ALL, &ptr->comp_data, ptr);
    bme280handler_print_sensor_data(&ptr->comp_data);
  }
  */
}
