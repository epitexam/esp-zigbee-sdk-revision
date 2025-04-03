#ifndef SGP30_H
#define SGP30_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define SGP30_ADDR 0x58

#define SGP30_CMD_IAQ_INIT 0x2003
#define SGP30_CMD_MEASURE_IAQ 0x2008
#define SGP30_CMD_READ_IAQ 0x2009

esp_err_t sgp30_send_command(uint16_t command);
esp_err_t sgp30_read_data(uint8_t *data, size_t len);
bool sgp30_check_crc(uint8_t *data, size_t len, uint8_t checksum);
esp_err_t sgp30_init();
esp_err_t sgp30_measure_iaq(uint16_t *co2, uint16_t *tvoc);

#endif // SGP30_H