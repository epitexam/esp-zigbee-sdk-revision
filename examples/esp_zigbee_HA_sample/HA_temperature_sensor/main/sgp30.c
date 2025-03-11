#include "sgp30.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "SGP30";

esp_err_t sgp30_send_command(uint16_t command)
{
    uint8_t cmd[2];
    cmd[0] = (command >> 8) & 0xFF;
    cmd[1] = command & 0xFF;

    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, SGP30_ADDR, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur d'envoi de la commande 0x%04X: %s", command, esp_err_to_name(err));
    }
    return err;
}

esp_err_t sgp30_read_data(uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, SGP30_ADDR, data, len, pdMS_TO_TICKS(1000));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur de lecture des données: %s", esp_err_to_name(err));
    }
    return err;
}

bool sgp30_check_crc(uint8_t *data, size_t len, uint8_t checksum)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc == checksum;
}

esp_err_t sgp30_init()
{
    esp_err_t err = sgp30_send_command(SGP30_CMD_IAQ_INIT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur d'initialisation du SGP30: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t sgp30_measure_iaq(uint16_t *co2, uint16_t *tvoc)
{
    uint8_t data[6];

    esp_err_t err = sgp30_send_command(SGP30_CMD_MEASURE_IAQ);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur lors de l'envoi de la commande de mesure IAQ.");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(12));

    err = sgp30_read_data(data, sizeof(data));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur lors de la lecture des données IAQ.");
        return err;
    }

    if (!sgp30_check_crc(&data[0], 2, data[2]))
    {
        ESP_LOGE(TAG, "Erreur de CRC pour les données CO2.");
        return ESP_ERR_INVALID_CRC;
    }
    if (!sgp30_check_crc(&data[3], 2, data[5]))
    {
        ESP_LOGE(TAG, "Erreur de CRC pour les données TVOC.");
        return ESP_ERR_INVALID_CRC;
    }

    *co2 = (data[0] << 8) | data[1];
    *tvoc = (data[3] << 8) | data[4];

    ESP_LOGI(TAG, "CO2 = %d ppm, TVOC = %d ppb", *co2, *tvoc);
    return ESP_OK;
}