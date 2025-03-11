// credit to https://github.com/RobinFrcd
// this issue helped me a lot : https://github.com/espressif/esp-zigbee-sdk/issues/518

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_temperature_sensor.h"
#include "string.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include <inttypes.h>
#include <stdint.h>
#include <math.h>

#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define SGP30_ADDR 0x58

#define SGP30_CMD_IAQ_INIT 0x2003
#define SGP30_CMD_MEASURE_IAQ 0x2008
#define SGP30_CMD_READ_IAQ 0x2009

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

#define DEFINE_PSTRING(var, str)   \
    const struct                   \
    {                              \
        unsigned char len;         \
        char content[sizeof(str)]; \
    }(var) = {sizeof(str) - 1, (str)}

static const char *TAG = "ESP_ZB_SGP_30_SENSOR";

float_t puissance_moins_six(uint16_t valeur)
{
    // Convertir la valeur uint16_t en float
    float_t valeur_float = (float_t)valeur;

    // Calculer la puissance -6
    float_t resultat = powf(valeur_float, 1.0f);

    return resultat;
}

esp_err_t i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur de configuration I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erreur d'installation du driver I2C: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

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

void reportAttribute(uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = HA_ESP_SENSOR_ENDPOINT,
            .src_endpoint = HA_ESP_SENSOR_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(HA_ESP_SENSOR_ENDPOINT, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void reportValue(uint16_t cluserID, uint16_t valueID, void *value)
{
    // esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        cluserID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        valueID,
        value,
        false);
    // esp_zb_lock_release();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            // ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *custom_sensor_clusters_create()
{
    esp_zb_basic_cluster_cfg_t basic_cfg;
    esp_zb_identify_cluster_cfg_t identify_cfg;

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    DEFINE_PSTRING(ManufacturerName, "CustomSensor");
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)&ManufacturerName));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_measured_value = 0,
        .max_measured_value = 60000,
    };

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(
        cluster_list,
        esp_zb_carbon_dioxide_measurement_cluster_create(&co2_meas_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_LOGI(TAG, "Carbon dioxide measurement cluster added");

    return cluster_list;
}

static esp_zb_ep_list_t *custom_sensor_ep_create(uint8_t endpoint_id)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, custom_sensor_clusters_create(), endpoint_config);
    return ep_list;
}

static void configure_cluster_reporting(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id)
{
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = endpoint,
        .cluster_id = cluster_id,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 10,
        .attr_id = attr_id,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);
}

static void init_zigbee(void)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT);

    esp_zb_device_register(esp_zb_sensor_ep);

    configure_cluster_reporting(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_stack_main_loop();
}

static void read_data_task(void *pvParameters)
{

    uint16_t co2 = 0;
    uint16_t tvoc = 0;

    while (1)
    {
        esp_err_t err = sgp30_measure_iaq(&co2, &tvoc);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Erreur lors de la lecture des données : %s", esp_err_to_name(err));
        }

        esp_zb_lock_acquire(portMAX_DELAY);

        float_t tt = puissance_moins_six(co2) / 1000000.0;
        ESP_LOGI(TAG, "CO2: %.6f ppm", tt);
        reportValue(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &tt);

        esp_zb_lock_release();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{

    if (i2c_master_init() == ESP_OK)
    {
        if (sgp30_init() == ESP_OK)
        {
            ESP_LOGI(TAG, "SGP30 : OK!");
        }
        else
        {
            ESP_LOGE(TAG, "💥 Échec de l'initialisation du SGP30 !");
        }
    }
    else
    {
        ESP_LOGE(TAG, "💥 Échec de l'initialisation I2C !");
    }

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    init_zigbee();

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(read_data_task, "data_reader", 16384, NULL, 4, NULL);
}
