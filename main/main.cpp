#include <stdio.h>
#include <string.h>
#include "esp_chip_info.h"
#include "esp_flash_spi_init.h"
#include "i2c_class.h"

extern "C" {
	void app_main(void);
}

static esp_err_t i2c_master_init()
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    printf("I2C mode %d", I2C_ESP_I2C_MODE);
    conf.mode = I2C_ESP_I2C_MODE;
    conf.sda_io_num = (gpio_num_t) I2C_SDA_PIN;
    conf.scl_io_num = (gpio_num_t) I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_CLOCK_SPEED;
    conf.clk_flags = 0;

    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

static void scan_port()
{
    // i2c init & scan
    if (i2c_master_init() != ESP_OK)
        ESP_LOGE(I2C_TAG, "i2c init failed\n");

     printf("i2c scan: \n");
     for (uint8_t i = 1; i < 127; i++)
     {
        int ret;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 100);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        //printf("Port %d \n", i);
        if (ret == ESP_OK)
        {
            printf("Found device at: 0x%2x\n", i);
        }
    }
}

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, \n", chip_info.revision);

    //printf("%dMB %s flash\n", esp_flash_get_size() / (1024 * 1024),
    //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    esp_err_t ret_val;
    i2c_esp32 i2c;
    i2c.i2c_scan();

    //scan_port();


/*
    i2c_master_bus_config_t i2c_mst_config_1; 
        i2c_mst_config_1.clk_source =   I2C_CLK_SRC_DEFAULT;
        i2c_mst_config_1.i2c_port =     I2C_MASTER_PORT;
        i2c_mst_config_1.scl_io_num =   I2C_SCL_PIN;
        i2c_mst_config_1.sda_io_num =   I2C_SDA_PIN;
        i2c_mst_config_1.glitch_ignore_cnt = 7;
        i2c_mst_config_1.flags.enable_internal_pullup = true;
        i2c_mst_config_1.intr_priority = 0; //low priority
        i2c_mst_config_1.trans_queue_depth = 2000;
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(ret_val = i2c_new_master_bus(&i2c_mst_config_1, &bus_handle));
    printf("master bus status %d \n", ret_val);

    ESP_ERROR_CHECK(ret_val = i2c_master_probe(bus_handle, 0x22, I2C_TIMEOUT));
    printf("probe status %d \n", ret_val);
    
    //add device
    i2c_device_config_t dev_config;
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = 0x22;
    dev_config.scl_speed_hz = I2C_CLOCK_SPEED;

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(ret_val = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
     printf("add device status %d \n", ret_val);
    char data[2] = {0x00, 0x00};
    //Try to transmit something to device
    ESP_ERROR_CHECK(ret_val = i2c_master_transmit(dev_handle, data, 2, I2C_TIMEOUT));
    printf("write status %d \n", ret_val);

    //remove device
    ESP_ERROR_CHECK(ret_val = i2c_master_bus_rm_device(dev_handle));
    printf("remove device status %d \n", ret_val);

    ESP_ERROR_CHECK(ret_val = i2c_del_master_bus(bus_handle));
    printf("del bus status %d \n", ret_val);

    */

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}