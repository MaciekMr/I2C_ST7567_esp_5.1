#pragma once

#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include "driver/i2c_master.h"
#include "driver/i2c.h"
//#include "driver/i2c_types.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#ifdef CONFIG_I2C_MODE_MASTER
    #define I2C_ESP_I2C_MODE I2C_MODE_MASTER
#else
    #define I2C_ESP_I2C_MODE I2C_MODE_SLAVE
#endif
#define I2C_MODE_SLAVE CONFIG_I2C_MODE_SLAVE
#define I2C_PORT_NO CONFIG_I2C_PORT_NO
#define I2C_SDA_PIN CONFIG_I2C_SDA_PIN
#define I2C_SCL_PIN CONFIG_I2C_SCL_PIN
#define I2C_CLOCK_SPEED CONFIG_I2C_CLOCK_SPEED
#define I2C_TIMEOUT CONFIG_I2C_TIMEOUT

#define I2C_MASTER_PORT -1
#define I2C_PORT_RANGE 0x7F //7bit address - 0-127
#define ACK_CHECK_EN   0x1

#define I2C_TAG "I2C_MASTER"

struct I2C_PINS
{
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
};

class i2c_esp32
{
//Static
private:
    i2c_cmd_handle_t            cmd_handle;
protected:
    static bool                 i2c_initiated;
protected:
    uint8_t                     device_port;
protected:
    i2c_config_t                bus_config;
    int                         i2c_port;
    I2C_PINS                    pins;

public:
    i2c_esp32();
    ~i2c_esp32();
    esp_err_t init_master();
    esp_err_t init_device(uint8_t _port_no);
    esp_err_t remove_device();
    void get_new_command_handler();
    esp_err_t execute_command_set();
    esp_err_t send_data(uint8_t port_no, uint8_t * write_buffer, size_t buffer_size, int time_out = I2C_TIMEOUT);
    esp_err_t send_byte(uint8_t write_buffer, int time_out = I2C_TIMEOUT);
    esp_err_t send_byte(uint8_t port_no, uint8_t *write_buffer, int time_out = I2C_TIMEOUT);
    esp_err_t send_multi_byte(uint8_t * write_buffer, size_t buffer_size, int time_out = I2C_TIMEOUT);
    esp_err_t send_multi_byte(uint8_t port_no, uint8_t * write_buffer, size_t buffer_size, int time_out = I2C_TIMEOUT);
    void i2c_scan();
    esp_err_t probe_device(uint8_t _port_no);
};