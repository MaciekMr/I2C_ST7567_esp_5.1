#include "i2c_class.h"


i2c_esp32::i2c_esp32()
{
    pins.scl_pin = (gpio_num_t) (int) I2C_SCL_PIN;
    pins.sda_pin = (gpio_num_t) (int) I2C_SDA_PIN;
    //bus_config = new i2c_master_bus_handle_t;
}

i2c_esp32::~i2c_esp32()
{
    //delete(i2c_master_bus_t)
    i2c_driver_delete(I2C_MASTER_NUM);
}

esp_err_t  i2c_esp32::init_master()
{
    ESP_LOGI(I2C_TAG, "%s", __PRETTY_FUNCTION__);

    printf("SCL %d SDA %d mode: %d \n",I2C_SCL_PIN,I2C_SDA_PIN, I2C_ESP_I2C_MODE);
    esp_err_t ret_val = 0;
    bus_config.mode = I2C_ESP_I2C_MODE;
    bus_config.sda_io_num = I2C_SDA_PIN;         // select SDA GPIO specific to your project
    bus_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    bus_config.scl_io_num = I2C_SCL_PIN;         // select SCL GPIO specific to your project
    bus_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    bus_config.master.clk_speed = I2C_CLOCK_SPEED;  // select frequency specific to your project
    bus_config.clk_flags = 0;                          // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    
    ESP_ERROR_CHECK(ret_val = i2c_param_config(I2C_MASTER_NUM, &bus_config));
    //install driver
    ESP_ERROR_CHECK(ret_val = i2c_driver_install(I2C_MASTER_NUM, bus_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    if(ret_val != ESP_OK)
        ESP_LOGI(I2C_TAG, "i2c_new_master_bus ret_val = %d  in: %s", ret_val, __PRETTY_FUNCTION__);

    return (ret_val);
}

esp_err_t  i2c_esp32::init_device(uint8_t _port_no = I2C_PORT_NO)
{
    ESP_LOGE(I2C_TAG, "%s", __PRETTY_FUNCTION__);
    esp_err_t ret_val = 0;
    /*
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = _port_no;
    dev_config.scl_speed_hz = I2C_CLOCK_SPEED;
    
    ESP_ERROR_CHECK(ret_val = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    */
    if(ret_val != ESP_OK)
        ESP_LOGI(I2C_TAG, "i2c_master_bus_add_device ret_val = %d  in: %s", ret_val, __PRETTY_FUNCTION__);
    
    return (ret_val);
}

esp_err_t i2c_esp32::remove_device()
{
    ESP_LOGE(I2C_TAG, "%s", __PRETTY_FUNCTION__);
    esp_err_t ret_val = 0;
    //ret_val = i2c_master_bus_rm_device(dev_handle);
    if(ret_val != ESP_OK)
        ESP_LOGI(I2C_TAG, "i2c_master_bus_rm_device ret_val = %d  in: %s", ret_val, __PRETTY_FUNCTION__);
    return(ret_val);
}

esp_err_t  i2c_esp32::send_data(uint8_t port_no, uint8_t * write_buffer, size_t buffer_size, int time_out = I2C_TIMEOUT)
{
    esp_err_t ret_val;
    ret_val = i2c_master_write_to_device(I2C_MASTER_NUM, port_no, write_buffer, buffer_size, time_out / portTICK_PERIOD_MS);
    return(ret_val);
}
/******************************************************
 * For a probe function, MASTER has to send to SLAVE
 * the full byte (8 bits), where A7->A1 is a port of SLAVE
 * and the A0 contains the R/W bit => 0 - write 1 - read
 * and simple 2 bytes {0,0} to receive ACK reponse from
 * SLAVE.
 * When SLAVE respons, it is connected 
*******************************************************/
esp_err_t i2c_esp32::probe_device(uint8_t _port_no)
{
    esp_err_t ret_val = ESP_OK;
    uint8_t data[2] = {0,0};
    //printf("status %d \n", ret_val);
    //ret_val = i2c_master_probe(bus_handle, _port_no, I2C_TIMEOUT);
    //ret_val = send_data(_port_no, data, sizeof(data));
    //printf("send_data status %d \n", ret_val);
    //ESP_ERROR_CHECK(ret_val);
    //printf("status %d \n", ret_val);
    //printf("i2c scan %d: \n", ((_port_no << 1) | I2C_MASTER_WRITE)); //A7->A1 - address, A0 - R/W flag
    /*
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (_port_no << 1) | I2C_MASTER_WRITE, true);
    //i2c_master_write_byte(cmd_handle, 0x3C, true);
    i2c_master_stop(cmd_handle);
    ret_val = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    */
    ret_val = i2c_master_write_to_device(I2C_MASTER_NUM, _port_no, data, 2, I2C_TIMEOUT/ portTICK_PERIOD_MS);
    //printf("Port %d \n", i);
    //if (ret == ESP_OK)
    //{
    //    printf("Found device at: 0x%2x\n", i);
    //}
    return(ret_val);
}

/**************************************************************
 * When port_no = -1, system needs to scan i2c ports to find
 * a valid port number. Set the range from 0x00 to 0xFF
 * and execute the tasks:
 * initialise i2c with port number
 * 
***************************************************************/
void i2c_esp32::i2c_scan()
{
    esp_err_t ret_val = ESP_OK;
    init_master();
    uint8_t port_i2c_scan = 1;
    if(I2C_PORT_NO == -1)
    {
        //Scan I2C ports
        do
        {
            //ESP_LOGI(I2C_TAG, "i2c scan port number = %d  in: %s", port_i2c_scan, __PRETTY_FUNCTION__);
            //init_device(port_i2c_scan);
            //Send short data to given address
            ret_val = probe_device(port_i2c_scan);
            if(ret_val == ESP_OK)
                ESP_LOGI(I2C_TAG, "Device is found on address: %d", port_i2c_scan);
            //if(ret_val == ESP_ERR_NOT_FOUND)
                //ESP_LOGE(I2C_TAG, "Device is not found on address: %d", port_i2c_scan);
            //remove device
            //remove_device();
        }while(++port_i2c_scan != I2C_PORT_RANGE);
        
    }
    //ret_val = i2c_del_master_bus(bus_handle);
    ret_val = i2c_driver_delete(I2C_MASTER_NUM);
}