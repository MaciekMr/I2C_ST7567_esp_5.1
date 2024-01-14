#include "lcd_SSD1306.h"


ssd1306::ssd1306()
{
    init_master();
    i2c_scan();
}

/***************************************************
 * The LCD controller has to be initiated
 * 1. Reset LCD 
 * 2. Switch LCD off
 * 3. Set low column address
 * 4. Set high column address
 * 5. Set line begin address
 * 6. Set page addressing mode
 * 7. Set contrast
 * 8. Set remapping
 * 9. Set scan direction
 * 10. Set no inversion display
 * 11. Set multiplex ratio
 * 12. Set display offset
 * 13. Set divide ratio/oscilatot freq
 * 14. Set the pre charge period
 * 15. Set driver connection configuration
 * 16. Set VCOMH
 * 17. Set charge pump
 * 18. Connect RAM with LCD panel
 * 19. Switch inversion off
 * 20. Switch on LCD
 * 21. Clean LCD
*/
void ssd1306::init_chip()
{
    esp_err_t ret_val = ESP_OK;
    //Send set of commands to chip
    //Set the init byte with slave address
    ret_val = send_byte(CMD_DISPLAY_SLEEP);
    ret_val = send_byte(CMD_SET_LOW_NIBBLE_COL_ADDR);
    //ret_val = send_byte(CMD_SET_HIG_NIBBLE_COL_ADDR);
    //ret_val = send_byte(CMD_SET_DISP_START_LINE);
    //ret_val = send_byte(CMD_MEM_ADDRESS_MODE);
    //ret_val = send_byte(0x02);
    ret_val = send_byte(CMD_SET_CONTRAST);
    ret_val = send_byte(0x0F);
    ret_val = send_byte(CMD_SEGMENT_REMAP_7F);
    ret_val = send_byte(CMD_SET_COM_OUT_SCAN_DIR | 0x08);
    ret_val = send_byte(CMD_DISPLAY_NORMAL);
    //ret_val = send_byte(CMD_SET_MULTIPLEX_RATIO);
    //ret_val = send_byte(0x3F);
    //ret_val = send_byte(CMD_SET_DISPLAY_OFFSET);
    //ret_val = send_byte(0x00);
    //ret_val = send_byte(CMD_SET_RATIO_FREQ);
    //ret_val = send_byte(0x80);
    //ret_val = send_byte(CMD_SET_PRE_CHARGE_PER);
    //ret_val = send_byte(0xF1);
    //ret_val = send_byte(CMD_SET_COM_PIN_CONF);
    //ret_val = send_byte(0x12);
    //ret_val = send_byte(CMD_SET_VCOM_H);
    //ret_val = send_byte(0x40);
    ret_val = send_byte(CMD_SET_CHARGE_PUMP);
    ret_val = send_byte(0x14);
    ret_val = send_byte(CMD_DISPLAY_FULL_ON);
    ret_val = send_byte(CMD_DISPLAY_NORMAL);
    ret_val = send_byte(CMD_DISPLAY_ON);
    ret_val = send_byte(CMD_SET_LOW_NIBBLE_COL_ADDR);
    ret_val = send_byte(0x00);
    ret_val = send_byte(0x10);
    ret_val = send_byte(CMD_SET_PAGE_START_ADDR);
    ret_val = execute_command_set();

    if (ret_val == ESP_OK) {
		ESP_LOGI(I2C_TAG, "OLED configured successfully");
	} else {
		ESP_LOGE(I2C_TAG, "OLED configuration failed. code: %d 0x%.2X", ret_val, ret_val);
	}
    clean_lcd();
}

void ssd1306::init_lcd()
{
    //
}

void ssd1306::clean_lcd()
{
    esp_err_t ret_val = ESP_OK;
    i2c_cmd_handle_t cmd;

	uint8_t zero[128];
    memset(zero, 0, 128);
	for (uint8_t i = 0; i < 8; i++) {
        get_new_command_handler();
        ret_val = send_byte(CMD_SET_BYTE_CMD_SINGLE);
        ret_val = send_byte(CMD_SET_PAGE_START_ADDR | i);
        ret_val = send_byte(CMD_SET_DISP_START_LINE);
        ret_val = send_multi_byte(zero, 128);
        ret_val = execute_command_set();
	}
}



void ssd1306::example()
{

    esp_err_t ret_val = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_LOGI(I2C_TAG, "Device = %d  %s", device_port, __PRETTY_FUNCTION__);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_port << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, CMD_SET_LOW_NIBBLE_COL_ADDR, true);

	i2c_master_write_byte(cmd, CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, CMD_SEGMENT_REMAP_7F, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, CMD_SET_COM_OUT_SCAN_DIR, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	ret_val = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (ret_val == ESP_OK) {
		ESP_LOGI(I2C_TAG, "OLED configured successfully");
	} else {
		ESP_LOGE(I2C_TAG, "OLED configuration failed. code: 0x%.2X", ret_val);
	}
	i2c_cmd_link_delete(cmd);
}