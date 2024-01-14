#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "string.h"
#include "i2c_class.h"

struct buffer{
    uint8_t page; //each page has 7 bits
};

#define CMD_SET_CONTRAST        0x81
#define CMD_DISPLAY_FULL_ON     0xA4  //connect RAM to display
#define CMD_DISPLAY_ON_NO_RAM   0xA5
#define CMD_DISPLAY_NORMAL      0xA6
#define CMD_DISPLAY_INVERSE     0xA7
#define CMD_DISPLAY_ON          0xAF
#define CMD_DISPLAY_SLEEP       0xAE
#define CMD_RIGHT_HOR_SCROLL    0x26
#define CMD_LEFT_HOR_SCROLL     0x27
#define CMD_RIGHT_VERT_SCROLL   0x29
#define CMD_LEFT_VERT_SCROLL    0x2A
#define CMD_STOP_SCROLLING      0x2E
#define CMD_START_SCROLLING     0x2F
#define CMD_ROWS_SCROLLING      0xA3
#define CMD_SET_LOW_NIBBLE_COL_ADDR 0x00  //->0x0F Set Lower Column Start Address for Page Addressing Mode
#define CMD_SET_HIG_NIBBLE_COL_ADDR 0x10  //->0x1F Set Higher Column Start Address for Page Addressing Mode
#define CMD_MEM_ADDRESS_MODE    0x20  // 0x20 -> horizontal addressing mode, 0x21 ->vertical addressing mode, 0x22 - page addressing mode (reset)
#define CMD_SET_COL_ADDRESS     0x21
#define CMD_SET_PAGE_ADDRESS    0x22
#define CMD_SET_PAGE_START_ADDR 0xB0 // + page number 0->7 valid only in page addressing mode
///////LCD panel configuration
#define CMD_SET_DISP_START_LINE 0x40 // -> + 0:63 (0x3F) line nummber (0x7F - max)
#define CMD_SEGMENT_REMAP_00    0xA0
#define CMD_SEGMENT_REMAP_7F    0xA1
#define CMD_SET_MULTIPLEX_RATIO 0xA8 //second byte 0x00 -> 0x3F
#define CMD_SET_COM_OUT_SCAN_DIR    0xC0 // | 0x00 - normal mode (RESET) scna from COM0 to COM[N-1] , N -multiplex ratio , 0x08 - remapped mode scan from COM[N-1] -> COM0
#define CMD_SET_DISPLAY_OFFSET  0xD3 // second byte - set vertical shift by COM from 0x00 to 0x3F
#define CMD_SET_COM_PIN_CONF    0xDA // second byte 0x02 - sequential COM pin config 0x12 - alternative COM pin config  0x22 - enable COM Left/Right remap 0x02 - disable COM left/right remao
#define CMD_SET_RATIO_FREQ      0xD5 //second byte freq of frames per second
#define CMD_SET_PRE_CHARGE_PER  0xD9
#define CMD_SET_VCOM_H          0xDB // Set V_COM_H
#define CMD_SET_CHARGE_PUMP     0x8D
#define CMD_SET_BYTE_CMD_SINGLE 0x80


#define LCD_I2C_INTERFACE   "I2C"

class ssd1306:
    protected i2c_esp32
{
protected:
    void refresh();
    void reset_column_counter();

public:
    ssd1306();
    void init_chip();  //Initialise chip before play with LCD
    void init_lcd();
    void clean_lcd();
    void set_point(uint8_t pos_x, uint8_t pos_y, uint8_t page);
    void example();
};