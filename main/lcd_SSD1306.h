

struct buffer{
    uint8_t page; //each page has 7 bits
}

#define CMD_SET_CONTRAST        0x81
#define CMD_DISPLAY_FULL_ON     0xA4
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



class ssd1306
{
protected:
    uint8_t i2c_address;
    void refresh();
    void reset_column_counter();


public:
    ssd1306();
    void init_lcd();
    void clean_lcd();
    void set_point(uint8_t pos_x, uint8_t pos_y, uint8_t page);




}