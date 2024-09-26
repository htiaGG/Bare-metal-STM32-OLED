#pragma once
#include "hal.h"

static struct i2c *I2C = I2C1;

#define SSD1306_I2C_ADDR 0x3C

#define SSD1306_HEIGHT 32
#define SSD1306_WIDTH 128

#define SSD1306_NUM_COLUMNS 128
#define SSD1306_NUM_PAGES 4
#define SSD1306_NUM_ROWS_PER_PAGE 8

typedef enum {
    Black = 0x00,
    White = 0x01,
} SSD1306_SCREEN_COLOR;

static uint8_t SSD1306_Screen_Buffer[SSD1306_NUM_PAGES * SSD1306_NUM_COLUMNS];

// SSD1306 control byte
#define CONTROL_COMMAND 0x00
#define CONTROL_DATA    0x40

#define SSD1306_DISPLAY_OFF             0xAE
#define SSD1306_DISPLAY_ON              0xAF
#define SSD1306_SET_DISPLAY_CLOCK       0xD5
#define SSD1306_SET_MULTIPLEX           0xA8
#define SSD1306_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_SET_START_LINE          0x40
#define SSD1306_CHARGE_PUMP             0x8D
#define SSD1306_MEMORY_MODE             0x20
#define SSD1306_SEG_REMAP               0xA0
#define SSD1306_COM_SCAN_DEC            0xC8
#define SSD1306_SET_COM_PINS            0xDA
#define SSD1306_SET_CONTRAST            0x81
#define SSD1306_SET_PRECHARGE           0xD9
#define SSD1306_SET_VCOM_DETECT         0xDB
#define SSD1306_DISPLAY_ALL_ON_RESUME   0xA4
#define SSD1306_NORMAL_DISPLAY          0xA6

#define COMMAND_COLUMN_ADDRESS          0x21
#define COMMAND_SET_PAGE_ADDRESS        0x22


void SSD1306_Send_Command(uint8_t address, uint8_t cmd)
{
    I2C_Write(I2C, address, CONTROL_COMMAND, &cmd, 1);
}


// Initialize SSD1306 display with a sequence of commands
void SSD1306_Init() {
    HAL_Delay(100);

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_DISPLAY_OFF);             // Display off

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_DISPLAY_CLOCK);       // Set display clock
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x80);                            // Recommended oscillator frequency

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_MULTIPLEX);           // Set multiplex ratio
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x1F);                            // 1/64 duty

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_DISPLAY_OFFSET);      // Set display offset
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x00);                            // No offset

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_START_LINE | 0x00);   // Set start line at 0

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_CHARGE_PUMP);             // Enable charge pump
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x14);                            // Enable charge pump

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_MEMORY_MODE);             // Set memory addressing mode
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x00);                            // Horizontal addressing mode

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SEG_REMAP | 0x01);        // Set segment re-map
    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_COM_SCAN_DEC);            // Set COM output scan direction

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_COM_PINS);            // Set COM pins hardware configuration
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x02);                            // Alternative COM pin configuration

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_CONTRAST);            // Set contrast control
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0xCF);                            // Max contrast

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_PRECHARGE);           // Set pre-charge period
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0xF1);                            // Phase 1 and 2 periods

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_SET_VCOM_DETECT);         // Set VCOMH deselect level
    SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x40);                            // Default value

    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_DISPLAY_ALL_ON_RESUME);   // Resume to RAM content display
    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_NORMAL_DISPLAY);          // Set normal display mode
    SSD1306_Send_Command(SSD1306_I2C_ADDR, SSD1306_DISPLAY_ON);              // Display on

    SSD1306_Fill(Black);
    SSD1306_Update_Display();
}

void SSD1306_Send_Data(const uint8_t* buffer, unsigned long length)
{
    for (uint32_t i = 0u; i < length; ++i)
    {
        SSD1306_Send_Command(SSD1306_I2C_ADDR, 0xB0 + i);
        SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x00);
        SSD1306_Send_Command(SSD1306_I2C_ADDR, 0x10);
        I2C_Write(I2C, SSD1306_I2C_ADDR, CONTROL_DATA, &buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

void SSD1306_Set_Column_Address(uint8_t start, uint8_t end)
{
    SSD1306_Send_Command(SSD1306_I2C_ADDR, COMMAND_COLUMN_ADDRESS);
    SSD1306_Send_Command(SSD1306_I2C_ADDR, start);
    SSD1306_Send_Command(SSD1306_I2C_ADDR, end);
}

void SSD1306_Set_Page_Address(uint8_t start, uint8_t end)
{
    SSD1306_Send_Command(SSD1306_I2C_ADDR, COMMAND_SET_PAGE_ADDRESS);
    SSD1306_Send_Command(SSD1306_I2C_ADDR, start);
    SSD1306_Send_Command(SSD1306_I2C_ADDR, end);
}

void SSD1306_Fill(SSD1306_SCREEN_COLOR color)
{
    for (int i = 0; i < sizeof(SSD1306_Screen_Buffer); i++)
    {
        SSD1306_Screen_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

void SSD1306_Draw_Pixel(uint32_t x, uint32_t y, uint8_t value)
{
    /* NOTE: Only upper bounds checking as coordinates are unsigned */
    if ((x > SSD1306_NUM_COLUMNS - 1) || (y > (SSD1306_NUM_PAGES * SSD1306_NUM_ROWS_PER_PAGE)))
    {
        /* NOTE: Error, logging? */
        return;
    }

    uint8_t page = y / SSD1306_NUM_ROWS_PER_PAGE;
    uint8_t bit = y % SSD1306_NUM_ROWS_PER_PAGE;
    uint16_t index = x + (page * SSD1306_WIDTH);

    if (value)
    {
        SSD1306_Screen_Buffer[index] |= (1 << bit);
    } else {
        SSD1306_Screen_Buffer[index] &= ~(1 << bit);
    }
}

void SSD1306_Update_Display()
{
    SSD1306_Send_Data(SSD1306_Screen_Buffer, sizeof(8));
}
