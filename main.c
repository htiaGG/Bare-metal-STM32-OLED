#include <stdint.h>
#include <string.h>
#include "Drivers/hal.h"
#include "Drivers/ssd1306.h"

void SysTick_Handler(void)
{
    s_ticks++;
}

void *memset(void *dest, int value, size_t len) {
    unsigned char *ptr = dest;
    while (len-- > 0) {
        *ptr++ = (unsigned char)value;
    }
    return dest;
}

int main()
{
    systick_init(DEFAULT_CPU_CLOCK / 1000);

    RCC->AHB1ENR |= 1 << 0; // Enable GPIOA clock
    RCC->AHB1ENR |= 1 << 1; // Enable GPIOB clock
    RCC->APB1ENR |= 1 << 21; // Enable I2C1 clock

    // Led setup
    volatile uint16_t led = PIN('A', 5);
    gpio_set_mode(led, GPIO_MODE_OUTPUT);

    volatile uint16_t I2C1_SCL = PIN('B', 8);
    volatile uint16_t I2C1_SDA = PIN('B', 9);

    // Configure I2C1 pinout
    struct gpio *gpiob = GPIO(PINBANK(I2C1_SCL));

    gpio_set_mode(I2C1_SCL, GPIO_MODE_AF);
    gpio_set_mode(I2C1_SDA, GPIO_MODE_AF);

    gpiob->OTYPER |= (1 << 8) | (1 << 9);
    gpiob->OSPEEDR |= (3 << 16) | (3 << 18);
    gpiob->PUPDR |= (1 << 16) | (1 << 18);
    gpiob->AFR[1] |= (4 << 0) | (4 << 4);

    //Configuring IC2 registers
    struct i2c *i2c1 = I2C1;
    i2c1->CR1 |= (1 << 15);
    i2c1->CR1 &= ~(1 << 15);

    i2c1->CR2 |= (45 << 0);
    i2c1->CCR |= (225 << 0);
    i2c1->TRISE = 46;

    i2c1->CR1 |= (1 << 0); // Enable I2C

    /* TEMP: Testing Display (Successful) */
    SSD1306_Init(i2c1);
    // SSD1306_Draw_Pixel(5, 5);
    SSD1306_Update_Display();

    uint32_t timer = 0, period = 100;
    uint32_t frame = 0;

    int x = 0, y = 0;
    int dx = 1, dy = 1;

    /* TEMP: Testing timing + board LED (Successful)*/
	for(;;)
    {
        // bool on;
        // gpio_write(led, on);
        // on = !on;

        // SSD1306_Fill(Black);
        drawSquare(x, y);
        SSD1306_Update_Display();
        x += dx;
        y += dy;

        // Bounce off the edges
        if (x <= 0 || x >= SSD1306_WIDTH - 4) dx = -dx;
        if (y <= 0 || y >= SSD1306_HEIGHT - 4) dy = -dy;

        // HAL_delay(100);
    }
	return 0;
}

void createWavePattern(int frame) {
    // SSD1306_Clear_Display();
    for (int x = 0; x < SSD1306_WIDTH; x++) {
        int y = (SSD1306_HEIGHT / 2) + (SSD1306_HEIGHT / 4) * ((x + frame) % SSD1306_WIDTH) / SSD1306_WIDTH;
        SSD1306_Draw_Pixel(x, y, 1);
    }
}

void createGradient() {
    for (int y = 0; y < SSD1306_HEIGHT; y++) {
        for (int x = 0; x < SSD1306_WIDTH; x++) {
            // Create a gradient by setting pixels based on their position
            SSD1306_Draw_Pixel(x, y, x % 2);
        }
    }
}

void drawSquare(int x, int y) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            SSD1306_Draw_Pixel(x + i, y + j, 1);
        }
    }
}
