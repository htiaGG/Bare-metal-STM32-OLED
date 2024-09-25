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
    SystemClock_Config();
    systick_init(MAX_CPU_CLOCK / 1000);

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

    //Configuring I2C registers
    struct i2c *i2c1 = I2C1;
    i2c1->CR1 |= (1 << 15);
    i2c1->CR1 &= ~(1 << 15);

    i2c1->CR2 |= (45 << 0);
    i2c1->CCR |= (225 << 0);
    i2c1->TRISE = 46;

    i2c1->CR1 |= (1 << 0); // Enable I2C

    //Configure USART2
    volatile uint16_t USART2_Tx = PIN('A', 2);
    volatile uint16_t USART2_Rx = PIN('A', 3);
    struct gpio *gpioa = GPIO(PINBANK(USART2_Tx));
    RCC->APB1ENR |= 1 << 17;
    gpioa->AFR[0] |= (7 << (PINNO(USART2_Tx) * 4)) | (7 << (PINNO(USART2_Rx) * 4));

    /* TEMP: Testing Display (Successful) */
    SSD1306_Init(i2c1);

    uint32_t timer = 0, period = 100;
    uint32_t frame = 0;

    int x = 0, y = 0;
    int dx = 1, dy = 1;

    /* TEMP: Testing timing + board LED (Successful)*/
	for(;;)
    {
        bool on;
        if(on)
        {
            SSD1306_Fill(White);
        } else {
            SSD1306_Fill(Black);
        }
        SSD1306_Update_Display();
        on = !on;
        // bool on;
        // gpio_write(led, on);
        // on = !on;

        // SSD1306_Fill(Black);
        // drawSquare(x, y);
        // SSD1306_Update_Display();
        // x += dx * 2;
        // y += dy * 2;

        // // Bounce off the edges
        // if (x <= 0 || x >= SSD1306_WIDTH - 4) dx = -dx;
        // if (y <= 0 || y >= SSD1306_HEIGHT - 4) dy = -dy;
    }
	return 0;
}

void drawSquare(int x, int y) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            SSD1306_Draw_Pixel(x + i, y + j, 1);
        }
    }
}
