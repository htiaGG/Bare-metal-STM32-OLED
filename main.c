#include <stdint.h>
#include <string.h>
#include "Drivers/hal.h"
#include "Drivers/ssd1306.h"

volatile uint32_t s_ticks;
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
    SSD1306_Draw_Pixel(5, 5);
    SSD1306_Update_Display();

    uint32_t timer = 0, period = 1000;

    /* TEMP: Testing timing + board LED (Successful)*/
	for(;;)
    {
        bool on;
        if (timer_expired(&timer, period, s_ticks)) {
            // static bool on;
            gpio_write(led, on);
            on = !on;
        }
    }
	return 0;
}
