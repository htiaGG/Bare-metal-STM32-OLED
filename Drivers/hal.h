#pragma once

#include <stdint.h>
#include <stdbool.h>

// Constants
#define DEFAULT_CPU_CLOCK 16000000
#define MAX_CPU_CLOCK 84000000

// Macros for peripheral programming
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
#define BIT(x) (1U << (x))

// Enums
enum GPIO_MODE {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_AF,
    GPIO_MODE_ANALOG
};

enum I2C_RW_Bit {
    RW_Write = 0,
    RW_Read = 1
};

// Struct declarations
struct flash;
struct gpio;
struct rcc;
struct usart;
struct i2c;
struct systick;

// Function declarations
void SystemClock_Config(void);
void systick_init(uint32_t ticks);
void gpio_set_mode(uint16_t pin, uint8_t mode);
void gpio_write(uint16_t pin, bool val);
void HAL_Delay(uint32_t delay);
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now);

// I2C functions
void I2C_Start(struct i2c *I2C);
void I2C_Write(struct i2c *I2C, uint16_t Addr, uint16_t MemAddress, uint8_t *data, uint16_t size);
void I2C_Address(struct i2c *I2C, uint8_t address, enum I2C_RW_Bit mode);
void I2C_Stop(struct i2c *I2C);
void I2C_Begin(struct i2c *I2C, uint8_t address, enum I2C_RW_Bit mode);

// External variables
extern volatile uint32_t s_ticks;

#include "hal_structs.h"


// Function Implementations
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
    if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
    if (*t == 0) *t = now + prd;                   // First poll? Set expiration
    if (*t > now) return false;                    // Not expired yet, return
    *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
    return true;                                   // Expired, return true
}
void HAL_Delay(uint32_t delay)
{
    uint32_t now = s_ticks;

    while((s_ticks - now) < delay) {}
}

void SystemClock_Config(void) {
    // 1. Enable HSI
    RCC->CR |= (1 << 0);
    while (!(RCC->CR & (1 << 1))); // Wait for HSI to be ready

    // 2. Configure PLL
    RCC->CR &= ~(1 << 24); // Disable the PLL
    RCC->PLLCFGR = (16 << 0) | // PLLM = 16 (HSI/16 = 1 MHz)
                   (336 << 6) | // PLLN = 168 (1 MHz * 168 = 168 MHz)
                   (1 << 16) |   // PLLP = 4 (168 MHz / 2 = 84 MHz)
                   (0 << 22) |      // PLL Source = HSI
                   (7 << 24);    // PLLQ = 7 (for USB OTG FS, SDIO, RNG)
    RCC->CR |= (1 << 24); // Enable PLL
    while (!(RCC->CR & (1 << 25))); // Wait for PLL to be ready

    // 4. Configure AHB, APB1, and APB2 prescalers
    RCC->CFGR &= ~(15 << 4); // Clear HPRE bits (no division for AHB, DIV1)
    RCC->CFGR &= ~(7 << 10); // Clear PPRE1 bits
    RCC->CFGR |= (4 << 10);  // Set PPRE1 bits to 100 (division by 2 for APB1)
    RCC->CFGR &= ~(7 << 13); // Clear PPRE2 bits
    RCC->CFGR |= (0 << 13);  // Set PPRE2 bits to 000 (no division for APB2, DIV1)

    // 4. Configure Flash latency
    FLASH->ACR &= ~(7 << 0); // Clear LATENCY bits
    FLASH->ACR |= (2 << 0);  // Set LATENCY bits to 010 (2 wait states for 84 MHz)

    // 5. Select PLL as system clock source
    RCC->CFGR &= ~(3 << 0); // Clear SW bits
    RCC->CFGR |= (2 << 0); // Select PLL as system clock
    while (((RCC->CFGR >> 2) & 3) != 2); // Wait for PLL to be used as system clock
}

void systick_init(uint32_t ticks)
{
    if ((ticks - 1) > 0xffffff) return;
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = (1 << 2) | (1 << 1) | (1 << 0); // CLKSOURCE, TICKINT, ENABLE bits

    // SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);
    RCC->APB2ENR |= BIT(14);
}

void gpio_set_mode(uint16_t pin, uint8_t mode)
{
    struct gpio *gpio = GPIO(PINBANK(pin)); // Pin bank
    uint8_t n = PINNO(pin);                 // Pin number
    gpio->MODER &= ~(0x3 << (n * 2));        // Clear current setting
    gpio->MODER |= (mode & 0x3) << (n * 2);   // Set new mode
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

void I2C_Start(struct i2c *I2C)
{
    I2C->CR1 |= (1 << 10);
    I2C->CR1 |= (1 << 8);
    while(!(I2C->SR1 & (1 << 0)));
}

void I2C_Write(struct i2c *I2C, uint16_t Addr, uint16_t MemAddress, uint8_t *data, uint16_t size)
{
    //TODO : COMPLETE THIS FUNCTION
    I2C_Begin(I2C, Addr, RW_Write);
    while(!(I2C->SR1 & (1 << 7)));
    I2C->DR = MemAddress;
    while(!(I2C->SR1 & (1 << 2)));

    for(uint16_t i = 0; i < size; i++)
    {
        while(!(I2C->SR1 & (1 << 7)));
        I2C->DR = data[i];
    }
    while(!(I2C->SR1 & (1 << 2)));

    I2C_Stop(I2C);
}

void I2C_Address(struct i2c *I2C, uint8_t address, enum I2C_RW_Bit mode)
{
    I2C->DR = ((address << 1) | (uint8_t)mode);
    while(!(I2C->SR1 & (1 << 1)));
    uint8_t temp = I2C->SR1 | I2C->SR2; // Clears the ADDR bit by reading SR1 followed by SR2
}

void I2C_Stop(struct i2c *I2C)
{
    I2C->CR1 |= (1 << 9);
}

void I2C_Begin(struct i2c *I2C, uint8_t address, enum I2C_RW_Bit mode)
{
    I2C_Start(I2C);
    I2C_Address(I2C, address, mode);
}
