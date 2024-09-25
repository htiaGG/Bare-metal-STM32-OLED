#pragma once
#include <stdint.h>
#include <stdbool.h>

volatile uint32_t s_ticks;

struct flash {
    volatile uint32_t ACR;      // Access Control Register, offset 0x00
    volatile uint32_t KEYR;     // Key Register, offset 0x04
    volatile uint32_t OPTKEYR;  // Option Key Register, offset 0x08
    volatile uint32_t SR;       // Status Register, offset 0x0C
    volatile uint32_t CR;       // Control Register, offset 0x10
    volatile uint32_t OPTCR;    // Option Control Register, offset 0x14
    volatile uint32_t OPTCR1;   // Option Control Register 1, offset 0x18
};

#define FLASH ((struct flash *) 0x40023C00)

struct gpio
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
};
#define GPIO(port) ((struct gpio *) (0x40020000 + 0x400 * (port)))  //GPIO port peripheral configuration

/*Peripheral programming macros*/
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
#define BIT(x) (1U << (x))


enum GPIO_MODE
{
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_AF,
    GPIO_MODE_ANALOG,
};

struct rcc
{
    volatile uint32_t CR;          //0x00
    volatile uint32_t PLLCFGR;     //0x04
    volatile uint32_t CFGR;        //0x08
    volatile uint32_t CIR;         //0x0C
    volatile uint32_t AHB1RSTR;    //0x10
    volatile uint32_t AHB2RSTR;    //0x14
    volatile uint32_t RESERVED1;   //0x18
    volatile uint32_t RESERVED2;   //0x1C
    volatile uint32_t APB1RSTR;    //0x20
    volatile uint32_t APB2RSTR;    //0x24
    volatile uint32_t RESERVED3;   //0x28
    volatile uint32_t RESERVED4;   //0x2C
    volatile uint32_t AHB1ENR;     //0x30
    volatile uint32_t AHB2ENR;     //0x34
    volatile uint32_t RESERVED5;   //0x38
    volatile uint32_t RESERVED6;   //0x3C
    volatile uint32_t APB1ENR;     //0x40
    volatile uint32_t APB2ENR;     //0x44
    volatile uint32_t RESERVED7;   //0x48
    volatile uint32_t RESERVED8;   //0x4C
    volatile uint32_t AHB1LPENR;   //0x50
    volatile uint32_t AHB2LPENR;   //0x54
    volatile uint32_t RESERVED9;   //0x58
    volatile uint32_t RESERVED10;  //0x5C
    volatile uint32_t APB1LPENR;   //0x60
    volatile uint32_t APB2LPENR;   //0x64
    volatile uint32_t RESERVED11;  //0x68
    volatile uint32_t RESERVED12;  //0x6C
    volatile uint32_t BDCR;        //0x70
    volatile uint32_t CSR;         //0x74
    volatile uint32_t RESERVED13;  //0x78
    volatile uint32_t RESERVED14;  //0x7C
    volatile uint32_t SSCGR;       //0x80
    volatile uint32_t PLLI2SCFGR;  //0x84
    volatile uint32_t RESERVED15;  //0x88
    volatile uint32_t DCKCFGR;     //0x8C
};
#define RCC ((struct rcc *) 0x40023800) // RCC peripheral

struct usart
{
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
};
#define USART1 ((struct usart *) 0x40011000)
#define USART2 ((struct usart *) 0x40004400)
#define USART6 ((struct usart *) 0x40011400)

struct i2c
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
};
#define I2C1 ((struct i2c *) 0x40005400)
#define I2C2 ((struct i2c *) 0x40005800)
#define I2C3 ((struct i2c *) 0x40005C00)

struct systick
{
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
};

#define DEFAULT_CPU_CLOCK 16000000
#define MAX_CPU_CLOCK 84000000
#define SYSTICK ((struct systick *) 0xe000e010) // SYSTICK peripheral


/*Functions*/

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
    // RCC->CR |= (1 << 0);
    // while (!(RCC->CR & (1 << 1))); // Wait for HSI to be ready

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

// I2C Functions
enum I2C_RW_Bit {
  RW_Write = 0,
  RW_Read = 1,
};

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
