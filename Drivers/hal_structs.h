#pragma once

#include <stdint.h>

struct flash {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
    volatile uint32_t OPTCR1;
};

struct gpio {
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

struct rcc {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED2[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t RESERVED3[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED4[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t RESERVED5[2];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVED6[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED7[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t RESERVED8;
    volatile uint32_t DCKCFGR;
};

struct usart {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
};

struct i2c {
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

struct systick {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
};

// Peripheral base addresses
#define FLASH ((struct flash *) 0x40023C00)
#define GPIO(port) ((struct gpio *) (0x40020000 + 0x400 * (port)))
#define RCC ((struct rcc *) 0x40023800)
#define USART1 ((struct usart *) 0x40011000)
#define USART2 ((struct usart *) 0x40004400)
#define USART6 ((struct usart *) 0x40011400)
#define I2C1 ((struct i2c *) 0x40005400)
#define I2C2 ((struct i2c *) 0x40005800)
#define I2C3 ((struct i2c *) 0x40005C00)
#define SYSTICK ((struct systick *) 0xe000e010)
