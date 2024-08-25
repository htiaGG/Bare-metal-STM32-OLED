#pragma once

#include <stdint.h>

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

enum GPIO_MODE
{
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_AF,
    GPIO_MODE_ANALOG,
};

struct rcc
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED3;
    volatile uint32_t RESERVED4;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t RESERVED5;
    volatile uint32_t RESERVED6;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED7;
    volatile uint32_t RESERVED8;
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t RESERVED9;
    volatile uint32_t RESERVED10;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED11;
    volatile uint32_t RESERVED12;
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t RESERVED13;
    volatile uint32_t DCKCFGR;
};

#define RCC ((struct rcc *) 0x40023800) // RCC peripheral

struct systick
{
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
};

#define SYSTICK ((struct systick *) 0xe000e010) // SYSTICK peripheral

#define GPIO(port) ((struct gpio *) (0x40020000 + 0x400 * (port)))  //GPIO port peripheral configuration

/*Peripheral programming macros*/
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

/*Functions*/
void delay(uint32_t Delay)
{
    for (volatile int i = 0; i < 80000; i++);
}

void gpio_set_mode(uint16_t pin, uint8_t mode)
{
    struct gpio *gpio = GPIO(PINBANK(pin)); // Pin bank
    uint8_t n = PINNO(pin);                 // Pin number
    gpio->MODER &= ~(0x3 << (n * 2));        // Clear current setting
    gpio->MODER |= (mode & 0x3) << (n * 2);   // Set new mode
}
