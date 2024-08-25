#include <stdint.h>
#include "hal.h"

int main()
{
    // *((volatile uint16_t *)(0x40023800 + 0x30)) |= 1 << 0; //Enable RCC_AHB1 (RCC_AHB1ENR)
    RCC->AHB1ENR |= 1 << 0;
    delay(1000);
    volatile uint16_t led = PIN('A', 5);
    gpio_set_mode(led, GPIO_MODE_OUTPUT);

	for(;;)
    {
        GPIO(PINBANK(led))->ODR |= (1 << 5);   // Set new mode
        delay(1000);
        GPIO(PINBANK(led))->ODR &= (0 << 5);   // Set new mode
        delay(1000);
    }
	return 0;
}
