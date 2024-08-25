#include <stdint.h>
#include "Drivers/hal.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void)
{
    s_ticks++;
}

int main()
{
    systick_init(DEFAULT_CPU_CLOCK / 1000);             // Tick every 1 ms

    // *((volatile uint16_t *)(0x40023800 + 0x30)) |= 1 << 0; //Enable RCC_AHB1 (RCC_AHB1ENR)
    RCC->AHB1ENR |= 1 << 0;

    volatile uint16_t led = PIN('A', 5);
    gpio_set_mode(led, GPIO_MODE_OUTPUT);

    uint32_t timer = 0, period = 1000;          // Declare timer and 500ms period

	for(;;)
    {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on;       // This block is executed
            gpio_write(led, on);  // Every `period` milliseconds
            on = !on;             // Toggle LED state
        }
    }
	return 0;
}
