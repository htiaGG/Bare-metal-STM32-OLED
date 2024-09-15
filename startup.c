#include <stdint.h>
// Create references to symbols defined in the linker script

extern unsigned int _data_start;
extern unsigned int _data_end;
extern unsigned int _data_load;
extern unsigned int _bss_start;
extern unsigned int _bss_end;

void _reset();
int main();

__attribute__((naked, noreturn))
void _reset(void)
{
    volatile uint16_t *src, *dst;

    for (src = &_data_load, dst = &_data_start; dst < &_data_end; src++, dst++)
    {
        *dst = *src;
    }

    for (dst = &_bss_start; dst < &_bss_end; dst++)
    {
        *dst = 0;
    }

    main();

    while(1);
}

extern void SysTick_Handler(void);  // Defined in main.c
extern void _estack(void);          // Defined in link.ld

__attribute__((section(".vectors")))
void (*const tab[16 + 91])(void) = {_estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
