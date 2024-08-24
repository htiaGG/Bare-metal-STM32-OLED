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
    while(1);   //NOTE: In case main returns
}
