#include "main.h"

#define LED_PIN PIN_A1

void main()
{
    while(true)
    {
        putc(0x78);
        output_toggle(LED_PIN);
        delay_ms(100);
    }
}
