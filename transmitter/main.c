#include "main.h"

#define LED_PIN PIN_A1

void main()
{
    delay_ms(100);
    putc('b');
    putc('b');
    delay_ms(100);
    while(true)
    {
        putc(0x78);
        output_toggle(LED_PIN);
        delay_ms(200);
    }
}
