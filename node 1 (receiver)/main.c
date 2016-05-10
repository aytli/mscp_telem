#include "main.h"

#define LED_PIN PIN_A1

void main()
{
    while(true)
    {
        output_toggle(LED_PIN);
        delay_ms(100);
    }
}
