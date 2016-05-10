#include "main.h"

#define LED_PIN PIN_A1

enum
{
    CAN_ID_TABLE(EXPAND_AS_ENUMERATION)
};

void send_data(int page_id, int len, int * data)
{
    int i;
    putc(page_id);
    putc(len);
    for (i = 0 ; i < len ; i++)
    {
        putc(*(data+i));
    }
}

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
