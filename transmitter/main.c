#include "main.h"
#include "can18F4580_mscp.c"

enum
{
    CAN_ID_TABLE(EXPAND_AS_ENUMERATION)
};

void xbee_init(void)
{
    delay_ms(100);
    putc('b');
    putc('b');
    delay_ms(100);
}

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
    int i;
    struct rx_stat rxstat;
    int32 rx_id;
    int in_data[8];
    int rx_len;
    
    //xbee_init();
    
    can_init();
    set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in (default)
    enable_interrupts(INT_TIMER2);   //enable timer2 interrupt (if want to count ms)
    enable_interrupts(GLOBAL);       //enable all interrupts
    delay_us(200);
    
    while(true)
    {
        if (can_kbhit())
        {
            if (can_getd(rx_id, in_data, rx_len, rxstat))
            {
                output_toggle(LED_PIN);
                
                printf("\r\nRECIEVED: BUFF=%U ID=%3LX LEN=%U OVF=%U ",
                       rxstat.buffer,
                       rx_id,
                       rx_len,
                       rxstat.err_ovfl);
                
                printf("FILT=%U RTR=%U EXT=%U INV=%U",
                       rxstat.filthit,
                       rxstat.rtr,
                       rxstat.ext,
                       rxstat.inv);
                
                printf("\r\n    DATA = ");
                
                for (i = 0 ; i < rx_len ; i++)
                {
                    printf("%X ",in_data[i]);
                }
                printf("\r\n");
            }
        }
    }
}
