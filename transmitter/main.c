#include "main.h"
#include "can18F4580_mscp.c"

#define HEARTBEAT_PERIOD_MS 500

// Creates a list of CAN packet IDs
enum
{
    CAN_ID_TABLE(EXPAND_AS_ID)
};

// Creates a list of CAN packet lengths
enum
{
    CAN_ID_TABLE(EXPAND_AS_LENGTH)
};

// Creates a list of telemetry packet IDs
enum
{
    TELEM_ID_TABLE(EXPAND_AS_ID)
};

// Creates a list of telemetry packet IDs
enum
{
    TELEM_ID_TABLE(EXPAND_AS_LENGTH)
};

static int8 g_motor_page[TELEM_MOTOR_LEN];

void xbee_init(void)
{
    // Puts the xbee into bypass mode
    // Documentation: http://xbee-sdk-doc.readthedocs.io/en/stable/doc/tips_tricks/
    delay_ms(100);
    putc('b');
    putc('b');
    delay_ms(100);
}

void send_data(int8 id, int len, int * data)
{
    int i;
    putc(id);
    putc(len);
    for (i = 0 ; i < len ; i++)
    {
        putc(*(data+i));
    }
}

#int_timer2
void isr_timer2(void)
{
    output_toggle(LED_PIN);
    send_data(TELEM_MOTOR_ID,TELEM_MOTOR_LEN,g_motor_page);
}

void main()
{
    int i;
    struct rx_stat rxstat;
    int32 rx_id;
    int in_data[8];
    int rx_len;
    
    // Set up and enable timer 2 with a period of HEARTBEAT_PERIOD_MS
    setup_timer2(TMR_EXTERNAL|TMR_DIV_BY_256,39*HEARTBEAT_PERIOD_MS);
    enable_interrupts(INT_TIMER2);
    
    xbee_init();
    
    can_init();
    set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in (default)
    enable_interrupts(INT_TIMER2);   //enable timer2 interrupt (if want to count ms)
    enable_interrupts(GLOBAL);       //enable all interrupts
    delay_us(200);
    
    int data[8] = {0,1,2,3,4,5,6,7};
    int data_len = 8;
    int id = TELEM_MOTOR_ID;
    int len = TELEM_MOTOR_LEN;
    
    while(true)
    {
        if (can_kbhit())
        {
            if (can_getd(rx_id, in_data, rx_len, rxstat))
            {
                switch(rx_id)
                {
                    case 0x501:
                        g_motor_page = in_data;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}
