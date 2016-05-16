#include "main.h"
#include "can18F4580_mscp.c"

#define HEARTBEAT_PERIOD_MS 500

// Creates a list of CAN packet IDs
enum
{
    CAN_ID_TABLE(EXPAND_AS_CAN_ID)
};

// Creates a list of telemetry packet IDs
enum
{
    TELEM_ID_TABLE(EXPAND_AS_TELEM_ID)
};

// Creates a list of telemetry packet IDs
enum
{
    TELEM_ID_TABLE(EXPAND_AS_LENGTH)
};

static int8 g_bps_temp_page[8];

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

int16 ms;
#int_timer2
void isr_timer2(void)
{
    if (ms >= HEARTBEAT_PERIOD_MS)
    {
        ms = 0;
        output_toggle(TX_PIN);
        send_data(TELEM_MOTOR_ID,8,g_bps_temp_page);
    }
    else
    {
        ms++;
    }
}

void main()
{
    int i;
    struct rx_stat rxstat;
    int32 rx_id;
    int8 in_data[8];
    int rx_len;
    
    // Set up and enable timer 2 to interrupt every 1ms with a 20MHz clock
    setup_timer_2(T2_DIV_BY_4,79,16);
    enable_interrupts(INT_TIMER2);
    enable_interrupts(GLOBAL);
    
    xbee_init();
    can_init();
    set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in (default)
    delay_us(200);
    
    while(true)
    {
        if (can_kbhit())
        {
            if (can_getd(rx_id, in_data, rx_len, rxstat))
            {
                output_toggle(RX_PIN);
                switch(rx_id)
                {
                    case CAN_BPS_VOLTAGE_ID:     // Motor data received
                        break;
                    case CAN_BPS_TEMPERATURE_ID: // BPS voltage data
                        memcpy(g_bps_temp_page,in_data,rx_len);
                        break;
                    case CAN_BPS_CURRENT_ID:     // BPS current data
                        break;
                    case CAN_BPS_BALANCING_ID:   // BPS balancing bits
                        break;
                    case CAN_BPS_STATUS_ID:      // BPS status
                        break;
                    default:
                        break;
                }
            }
        }
    }
}
