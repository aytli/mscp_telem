#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

#define SENDING_PERIOD_MS 50
#define POLLING_PERIOD_MS 200

// CAN bus defines
#define TX_PRI 3
#define TX_EXT 0
#define TX_RTR 1

#define TELEM_SEND_PACKET(i) \
    send_data(g_telem_id[i],g_telem_len[i],gp_telem_page[i]);

// Creates an array of telemetry packet IDs
static int16 g_telem_id[N_TELEM_ID] =
{
    TELEM_ID_TABLE(EXPAND_AS_TELEM_ID_ARRAY)
};

// Creates an array of telemetry packet lengths
static int16 g_telem_len[N_TELEM_ID] =
{
    TELEM_ID_TABLE(EXPAND_AS_TELEM_LEN_ARRAY)
};

// Creates an array of polling IDs
static int16 g_polling_id[N_CAN_POLLING_ID] =
{
    CAN_POLLING_TABLE(EXPAND_AS_POLLING_ID_ARRAY)
};

TELEM_ID_TABLE(EXPAND_AS_PAGE_DECLARATIONS)
static int * gp_telem_page[N_TELEM_ID] =
{
    TELEM_ID_TABLE(EXPAND_AS_PAGE_ARRAY)
};

static int1 gb_poll;

// Puts the xbee into bypass mode
// Documentation: http://xbee-sdk-doc.readthedocs.io/en/stable/doc/tips_tricks/
void xbee_init(void)
{
    delay_ms(100);
    putc('b');
    putc('b');
    delay_ms(100);
}

// Sends a page of data over the radio module
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

// INT_TIMER2 programmed to trigger every 1ms with a 20MHz clock
// Telemetry data will be sent out one page at a time with a period of SENDIN_PERIOD_MS
#int_timer2
void isr_timer2(void)
{
    static int16 ms;
    static int8 i = 0;
    
    if (ms >= SENDING_PERIOD_MS)
    {
        ms = 0; // Reset timer
        output_toggle(TX_PIN);
        TELEM_SEND_PACKET(i);
        
        if (i >= (N_TELEM_ID-1))
        {
            i = 0;
        }
        else
        {
            i++;
        }
    }
    else
    {
        ms++;
    }
}

// INT_TIMER4 programmed to trigger every 1ms with a 20MHz clock
// Polling request flag will be set with a period of POLLING_PERIOD_MS
#int_timer4
void isr_timer4(void)
{
    static int16 ms;
    if (ms >= POLLING_PERIOD_MS)
    {
        ms = 0;         // Reset timer
        gb_poll = true; // Raise polling request flag
    }
    else
    {
        ms++;
    }
}

// CAN receive buffer 1 interrupt
int32 g_can1_id;
int8  g_can1_data[8] = {0,0,0,0,0,0,0,0};
int8  g_can1_len;
int1  gb_can1_hit = false;
#int_canrx0
void isr_canrx0() { // CAN receive buffer 0 interrupt
    struct rx_stat rxstat;
    
    output_toggle(RX_PIN);
    if (can_getd(g_can1_id, g_can1_data, g_can1_len, rxstat))
    {
        gb_can1_hit = true;
    }
    else
    {
        gb_can1_hit = false;
    }
}

// CAN receive buffer 2 interrupt
int32 g_can2_id;
int8  g_can2_data[8] = {0,0,0,0,0,0,0,0};
int8  g_can2_len;
int1  gb_can2_hit = false;
#int_canrx1
void isr_canrx1()
{ // CAN receive buffer 1 interrupt
    struct rx_stat rxstat;

    output_toggle(RX_PIN);
    if (can_getd(g_can2_id, g_can2_data, g_can2_len, rxstat))
    {
        gb_can2_hit = true;
    }
    else
    {
        gb_can2_hit = false;
    }
}

void main()
{
    int i;
    struct rx_stat rxstat;
    int32 rx_id;
    int8 in_data[8];
    int rx_len;
    
    clear_interrupt(INT_CANRX0);
    enable_interrupts(INT_CANRX0);
    clear_interrupt(INT_CANRX1);
    enable_interrupts(INT_CANRX1);
    
    setup_timer_2(T2_DIV_BY_4,79,16); // Timer 2 set up to interrupt every 1ms with a 20MHz clock
    setup_timer_4(T4_DIV_BY_4,79,16); // Timer 4 set up to interrupt every 1ms with a 20MHz clock
    enable_interrupts(INT_TIMER2);
    enable_interrupts(INT_TIMER4);
    enable_interrupts(GLOBAL);
    
    xbee_init();
    can_init();
    set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in (default)
    delay_us(200);
    
    int empty[8] = {0,0,0,0,0,0,0,0};
    
    while(true)
    {
        if (gb_can1_hit == true)
        {
            // Data in buffer 1, transfer contents
            rx_id = g_can1_id;
            rx_len = g_can1_len;
            memcpy(&in_data[0],g_can1_data,8);
        }
        else if (gb_can2_hit == true)
        {
            // Data in buffer 2, transfer contents
            rx_id = g_can2_id;
            rx_len = g_can2_len;
            memcpy(&in_data[0],g_can2_data,8);
        }
        else
        {
            // no can data, do nothing
        }
        
        // If CAN data was received
        if (gb_can1_hit || gb_can2_hit)
        {
            // Check the ID of the received packet and update the corresponding page
            switch(rx_id)
            {
                // MOTOR DATA
                case CAN_MOTOR_BUS_VI_ID:       // Motor voltage and current
                    memcpy(&g_motor_bus_vi_page[0],in_data,rx_len);
                    break;
                case CAN_MOTOR_VELOCITY_ID:     // Motor velocity
                    memcpy(&g_motor_velocity_page[0],in_data,rx_len);
                    break;
                case CAN_MOTOR_HS_TEMP_ID:  // Motor heatsink temperature
                    memcpy(&g_motor_hs_temp_page[0],in_data,rx_len);
                    break;
                case CAN_MOTOR_DSP_TEMP_ID:  // Motor DSP temperature
                    memcpy(&g_motor_dsp_temp_page[0],in_data,rx_len);
                    break;
                
                // EV DRIVER CONTROLS DATA
                case CAN_EVDC_DRIVE_ID:
                    memcpy(&g_evdc_drive_page[0],in_data,rx_len);
                    break;
                
                // BPS DATA
                case CAN_BPS_VOLTAGE1_ID:       // BPS voltage 1
                    memcpy(&g_bps_voltage_page[0],in_data,rx_len);
                    break;
                case CAN_BPS_VOLTAGE2_ID:       // BPS voltage 2
                    memcpy(&g_bps_voltage_page[8],in_data,rx_len);
                   break;
                case CAN_BPS_VOLTAGE3_ID:       // BPS voltage 3
                    memcpy(&g_bps_voltage_page[16],in_data,rx_len);
                    break;
                case CAN_BPS_VOLTAGE4_ID:       // BPS voltage 4
                    memcpy(&g_bps_voltage_page[24],in_data,rx_len);
                    break;
                case CAN_BPS_VOLTAGE5_ID:       // BPS voltage 5
                    memcpy(&g_bps_voltage_page[32],in_data,rx_len);
                    break;
                case CAN_BPS_VOLTAGE6_ID:       // BPS voltage 6
                    memcpy(&g_bps_voltage_page[40],in_data,rx_len);
                    break;
                case CAN_BPS_VOLTAGE7_ID:       // BPS voltage 7
                    memcpy(&g_bps_voltage_page[48],in_data,rx_len);
                    break;
                case CAN_BPS_VOLTAGE8_ID:       // BPS voltage 8
                    memcpy(&g_bps_voltage_page[56],in_data,rx_len);
                    break;
                case CAN_BPS_TEMPERATURE1_ID:   // BPS temperature 1
                    memcpy(&g_bps_temperature_page[0],in_data,rx_len);
                    break;
                case CAN_BPS_TEMPERATURE2_ID:   // BPS temperature 2
                    memcpy(&g_bps_temperature_page[8],in_data,rx_len);
                    break;
                case CAN_BPS_TEMPERATURE3_ID:   // BPS temperature 3
                    memcpy(&g_bps_temperature_page[16],in_data,rx_len);
                    break;
                case CAN_BPS_CURRENT_ID:        // BPS current
                    memcpy(&g_bps_current_page[0],in_data,rx_len);
                    break;
                case CAN_BPS_BALANCING_ID:      // BPS balancing bits
                    memcpy(&g_bps_balancing_page[0],in_data,rx_len);
                    break;
                case CAN_BPS_STATUS_ID:         // BPS status
                    memcpy(&g_bps_status_page[0],in_data,rx_len);
                    break;
                
                // PMS DATA
                case CAN_PMS_DATA_ID:           // PMS data (aux voltage/temperature, DC/DC temperature)
                    memcpy(&g_pms_page[0],in_data,rx_len);
                    break;
                
                // MPPT DATA
                case CAN_MPPT1_ID:              // MPPT 1 data
                    memcpy(&g_mppt_page[0],in_data,rx_len);
                    break;
                case CAN_MPPT2_ID:              // MPPT 2 data
                    memcpy(&g_mppt_page[7],in_data,rx_len);
                    break;
                case CAN_MPPT3_ID:              // MPPT 3 data
                    memcpy(&g_mppt_page[14],in_data,rx_len);
                    break;
                case CAN_MPPT4_ID:              // MPPT 4 data
                    memcpy(&g_mppt_page[21],in_data,rx_len);
                    break;
                
                // Invalid CAN id
                default:
                    break;
            }
        }
        
        // Periodically poll for data on the CAN bus
        // Polls one destination at a time, when the gb_poll flag is set
        if (can_tbe() && (gb_poll == true))
        {
            gb_poll = false;
            can_putd(g_polling_id[i],0,8,TX_PRI,TX_EXT,TX_RTR);
            
            if (i >= (N_CAN_POLLING_ID-1))
            {
                i = 0;
            }
            else
            {
                i++;
            }
        }
    }
}
