// Spitfire telemetry code
// Author: Andy Li
// Copyright 2016, McMaster Solar Car Project
// Reads vehicle information over CAN bus and periodically sends it over a radio
// module to a receiving PC

// Includes
#include "main.h"
#include "ieeefloat.c"
#include "can_telem.h"
#include "can18F4580_mscp.c"

// Timing periods
#define SENDING_PERIOD_MS  50
#define POLLING_PERIOD_MS  200

// CAN bus defines
#define TX_PRI 3
#define TX_EXT 0
#define TX_RTR 1

// Sends a packet of telemetry data to the radio module over uart
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

// Declares and creates an array of telemetry pages
TELEM_ID_TABLE(EXPAND_AS_TELEM_PAGE_DECLARATIONS)
static int * gp_telem_page[N_TELEM_ID] =
{
    TELEM_ID_TABLE(EXPAND_AS_TELEM_PAGE_ARRAY)
};

static int1          gb_send;
static int1          gb_poll;
static int32         g_can0_id;
static int8          g_can0_data[8];
static int8          g_can0_len;
static int1          gb_can0_hit = false;
static int32         g_can1_id;
static int8          g_can1_data[8];
static int8          g_can1_len;
static int1          gb_can1_hit = false;
static int32         g_rx_id;
static int8          g_rx_len;
static int8          g_rx_data[8];
static telem_state_t g_state;

// Puts the xbee into bypass mode, toggles Xbee reset pins
// Documentation: http://xbee-sdk-doc.readthedocs.io/en/stable/doc/tips_tricks/
void xbee_init(void)
{
    delay_ms(10);
    output_low(XBEE_PIN);
    delay_ms(1);
    output_high(XBEE_PIN);
    delay_ms(1);
    putc('b');
    putc('b');
    delay_ms(10);
}

// Sends a page of data over the radio module
void send_data(int8 id, int len, int * data)
{
    int i;
    putc(id);
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
    if (ms >= SENDING_PERIOD_MS)
    {
        ms = 0;         // Reset timer
        gb_send = true; // Raise data sending flag
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

// CAN receive buffer 0 interrupt
#int_canrx0
void isr_canrx0()
{
    struct rx_stat rxstat;
    output_toggle(RX_PIN);
    
    if (can_getd(g_can0_id, g_can0_data, g_can0_len, rxstat))
    {
        gb_can0_hit = true;
    }
    else
    {
        gb_can0_hit = false;
    }
}

// CAN receive buffer 1 interrupt
#int_canrx1
void isr_canrx1()
{
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

void idle_state(void)
{
    if (gb_can0_hit == true)
    {
        // Data received in buffer 0, transfer contents
        g_rx_id = g_can0_id;
        g_rx_len = g_can0_len;
        memcpy(g_rx_data,g_can0_data,8);
        gb_can0_hit = false;
        g_state = DATA_RECEIVED;
    }
    else if (gb_can1_hit == true)
    {
        // Data received in buffer 1, transfer contents
        g_rx_id = g_can1_id;
        g_rx_len = g_can1_len;
        memcpy(g_rx_data,g_can1_data,8);
        gb_can1_hit = false;
        g_state = DATA_RECEIVED;
    }
    else if (gb_send == true)
    {
        // Ready to send data
        g_state = DATA_SENDING;
    }
    else if (can_tbe() && (gb_poll == true))
    {
        // Ready to poll
        g_state = DATA_POLLING;
    }
    else
    {
        // Nothing, remain idle
        g_state = IDLE;
    }
}

void data_received_state(void)
{
    // Check the ID of the received packet and update the corresponding page
    switch(g_rx_id)
    {
        // MOTOR DATA
        case CAN_MOTOR_STATUS_ID:       // Motor status bits
            memcpy(&g_motor_status_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_MOTOR_BUS_VI_ID:       // Motor voltage and current
            memcpy(&g_motor_bus_vi_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_MOTOR_VELOCITY_ID:     // Motor velocity
            memcpy(&g_motor_velocity_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_MOTOR_HS_TEMP_ID:  // Motor heatsink temperature
            memcpy(&g_motor_hs_temp_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_MOTOR_DSP_TEMP_ID:  // Motor DSP temperature
            memcpy(&g_motor_dsp_temp_page[0],g_rx_data,g_rx_len);
            break;
        
        // EV DRIVER CONTROLS DATA
        case CAN_EVDC_DRIVE_ID:
            memcpy(&g_evdc_drive_page[0],g_rx_data,g_rx_len);
            break;
        
        // BPS DATA
        case CAN_BPS_VOLTAGE1_ID:       // BPS voltage 1
            memcpy(&g_bps_voltage_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_BPS_VOLTAGE2_ID:       // BPS voltage 2
            memcpy(&g_bps_voltage_page[8],g_rx_data,g_rx_len);
           break;
        case CAN_BPS_VOLTAGE3_ID:       // BPS voltage 3
            memcpy(&g_bps_voltage_page[16],g_rx_data,g_rx_len);
            break;
        case CAN_BPS_VOLTAGE4_ID:       // BPS voltage 4
            memcpy(&g_bps_voltage_page[24],g_rx_data,g_rx_len);
            break;
        case CAN_BPS_TEMPERATURE1_ID:   // BPS temperature 1
            memcpy(&g_bps_temperature_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_BPS_TEMPERATURE2_ID:   // BPS temperature 2
            memcpy(&g_bps_temperature_page[8],g_rx_data,g_rx_len);
            break;
        case CAN_BPS_TEMPERATURE3_ID:   // BPS temperature 3
            memcpy(&g_bps_temperature_page[16],g_rx_data,g_rx_len);
            break;
        case CAN_BPS_CUR_BAL_STAT_ID:   // BPS current, balancing bits, status
            memcpy(&g_bps_cur_bal_stat_page[0],g_rx_data,g_rx_len);
            break;
        
        // PMS DATA
        case CAN_PMS_DATA_ID:           // PMS data (aux voltage/temperature, DC/DC temperature)
            memcpy(&g_pms_page[0],g_rx_data,g_rx_len);
            break;
        
        // MPPT DATA
        case CAN_MPPT1_ID:              // MPPT 1 data
            memcpy(&g_mppt_page[0],g_rx_data,g_rx_len);
            break;
        case CAN_MPPT2_ID:              // MPPT 2 data
            memcpy(&g_mppt_page[7],g_rx_data,g_rx_len);
            break;
        case CAN_MPPT3_ID:              // MPPT 3 data
            memcpy(&g_mppt_page[14],g_rx_data,g_rx_len);
            break;
        case CAN_MPPT4_ID:              // MPPT 4 data
            memcpy(&g_mppt_page[21],g_rx_data,g_rx_len);
            break;
        
        // Invalid CAN id
        default:
            break;
    }
    
    // Data received, return to idle
    g_state = IDLE;
}

void data_sending_state(void)
{
    static int i = 0;
    
    gb_send = false;
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
    
    g_state = IDLE;
}

void data_polling_state(void)
{
    static int i = 0;
    unsigned int8 motor_speed_current_data[TELEM_MOTOR_SPEED_CURRENT_LEN];
    int8  bytes;
    int32 raw_speed;
    int32 raw_current;
    
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
    
    // Driver display uses a PIC24, cannot figure out how to convert floating
    // point numbers in IEEE 754 format, telemetry will do the conversion and
    // send it to the driver display over CAN bus
    for (bytes = 0 ; bytes < 4 ; bytes++)
    {
        raw_speed   += (int32) (g_motor_velocity_page[i+4] << (8*i)); // Upper 4 bytes of motor speed packet
        raw_current += (int32) (g_motor_bus_vi_page[i+4]   << (8*i)); // Upper 4 bytes of motor vi packet
    }
    motor_speed_current_data[0] = (unsigned int8) (f_IEEEtoPIC((float)(raw_speed*3.6))); // Motor speed in km/h
    motor_speed_current_data[1] = (unsigned int8) (f_IEEEtoPIC((float)(raw_current)));   // Motor current
    
    can_putd(TELEM_MOTOR_SPEED_CURRENT_ID,
             &motor_speed_current_data[0],
             TELEM_MOTOR_SPEED_CURRENT_LEN,
             3,0,0);
    
    // Polling data sent, return to idle
    g_state = IDLE;
}

void main()
{
    // Enable CAN receive interrupts
    clear_interrupt(INT_CANRX0);
    enable_interrupts(INT_CANRX0);
    clear_interrupt(INT_CANRX1);
    enable_interrupts(INT_CANRX1);
    
    // Setup timer interrupts
    setup_timer_2(T2_DIV_BY_4,79,16); // Timer 2 set up to interrupt every 1ms with a 20MHz clock
    setup_timer_4(T4_DIV_BY_4,79,16); // Timer 4 set up to interrupt every 1ms with a 20MHz clock
    enable_interrupts(INT_TIMER2);
    enable_interrupts(INT_TIMER4);
    enable_interrupts(GLOBAL);
    
    xbee_init();
    can_init();
    
    // Setup CAN gpio pins
    set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in (default)
    delay_us(200);
    
    // Start in idle state
    g_state = IDLE;
    
    while(true)
    {
        switch(g_state)
        {
            case IDLE:
                idle_state();
                break;
            case DATA_RECEIVED:
                data_received_state();
                break;
            case DATA_SENDING:
                data_sending_state();
                break;
            case DATA_POLLING:
                data_polling_state();
                break;
            default:
                break;
        }
    }
}
