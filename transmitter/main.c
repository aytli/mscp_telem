#include "main.h"
#include "can18F4580_mscp.c"

#define SENDING_PERIOD_MS 100
#define POLLING_PERIOD_MS  50

// Creates a list of CAN packet IDs
enum
{
    CAN_ID_TABLE(EXPAND_AS_ID_ENUM)
};

// Creates a list of telemetry packet IDs
enum
{
    TELEM_ID_TABLE(EXPAND_AS_ID_ENUM)
};

// Creates a list of telemetry packet lengths
enum
{
    TELEM_ID_TABLE(EXPAND_AS_LEN_ENUM)
};

// Creates a list of polling IDs
enum
{
    CAN_POLLING_TABLE(EXPAND_AS_ID_ENUM)
};

// Creates an array of polling IDs
static int16 g_polling_id[N_CAN_POLLING_ID] =
{
    POLLING_MOTOR_HS_TEMP_ID,
    POLLING_MOTOR_DSP_TEMP_ID,
    POLLING_MPPT1_ID,
    POLLING_MPPT2_ID,
    POLLING_MPPT3_ID,
    POLLING_MPPT4_ID
};

static int8 g_motor_bus_vi_page[TELEM_MOTOR_BUS_VI_LEN];
static int8 g_motor_velocity_page[TELEM_MOTOR_VELOCITY_LEN];
static int8 g_motor_hs_temp_page[TELEM_MOTOR_HS_TEMP_LEN];
static int8 g_motor_dsp_temp_page[TELEM_MOTOR_DSP_TEMP_LEN];
static int8 g_evdc_drive_page[TELEM_EVDC_DRIVE_LEN];
static int8 g_bps_voltage_page[TELEM_BPS_VOLTAGE_LEN];
static int8 g_bps_temperature_page[TELEM_BPS_TEMPERATURE_LEN];
static int8 g_bps_current_page[TELEM_BPS_CURRENT_LEN];
static int8 g_bps_balancing_page[TELEM_BPS_BALANCING_LEN];
static int8 g_bps_status_page[TELEM_BPS_STATUS_LEN];
static int8 g_pms_page[TELEM_PMS_DATA_LEN];
static int8 g_mppt_page[TELEM_MPPT_LEN];
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
// Telemetry data will be sent out with a period of SENDING_PERIOD_MS
#int_timer2
void isr_timer2(void)
{
    static int16 ms;
    
    static int32 motor_current  = 0x41a00000; // 20.0A
    static int32 motor_voltage  = 0x42c80000; // 100.0V
    g_motor_bus_vi_page[0] = (int8)((motor_current>>24)&0xFF);
    g_motor_bus_vi_page[1] = (int8)((motor_current>>16)&0xFF);
    g_motor_bus_vi_page[2] = (int8)((motor_current>> 8)&0xFF);
    g_motor_bus_vi_page[3] = (int8)((motor_current>> 0)&0xFF);
    g_motor_bus_vi_page[4] = (int8)((motor_voltage>>24)&0xFF);
    g_motor_bus_vi_page[5] = (int8)((motor_voltage>>16)&0xFF);
    g_motor_bus_vi_page[6] = (int8)((motor_voltage>> 8)&0xFF);
    g_motor_bus_vi_page[7] = (int8)((motor_voltage>> 0)&0xFF);
    
    static int32 motor_velocity = 0x41600000; // 14 m/s
    static int32 motor_rpm      = 0x447a0000; // 1000 rpm
    g_motor_velocity_page[0] = (int8)((motor_velocity>>24)&0xFF);
    g_motor_velocity_page[1] = (int8)((motor_velocity>>16)&0xFF);
    g_motor_velocity_page[2] = (int8)((motor_velocity>> 8)&0xFF);
    g_motor_velocity_page[3] = (int8)((motor_velocity>> 0)&0xFF);
    g_motor_velocity_page[4] = (int8)((motor_rpm>>24)&0xFF);
    g_motor_velocity_page[5] = (int8)((motor_rpm>>16)&0xFF);
    g_motor_velocity_page[6] = (int8)((motor_rpm>> 8)&0xFF);
    g_motor_velocity_page[7] = (int8)((motor_rpm>> 0)&0xFF);
    
    static int32 motor_hs_temp  = 0x42480000; // 50C
    static int32 motor_int_temp = 0x42700000; // 60C
    g_motor_hs_temp_page[0] = (int8)((motor_hs_temp>>24)&0xFF);
    g_motor_hs_temp_page[1] = (int8)((motor_hs_temp>>16)&0xFF);
    g_motor_hs_temp_page[2] = (int8)((motor_hs_temp>> 8)&0xFF);
    g_motor_hs_temp_page[3] = (int8)((motor_hs_temp>> 0)&0xFF);
    g_motor_hs_temp_page[4] = (int8)((motor_int_temp>>24)&0xFF);
    g_motor_hs_temp_page[5] = (int8)((motor_int_temp>>16)&0xFF);
    g_motor_hs_temp_page[6] = (int8)((motor_int_temp>> 8)&0xFF);
    g_motor_hs_temp_page[7] = (int8)((motor_int_temp>> 0)&0xFF);
    
    static int32 motor_dsp_temp  = 0x42200000; // 40C
    g_motor_dsp_temp_page[4] = (int8)((motor_dsp_temp>>24)&0xFF);
    g_motor_dsp_temp_page[5] = (int8)((motor_dsp_temp>>16)&0xFF);
    g_motor_dsp_temp_page[6] = (int8)((motor_dsp_temp>> 8)&0xFF);
    g_motor_dsp_temp_page[7] = (int8)((motor_dsp_temp>> 0)&0xFF);
    
    static int32 evdc_current    = 0x42200000; // 40% of max current
    static int32 evdc_velocity   = 0x447a0000; // 1000 rpm
    g_evdc_drive_page[0] = (int8)((evdc_current>>24)&0xFF);
    g_evdc_drive_page[1] = (int8)((evdc_current>>16)&0xFF);
    g_evdc_drive_page[2] = (int8)((evdc_current>> 8)&0xFF);
    g_evdc_drive_page[3] = (int8)((evdc_current>> 0)&0xFF);
    g_evdc_drive_page[4] = (int8)((evdc_velocity>>24)&0xFF);
    g_evdc_drive_page[5] = (int8)((evdc_velocity>>16)&0xFF);
    g_evdc_drive_page[6] = (int8)((evdc_velocity>> 8)&0xFF);
    g_evdc_drive_page[7] = (int8)((evdc_velocity>> 0)&0xFF);
    
    if (ms >= SENDING_PERIOD_MS)
    {
        ms = 0; // Reset timer
        output_toggle(TX_PIN);
        send_data(TELEM_MOTOR_BUS_VI_ID     , TELEM_MOTOR_BUS_VI_LEN     , g_motor_bus_vi_page);
        send_data(TELEM_MOTOR_VELOCITY_ID   , TELEM_MOTOR_VELOCITY_LEN   , g_motor_velocity_page);
        send_data(TELEM_MOTOR_HS_TEMP_ID    , TELEM_MOTOR_HS_TEMP_LEN    , g_motor_hs_temp_page);
        send_data(TELEM_MOTOR_DSP_TEMP_ID   , TELEM_MOTOR_DSP_TEMP_LEN   , g_motor_dsp_temp_page);
        send_data(TELEM_EVDC_DRIVE_ID       , TELEM_EVDC_DRIVE_LEN       , g_evdc_drive_page);
        send_data(TELEM_BPS_VOLTAGE_ID      , TELEM_BPS_VOLTAGE_LEN      , g_bps_voltage_page);
        send_data(TELEM_BPS_TEMPERATURE_ID  , TELEM_BPS_TEMPERATURE_LEN  , g_bps_temperature_page);
        send_data(TELEM_BPS_CURRENT_ID      , TELEM_BPS_CURRENT_LEN      , g_bps_current_page);
        send_data(TELEM_BPS_BALANCING_ID    , TELEM_BPS_BALANCING_LEN    , g_bps_balancing_page);
        send_data(TELEM_BPS_STATUS_ID       , TELEM_BPS_STATUS_LEN       , g_bps_status_page);
        send_data(TELEM_PMS_DATA_ID         , TELEM_PMS_DATA_LEN         , g_pms_page);
        send_data(TELEM_MPPT_ID             , TELEM_MPPT_LEN             , g_mppt_page);
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
        ms = 0;                 // Reset timer
        gb_poll = true;        // Raise polling request flag
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
    
    int tx_pri = 3;
    int1 tx_ext = 0;
    int1 tx_rtr = 1;
    
    setup_timer_2(T2_DIV_BY_4,79,16); // Timer 2 set up to interrupt every 1ms with a 20MHz clock
    setup_timer_4(T4_DIV_BY_4,79,16); // Timer 4 set up to interrupt every 1ms with a 20MHz clock
    enable_interrupts(INT_TIMER2);
    enable_interrupts(INT_TIMER4);
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
                output_toggle(PIN_C0);  // CAN data received
                
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
                        memcpy(&g_mppt_page[8],in_data,rx_len);
                        break;
                    case CAN_MPPT3_ID:              // MPPT 3 data
                        memcpy(&g_mppt_page[16],in_data,rx_len);
                        break;
                    case CAN_MPPT4_ID:              // MPPT 4 data
                        memcpy(&g_mppt_page[24],in_data,rx_len);
                        break;
                    
                    // Invalid CAN id
                    default:
                        break;
                }
            }
        }
        
        // Periodically poll for data on the CAN bus
        // Polls one destination at a time, when the gb_poll flag is set
        if (can_tbe() && (gb_poll == true))
        {
            gb_poll = false;
            can_putd(g_polling_id[i],0,8,tx_pri,tx_ext,tx_rtr);
            
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
