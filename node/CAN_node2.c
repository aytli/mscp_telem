#include <18F26K80.h>

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES SOSC_DIG                 //Digital mode, I/O port functionality of RC0 and RC1
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES HSH                      //High speed Osc, high power 16MHz-25MHz
#FUSES NOPLLEN                  //4X HW PLL disabled, 4X PLL enabled in software
#FUSES BROWNOUT               
#FUSES PUT
#FUSES NOIESO
#FUSES NOFCMEN
#FUSES NOPROTECT

#use delay(clock = 20000000)
#use rs232(baud = 9600, parity = N, UART1, bits = 8)
#define LED PIN_C0
#define RTS PIN_C5

#include <can18F4580_mscp.c>  // Modified CAN library includes default FIFO mode, timing settings match MPPT, 11-bit instead of 24-bit addressing


// NOTE: CAN_ID_TABLE and TELEM_ID_TABLE are x macros
// X macro tutorial: http://www.embedded.com/design/programming-languages-and-tools/4403953/C-language-coding-errors-with-X-macros-Part-1

#define EXPAND_AS_ID_ENUM(a,b,c)   a##_ID  = b,
#define EXPAND_AS_LEN_ENUM(a,b,c)  a##_LEN = c,

// X macro table of CANbus packets
//        Packet name            ,    ID, Length
#define CAN_ID_TABLE(ENTRY)                   \
    ENTRY(CAN_MOTOR_BUS_VI       , 0x402,  8) \
    ENTRY(CAN_MOTOR_VELOCITY     , 0x403,  8) \
    ENTRY(CAN_MOTOR_TEMPERATURE  , 0x40B,  8) \
    ENTRY(CAN_BPS_VOLTAGE1       , 0x500,  8) \
    ENTRY(CAN_BPS_VOLTAGE2       , 0x501,  8) \
    ENTRY(CAN_BPS_VOLTAGE3       , 0x502,  8) \
    ENTRY(CAN_BPS_VOLTAGE4       , 0x503,  8) \
    ENTRY(CAN_BPS_VOLTAGE5       , 0x504,  8) \
    ENTRY(CAN_BPS_VOLTAGE6       , 0x505,  8) \
    ENTRY(CAN_BPS_VOLTAGE7       , 0x506,  8) \
    ENTRY(CAN_BPS_VOLTAGE8       , 0x507,  4) \
    ENTRY(CAN_BPS_TEMPERATURE1   , 0x508,  8) \
    ENTRY(CAN_BPS_TEMPERATURE2   , 0x509,  8) \
    ENTRY(CAN_BPS_TEMPERATURE3   , 0x50A,  8) \
    ENTRY(CAN_BPS_CURRENT        , 0x50B,  2) \
    ENTRY(CAN_BPS_BALANCING      , 0x50C,  4) \
    ENTRY(CAN_BPS_STATUS         , 0x50D,  2) \
    ENTRY(CAN_MPPT1              , 0x701,  8) \
    ENTRY(CAN_MPPT2              , 0x702,  8) \
    ENTRY(CAN_MPPT3              , 0x703,  8) \
    ENTRY(CAN_MPPT4              , 0x704,  8) \

// X macro table of telemetry packets
//        Packet name            ,    ID, Length
#define TELEM_ID_TABLE(ENTRY)                 \
    ENTRY(TELEM_MOTOR_BUS_VI     ,  0x03,  8) \
    ENTRY(TELEM_MOTOR_VELOCITY   ,  0x05,  8) \
    ENTRY(TELEM_MOTOR_TEMPERATURE,  0x07,  8) \
    ENTRY(TELEM_BPS_VOLTAGE      ,  0x0B, 60) \
    ENTRY(TELEM_BPS_TEMPERATURE  ,  0x0D, 24) \
    ENTRY(TELEM_BPS_CURRENT      ,  0x11,  2) \
    ENTRY(TELEM_BPS_BALANCING    ,  0x13,  4) \
    ENTRY(TELEM_BPS_STATUS       ,  0x17,  2) \
    ENTRY(TELEM_MPPT             ,  0x1D, 32)

// Creates a list of CAN packet IDs
enum
{
    CAN_ID_TABLE(EXPAND_AS_ID_ENUM)
};

// Creates a list of CAN packet lengths
enum
{
    CAN_ID_TABLE(EXPAND_AS_LEN_ENUM)
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

static unsigned int16 g_can_id[17] = {
    CAN_MOTOR_BUS_VI_ID,
    CAN_MOTOR_VELOCITY_ID,
    CAN_MOTOR_TEMPERATURE_ID,
    CAN_BPS_VOLTAGE1_ID,
    CAN_BPS_VOLTAGE2_ID,
    CAN_BPS_VOLTAGE3_ID,
    CAN_BPS_VOLTAGE4_ID,
    CAN_BPS_VOLTAGE5_ID,
    CAN_BPS_VOLTAGE6_ID,
    CAN_BPS_VOLTAGE7_ID,
    CAN_BPS_VOLTAGE8_ID,
    CAN_BPS_TEMPERATURE1_ID,
    CAN_BPS_TEMPERATURE2_ID,
    CAN_BPS_TEMPERATURE3_ID,
    CAN_BPS_CURRENT_ID,
    CAN_BPS_BALANCING_ID,
    CAN_BPS_STATUS_ID
};

static int8 g_motor_bus_vi_page[TELEM_MOTOR_BUS_VI_LEN]           = {0};
static int8 g_motor_velocity_page[TELEM_MOTOR_VELOCITY_LEN]       = {0};
static int8 g_motor_temperature_page[TELEM_MOTOR_TEMPERATURE_LEN] = {0};
static int8 g_bps_voltage_page[TELEM_BPS_VOLTAGE_LEN]             = {0};
static int8 g_bps_temperature_page[TELEM_BPS_TEMPERATURE_LEN]     = {0};
static int8 g_bps_current_page[TELEM_BPS_CURRENT_LEN]             = {0};
static int8 g_bps_balancing_page[TELEM_BPS_BALANCING_LEN]         = {0};
static int8 g_bps_status_page[TELEM_BPS_STATUS_LEN]               = {0};
static int8 g_mppt_page[TELEM_MPPT_LEN]                           = {0};

int16 ms;
#int_timer2
void isr_timer2(void)
{
    ms++; //keep a running timer interupt that increments every milli-second
}

void main()
{
    int8 i = 0;
    int1 tx_rtr = 0;
    int1 tx_ext = 0;
    int tx_pri = 3;
    
    setup_timer_2(T2_DIV_BY_4,79,16);   //setup up timer2 to interrupt every 1ms if using 20Mhz clock
    enable_interrupts(INT_TIMER2);   //enable timer2 interrupt (if want to count ms)
    enable_interrupts(GLOBAL);       //enable all interrupts
    
    can_init();
    set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in (default)
    
    int16 id;
    
    while(true)
    {
        if (can_tbe() && (ms > 200))
        {
            ms = 0; // resets the timer interupt
            output_toggle(LED);
            
            switch(i)
            {
                case 0:       // Motor voltage and current
                    can_putd(CAN_MOTOR_BUS_VI_ID,
                             g_motor_bus_vi_page,
                             CAN_MOTOR_BUS_VI_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_motor_bus_vi_page[0]++;
                    break;
                case 1:     // Motor velocity
                    can_putd(CAN_MOTOR_VELOCITY_ID,
                             g_motor_velocity_page,
                             CAN_MOTOR_VELOCITY_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_motor_velocity_page[1]++;
                    break;
                case 2:  // Motor temperature
                    can_putd(CAN_MOTOR_TEMPERATURE_ID,
                             g_motor_temperature_page,
                             CAN_MOTOR_TEMPERATURE_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_motor_temperature_page[2]++;
                    break;
                case 3:       // BPS voltage 1
                    can_putd(CAN_BPS_VOLTAGE1_ID,
                             g_bps_voltage_page,
                             CAN_BPS_VOLTAGE1_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[1]++;
                    break;
                case 4:       // BPS voltage 2
                    can_putd(CAN_BPS_VOLTAGE2_ID,
                             g_bps_voltage_page+8,
                             CAN_BPS_VOLTAGE2_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[9]++;
                    break;
                case 5:       // BPS voltage 3
                    can_putd(CAN_BPS_VOLTAGE3_ID,
                             g_bps_voltage_page+16,
                             CAN_BPS_VOLTAGE3_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[17]++;
                    break;
                case 6:       // BPS voltage 4
                    can_putd(CAN_BPS_VOLTAGE4_ID,
                             g_bps_voltage_page+24,
                             CAN_BPS_VOLTAGE4_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[25]++;
                    break;
                case 7:       // BPS voltage 5
                    can_putd(CAN_BPS_VOLTAGE5_ID,
                             g_bps_voltage_page+32,
                             CAN_BPS_VOLTAGE5_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[33]++;
                    break;
                case 8:       // BPS voltage 6
                    can_putd(CAN_BPS_VOLTAGE6_ID,
                             g_bps_voltage_page+40,
                             CAN_BPS_VOLTAGE6_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[41]++;
                    break;
                case 9:       // BPS voltage 7
                    can_putd(CAN_BPS_VOLTAGE7_ID,
                             g_bps_voltage_page+48,
                             CAN_BPS_VOLTAGE7_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[49]++;
                    break;
                case 10:       // BPS voltage 8
                    can_putd(CAN_BPS_VOLTAGE8_ID,
                             g_bps_voltage_page+56,
                             CAN_BPS_VOLTAGE8_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_voltage_page[57]++;
                    break;
                case 11:   // BPS temperature 1
                    can_putd(CAN_BPS_TEMPERATURE1_ID,
                             g_bps_temperature_page,
                             CAN_BPS_TEMPERATURE1_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_temperature_page[1]++;
                    break;
                case 12:   // BPS temperature 2
                    can_putd(CAN_BPS_TEMPERATURE2_ID,
                             g_bps_temperature_page+8,
                             CAN_BPS_TEMPERATURE2_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_temperature_page[9]++;
                    break;
                case 13:   // BPS temperature 3
                    can_putd(CAN_BPS_TEMPERATURE3_ID,
                             g_bps_temperature_page+16,
                             CAN_BPS_TEMPERATURE3_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_temperature_page[17]++;
                    break;
                case 14:        // BPS current
                    can_putd(CAN_BPS_CURRENT_ID,
                             g_bps_current_page,
                             CAN_BPS_CURRENT_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_current_page[0]++;
                    break;
                case 15:      // BPS balancing bits
                    can_putd(CAN_BPS_BALANCING_ID,
                             g_bps_balancing_page,
                             CAN_BPS_BALANCING_LEN,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_balancing_page[0]++;
                    break;
                case 16:         // BPS status
                    can_putd(CAN_BPS_STATUS_ID,
                             g_bps_status_page,
                             CAN_BPS_STATUS_len,
                             tx_pri,tx_ext,tx_rtr);
                    g_bps_status_page[0]++;
                    break;
                case 17:         // MPPT data
                    can_putd(CAN_MPPT1_ID,
                             g_mppt_page,
                             CAN_MPPT1_len,
                             tx_pri,tx_ext,tx_rtr);
                    g_mppt_page[1]++;
                    break;
                case 18:         // MPPT data
                    can_putd(CAN_MPPT2_ID,
                             g_mppt_page+8,
                             CAN_MPPT2_len,
                             tx_pri,tx_ext,tx_rtr);
                    g_mppt_page[9]++;
                    break;
                case 19:         // MPPT data
                    can_putd(CAN_MPPT3_ID,
                             g_mppt_page+16,
                             CAN_MPPT3_len,
                             tx_pri,tx_ext,tx_rtr);
                    g_mppt_page[17]++;
                    break;
                case 20:         // MPPT data
                    can_putd(CAN_MPPT4_ID,
                             g_mppt_page+24,
                             CAN_MPPT4_len,
                             tx_pri,tx_ext,tx_rtr);
                    g_mppt_page[25]++;
                    break;
                default:                        // Invalid CAN id
                    break;
            }
            
            if (i >= 20)
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

