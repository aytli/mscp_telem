#ifndef CAN_TELEM_H
#define CAN_TELEM_H

// NOTE: The following tables are x-macros
// X macro tutorial: http://www.embedded.com/design/programming-languages-and-tools/4403953/C-language-coding-errors-with-X-macros-Part-1

//////////////////////////
// CAN BUS DEFINES ///////
//////////////////////////

#define EXPAND_AS_CAN_ID_ENUM(a,b,c)  a##_ID  = b,
#define EXPAND_AS_CAN_LEN_ENUM(a,b,c) a##_LEN = c,
#define EXPAND_AS_CAN_ID_ARRAY(a,b,c)           b,

// X macro table of CANbus packets
//        Packet name            ,    ID, Length
#define CAN_ID_TABLE(ENTRY)                   \
    ENTRY(CAN_MOTOR_BUS_VI       , 0x402,  8) \
    ENTRY(CAN_MOTOR_VELOCITY     , 0x403,  8) \
    ENTRY(CAN_MOTOR_HS_TEMP      , 0x40B,  8) \
    ENTRY(CAN_MOTOR_DSP_TEMP     , 0x40C,  8) \
    ENTRY(CAN_EVDC_DRIVE         , 0x501,  8) \
    ENTRY(CAN_BPS_VOLTAGE1       , 0x600,  8) \
    ENTRY(CAN_BPS_VOLTAGE2       , 0x601,  8) \
    ENTRY(CAN_BPS_VOLTAGE3       , 0x602,  8) \
    ENTRY(CAN_BPS_VOLTAGE4       , 0x603,  8) \
    ENTRY(CAN_BPS_VOLTAGE5       , 0x604,  8) \
    ENTRY(CAN_BPS_VOLTAGE6       , 0x605,  8) \
    ENTRY(CAN_BPS_VOLTAGE7       , 0x606,  8) \
    ENTRY(CAN_BPS_VOLTAGE8       , 0x607,  4) \
    ENTRY(CAN_BPS_TEMPERATURE1   , 0x608,  8) \
    ENTRY(CAN_BPS_TEMPERATURE2   , 0x609,  8) \
    ENTRY(CAN_BPS_TEMPERATURE3   , 0x60A,  8) \
    ENTRY(CAN_BPS_CUR_BAL_STAT   , 0x60B,  7) \
    ENTRY(CAN_PMS_DATA           , 0x60E,  7) \
    ENTRY(CAN_MPPT1              , 0x771,  7) \
    ENTRY(CAN_MPPT2              , 0x772,  7) \
    ENTRY(CAN_MPPT3              , 0x773,  7) \
    ENTRY(CAN_MPPT4              , 0x774,  7)
#define N_CAN_ID 21

enum {CAN_ID_TABLE(EXPAND_AS_CAN_ID_ENUM)};
enum {CAN_ID_TABLE(EXPAND_AS_CAN_LEN_ENUM)};


//////////////////////////
// TELEMETRY DEFINES /////
//////////////////////////

#define EXPAND_AS_TELEM_ID_ENUM(a,b,c,d)  a##_ID  = b,
#define EXPAND_AS_TELEM_LEN_ENUM(a,b,c,d) a##_LEN = c,
#define EXPAND_AS_TELEM_ID_ARRAY(a,b,c,d)           b,
#define EXPAND_AS_TELEM_LEN_ARRAY(a,b,c,d)          c,
#define EXPAND_AS_TELEM_PAGE_ARRAY(a,b,c,d)         d,
#define EXPAND_AS_TELEM_PAGE_DECLARATIONS(a,b,c,d) static int8 d[c];

// X macro table of telemetry packets
//        Packet name            ,    ID, Length, Page array
#define TELEM_ID_TABLE(ENTRY)                                          \
    ENTRY(TELEM_MOTOR_BUS_VI     ,  0x03,  8, g_motor_bus_vi_page)     \
    ENTRY(TELEM_MOTOR_VELOCITY   ,  0x05,  8, g_motor_velocity_page)   \
    ENTRY(TELEM_MOTOR_HS_TEMP    ,  0x07,  8, g_motor_hs_temp_page)    \
    ENTRY(TELEM_MOTOR_DSP_TEMP   ,  0x09,  8, g_motor_dsp_temp_page)   \
    ENTRY(TELEM_EVDC_DRIVE       ,  0x0A,  8, g_evdc_drive_page)       \
    ENTRY(TELEM_BPS_VOLTAGE      ,  0x0B, 60, g_bps_voltage_page)      \
    ENTRY(TELEM_BPS_TEMPERATURE  ,  0x0D, 24, g_bps_temperature_page)  \
    ENTRY(TELEM_BPS_CUR_BAL_STAT ,  0x11,  7, g_bps_cur_bal_stat_page) \
    ENTRY(TELEM_PMS_DATA         ,  0x19,  7, g_pms_page)              \
    ENTRY(TELEM_MPPT             ,  0x1D, 28, g_mppt_page)
#define N_TELEM_ID 10

enum {TELEM_ID_TABLE(EXPAND_AS_TELEM_ID_ENUM)};
enum {TELEM_ID_TABLE(EXPAND_AS_TELEM_LEN_ENUM)};


//////////////////////////
// POLLING DEFINES ///////
//////////////////////////

#define EXPAND_AS_POLLING_ID_ENUM(a,b)  a##_ID  = b,
#define EXPAND_AS_POLLING_ID_ARRAY(a,b)           b,

// X macro table of CAN bus destinations to be polled
//        Packet name            ,    ID
#define CAN_POLLING_TABLE(ENTRY)          \
    ENTRY(POLLING_MOTOR_HS_TEMP  , 0x40B) \
    ENTRY(POLLING_MOTOR_DSP_TEMP , 0x40C) \
    ENTRY(POLLING_MPPT1          , 0x711) \
    ENTRY(POLLING_MPPT2          , 0x712) \
    ENTRY(POLLING_MPPT3          , 0x713) \
    ENTRY(POLLING_MPPT4          , 0x714)
#define N_CAN_POLLING_ID 6

enum {CAN_POLLING_TABLE(EXPAND_AS_POLLING_ID_ENUM)};

#endif
