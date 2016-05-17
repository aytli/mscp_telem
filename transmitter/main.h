#include <18F26K80.h>
#device adc=16

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
#use rs232(baud = 9600, xmit = PIN_C6, rcv = PIN_C7)

#define RX_PIN PIN_A0
#define TX_PIN PIN_A1

// NOTE: CAN_ID_TABLE and TELEM_ID_TABLE are x macros
// X macro tutorial: http://www.embedded.com/design/programming-languages-and-tools/4403953/C-language-coding-errors-with-X-macros-Part-1

#define EXPAND_AS_ID_ENUM(a,b,c)  a##_ID  = b,
#define EXPAND_AS_LEN_ENUM(a,b,c) a##_LEN = c,

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
    ENTRY(CAN_BPS_STATUS         , 0x50D,  2)

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
    ENTRY(TELEM_BPS_STATUS       ,  0x17,  2)
