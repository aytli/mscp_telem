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

#define EXPAND_AS_CAN_ID(a,b)     a##_ID  = b,
#define EXPAND_AS_TELEM_ID(a,b,c) a##_ID  = b,
#define EXPAND_AS_LENGTH(a,b,c)   a##_LEN = c,

// X macro table of CANbus packets
//        Packet name          ,    ID
#define CAN_ID_TABLE(ENTRY)             \
    ENTRY(CAN_MOTOR_BUS_VI     , 0x402) \
    ENTRY(CAN_MOTOR_VELOCITY   , 0x403) \
    ENTRY(CAN_MOTOR_TEMPERATURE, 0x40B) \
    ENTRY(CAN_BPS_VOLTAGE      , 0x500) \
    ENTRY(CAN_BPS_TEMPERATURE  , 0x501) \
    ENTRY(CAN_BPS_CURRENT      , 0x502) \
    ENTRY(CAN_BPS_BALANCING    , 0x503) \
    ENTRY(CAN_BPS_STATUS       , 0x504)

// X macro table of telemetry packets
//        Packet name          ,    ID, Length
#define TELEM_ID_TABLE(ENTRY)               \
    ENTRY(TELEM_MOTOR          ,  0x12,  8) \
    ENTRY(TELEM_BPS_VOLTAGE    ,  0x36, 60) \
    ENTRY(TELEM_BPS_TEMPERATURE,  0x45, 24) \
    ENTRY(TELEM_BPS_CURRENT    ,  0x55,  2) \
    ENTRY(TELEM_BPS_BALANCING  ,  0x69,  4) \
    ENTRY(TELEM_BPS_STATUS     ,  0x53,  2)
