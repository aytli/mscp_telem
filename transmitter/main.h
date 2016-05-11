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

#define LED_PIN PIN_A1

#define EXPAND_AS_ENUMERATION(a,b,c) a = b,

// Table of CANbus IDs for the motor controller and bps
//        Packet name      ,    ID, Length
#define CAN_ID_TABLE(ENTRY)             \
    ENTRY(MOTOR_BUS_VI     , 0x402,  8) \
    ENTRY(MOTOR_VELOCITY   , 0x403,  8) \
    ENTRY(MOTOR_TEMPERATURE, 0x40B,  8) \
    ENTRY(BPS_VOLTAGE      , 0x500, 60) \
    ENTRY(BPS_TEMPERATURE  , 0x501, 24) \
    ENTRY(BPS_CURRENT      , 0x502,  2) \
    ENTRY(BPS_BALANCING    , 0x503,  4) \
    ENTRY(BPS_STATUS       , 0x504,  2)
