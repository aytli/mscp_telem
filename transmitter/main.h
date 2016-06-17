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
#use rs232(baud = 115200, xmit = PIN_C6, rcv = PIN_C7)

#define RX_PIN PIN_C2
#define TX_PIN PIN_C3

// State machine states
typedef enum
{
    IDLE,
    DATA_RECEIVED,
    DATA_POLLING,
    N_STATES
} telem_state_t;
