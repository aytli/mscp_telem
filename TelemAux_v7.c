/****************************************************************************
 * TelemAux_v6.c
 * -------------
 *
 * Telemetry for Spitfire. Reads data on CAN bus from BPS, WaveSculptor22,
 * drivetek MPPTs. Takes analog measurements (aux pack voltage, array current)
 * Outputs aux pack voltages onto CAN for driver display. Data is transmitted
 * on radio port (timer interrupt based).
 *
 * Software developers: James Strack
 *
 * McMaster Solar Car Project, 2014
 *
 * COMPILER: CCS PCD 4.120
 *
 * TESTED WITH:
 * ------------
 * PIC18F26K80
 *
 * CURRENTLY IMPLEMENTED:
 * ----------------------
 * - Work in progress!
 * - Tested on bench with BPS, WS22, drive control from PC, and one MPPT 
 *
 * CODED BUT NOT TESTED:
 * ---------------------
 *
 * FUTURE WORK / BUGS:
 * -------------------
 * - Add the broadcast of aux voltages and array current to driver display over
 *   CAN bus (DONE in revision 6 - JMS)
 * - The input impedance on the analog pins for the AUX pack (AN0-AN3) is high
 *   due to the use of a voltage divider network. This requires long acq times
 *   to get accurate voltage readings. In a future hardware revision this 
 *   should be addressed (JMS)
 * - It seems like only one of two CAN interrupts is triggered; investigate (JMS)
 *
 * REVISION LOG
 * ------------
 * - June 8, 2014: revision 1, testing with BPS only (JMS)
 *
 * - June 9, 2014: rev 2, add routines for drivetek MPPT communication (JMS)
 *
 * - June 11, 2014: WS22 broadcasts packets in quick succession and this
 *   causes CAN buffer overflow. Need to limit data parsing, etc to times when
 *   there is no CAN traffic or the buffer fills up too quickly. As a result, 
 *   the CAN receive routine will be rewritten to be interrupt driven. The 
 *   radio broadcast will be performed in the main loop. This is the opposite
 *   of the original approach. This will be implemented in revision 4 (JMS).
 *   Also decided to disable WS22 broadcasting (except for velocity). All
 *   other data will be explicitly requested by telemetry for better control
 *   over data flow.
 *
 * - June 12, 2014: Revision 4. Moved CAN receive to interrupt. Fixed math
 *   error in AuxMeasure() (JMS)
 *
 * - June 13, 2014: Parsing of WS22 data added. Note that data from the motor
 *   controller is in IEEE float format so the use of the f_IEEEtoPIC() CCS
 *   library function is required (see TelemCANparse() below). (JMS)
 *
 * - June 14, 2014: Revision 5. Change output formatting of BPS module
 *   voltages, now fprintf outputs to ASCII formatted in hex rather than
 *   decimal. Increased baud rate of radio interface to 38400 (RF baud of
 *   9XCite modules / fastest that can be used without flow control) 
 * 
 ****************************************************************************/

#include <18F26K80.h>
#device adc=12

#FUSES WDT 
#FUSES WDT32            //Watch Dog Timer uses 1:32 Postscale
#FUSES HSH              // High speed Osc, high power 16MHz-25MHz
#FUSES NOPLLEN          // 4X HW PLL disabled, 4x PLL enabled in software

//#FUSES SOSC_DIG        // Digital mode, I/O port functionality of RC0 and RC1
#FUSES NOXINST          // Extended set extension and Indexed Addressing mode disabled(Legacy mode)
#FUSES BROWNOUT         // Reset when brownout detected
#FUSES PUT              // Power Up Timer
#FUSES NOIESO           // Internal External Switch Over mode disabled
#FUSES FCMEN            // Fail-safe clock monitor enabled
#FUSES NOPROTECT        // Code not protected from reading

#use delay(clock=20000000,restart_wdt)
#use rs232(baud=38400,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,stream=RADIO,restart_wdt)
#use i2c(Master,Fast,sda=PIN_C4,scl=PIN_C3,restart_wdt,force_hw)

// System constants
#define LED         pin_C2
#define ADC_HALL    10      // Analog port for array current sensor
#define UP_INTERVAL 50      // Update interval for polled CAN measurements (ms). Set to be longer than time taken for radio TX
#define TX_INTERVAL 250     // Update interval for radio TX; must be larger than UP_INTERVAL
#define canTimeout  8       // Interval to reset data arrays after not hearing from a system over CAN (multiples of TX_INTERVAL)
#define AuxCount    4       // Number of cells in aux pack
#define AuxWarn     2785    // Low voltage warning for aux pack cell in 12 bit units (3.4V)
#define AuxErr      2539    // Low voltage error for aux pack cell in 12 bit units (3.1V)
#define BattCount   26      // Number of modules in main pack
#define MPPTCount   4       // Number of MPPTs/subarrays
#define N_CAN_poll  6       // Total number of CAN packets to be requested from WS22 and MPPTs 

// Print to Radio Codes
#define btemCode    0       // Battery temperatures 
#define bvolCode    1       // Battery voltages
#define miscCode    2       // Analog measurements and motor controller info
#define mpptCode    3       // Data from MPPTs        

// CAN bus IDs
#define wsID         0x400    // WaveSculptor (motor controller) ID
#define wsStatus     wsID+1   // WaveSculptor status bits
#define wsBus        wsID+2   // WaveSculptor bus voltage and current
#define wsVeloc      wsID+3   // WaveSculptor motor speed
#define wsHStemp     wsID+11  // WaveSculptor heat-sink and motor temperature
#define wsDSPtemp    wsID+12  // WaveSculptor DSP board temperature

#define evID         0x500    // EV driver controls ID (TBA)
#define evDrive      evID+1   // Desired motor current and velocity
#define evPower      evID+2

#define bpsID        0x600    // Battery protection system ID

#define mpptID       0x700    // MPPT masked ID (ignoring second LSB)
#define mpptReq      0x710    // MPPT request base ID
#define mpptRes      0x770    // MPPT response base

#define telemID      0x7A0    // ID for telem broadcast of aux voltages

// NOTE!! CAN IDs 0x7F0-0x7FF are reserved for bootloader updates by the WS22 
// DO NOT USE CAN IDs in this range!

// GLOBAL VARIABLES

// Battery pack info (do we want more info from the BPS?)
unsigned int8 battVolt[BattCount]={0};
int8 battTemp[BattCount]={0};
unsigned long battCurr=0;

// Aux pack info
unsigned long AuxVoltRaw[AuxCount]={0}; // Raw readouts from voltage divider network
unsigned long AuxCell[AuxCount]={0}; // Aux cell batt voltages in ADC units
int8 AuxFlags=0;

// MPPT info
unsigned int8 mpptTemp[MPPTCount]={0};
unsigned int8 mpptStat[MPPTCount]={0};
unsigned long mpptVoltIn[MPPTCount]={0};
unsigned long mpptCurrIn[MPPTCount]={0};
unsigned long mpptVoltOut[MPPTCount]={0};

// Motor/controller/drive info; controller data is in floating point format
float motorBusV=0;
float motorBusI=0;
float rpm=0;
float contHStemp=0;
float contDSPtemp=0;
float driveCurr=0;              // Desired motor current (%) from driver controls
float driveVel=0;               // Desired motor speed (RPM) from driver controls

// Other variables
unsigned long arrayCurr=0;      // ADC reading for array current
unsigned long ms = 0;           // Clock tick for timer2 (ms)
unsigned int  Gout_id = 0;       // Message identifier (print to radio code)
unsigned int  TX_next=0;        // Interrupt flags
unsigned int  can_poll=0;
unsigned int  new_can0_data=0;
unsigned int  new_can1_data=0;
unsigned int  bpsTimer=0;       // Timeout counters
unsigned int  wsTimer=0;
unsigned int  evTimer=0;
unsigned int  mpptTimer[mpptCount]={0};

// CAN variables
int     C_empty[8]   = {0,0,0,0,0,0,0,0};
int     Cin_data0[8] = {0,0,0,0,0,0,0,0};
int     Cin_data1[8] = {0,0,0,0,0,0,0,0};
int32   Crx0_id=0;
int     Crx0_len;
int32   Crx1_id=0;
int     Crx1_len;

// Function prototypes
void system_init(void);
void AuxMeasure(void);
void HallMeasure(void);
void RadioTransmit(int out_id);
void TelemCANparse(int *CANdata, int32 CANid);
void TelemCANtx(void);
// void TelemCANpoll(int32 CANid);

// Modified CAN library includes default FIFO mode, timing settings match MPPT,
// and 11-bit instead of 24-bit addressing
#include "Telem_can-18F4580_mscp.c" 

// Library to convert IEEE float data from WS22 to PIC format
#include <ieeefloat.c>

// Interrupt service routines
#int_canrx0
void isr_canrx0() { // CAN receive buffer 0 interrupt
    struct rx_stat rxstat;
    
    output_toggle(LED);
    if(can_getd(Crx0_id, Cin_data0, Crx0_len, rxstat)){
        new_can0_data=1;
    } else {
        new_can0_data=0;
    }
}

#int_canrx1
void isr_canrx1() { // CAN receive buffer 1 interrupt
    struct rx_stat rxstat;

    output_toggle(LED);
    if(can_getd(Crx1_id, Cin_data1, Crx1_len, rxstat)){
        new_can1_data=1;
    } else {
        new_can1_data=0;
    }
}

#int_timer2
void isr_timer2(void) {
    ms++;
    if(ms == UP_INTERVAL){
        CAN_poll=1;
    }
    if (ms == TX_INTERVAL){
        TX_next=1;
        ms=0;
    }
} 

void main() {

    int poll_count=0;
    int32 IDpoll[N_CAN_poll];
    
    // These are CAN IDs of the packets that are requested on a rotating basis
    //IDpoll[0]=wsStatus;
    //IDpoll[1]=wsBus;
    //IDpoll[2]=wsVeloc;
    IDpoll[0]=wsHStemp;
    IDpoll[1]=wsDSPtemp;
    IDpoll[2]=mpptReq+1;
    IDpoll[3]=mpptReq+2;
    IDpoll[4]=mpptReq+3;
    IDpoll[5]=mpptReq+4;
    
    system_init();
    
    while(true){
        delay_us(1);   // Do we need this delay for stability?
        
        if(new_can1_data){  //CAN1 seems to interrupt more often than CAN0 so check it first
            TelemCANparse(Cin_data1, Crx1_id);
            new_can1_data=0;
        } else if (new_can0_data){
            TelemCANparse(Cin_data0, Crx0_id);
            new_can0_data=0;
        }
        if(CAN_poll){
            // Request a CAN data packet, on a rotating basis            
            // TelemCANPoll(IDpoll[poll_count++]);
            can_putd(IDpoll[poll_count++],C_empty,8,3,0,1);
            if(poll_count > (N_CAN_poll-1))poll_count=0;
            CAN_poll=0;
            RadioTransmit(miscCode); // Send priority info quickly
        }
        if(TX_next){
            RadioTransmit(Gout_id);
         Gout_id++;
         if(Gout_id>mpptCode)Gout_id=0;   
            TX_next=0;
        }
    }
}

void system_init(){
   setup_adc(ADC_CLOCK_DIV_64|ADC_TAD_MUL_20); // Input impedance on ADC pins is high from voltage divider network. Need long A/D time
   setup_adc_ports(ALL_ANALOG);
   setup_timer_2(T2_DIV_BY_4,79,16);
   setup_comparator(NC_NC_NC_NC);// This device COMP currently not supported by the PICWizard
   
   can_init();
   set_tris_b((*0xF93 & 0xFB) | 0x08);  //b3 is out, b2 is in (default CAN pins)
   
   clear_interrupt(INT_TIMER2);
   enable_interrupts(INT_TIMER2);
   clear_interrupt(INT_CANRX0);
   enable_interrupts(INT_CANRX0);
   clear_interrupt(INT_CANRX1);
   enable_interrupts(INT_CANRX1);
   enable_interrupts(GLOBAL);
   
   output_low(LED); 
}

void AuxMeasure(void) {
    int k=0;
    
    for(k=0;k<AuxCount;k++){
        set_adc_channel(k);
        delay_us(10);
        AuxVoltRaw[k] = read_adc();
    }
    
    // Convert to individual cell voltages (take into account voltage dividers) 
    AuxCell[0] = AuxVoltRaw[3]; // Lowest cell in aux pack
    AuxCell[1] = 2*AuxVoltRaw[2] - AuxVoltRaw[3];
    AuxCell[2] = 3*AuxVoltRaw[1] - 2*AuxVoltRaw[2];
    AuxCell[3] = 4*AuxVoltRaw[0] - 3*AuxVoltRaw[1];
    
    // Set voltage warning flags
    // Bits 0-3 are voltage warning flags for cells 0-3
    // Bits 4-7 are voltage error flags for cells 0-3
    
    for (k=0;k<AuxCount;k++){
        if(AuxCell[k]>AuxWarn){
            bit_clear(AuxFlags,k);
            bit_clear(AuxFlags,k+4);
        } else if (AuxCell[k]>AuxErr){
            bit_set(AuxFlags,k);
        } else {
            bit_set(AuxFlags,k);
            bit_set(AuxFlags,k+4);
        }
    }
}

void HallMeasure(void) {
    set_adc_channel(ADC_HALL);
    delay_us(10);
    arrayCurr = read_adc();
}

void RadioTransmit(int out_id){
    int j=0;
    
    fprintf(RADIO,"#%d",out_id); //Start sequence (#0)
    restart_wdt();
    
    // Output most recent information stored, request data when appropriate
    
    switch(out_id){
    
        case btemCode:     // Battery temperatures
            bpsTimer++;    // Increment timeout counter for BPS
            if (bpsTimer>canTimeout){ // Zero data/arrays if no new data received for a long time
                memset(&battTemp[0],0,BattCount);
            }
            for(j=0; j<BattCount; j++){
                fprintf(RADIO,",%d",battTemp[j]);
                restart_wdt();
            }
            break;
           
        case bvolCode:     // Battery voltages
            if (bpsTimer>canTimeout){ // Zero data/arrays if no new data received for a long time
                memset(&battVolt[0],0,BattCount);
                bpsTimer=0;
            }
            for(j=0; j<BattCount; j++){
                fprintf(RADIO,",%0X",battVolt[j]);
                restart_wdt();
            }
            break;
            
        case miscCode:
            wsTimer++;  // Increment timeout counters for motor controller
            evTimer++;
            
            if (wsTimer>canTimeout){ // Zero data/arrays if no new data received for a long time
                wsTimer=0;
                motorBusV=0;
                motorBusI=0;
                rpm=0;
                contHStemp=0;
                contDSPtemp=0;
            }
            
            if (evTimer>canTimeout){
                evTimer=0;
                driveCurr=0;
                driveVel=0;
            }
            // Update local analog measurements and announce to CAN bus
            AuxMeasure();
            HallMeasure();
            TelemCANtx();
            
            // Battery and array current
            fprintf(RADIO,",%lu,%lu",battCurr,arrayCurr);
            
            // Aux voltages (report in 10 bit precision)
            for(j=0;j<AuxCount;j++)fprintf(RADIO,",%lu",(int16)AuxCell[j]>>2);
            
            // Motor controller info
            fprintf(RADIO,",%.1f,%.1f,%.0f,%.0f,%.0f,%.2f,%.0f",motorBusV,motorBusI,rpm,contHStemp,contDSPtemp,driveCurr,driveVel);
            break;
            
        case mpptCode:     // MPPT info
            for(j=0; j<MPPTCount; j++){
                mpptTimer[j]++;   // Increment timeout counter for MPPTs
                if(mpptTimer[j]>canTimeout){
                    mpptTimer[j]=0;
                    mpptVoltIn[j]=0;
                    mpptCurrIn[j]=0;
                    mpptVoltOut[j]=0;
                    mpptTemp[j]=0;
                }
                fprintf(RADIO,",%lu,%lu,%lu,%d",mpptVoltIn[j],mpptCurrIn[j],mpptVoltOut[j],mpptTemp[j]);
            }
            break;
            
        default:
            fprintf(RADIO,",0");
            break;
    }
    fputc(0x0D,RADIO); // [CR]
    fputc(0x0A,RADIO); // [LF], maybe remove this later?
    
    //out_id++;
    //if(out_id>mpptCode)out_id=0;
    restart_wdt();
}

void TelemCANparse(int *CANdata, int32 CANid){
    int32 id_hi=CANid & 0xFF00;
    int id_lo=CANid & 0xFF;
    int i=0;
    int32 raw1=0;
    int32 raw2=0;
    
    switch (id_hi){

        case wsID:      // Motor controller
            wsTimer=0;  // Reset WS22 timeout counter;
            // Collect groups of 32 bit data before converting to float            
            for(i=0;i<4;i++){
                raw1+=(int32)CANdata[i]<<(8*i);    //Lowest 32 bits
                raw2+=(int32)CANdata[i+4]<<(8*i);  //Higher 32 bits
            }
            
            switch (id_lo){
            
            //case wsStatus:
            //break;
            
            case wsBus:
                motorBusV=f_IEEEtoPIC((float)raw1);
                motorBusI=f_IEEEtoPIC((float)raw2);
            break;
            
            case wsVeloc:
                rpm=f_IEEEtoPIC((float)raw1);
            break;
            
            case wsHStemp:
                contHStemp=f_IEEEtoPIC((float)raw2);
            break;
            
            case wsDSPtemp:
                contDSPtemp=f_IEEEtoPIC((float)raw1);
            break;
            
            default:
            break;
            }
            
        break;
        
        case evID:      // Drive command from EV driver controls
            evTimer=0;  // Reset drive command timeout counter
            if(id_lo==evDrive){
                for(i=0;i<4;i++){
                    raw1+=(int32)CANdata[i]<<(8*i);    //Lowest 32 bits
                    raw2+=(int32)CANdata[i+4]<<(8*i);  //Higher 32 bits
                }
                driveCurr=f_IEEEtoPIC((float)raw2);
                driveVel=f_IEEEtoPIC((float)raw1);
            }         
        break;
        
        case bpsID:     // BPS
            bpsTimer=0; // Reset BPS timeout counter
            if (id_lo < 6) {
                for (i = 0; i < 4; i++) {
                    battVolt[4*id_lo + i] = CANdata[2*i + 0];
                    battTemp[4*id_lo + i] = CANdata[2*i + 1];
                }
            } else if (id_lo == 6){
                battVolt[24] = CANdata[0];
                battTemp[24] = CANdata[1];
                battVolt[25] = CANdata[2];
                battTemp[25] = CANdata[3];
                battCurr = ((unsigned int16)CANdata[4] << 8) + (unsigned int16)CANdata[5];
            }
        break;
        
        case mpptID:     // drivetek MPPTs       
            id_lo = id_lo & 0x0F; // mask out 0x0770 in response from MPPT, this is the MPPT specific ID
            mpptTimer[id_lo-1] = 0; // Reset MPPT timeout counter
            
            mpptStat[id_lo-1]   = CANdata[0] >> 4;
            mpptVoltIn[id_lo-1] = ((unsigned int16)(CANdata[0] & 0x03) << 8) + (unsigned int16)CANdata[1];
            mpptCurrIn[id_lo-1] = ((unsigned int16)CANdata[2] << 8) + (unsigned int16)CANdata[3];
            mpptVoltOut[id_lo-1]= ((unsigned int16)CANdata[4] << 8) + (unsigned int16)CANdata[5];
            mpptTemp[id_lo-1]   = CANdata[6];
        break;
        
        default:        // Unknown / ignored CAN packet
        break;        
    }
}

void TelemCANtx(void){
    int     out_data[8]= {0,0,0,0,0,0,0,0};
    int1    tx_rtr = 0;
    int1    tx_ext = 0;
    int     tx_len = 8;
    int     tx_pri = 3;
    int     i=0;
    
    // Collect analog measurements
    for(i=0;i<4;i++)out_data[i]=(int8)(AuxCell[i]>>4);
    out_data[4]=(int8)(arrayCurr&0xFF);    // LSB
    out_data[5]=(int8)((arrayCurr&0xFF00)>>8);      // MSB
    out_data[6]=AuxFlags;
    
    // Send message
    can_putd(telemID,out_data,tx_len,tx_pri,tx_ext,tx_rtr);
}

/*
void TelemCANpoll(int32 CANid){
    int     out_data[8] = {0,0,0,0,0,0,0,0};
    int1    tx_rtr = 1; // Set RTR bit to 1 for requests
    int1    tx_ext = 0;
    int     tx_len = 8;
    int     tx_pri = 3;
    
    can_putd(CANid, out_data,tx_len,tx_pri,tx_ext,tx_rtr);
}
*/

