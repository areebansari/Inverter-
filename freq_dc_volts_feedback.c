/*
# This program is reading 2 signal frequencies on interrupts and calculating dc voltage and frequency accordingly
  also an ADC channel is used for reading the analog values from temperature sensor and calculating the 
  temperature. These are are polled by HMI (Master) and this controller is acting as a slave. Communication 
  protocol used is MODBUS RTU.
  
# Line frequency interrupt will occur after every 16.67ms
# Max frequency for DC voltage frequency is 100kHz 
*/

//-- Controller Configuration --//
#include <18f252.h>
#DEVICE ADC=10
#use delay (clock=20M)
#fuses NOLVP, NOPROTECT, HS, NOWDT, NOBROWNOUT 
//!#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7)
//-- END --//

//-- Modbus declaration --//
#define MODBUS_TYPE MODBUS_TYPE_SLAVE 
#define MODBUS_SERIAL_RX_BUFFER_SIZE 40                                        
#define MODBUS_SERIAL_BAUD 38400               

#define MODBUS_SERIAL_INT_SOURCE MODBUS_INT_RDA                    
#define MODBUS_SERIAL_TX_PIN  PIN_C6                                 
#define MODBUS_SERIAL_RX_PIN  PIN_C7 
#define MODBUS_SERIAL_ENABLE_PIN   PIN_C3 //c3  // Controls DE pin for RS485

#include "modbus.c"

#define MODBUS_ADDRESS 0x02 //0x02
//-- END --//

//-- Variables Declaration --//
long temp = 0; // temperature variable
int1 int_flag = FALSE; // indicate 200ms interrupt has occurred
int1 ccp_int = FALSE; // indicate line frequency interrupt has occurred (CCP1)
int1 edge_flag = TRUE;
static unsigned int16 counter = 0;
int16 loopcnt = 0; // loop counter if there is no line freq int 
static unsigned int32 dc_counter = 0; //will increment by 1 after every dc interrupt (CCP2)
unsigned int8 timer_counter = 0; // will increment every 100ms interrupt to calculate 200ms time
static unsigned int32 dc_volts = 0; // copying the value of dc counter 
unsigned int16 dc_mult = 7250; // scaling factor of dc voltage
const int reg_num = 3; // total no of holding registers
unsigned int16 hold_regs[reg_num]={0x00, 0x00, 0x00}; //initializing hold regs 
static int16 isr_ccp_delta = 0; // for storing the period calculated (line freq)
int c = 1; // will increment on every line freq interrupt (CCP1)
static unsigned int16 f[] = {0,0,0,0,0}; // for storing 5 line freq values
static int16 avg = 0; // for storing line freq avg  
int d = 0; // will increment on every 200ms interrupt dc samples
int16 dv[] = {0,0,0}; // storing dc voltages value
static int16 dc_avg = 0; // dc avg values
//-- END --//

/*
# Line Frequency interrupt ISR 
  This will calculate a period between two consecutive rising edges also increment a variable of c by one 
  for sampling and will raise a flag to indicate that interrupt has occurred and period is calculated 
*/
#INT_CCP1
void ccp_isr()
{
//!   int16 current_ccp = 0;
//!   static int16 old_ccp = 0;
//!   
//!   current_ccp = CCP_1;
//!   
//!   isr_ccp_delta = current_ccp - old_ccp;
//!   
//!   old_ccp = current_ccp;
//!   
//!   ccp_int = TRUE;
//!   
//!   c++;
if(edge_flag)
   {
      setup_timer_1(T1_INTERNAL | T1_DIV_BY_8); 
      setup_CCP1(CCP_CAPTURE_FE);
      edge_flag = FALSE;
   }
   else
   {
      counter = get_timer1();
      set_timer1(0);
      setup_CCP1(CCP_CAPTURE_RE);
      edge_flag = TRUE;
      ccp_int = TRUE;
      c++;
   }
}

/*
# DC Frequency interrupt ISR 
  A variable is incrementing on every interrupt 
*/
#INT_CCP2
void ccp2_isr()
{
   dc_counter++;  
}

/*
# 200ms interrupt ISR 
  This will raise a flag after every 200ms and increment a variable d for DC samples
*/
#INT_TIMER3
void timer3_isr()
{
   set_timer3(0);
   timer_counter++;
   if(timer_counter == 2) //reading counter after 200msec
   {
      int_flag = TRUE;
      d++;
   }
}

unsigned int16 round(unsigned int16 num)
{
   unsigned int16 _4bits = 0;
   unsigned int8 remainder = 0;
   
   _4bits = (num & 0x0F) - 4;
   if(_4bits > 5)
   {
      remainder = 10 - _4bits;
      num = num + remainder;
   }
   else if(_4bits < 5)
   {
      num = num - _4bits;
   }
   return num;
}

void main()
{
   //-- Local variable Declaration --//
   static unsigned int32 dc = 0;
   static unsigned int16 freq = 0;
   static unsigned int16 f_count = 0;
   int i = 0;
   //-- END --//
   
   //-- Setting up ADC --//
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_adc_ports(ALL_ANALOG);
   set_adc_channel(0);
   //-- END --//
//!   set_timer1(65535); // T = 1/fosc/4 * prescalar * (65535 - tmr)

   //-- Setting up ccp1 for frequency interrupt --//
   clear_interrupt(INT_CCP1);
   setup_CCP1(CCP_CAPTURE_RE);
   enable_interrupts(INT_CCP1);
   //-- END --//
   
   //-- Setting up timer for calculating period --//
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
   //-- END --//
   
   //-- Setting up ccp2 for dc frequency interrupt --//
   clear_interrupt(INT_CCP2);
   setup_CCP2(CCP_CAPTURE_DIV_16);
   enable_interrupts(INT_CCP2);
   //-- END --//
   
   //-- Setting up timer3 for 200ms interrupt --//
   clear_interrupt(INT_TIMER3);
   setup_timer_3(T3_INTERNAL | T3_DIV_BY_8);
   enable_interrupts(INT_TIMER3);
   //-- END --//
   
   enable_interrupts(GLOBAL); // Enabling global interrupts
   
   modbus_init(); // Initializing modbus 
   
   //infinite loop
   while(1)
   {
      //if frequency interrupt has occured it will disable all the interrupts and will calcuate frequency
      //once done will enable interrupts again
      // frequency is calculated by taking average of 5 samples
      if(ccp_int) 
      {       
         disable_interrupts(GLOBAL);
         f_count = counter;
         enable_interrupts(GLOBAL);
         freq = (62500000/f_count);
         f[c] = freq;
         counter = 0;
         loopcnt = 0;
         ccp_int = FALSE;
         if(c=>6)
         {
            c = 0;
            for(i=0;i<5;i++)
            {
               avg += f[i];
            }
            avg = (avg/5); delay_us(10);
            avg = round(avg);
//!            if(avg<6025) avg = 6020;
//!            else if (avg>=6025) avg = 6030;
//!            avg = (ceil(avg/100))*100;
         }
      }
      
      //this is a loop counter to check line frequency interrupt. if no interrupt is occuring frequency is zero
      else
         if(loopcnt >= 10000)
         {
            avg = 0;
         }
         else
            ++loopcnt;
      //------------------------------------------------------------------------------------------------//
      
      //if 200ms interrupt has occured it will disable all the interrupts and will calcuate dc votlage
      //once done will enable interrupts again
      // dc votlage is calculated by taking average of 3 samples
      // Also, temperature is being calculated in this routine
      if(int_flag)
      {
         disable_interrupts(GLOBAL);
         dc_volts = dc_counter; 
         enable_interrupts(GLOBAL);
         dc = ((dc_volts*7250*5)/100000);
         dv[d-1] = dc;
         temp = (read_adc()*488)/1000;
         dc_counter = 0;
         timer_counter = 0;
         if(d=>4)
         {
            d = 0;
            dc_avg = (dv[0]+dv[1]+dv[2])/3;      
         }
         int_flag = FALSE;
         
      }
      //-----------------------------------------------------------------------------------------------//
      
      //MODBUS CODE//
      
         // setting up holding registers for master to read //
         hold_regs[0] = temp;
         hold_regs[1] = avg;
         hold_regs[2] = dc;
         // END //
      
         if(modbus_kbhit())
         {
            //check address against our address, 0 is broadcast
            if((modbus_rx.address == MODBUS_ADDRESS) || modbus_rx.address == 0)
            {     
               switch(modbus_rx.func)
               {                                             
                  case FUNC_READ_HOLDING_REGISTERS:
                  if(modbus_rx.data[0] || modbus_rx.data[2] ||  modbus_rx.data[1]>=reg_num || 
                     modbus_rx.data[3]+modbus_rx.data[1]>reg_num)
                  {                          
                     modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
                  }
                  else                                                  
                  {    
                     modbus_read_holding_registers_rsp(MODBUS_ADDRESS,(modbus_rx.data[3]*2),
                     hold_regs+modbus_rx.data[1]);
                  }                                                                      
                  break;  
               }
            }
      }  
   }
}
   
      //MODBUS CODE END//
   

