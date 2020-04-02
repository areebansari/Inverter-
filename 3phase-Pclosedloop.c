#include <24HJ128GP306A.h>
#fuses HS,NOWDT,PR
#fuses ICSP2
#FUSES CKSNOFSM
#FUSES PUT128

#use delay(clock=20M)
//!#use rs232(baud=9600, xmit=PIN_F3,rcv=PIN_F2)
//#use rs232(baud=9600, xmit=PIN_F5,rcv=PIN_F4)
#use rs232(baud=9600, xmit=PIN_F5,rcv=PIN_F4, stream=PC)

#include <stdlib.h>
#include <math.h>


/*------------------------------Modbus Defines*/
#define MODBUS_TYPE MODBUS_TYPE_SLAVE 
#define MODBUS_SERIAL_RX_BUFFER_SIZE 40                                        
#define MODBUS_SERIAL_BAUD 38400               

#define MODBUS_SERIAL_INT_SOURCE MODBUS_INT_RDA                    
#define MODBUS_SERIAL_TX_PIN  PIN_F3                                  
#define MODBUS_SERIAL_RX_PIN  PIN_F2 
#define MODBUS_SERIAL_ENABLE_PIN   PIN_D1   // Controls DE pin for RS485

#include "modbus.c"

#define MODBUS_ADDRESS 0x01

/*--------------------------------------------*/

const int reg_num = 19;
int16 hold_regs[reg_num]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//int16 hold_regs[reg_num]={R_V, R_I, S_V, S_I, T_V, T_I, EEPROM, Kp, volt_set, Ki, Ki_delay, volt_err, integ_comp, prop_comp, R_volt_val, V_mult, I_mult, R_curr_val, T_count};

#define ReadEnable   output_high(PIN_F0);
#define ReadDisable  output_low(PIN_F0);

#define WriteEnable  output_high(PIN_F1);
#define WriteDisable output_low(PIN_F1);

//!#define  V_F_high    output_high(PIN_F2);
//!#define  V_F_low     output_low(PIN_F2);

#define  R_latch_High   output_high(PIN_D7);
#define  R_latch_low    output_low(PIN_D7);

#define  S_latch_High   output_high(PIN_D6);
#define  S_latch_low    output_low(PIN_D6);

#define  T_latch_High   output_high(PIN_D5);
#define  T_latch_low    output_low(PIN_D5);

#define  FiberEnable    output_high(PIN_G6);
#define  FiberDisable   output_low(PIN_G6);

#define  IGBTsEnable    output_high(PIN_G7);
#define  IGBTsDisable   output_low(PIN_G7);

#define  Start_Led_ON   output_high(PIN_F3);
#define  Start_Led_OFF  output_low(PIN_F3);

#define  Trip_LED       output_high(PIN_F2);
#define  Trip_Led_OFF   output_low(PIN_F2);

#define  Error_Led_ON   output_high(PIN_D1);
#define  Error_Led_OFF  output_low(PIN_D1);

#define  IncVal   PIN_G14
#define  DecVal   PIN_G12

#define  Pri_Sat  PIN_G13

//!#define  StartButton  PIN_D3;

//!volatile int1  R_data = FALSE, S_data = FALSE, T_data = FALSE;
static int1  Display_R = FALSE;
static int1  Display_S = FALSE;
static int1  Display_T = FALSE;
//!volatile int1  Disable_flag = FALSE;
//!volatile int1  zero_flag = TRUE;
      
//!volatile int freq_1 = 0, freq_2 = 0, freq_3 = 0;

unsigned int16 R_voltage_value = 0, R_current_value = 0;
unsigned int16 S_voltage_value = 0, S_current_value = 0;
unsigned int16 T_voltage_value = 0, T_current_value = 0;
unsigned int16 High_current = 0;

int16 T_count = 0;
int   current_checking = FALSE;
int   intr_count = 0;

union input_data
{
   unsigned int8  bits_8[4];
   unsigned int16 bits_16[2];
   unsigned int32 bits_32;
}R_voltage,R_current,S_voltage,S_current,T_voltage,T_current;

void Decoder_address(int1 a3,int1 a2,int1 a1,int1 a0)
{
   output_bit(PIN_G1,a0);
   output_bit(PIN_G0,a1);
   output_bit(PIN_G3,a2);
   output_bit(PIN_G2,a3);
}

//unsigned int8 data_8bit=0;


   
int Volt_dec_to_Eeprom = 255; // convert to binary for output to eeprom
static int integral_c = 0;
static int integral_en = 0;

int16 set_volt = 250; // REQUIRED VOLTAGE = (set_volt/10) [--Modified Value 3 --]

//!int integral_band = 5;// = (integral_band/10)  ----ceil((set_volt*0.005)+0.5); // Error allowance = 0.5%  [--Original Value--]
int integral_band = 10;//15;// [--1st Modified Value--]
int prop_band = 20;

int16 FreqIC_volts = 0; // (FreqIC_volts/10)
unsigned int16 R_V = 0;
unsigned int16 R_I = 0;
unsigned int16 S_V = 0;
unsigned int16 S_I = 0;
unsigned int16 T_V = 0;
unsigned int16 T_I = 0;

unsigned int16 V_mult = 65535; //FreqIC scaling factor
unsigned int16 I_mult = 10000;

int const Kp_initial = 38;  // [--2nd Modified Value--]
int Kp = 0;           // [--Added with 2nd Modification--]

int Ki = 2; // Ki/10 = 0.2 (NOTE: when change in value of Ki is required,then also change the value of Ki_inv).
int Ki_delay = 8; // cycle division factor = 2
int32 Ki_inv = 5000; // (Ki_inv = 10000/Ki) (NOTE: change value of Ki_inv according to the value of Ki). (E.g Ki = 2 then Ki_inv = 1/2 = 0.5*10 = 5)

int const EEprom_MAX = 254; // = 254 .Volt(255) = 0V
int const EEprom_MIN = 5; // steep voltage change from 0 to 5 - PID off below 5

signed int16 volt_error;
int16 abs_v_err;
signed int16 volt_out;
static signed int32 integ_comp;
static int pid_en = 1;
static int32 integ_comp_MAX = (Ki_inv*EEprom_MAX)/1000;

static signed int16 prop_comp = 0;
static signed int16 i_temp = 0;

int PID_setpoint = 5;
int PID_delay = 3; // delay after (PID_setpoint == Volt_dec_to_Eeprom)
int PID_c = 0;

//!int soft_steps[] = {50, 100, 150, 200, 250};
//!int soft_steps_size = sizeof(soft_steps); // ----------------check--!!!!!!!!!
static int soft_start = 0;
static int soft_stop  = 0;
//!static int step_num   = 0;

//!static int1 Sync_signal = FALSE;
static int Sync_timer = 0;
static const int Sync_timer_setpoint = 5;

int Volt_20pc = 0;

//--------------------
static int  OFiber_EN = 0;
int   button_state = 0;
//!static int IGBT_EN = 0;
//!int start_signal = 0; // Assign Pin
//--------------------

signed int16 diff = 0;

int16 FreqIC_to_actual_volts(unsigned int32 Freq_value)
{
   //int32 result = 68940; //low voltage /10
//!   int32 result = 15271;  //  5.71/3739 = 15271/10,000,000 (result = ((actual_Volts / VFcounter_value)*10,000,000))
   

   unsigned int32 result = Freq_value*V_mult;
//!   result = result/1000000; //1:5 sensing transformer
   result = result/100000; //1:1 sensing transformer

   return (result);
}

int16 FreqIC_to_actual_current (unsigned int32 Freq_I_value)
{
   unsigned int32 result_I = Freq_I_value*I_mult;
   
   result_I = result_I/100000;
   return (result_I);
}

//!int16 FreqIC_to_actual_current (int16 Freq1_value)
//!{
//!   int32 result1;
//!   
//!   if(Freq1_value==0) result1 = 0;
//!   else
//!   {
//!      result1 = Freq1_value + 66;
//!      result1 = result1 / 22;
//!   }
//!   return (result1);
//!}

void Soft_Start_Seq()
{
   if (Volt_dec_to_EEprom < 5)//if E2PROM step is < 5
   {
      Volt_dec_to_EEprom++;
      integ_comp = (Volt_dec_to_EEprom*Ki_inv)/1000;//update 'interg_comp' here so that when enable PID it uses updated value.
   }
   else
   {
      volt_error = set_volt - R_V; // Caculate error
      if (volt_error > (set_volt/20))
      {
         if(Volt_dec_to_EEprom < EEprom_MAX)
            Volt_dec_to_EEprom++;
         else
            Volt_dec_to_EEprom = EEprom_MAX;
         
         integ_comp = (Volt_dec_to_EEprom*Ki_inv)/1000;
      }
      else
      {
         integral_c++;
         soft_start = 0;
         pid_en = 1;
         PID_setpoint = Volt_dec_to_Eeprom; // 17-2-2018 new
      }
   }
}

void Soft_Stop_Seq()
{
   if(Volt_dec_to_EEprom <255)
   {
      if (Volt_dec_to_EEprom > 0) Volt_dec_to_EEprom--;
      else Volt_dec_to_EEprom = 255;
   }
   else
   {
      disable_interrupts(INT_EXT4);  // sync A enable
      IGBTsDisable;
      delay_us(10);
      FiberDisable;
      OFiber_EN = 0;
      button_state = 1;
      soft_stop = 0;
   }
}

#INT_EXT0
void EXT0_isr(void)                                                        /* R-interrupt */
{
   ReadEnable;
   Decoder_address(0,0,0,0); delay_us(1);   R_voltage.bits_16[1] = input_b();   //reading addr 0
   delay_us(2); 
   Decoder_address(0,0,0,1); delay_us(1);   R_voltage.bits_16[0] = input_b();   //reading addr 1
   delay_us(2); 
   Decoder_address(0,0,1,0); delay_us(1);   R_current.bits_16[1] = input_b();   //reading addr 2
   delay_us(2);
   Decoder_address(0,0,1,1); delay_us(1);   R_current.bits_16[0] = input_b();   //reading addr 3 
   delay_us(2);
   ReadDisable;
   
   Display_R = TRUE;
   
   intr_count++;
   if(intr_count >= 10) current_checking = TRUE;
}

#INT_EXT1                                                         /* S-interrupt */
void EXT1_isr()
{
   ReadEnable;
   Decoder_address(0,1,0,0); delay_us(1);   S_voltage.bits_16[1] = input_b();   //reading addr 4
   delay_us(2);
   Decoder_address(0,1,0,1); delay_us(1);   S_voltage.bits_16[0] = input_b();   //reading addr 5
   delay_us(2);
   Decoder_address(0,1,1,0); delay_us(1);   S_current.bits_16[1] = input_b();   //reading addr 6
   delay_us(2);
   Decoder_address(0,1,1,1); delay_us(1);   S_current.bits_16[0] = input_b();   //reading addr 7
   delay_us(2);
   ReadDisable;
   
   Display_S = TRUE;
}


#INT_EXT2                                                         /* T-interrupt */
void EXT2_isr()
{
   ReadEnable;
   Decoder_address(1,0,0,0); delay_us(1);   T_voltage.bits_16[1] = input_b();   //reading addr 8
   delay_us(2);
   Decoder_address(1,0,0,1); delay_us(1);   T_voltage.bits_16[0] = input_b();   //reading addr 9
   delay_us(2);
   Decoder_address(1,0,1,0); delay_us(1);   T_current.bits_16[1] = input_b();   //reading addr 10
   delay_us(2);
   Decoder_address(1,0,1,1); delay_us(1);   T_current.bits_16[0] = input_b();   //reading addr 11
   delay_us(2);
   ReadDisable;
   
   Display_T = TRUE;
}


#INT_EXT4
void EXT4_isr(void)
{
//!   Sync_signal = TRUE;
   Sync_timer++;
   output_b(Volt_dec_to_Eeprom);
   delay_us(5);

   R_latch_high;
   S_latch_high;
   T_latch_High;  
  
   delay_us(5);
   
   R_latch_low;
   S_latch_low;
   T_latch_low;
}

int PIDcal(int16 volt_setpoint,int16 actual_volts)
{
  
   volt_error = volt_setpoint - actual_volts; // Caculate error
   
   abs_v_err = abs(volt_error);
   if (abs_v_err > integral_band) // Ignore minimum error  for integral
   {
      if(integral_en == 1)
      {
         if (abs_v_err >= 15)// original value
            integ_comp = integ_comp + (volt_error/10);
         else if (volt_error > 0)
            integ_comp = integ_comp + 2;
         else
            integ_comp = integ_comp - 2;

         integral_en = 0;
         if (integ_comp > integ_comp_MAX) integ_comp = integ_comp_MAX; // limits integral comp to 254
         else if (integ_comp < 0) integ_comp = 0;
      }
      if (abs_v_err > prop_band)
      {
         prop_comp = (Kp*volt_error)/100; // can put in else to make P and I execution exclusive
      }
   }
   
   if (integ_comp == 0) i_temp = 0; // implementing ceil without float ===>>>> i_temp = ceil((Ki*integ_comp)/10);
   else  i_temp = 1 + (((Ki*integ_comp) - 1) / 10); // if x != 0
   
//!   fprintf(PC,"integ =%lu\t",integ_comp);
   //i_temp = ceil((Ki*integ_comp)/100);// i_temp = ceil((Ki*integ_comp)/10); //using 100 because of mult above

//!   fprintf(PC,"i_comp=%lu\t",i_temp);
   volt_out = prop_comp + i_temp ; // PI Ouput [Original]

   diff = Volt_dec_to_Eeprom - volt_out;
   if (diff > 20)
   {
      if(Volt_dec_to_Eeprom > 24)
      {
         volt_out = Volt_dec_to_Eeprom - 20;// + Eeprom_min!!!!!!!!!!!!!!!!!!!!!
         integ_comp = (volt_out*Ki_inv)/1000; //new 10-2-2018
      }
      else
      {
         volt_out = EEprom_MIN;
         integ_comp = (volt_out*Ki_inv)/1000; //new 10-2-2018
      }
   }
   else if (diff < -20)
   {
      if(Volt_dec_to_Eeprom < 235)
      {
         volt_out = Volt_dec_to_Eeprom + 20;
         integ_comp = (volt_out*Ki_inv)/1000; //new 10-2-2018
      }
      else
      {
         volt_out = EEprom_MAX;
         integ_comp = (volt_out*Ki_inv)/1000; //new 10-2-2018
      }
   }

   if(volt_out > EEprom_MAX) // Saturation Limits Max and Min
   {
      volt_out= EEprom_MAX;
      integ_comp = (volt_out*Ki_inv)/1000; //new 10-2-2018
   }
   else if(volt_out < EEprom_MIN)
   {
      volt_out = EEprom_MIN;
      integ_comp = (volt_out*Ki_inv)/1000; //new 10-2-2018
   }
//!   printf("V_return = %ld",volt_out); //[Modified]
   return volt_out;
}


void main()
{
   setup_adc_ports(NO_ANALOGS);
   
   //----------------------------
   output_b(255);
   
   R_latch_high;
   S_latch_high;
   T_latch_high;
   
   delay_us(10);
   
   R_latch_low;
   S_latch_low;
   T_latch_low;
   //----------------------------
   
   
   clear_interrupt(INT_EXT0);    //
   ext_int_edge(0,L_TO_H);       // R-Interrupt
   enable_interrupts(INT_EXT0);  //

   clear_interrupt(INT_EXT1);    //
   ext_int_edge(1,L_TO_H);       // R-Interrupt
   enable_interrupts(INT_EXT1);  //
   
   clear_interrupt(INT_EXT2);    //
   ext_int_edge(2,L_TO_H);       // R-Interrupt
   enable_interrupts(INT_EXT2);  //
   
   clear_interrupt(INT_EXT4);    //
   ext_int_edge(4,L_TO_H);       // Sync-Interrupt
//   enable_interrupts(INT_EXT4);  //
   
/*****************************************************************************************/   
/**/   setup_compare(5, COMPARE_PWM | COMPARE_TIMER3); // Using Compare Module 5    /**/
/**/                                                                                /**/
/**/   set_pwm_duty(5,1);                              // 50% duty cycle            /**/
/**/   setup_timer3(TMR_INTERNAL | TMR_DIV_BY_1, 2);   // Clock Freq = 3.3 MHz      /**/
/*****************************************************************************************/
   
   enable_interrupts(INTR_GLOBAL);

//!   printf("\r\nTesting\r\n");

   /*----------------------------*/
   ReadDisable;
   Decoder_address(1,1,0,0);  // reading addr 12 data
   WriteEnable;
   OUTPUT_B(1);               // Possible values are 1,2 & 4 only.
   delay_ms(1);
   Decoder_address(0,0,0,0);  // send 0 address.
   delay_ms(1);
   WriteDisable;
   /*----------------------------*/
   
   output_low(PIN_D0);        // reset V_F counter pin
//!   output_high(PIN_G6)
   modbus_init();
   while(TRUE)
   {
      if(Display_R == TRUE)
      {
         R_voltage_value = make16(R_voltage.bits_8[2],R_voltage.bits_8[0]);
         R_current_value = make16(R_current.bits_8[2],R_current.bits_8[0]);
         R_V = FreqIC_to_actual_volts(R_voltage_value);
         R_I = FreqIC_to_actual_current(R_current_value);
         Display_R = FALSE;
      }
      
      if(Display_S == TRUE)
      {
         S_voltage_value = make16(S_voltage.bits_8[2],S_voltage.bits_8[0]);
         S_current_value = make16(S_current.bits_8[2],S_current.bits_8[0]);
         S_V = FreqIC_to_actual_volts(S_voltage_value);
         S_I = FreqIC_to_actual_current(S_current_value);
         Display_S = FALSE;
      }
      
      
      if(Display_T == TRUE)
      {
         T_voltage_value = make16(T_voltage.bits_8[2],T_voltage.bits_8[0]);
         T_current_value = make16(T_current.bits_8[2],T_current.bits_8[0]);
         T_V = FreqIC_to_actual_volts(T_voltage_value);
         T_I = FreqIC_to_actual_current(T_current_value);
         Display_T = FALSE;
      }
      
      /*---------COMM START-----------------------------------------------*/
      
      hold_regs[0] = R_V;
      hold_regs[1] = R_I;
      
      hold_regs[2] = S_V;
      hold_regs[3] = S_I;
      
      hold_regs[4] = T_V;
      hold_regs[5] = T_I;
      
      hold_regs[6] = Volt_dec_to_Eeprom;
      
      hold_regs[11] = volt_error;
      hold_regs[12] = integ_comp;
      hold_regs[13] = prop_comp;
      
      hold_regs[14] = R_voltage_value;
      hold_regs[17] = R_current_value;
      
      hold_regs[18] = T_count;
           

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
               
               case FUNC_WRITE_MULTIPLE_REGISTERS:
               if(modbus_rx.data[0] || modbus_rx.data[2] ||  modbus_rx.data[1]>=reg_num || 
                  modbus_rx.data[3]+modbus_rx.data[1]>reg_num)
               {
                  modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               }
               else
               {
                  hold_regs[modbus_rx.data[1]]= make16(modbus_rx.data[5],modbus_rx.data[6]);
                  
                  modbus_write_multiple_registers_rsp(MODBUS_ADDRESS,
                                 make16(modbus_rx.data[0],modbus_rx.data[1]),
                                 make16(modbus_rx.data[2],modbus_rx.data[3]));
               }
               break;
               
               default:    //We don't support the function, so return exception
               modbus_exception_rsp(MODBUS_ADDRESS,modbus_rx.func,ILLEGAL_FUNCTION);
            }                                      
         }
      }
      
      Kp = hold_regs[7];
      set_volt = hold_regs[8];
      Ki = hold_regs[9];
      Ki_delay = hold_regs[10];
      V_mult = hold_regs[15];
      I_mult = hold_regs[16];
      
      if (Ki == 0)   Ki_inv = 0;
      else  Ki_inv = 10000/Ki; //!!!!!!!!!!!!!!!!!!zero division!!!!!!!!!!!!!!!!!!!
      integ_comp_MAX = (Ki_inv*EEprom_MAX)/1000;
      //--------------------------- COMM END---------------------
      
      if(input(PIN_G9)) // Start Button 
      {
         Start_Led_ON;
         
         //delay_ms(1);
         if(OFiber_EN == 0 && button_state == 0)
         {
//!            printf("A\r\n");
            OFiber_EN = 1;
            FiberEnable;
            delay_ms(1);
            IGBTsEnable;
            delay_ms(1);
            enable_interrupts(INT_EXT4);  // sync A enable
            Volt_dec_to_EEprom = 0;
            soft_start = 1;
            soft_stop  = 0;
            pid_en = 0;
         }
      }
      else  
      {
         Start_Led_OFF;
         button_state = 0;
         
         if(OFiber_EN==1)
         {
            //step_num = 0;// new 10-2-2018
            soft_start = 0;
            pid_en = 0;
            soft_stop  = 1;
         }
      }
           
      if(Sync_timer >= Sync_timer_setpoint)//after 'Sync_timer_setpoint' times execute this loop.
      {
         if(soft_start == 1)     Soft_Start_Seq();
         else if(soft_stop == 1) Soft_Stop_Seq();
         else if(pid_en == 1)
         {
            if (PID_setpoint == Volt_dec_to_Eeprom) 
            {
               if (PID_c == PID_delay) // delay after (PID_setpoint == Volt_dec_to_Eeprom) for voltage settling
               {
                  PID_c = 0;
                  if (Ki_delay == 0)   Ki_delay = 3; // 12-2-2018
                  if (integral_c == Ki_delay)
                  {
                     integral_en = 1;
                     integral_c = 0;
                  }
                  else if (integral_c > Ki_delay)  integral_c = 0;
                  
                  integral_c++;
                  PID_setpoint = PIDcal(set_volt, R_V);
               }
               PID_c++;
            }
            else if (PID_setpoint > Volt_dec_to_Eeprom)    Volt_dec_to_Eeprom++;
            else if (PID_setpoint < Volt_dec_to_Eeprom)    Volt_dec_to_Eeprom--;
         }
//!         Sync_signal = FALSE;
         Sync_timer = 0;
      }

//!      /*----------High_current----------*/
//!      if( R_current_value > S_current_value ) High_current = R_current_value;
//!      else High_current = S_current_value;
//!      if( High_current < T_current_value) High_current = T_current_value;
//!      /*--------------------------------*/

      /*----------Primary_Saturation---------*/
      if(input(Pri_Sat)) 
      {
         if(Volt_dec_to_Eeprom > 5) Volt_dec_to_Eeprom = 5;                     
         soft_start = 0;
         pid_en = 0;
         soft_stop = 1;
         while(Volt_dec_to_Eeprom != 255) Soft_Stop_Seq();
      }
      /*--------------------------------*/ 
      
      /*----------High_current----------*/
      if( R_I > S_I ) High_current = R_I;
      else High_current = S_I;
      if( High_current < T_I) High_current = T_I;
      /*--------------------------------*/
      
      /*---------Load Current Checking-------------------------*/
      if( current_checking )
      {
         /*if ( High_current > 10322 )     T_count+=300;    //actual current = 416
         else if ( High_current > 7704 )   T_count+=60;     //actual current = 312
         else if ( High_current > 6161 )   T_count+=10;     //actual current = 250
         else if ( High_current > 5082 )   T_count+=1;      //actual current = 208*/
         
         
         /*----------------For Testing --------------------*/
         
//!         if ( High_current > 8700 )       T_count+=50;    //actual current = 400
//!         
//!         else if ( High_current > 7400 )   T_count+=10;     //actual current = 305
//!         
//!         else if ( High_current > 6000 )   T_count+=1;     //actual current = 255
         
//!         if ( High_current > 85 )       T_count+=50;    //actual current = 400
//!         
//!         else if ( High_current > 60 )   T_count+=10;     //actual current = 305
//!         
//!         else if ( High_current >  40)   T_count+=1;     //actual current = 255

         if ( High_current > 400 )       T_count+=50;    //actual current = 400
         
         else if ( High_current > 305 )   T_count+=10;     //actual current = 305
         
         else if ( High_current >  255)   T_count+=1;     //actual current = 255

      // if (High_current > 3000) T_count+=5;  // actual cuurent ~= 110 
       //else if ( High_current > 4004 )   T_count+=10;      //actual current ~= 160

         if( T_count > 7200 )
         {
            //step_num = 0;// new 10-2-2018
            soft_start = 0; // new 10-2-2018
            pid_en = 0;     
            soft_stop  = 1;
            T_count = 0;
         }
//!         if ( T_count <=0 ) T_count = 0;
//!         else if ( High_current<5100 ) T_count-=2; //current < 208
         
         if ( T_count <=0 ) T_count = 0;
         else if ( High_current<208 ) T_count-=2; //current < 208
         
         intr_count = 0;
         current_checking = FALSE;

      } 
      
      
//!      printf("set_p=%lu\t",set_volt);
//!      printf("Step=%lu\t",Volt_dec_to_Eeprom);//--------------------------------
//!      
//!      printf("V1=%lu\t",R_voltage_value);
//!      printf("I1=%lu\t",R_current_value);
//!      
//!      printf("V2=%lu\t",S_voltage_value);
//!      printf("I2=%lu\t",S_current_value);
//!      
//!      printf("V3=%lu\t",T_voltage_value);
//!      printf("I3=%lu\t",T_current_value);
//!      printf("count=%lu\r\n",T_count);
      //if(over_current)  printf("Overload\r\n");
//!      printf("Error=%lu\r\n",volt_error);
//      printf("fIC_v=%u\r\n",FreqIC_volts);
//!      printf("fIC_v=%u\r\n",R_V);
      
//!      fprintf(PC,"RV=%lu\t",hold_regs[0]);
//!      fprintf(PC,"ROM=%lu\t",Volt_dec_to_Eeprom);
//!      fprintf(PC,"Kp=%lu\t",Kp);
//!      fprintf(PC,"Ki_delay=%lu\t",Ki_delay);
//!      fprintf(PC,"Ki_inv=%lu\t",Ki_inv);
//!      fprintf(PC,"set volt=%lu\t\r\n",R_V); 
   }
}  


