/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "chopshop.h"
#include "adc.h"
#include "gyro.h"

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/

/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")

void InterruptHandlerLow ()     
{

static char hold3 = 0;						//hold value for timer3

	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
	 
							//holds know of interrupt hits gotten

	if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)
	{
     	INTCON3bits.INT2IE = 0;        //disable the interrupt to prevent it from being recalled
    	/*Perform code needed here. WARNING: This must be quick because if the code stays here to long a code error will occur. */
		rps(0);
		INTCON3bits.INT2IF = 0;       //clear the flag bit
      	INTCON3bits.INT2IE = 1;      //re enable the interrupt
  	} 
	else if(PIE2bits.TMR3IE && PIR2bits.TMR3IF)
	{
		PIE2bits.TMR3IE = 0; //turn interrupt off to prevent it from being recalled
		PIR2bits.TMR3IF = 0; //reset the flag to 0
		hold3++; 			 //adds one to the hold
		if (hold3 == 20)		 //is hold equal to 20
		{
			timer_data(3,0); //call timer_data function to add a second
			hold3 = 0;       //reset hold to 0 so the next second can be counted
		}
		PIE2bits.TMR3IE = 1; // enable interrupt
	} 
	else if (PIE1bits.TMR1IE && PIR1bits.TMR1IF)		//is the interrupt enabled and the flag triggered
		 {		
		      PIE1bits.TMR1IE = 0;					//turn interrupt off, this prevents calling it again while the following code is executed
		      PIR1bits.TMR1IF = 0;					//flag reset
		      timer_data(1, 0);						//call timer_data function
		      PIE1bits.TMR1IE = 1;					//enable interrupt again
       }
	else if(PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
		Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in adc.c)
	}
    else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
	{
		PIR1bits.ADIF = 0; // clear the ADC interrupt flag
		ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
	}               

//  ***  IFI Code Starts Here***
//                              
//  unsigned char int_byte;       
//  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
//  { 
//    INTCON3bits.INT2IF = 0;
//  }
//  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
//  {
//    INTCON3bits.INT3IF = 0;
//  }
//  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
//  {
//    int_byte = PORTB;          /* You must read or write to PORTB */
//    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
//  }                                        /*     to clear the interrupt condition.  */
//  else
//  { 
//    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
//  }
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{

unsigned char auto_selection = 0;					//determines that autonomous mode the user wishes to perform

  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
#if 1
		//can be 0-3, dependin on state of 2 dig inputs
		auto_selection = //rc_dig_in01 + (rc_dig_in02 * 2);
							1; //Auto mode 2, no matter what (drive forward)
printf("\rAuto_selection1 %d autoselect2 %d auto select is %d", rc_dig_in01, rc_dig_in02, auto_selection);

		switch(auto_selection)
		{
		case 0:
			User_Autonomous_Code_0();	
		break;
		case 1:
			User_Autonomous_Code_2();
		break;
		case 2:
			User_Autonomous_Code_3();
		break;
		case 3:
			User_Autonomous_Code_4();
		break;
		default:
			User_Autonomous_Code_4();
		break;
		}
#endif


        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);





        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */
 if(Get_ADC_Result_Count())
  {
    Process_Gyro_Data();
	
    Reset_ADC_Result_Count();
  }	
	  

}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
