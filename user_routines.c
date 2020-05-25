/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "chopshop.h"
#include "eeprom.h"
#include "gyro.h"
#include "adc.h"


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/
extern T_Packet_Data_Type T_Packet_Data;					//This is a local verision of the camera
															//information that we can work with
unsigned char which_drive;		//setting for the drive function
long temp_gyro_angle;
typedef enum{PREP,
			 FINE_ADJ}flywheel_stat;

flywheel_stat flywheel_adj_flag;	

unsigned char prev_prepared = 0;
unsigned char auto_aim_override = 1;			//if this is set to 1, then the automatic aiming function returns 1							

/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  //Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */
 
  Init_Serial_Port_One();
  Init_Serial_Port_Two();
  Initialize_ADC();
  Initialize_Gyro();
  


/***********************************Drive setting read********************************************/
   which_drive = EEPROM_read(9);					//read from EEPROM the value for which_drive

   if (which_drive == 1 || which_drive == 2)		//is the value one of the two accepted values?
   {												//do nothing
   }
   else												//else
   {
	   which_drive = DEFAULT_DRIVE;					//set the value to the default
	   EEPROM_prep((unsigned int)9, which_drive);	//write the setting to EEPROM
   }

/***************************************************************************************************/

  INTCON3bits.INT2IE = 1;		//dig i/o 1 interrupt
  initialize_timer1();
  initialize_timer3();

#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
#endif

  Putdata(&txdata);             /* DO NOT CHANGE! */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
static unsigned int i = 0;
static unsigned int j = 0;
static char gyro_cal = 0;
int temp_gyro_rate;
int temp_gyro_bias;


	Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
	i++;
	j++; 				// this will rollover every ~1000 seconds

	
	if(!gyro_cal)
	{
		if(j == 10)
		{
			printf("\rCalculating Gyro Bias...");
		}

		if(j == 60)
		{
			// start a gyro bias calculation
			Start_Gyro_Bias_Calc();
		}

		if(j == 300)
		{
			// terminate the gyro bias calculation
			Stop_Gyro_Bias_Calc();

			// reset the gyro heading angle
			Reset_Gyro_Angle();

			printf("Done\r");
			gyro_cal = 1;
		}
	}

	if(i >= 30 && j >= 300)
	{
		temp_gyro_bias = Get_Gyro_Bias();
		temp_gyro_rate = Get_Gyro_Rate();
		temp_gyro_angle = Get_Gyro_Angle();
		//printf(" Gyro Bias=%d\r\n", temp_gyro_bias);
		//printf(" Gyro Rate=%d\r\n", temp_gyro_rate);
		//printf(" Gyro Angle=%li\r\n\r\n", temp_gyro_angle);
		
	i = 0;
	}


	// This function is responsable for camera initialization 
	// and camera serial data interpretation. Once the camera
	// is initialized and starts sending tracking data, this 
	// function will continuously update the global T_Packet_Data 
	// structure with the received tracking information.
	Camera_Handler();


	EEPROM_write();					//if there is data in the queue, write to the data EEPROM
    Default_Routine();  /* Optional.  See below. */

    Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

    Putdata(&txdata);             /* DO NOT CHANGE! */
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
static char is_it_still_being_held_drive = 0;															//debounce for the drive selection
static int counter2 = 0;
static int last_time_value = 0;
static unsigned char last_rps_calculation = 0;
static char last_val_left = 0;
static char counter_L = 0;
static int counter = 0;
static char last_val = 0;
static char last_val_right = 0;
static int counter_R = 0;
	
//printf("\rinput is %d or %d", rc_dig_in05, rc_dig_in06);

if(L_encoder != last_val_left)		//has the value of the encoder changed?		
		{
			counter_L++;					//if so, add one to counter
			last_val_left = L_encoder;		//save the value
			printf("\rThe counter on left is %d", counter_L);
		}

if(R_encoder != last_val_right)		//has the value of the encoder changed?		
		{
			counter_R++;					//if so, add one to counter
			last_val_right = R_encoder;		//save the value
			printf("\rThe counter on right is %d", counter_R);
		}

#if 1
if(p1_sw_top && p2_sw_top)
{
counter++;
	if(counter % 32 == 0)
			{	
				shoot(1);
				printf("\rNow opening for a shoot");
				last_val = counter;
			}
			
			 if(!(counter % (10 + last_val)))
				{
					shoot(0);
					last_val += 2;
					printf("\rNow closing for a shoot");
				}

			if(counter > 200)
			{
				shoot(0);
				printf("\rDONE DONE DONE!!! FOR THE HORDE!!!!");
			}
}
#endif	
/*this is the tested values for the sensor*/
#if 0
if(timer_data(1,1) != last_time_value)					//do this only every quater for sample rate
		{
				last_time_value = timer_data(1,1);
				INTCON3bits.INT2IE = 0;						//turns sensor interrupt off to prevent more data from coming in
				last_rps_calculation = rps(1);				//determine what the rps of the wheel is from the rps function
				User_Byte3 = last_rps_calculation;			//assign value of user byte 
				rps(2);										//reset the counter in the rps function
				timer_data(1,2);							//reset the timer
				INTCON3bits.INT2IE = 1;
		}
printf("\rrps %d", last_rps_calculation);
#endif

if(counter2 == 20)
{     
//printf("\rdone is %d: interrupt is %d:", done, INTCON3bits.INT2IE);     
//printf("\rx %d, y %d", T_Packet_Data.mx, T_Packet_Data.my);	//used to determine 
counter2 = 0;
}
counter2++;

//printf("\rl_encouder %d flywheel_ sensor %d", L_encoder, rc_dig_in01);
/*********************************Setting code***********************************/
	if(p4_sw_trig)																				//is p4_sw_trig pressed down?
	{																							//
		if(p4_sw_top == 1 && which_drive == 1 && is_it_still_being_held_drive == 0)				//is p4_sw_top compressed and drive setting is 1?
		{
			which_drive = 2;																	//change the drive setting to two stick drive 	
`			EEPROM_prep((unsigned int)9, which_drive);											//write the new value to the EEPROM
			is_it_still_being_held_drive = 1;													//the button is being held now
		}
		else if(p4_sw_top == 1 && which_drive == 2 && is_it_still_being_held_drive == 0)		//else, is the top switch pressed and which_drive = 2 
			 {
				which_drive = 1;																//change which drive to 1
				EEPROM_prep((unsigned int)9, which_drive);										//write the new value to EEPROM
				is_it_still_being_held_drive = 1;												//the button is being held now
			}
    		else if(p4_sw_top == 0 && is_it_still_being_held_drive == 1)						//is the bottom not pressed and the debounce is 1				
				 {
					is_it_still_being_held_drive = 0;											//reset the debounce to 0
				 }
	}

/***********************************Drive code**************************************/
/*Using the global which_drive variable, we decide which type of drive should be used*/
	
	if(which_drive == 1)
	{
		Squishy_Drive_1stickdrive(&pwm01, &pwm02, 100);										//one stick drive
#if 0
		printf("drive:   p1_y = %d,  p1_x = %d\r pwm01 = %d,  pwm02 = %d\r\r", p1_y, p1_x, pwm01, pwm02);	
#endif
	}
	else if(which_drive == 2)
		{
			two_stick_drive(p1_y , p2_y, &pwm01, &pwm02, 100);								//two stick drive
#if 0
			printf("drive:   p1_y = %d,  p2_y = %d\r pwm01 = %d,  pwm02 = %d\r\r", p1_y, p2_y, pwm01, pwm02);
#endif
		}

/**********************************End drive code**********************************/
/*********************************************************************************/
/*************************gear switching function******************/
//();			//this has been changed to be called from the user_byte call												
/*************************feetContol function*************************************/
//feetControl();		//this has been chagned to be called rom the user_byte call
/************************pressure control function*****************/
pressure_control(PRESSURE_SENSOR);
/******************************************************************/
/***********************Gatherer function***************************/
Six_Lines_of_Gatherer_Code();
/*******************************************************************/
/*************************USER BYTE ASSIGNING************************
These are bytes that are sent back to the operator controler. By using a drain program,
you are able to pull these numbers out of the radio packets and use excel to graph them. 
It also gives you the ability to look at data recorded during a match at a later date so 
that having the robot can be unnecessary. The below code will assign the values to the user bytes. 
Line by line comments should be unnecessary
*********************************************************************/	
     User_Byte1 = T_Packet_Data.mx;
     User_Byte2 = T_Packet_Data.my;;
	 User_Byte4 = 0;	//clear all flags
	 if (gearSwitch())
		 User_Byte4 |= 1; /*indicate we're in high gear, or whatever gear out is */
	 if (PRESSURE_SENSOR)
		User_Byte4 |= 2; /*the compressor is on*/
	 if (p3_sw_trig)
		User_Byte4 |= 4; /*Indicate ball trigger depressed*/
	 if (p4_sw_aux1)
		User_Byte4 |= 8; /*Indicate fire button depressed */
	 User_Byte5 = rc_ana_in05; //slide pot on warp drive

#if 0
	printf("Userbytes1 %d\r 2: %d\r 4: %d\r 5: %d\r", User_Byte1, User_Byte2, User_Byte4, User_Byte5);
#endif
//warp_pulse();
/******************************************End user byte assigning**********************/
 
	if(PREPARE_TO_FIRE)						//are we prepared to just firing?
	{
		aimed_shot();						//get rdy!
	}
	else
	{
        flywheel = 127;						//else turn the flywheel
		flywheel_adj_flag = PREP;
	}

	
} 
/********************************End Default Routines******************************/


myfunction()
{
}

/***********************************************************************
Created by: 	Eric Finn - General coding and debugging
				Steven Shidlovsky - Assistance with code
				Per - Mentor
Date modified:  1-15-2006
				2-16-2006 (Sarah) function now returns whether we are in high or low gear 
Function name:  gearSwitch
Parameters:     none
Returns: 		relay1_fwd  = Switch to high gear
		    	relay2_fwd  = Switch to low gear
				gear_state  = what gear we're in
Purpose: This function changes the gear of the robot. 
Notes:	 None
**********************************************************************/
char gearSwitch(void)
{
	static char whichout = 0;								//tells the program which output to use
	static char hold = 0; 									//tells the program if the trigger is being held
	static char gear_state;										//return value.  Tells program if we are in high gear or low gear
															//1 = out, 0 = in, 2 = no change

	//if a trigger is pressed and output 0 is supposed to be on...
	if(p2_sw_trig && whichout == 0 && hold == 0 || p1_sw_trig && whichout == 0 && hold == 0)
	{
		gear_out = 1;	 									//turn it on...
		gear_in = 0; 										//turn output 2 off.
		whichout = 1; 										//say that output 1 should be turned on next...
		hold = 1; 											//tell program trigger is being held...
		gear_state = 1;										//tell program that gear is out
	}
	//if a trigger is pressed and output 1 is supposed to be on...
	else if(p2_sw_trig && whichout == 1 && hold == 0 || p1_sw_trig && whichout == 1 && hold == 0)
	{
		gear_in = 1; 										//turn output 1 on...
		gear_out = 0; 										//turn output 0 off
		whichout = 0; 										//tell program that output 0 is supposed to be on...
		hold = 1; 											//tell program that trigger is held
		gear_state = 0;										//tell program that gear is in
	}
	//if an output is on...
	else if(gear_in == 1 || gear_out == 1)
	{
		//Turn both of the relays off
		gear_in = 0;
		gear_out = 0;
	}
	//if the triggers are not held, and program thinks it is being held...
	else if(!p2_sw_trig && !p1_sw_trig)
	{
		//if both trigger buttons aren't being held, tell program it's not being held
		hold = 0;										
	}
#if 0
	printf("Gear_state: %d\r", gear_state);
#endif
    return gear_state;
}

/***********************************************************************
Created by: 	Steven Shidlovsky - General coding and debugging
Date modified:  
Function name:  aimed_shot
Parameters:     none
Returns: 		none
Purpose: To control horizontal aiming and flywheel speed, 
Notes:	 None
**********************************************************************/
void aimed_shot(void)
{

unsigned char known_rps = 31;
static unsigned char last_rps_calculation = 0;
static unsigned char last_time_value = 0;
static char counter = 0;
int hold;
static char firing = 0;
static char counter2 = 0;

/*This is full fire mode*/
if(FIRE_MODE > 200)
{
	if(p3_sw_trig)		//if they are pressing the trigger
	{
		shoot(1);		//keep the piston in
	}
	else				//else
	{
		shoot(0);		//u need to push the piston back out
	}
	flywheel = BASE_FIRE_SPEED;	//the flywheel is at base fire speed
}
else if(FIRE_MODE < 50)			//semiauto without sensor
	{
		flywheel = BASE_FIRE_SPEED;
		if(p3_sw_trig && !firing)						//is the fire trigger pulled?
		{
			firing = 1;									//start firing 
		}

		if(firing)										//are we firing?
		{	
			counter2++;									//add one to the counter
			if(counter2 < 40)							//if the counter is less than 40
			{
				shoot(1);						//let balls through
			}
			else if (counter2 < 80)
				 {
						shoot(0);				//close off the flow of balls
						flywheel = BASE_FIRE_SPEED; //speed up the flywheel
				 }
				else if(counter2 > 80)
					{
						counter2 = 0;					//reset counter
						firing = 0;						//we are done firing
					}
		}
	}

#if 1 //if 1 then the flywheel sensor works, if 0 it is broken
/*this is an experimental mode in the event that we have time to make it work*/
else if(FIRE_MODE < 200 && FIRE_MODE > 50)
{	
  /*Will use full auto but can be changed easily if necessary to semiauto*/
	if(p3_sw_trig)
	{
		shoot(1);
	}
	else
	{
		shoot(0);
		flywheel_adj_flag = PREP;
	}

		if(timer_data(1,1) != last_time_value)					//do this only every second for sample rate
		{
				last_time_value = timer_data(1,1);
				INTCON3bits.INT2IE = 0;						//turns sensor interrupt off to prevent more data from coming in
				last_rps_calculation = rps(1);				//determine what the rps of the wheel is from the rps function
				User_Byte3 = last_rps_calculation;			//assign value of user byte 
				rps(2);										//reset the counter in the rps function
				timer_data(1,2);							//reset the timer
				printf("\rrps is %d flywheel %d case %d", last_rps_calculation, flywheel, flywheel_adj_flag);
				switch(flywheel_adj_flag)					//switch this flag
				{
				case PREP:									//case PREP
					flywheel = BASE_FIRE_SPEED;				//set the flywheel to BASE_FIRE_SPEED
					if(counter > 12)							//wiat for half a second
					{
						flywheel_adj_flag = FINE_ADJ;		//move to fine adj
						counter = 0;						//reset counter
					}
					counter++;								//add one to counter
				break;										//add one to counter

				case FINE_ADJ:
					if(last_rps_calculation == 0)			//if the interrupt has broken
					{
						printf("\rERROR! NO INPUT!");		//error has occureed
						flywheel = BASE_FIRE_SPEED;
					}
					else
					{
						if((known_rps - last_rps_calculation) > ALLOWED_Y_ERROR)
						{
							hold = (known_rps - last_rps_calculation);	//add a small amount for fine adjustment				
							if(hold < 5)
							{
								flywheel += hold;
							}
							
						}
					}
					if(flywheel > 255)
					{
						flywheel = 254;
					}
				break;
				}
		}	
		else
		{
			INTCON3bits.INT2IE = 1;	
		}	
}
#endif

//		printf("\rlast rps %d looking for %d flywheel %d CASE %d", last_rps_calculation, known_rps, flywheel, flywheel_adj_flag);		

	    //the center for x is 79
		/*is the x coordinate from the camera within the allowed error of the center?*/	
	
}


/***********************************************************************
Created by: 	Steven Shidlovsky - General coding and debugging
				Per - Mentor
Date modified:  2-04-2006
Function name:  rps
Parameters:     char flag   = determines what operation that needs to be done
Returns: 		The rps value of the wheel
Purpose: To determine the rps of the flywheel 
Notes:	 This is intended to be used with an interrupt. Due to change in how we are going to input the
		 rps, this function will not likely be used.
**********************************************************************/
float rps(char flag)
{
#if 1
static unsigned int clicks = 0;					    //number of hits we have gotten on the interrupt

    switch(flag)									//determine what operation do perform
	{
	case 0:											//is the flag equal to 0?
		clicks++;							//add one to the clicks value
		//printf("\rclick");
		break;
	case 1:											//is it 1?
		return((clicks/FLYWHEEL_TRANS) * 4);// * 3.167);		   			//the equation is (number of clicks / clicks per rotation) * circucmfirence of wheel in feet)
	break;
	case 2:											//is the flag 2?
		clicks = 0;									//reset the click value
	break;
	}
#endif
}	

/***********************************************************************
Created by: 	Steven Shidlovsky - General coding and debugging
Date modified:  3-3-2006
Function name:  shoot
Parameters:     char flag   = determines what operation that needs to be done
Returns: 		nothing
Purpose: To operate the solinod that is on the shooter 
Notes:	 Able to be used anywhere as long as flag is called properly. Call flag 1 first than flag 0
**********************************************************************/
void shoot(char flag)
{
	static char whichout = 0;								//tells the program which output to use
	static char hold = 0; 									//tells the program if the trigger is being held
	static char prev_flag;														//1 = out, 0 = in, 2 = no change
	
	
	//if a trigger is pressed and output 0 is supposed to be on...
	if(whichout == 0 && hold == 0 && flag == 1)
	{
		loader_out = 1;	 									//turn it on...
		loader_in = 0; 										//turn output 2 off.
		whichout = 1; 										//say that output 1 should be turned on next...
		hold = 1; 											//tell program trigger is being held...
	}
	//if a trigger is pressed and output 1 is supposed to be on...
	else if(whichout == 1 && hold == 0 && flag == 0)
	{
		loader_in = 1; 										//turn output 1 on...
		loader_out = 0; 										//turn output 0 off
		whichout = 0; 										//tell program that output 0 is supposed to be on...
		hold = 1; 											//tell program that trigger is held
	}
	//if an output is on...
	else if(loader_in == 1 || loader_out == 1)
	{
		//Turn both of the relays off
		loader_in = 0;
		loader_out = 0;
	}
	//if the triggers are not held, and program thinks it is being held...
	else if(prev_flag != flag)
	{
		//if both trigger buttons aren't being held, tell program it's not being held
		hold = 0;										
	}

//printf("\rflag %d prev_flag %d, in %d out %d hold %d", flag, prev_flag, loader_in, loader_out, hold);
prev_flag = flag;

//printf("\rfiring %d, loader %d, fire mode %d flywheel %d mode in %d", firing, loader, FIRE_MODE, flywheel, mode_in);


#if SENSOR_DEAD
	User_Byte3 = flywheel;
#else
	User_Byte3 = rps(1);
#endif

}

/***********************************************************************
Created by: 	Pat - General coding and debugging
Date modified:  2-02-2006
				2-16-2006 (Sarah) returns whether the feet are down or up
Function name:  feetControl
Parameters:     none
Returns: 		foot_down  = Push foot down
		    	foot_up    = Pull foot in
				foot_state = whether foot is in or out
Purpose: This function changes the state of the foot on the robot. 
Notes:	 None
**********************************************************************/
#if 0 //we're not using feet
char feetControl(void)
{
	static char whichfout = 0;								//tells the program which output to use
	static char fhold = 0; 									//tells the program if the trigger is being held
	static char foot_state;										//return value.  Tells whether foot is down or up
															//1 if up, 0 if down

	//if a trigger is pressed and output 0 is supposed to be on...
	if(p1_sw_top && whichfout == 0 && fhold == 0 || p2_sw_top && whichfout == 0 && fhold == 0)
	{
		foot_up = 1;	 									//turn it on...
		foot_down = 0; 										//turn output 2 off.
		whichfout = 1; 										//say that output 1 should be turned on next...
		fhold = 1; 											//tell program trigger is being held...
		foot_state = 1;										//tell program foot is up
	}
	//if a trigger is pressed and output 1 is supposed to be on...
	else if(p1_sw_top && whichfout == 1 && fhold == 0 || p2_sw_top && whichfout == 1 && fhold == 0)
	{
		foot_down = 1; 										//turn output 1 on...
		foot_up = 0; 										//turn output 0 off
		whichfout = 0; 										//tell program that output 0 is supposed to be on...
		fhold = 1; 											//tell program that trigger is held
		foot_state = 0;										//tell program that foot is down
	}
	//if an output is on...
	else if(foot_down == 1 || foot_up == 1)
	{
		//Turn both of the relays off
		foot_down = 0;
		foot_up = 0;
	}
	//if the triggers are not held, and program thinks it is being held...
	else if(!p1_sw_top && !p2_sw_top)
	{
		//if both trigger buttons aren't bein held, tell program it's not being held
		fhold = 0;
	}

 //printf("foot_state: %d\r", foot_state);
 return foot_state;
}
#endif
/***********************************************************************
Created by: 	Robert Harris - General coding and debugging
Date modified:  2-10-2006
Function name:  Six_Lines_of_Gatherer_Code
Parameters:     none
Returns: 		None
Purpose: This operates the gatherer motor on the robot 
Notes:	 None
**********************************************************************/
void Six_Lines_of_Gatherer_Code(void)
{
/*This code checks the the switch that set set to an anolog value 
  to determine what action needs to be done*/

#if 0
printf("\rinput is %d motor is %d\r", three_position_switch_for_gatherer_motor, gatherer_motor);
#endif

gatherer_motor = 127;

    if (three_position_switch_for_gatherer_motor < 30 || p4_wheel < 30)
	{
        gatherer_motor = GATHER_MOTOR_IN;
	}
	else if(three_position_switch_for_gatherer_motor > 200 || p4_wheel > 200)
		{ 
			gatherer_motor = GATHER_MOTOR_OUT;
		}
	else 
		 {
			gatherer_motor = 127;
		 }
}

/*********************************************************************
 Created by; Patrick 
	This function will let us slightly warp the shooter to give us some
	control over the x-axis allowing us to change the x-axis without 
	needing to turn the entire robot to aim the shooter.
Pramaters: None.
Variables: warp_lv counter for how many loops the motor has been on.
		   warp_speed controls how fast the positive turn of the motor is.
		   warp_rspeed controls how fast the negative turn of the motor is.
		   WARP_DRIVE_PULSE controls th max loops the motor will be on for.
		   warp_drive is the window motor, defined in user_routines.h
Extra coment: WARP_DRIVE_PULSE and warp_drive are defined in user_routines.h
*********************************************************************/ 
void warp_pulse(void)
{
#if 0
	static char warp_lv=0;										//counter for if warp is activated for x loops.
	char warp_speed= 167;     //170;										//easy control for how fast to have the motor go forward.
	char warp_rspeed= 77;     //102;										//easy control for how fast to have the motor go backward.
	
	auto_aim_override = 1; //we're pushing this manually, so don't go through 

	if ((p3_x > 210) || (p3_x < 40))                               // Was either button 1 or two depressed?
        {
        if (warp_lv < WARP_DRIVE_PULSE)                         // Can we make one more lap?
            {
			warp_lv++;											//add to counter for how long the warp drive is running.
			if (p3_x > 210)										//check if button is pressed and debounce has not been activated.
				{
				//printf("Warp engage forward\r\n");			//printf to tell us if the warp drive has been engaged to go forward.
				warp_drive=warp_speed;							//speed at which the motor is being run.
				}
            else
				{
				//printf("Warp reverse\r\n");					//printf to tell us if the warp drive has been engaged to go backward.
				warp_drive=warp_rspeed;							//speed at which the motor is being run.
				}
            }
        else
			{
		    //printf("Warp disengage after running out\r\n");	//printf to tell us that we have disengaged the warp_drive.
		    warp_drive=127;										//neturalize the warp_drive to stop the motor.
			}
        }
	else
		{
		//printf("Warp disengage through button release\r\n");	//printf to tell us that we have disengaged the warp_drive.
		warp_drive=127;											//neturalize the warp_drive to stop the motor.
		warp_lv=0;												//reset the counter.
	    }
#endif
}
/**************************************************************/

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

void Aim_High(int input_from_camera)
{
#if 0

/*static int warp_drive_position = 0; // substitute for slide pot
static int number_to_reach = 0; // target slide pot position


if (auto_aim_override != 1) // if the override is off
	{
	//number_to_reach = [[[[equation based on input from camera]]]];
	

	if (number_to_reach < warp_drive_position - warp_drive_tolerance) // adjust one direction
		warp_drive = 167;
	else if (number_to_reach > warp_drive_position + warp_drive_tolerance) // adjust in the other direction
		warp_drive = 77;
	else	// otherwise, we've reached the target adjustment
		Aim_High_Handler = 0;  // set to 0 because we're done

	Aim_High_Handler = 1;  //set to 1 because we're not done
	}
*/


static int pulse_count_maker = 0;
static int current_count = 0;


if ((current_count < count_too_high || current_count > count_too_low)&& auto_aim_override != 1) // if the override is off
	{

	if (input_from_camera < CENTER_OF_WINDOW - warp_drive_tolerance) // adjust one direction
		{	
		warp_drive = 167;

		if (pulse_count_maker < 5)                         // Can we make one more lap?
			pulse_count_maker++;
		else 
			current_count++;

			pulse_count_maker = -5;											//add to counter for how long the warp drive is running
		}
	else if (input_from_camera > CENTER_OF_WINDOW + warp_drive_tolerance) // adjust in the other direction
		{
		warp_drive = 77;

		if (pulse_count_maker > -5)                         // Can we make one more lap?
			pulse_count_maker = pulse_count_maker -1;
		else 
			current_count = current_count - 1;

			pulse_count_maker = 5;											//add to counter for how long the warp drive is running
		}

	else	// otherwise, we've reached the target adjustment
		auto_aim_override = 0;  // set to 0 because we're done

	auto_aim_override = 1;  //set to 1 because we're not done
	}



else // override is on   
	auto_aim_override = 0; //set to 0 because we're overriden
#endif
}


/***************************************************************************************************************
	Created by : Daniel Judd
		this function will allow for an increase or decrease in shooter speed as long as
	the correct buttons are pressed 	
*	Parameters: none
	Returns: none                                                                                              *
*   Variables: p4_sw_top for the button on the drive pressed
		pwm01 for the motors
		p3_sw_aux1 for the controller to increase and decrease speed
***************************************************************************************************************/
	
void control_shooter_speed (void)
{
#if 0
	if (p3_sw_top == 1 ) //when the trigger is pressed//
	{	
		if ( p4_wheel > 127 ) //when the analog switch is upward//
		{
			flywheel += 20; //adds speed//
			printf("\rfaster nudge\r");
		}
		
		if ( p4_wheel < 127 )//when the analog swithc is downward
		{
			flywheel -= 20; //subtracts speed//
			printf("\rslower nudge\r");
		}
	}
#endif
}
