/*******************************************************************************
* FILE NAME: chopshop.c
*
* DESCRIPTION:
*  This file contains functions that the team has created for use with the robot. These functions have been created
*   to be universal. They can be used anywhere chopshop.h is included. Please feel free to aid to this bank any common functions that are created.
*
*******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"

/***********************************************************************
Created by: 	Robert Harris - Reduction equation
				Sarah Judd - General coding and debugging
				Steven Shidlovsky - General coding and debugging
				Per - Mentor
Date modified:  12-14-2005
Function name:  two_stick_drive
Parameters:     unsigned char input_L      = Input for the left drive motor  -> Value range 0-254 
		        unsigned char input_R      = Input for the right drive motor -> Value range 0-254
		        unsigned char *pwm_left    = Pwm pin left drive motor is on  
		        unsigned char *pwm_right   = pwm pin right drive motor is on
				char reduction             = % speed to run the motors (100 = full 0 = none)
Returns: 		*pwm_left  = Drive value for the left drive motor
		    	*pwm_right = Drive value for the right drive motor
Purpose: This function uses the input values to determine the output values for the PWM's that drive the robot. 
Notes:	 To allow the driver more control, a dead zone in incorperated into the function. This dead zone can be 
		 extended or shortened by redefining the variable "dead_zone" in CHOPSHOP.h. Another feature is the
	     reduction equation. This allows the output values to be reduced to only a fraction of their full strength.
		 For example, using a reduction value of 90 will make all input values be outputted at 9/10 of their normal 
         strength. The main advantage of this is it allows the drivers to gain greater control on the robot. Instead 
         of holding the joystick half way to get 50% power, a button on the joystick can reduce all values the joystick 
         inputs by 50%. This makes it much easier on the drivers. The calculations for this function are done in integer
         math in order to save memory.

		 Equation for when the input is less than 127: -(((-input + 127 ) * reduction)) / 100 + 127
		 Equation for when the input is greater than 127: 127 + (((input - 127) * reduction) / 100)
**********************************************************************/
void two_stick_drive(unsigned char input_L, unsigned char input_R, unsigned char *pwm_left, unsigned char *pwm_right, char reduction)
{

int hold_data;										//holds values for the calculations

    /*For input one, are we in the dead zone? If we are, set the left value to 127, which is neutral*/
    if((input_L >= (127 - dead_zone)) && (input_L <= (127 + dead_zone)))
       *pwm_left = 127;

       /*
       This next set of code is the left reduction equation. This will reduce the input value by the amount
       sent by the user from the parameter, reduction. This allows the driver to have greater control of
       the robot without needing to worry about holding the joystick at other positions than full.
       */
    else if (input_L < 127)
	 {
	    hold_data = -1 * input_L + 127;
	    hold_data = hold_data * reduction * -1;
	    hold_data /= 100;
	    hold_data += 127;
		*pwm_left = hold_data;
	 }
	 else if (input_L > 127)
	      {
		      hold_data = input_L - 127;
		      hold_data *= reduction;
		      hold_data /= 100;
		      hold_data += 127;
	          *pwm_left = hold_data;
	      }

    /*For input two, are we in the dead zone? If we are, set the left value to 127, which is neutral*/
    if(input_R >= 127 - dead_zone && input_R <= 127 + dead_zone)
       *pwm_right = 127;

       /*
       This next set of code is the right reduction equation. This will reduce the input value by the amount
       sent by the user from the parameter, reduction. This allows the driver to have greater control of
       the robot without needing to worry about holding the joystick at other positions than full.
       */
    else if (input_R < 127)
	 {
	    hold_data = -1 * input_R + 127;
	    hold_data = hold_data * reduction * -1;
	    hold_data /= 100;
	    hold_data += 127;
		*pwm_right = hold_data;
	 }
	 else if (input_R > 127)
	      {
		      hold_data = input_R - 127;
		      hold_data *= reduction;
		      hold_data /= 100;
		      hold_data += 127;
	    	  *pwm_right = hold_data;	
	      }
/*
This statement is present for easy debugging. In chopshop.h, there is a section that contains this flag, should the function need
to be debugged for any reason, a preliminary examination can be done by turning on the flag. If the flag is off, this section will
not be compiled.
*/
#if TWO_STICK_DEBUG
    printf("\rInputs: %d / %d. Reduction: %d. Outputs-> left:%d right:%d.", input_L, input_R, reduction,*pwm_left,*pwm_right);
#endif
}

/********************  uber_stick_drive function  *********************** 
Created by:    Robert Harris - General Coding
Date modified: November 22, 2005
Function name: uber_one_stick_drive
Parameters:    xaxis   = input to determine if turn will occur
			   yaxis   = input to determine the foward and reverse motion of robot
			   *pwmL   = pwm operating the left drive motor
			   *pwmR   = pwm operating the right drive motor  
Returns:       *pwmL   = left drive motor value (as a pointer)
			   *pwmR   = right drive motor value (as a pointer)
Purpose:       This allows the robot to be controlled with one joystick. 
Notes:         This function causes the robot to turn on its center when the user moves the x-axis of the joystick out of the deadzone.
				It will also be able to move the robot foward and reverse. The reduction equation is also used in this function.
				Should the function need to be debugged, set the one_stick_debug flag in the .h file to 1
Equation for when the input is less than 127: -(((-input + 127 ) * reduction)) / 100 + 127
Equation for when the input is greater than 127: 127 + (((input - 127) * reduction) / 100)			   
*/

void uber_one_stick_drive(unsigned char xaxis, unsigned char yaxis, unsigned char *pwmL, unsigned char *pwmR, unsigned char reduction)
{

int hold_data;
													//holds values for calculations

/*If the x-axis is less than 127 - xdead, then the robot will turn left*/
    if (xaxis < 127 - xdead)
	{
		/*this next section will perform the reduction equation*/
		hold_data = -1 * xaxis + 127;
	    hold_data = hold_data * reduction * -1;
	    hold_data /= 100;
	    hold_data += 127;

		/*This section assigns left drive the hold data value and the right drive value the oppisite value*/
		*pwmR = (127 - hold_data) + 127;
		*pwmL = hold_data;
	 }
     /*If the xaxis is greater than 127 + xdead, then the robot will turn right*/
    else if (xaxis  > 127 + xdead)
	     {
		      /*this next section will perform the reduction equation*/
  		      hold_data = xaxis - 127;
		      hold_data *= reduction;
		      hold_data /= 100;
		      hold_data += 127;
		      /*This section assigns right drive the hold data value and the left drive value the oppisite value*/
	          *pwmR = 127 - (hold_data - 127);
	          *pwmL = hold_data;
	     }
    	else													//This next section is for foward motion
	     {
			if (yaxis < 127 - ydead || yaxis > 127 + ydead)		//will foward motion occur?
		    {
	        	if (yaxis < 127 - ydead)						//is the yaxis value less than 127 - ydead?
	         	{
	                 /*this next section will perform the reduction equation*/
					 hold_data = -1 * yaxis + 127;
	                 hold_data = hold_data * reduction * -1;
	                 hold_data /= 100;
	                 hold_data += 127;
	            }
	 			else if (yaxis > 127 + ydead)					//is the yaxis value greater than 127 + ydead
	      			 {
						/*this next section will perform the reduction equation*/
		  				hold_data = yaxis - 127;
		  				hold_data *= reduction;
		  				hold_data /= 100;
		  				hold_data += 127;
		             }
					else										//else the drive must be nuetral currently
					  {
						  *pwmL = 127;
						  *pwmR = 127;
					  }
	
	 			*pwmL = hold_data;								//assign hold_data value to pwmL
				*pwmR = hold_data;								//assign hold_data value to pwmR
			}
    	}
/*This is the debug statement for the one stick drive. Should it be needed, activate it in chopshop.h*/
#if ONE_STICK_DEBUG
    printf("\rInputs x/y: %d / %d. Reduction: %d Outputs-> left:%d right:%d.", xaxis, yaxis, reduction,*pwmL,*pwmR);
#endif
}


/***********************************************************************
Created by: 	Robert Harris - General coding and debugging
				David "Squishy" Zaharee - Original Concept
Date modified:  1-27-2006
Function name:  Squishy_Drive_1stickdrive
Parameters:     *L_output, *R_output, reduction
Returns: 		(As pointers) L_output, R_output
Purpose: 		This function allows for intuitive 1-stick drive
Notes:	 		reduction (must be 0 to 100)
**********************************************************************/
void Squishy_Drive_1stickdrive(unsigned char *L_output, unsigned char *R_output, unsigned char reduction)
{
	
char x;							// x-axis in -127 to 127 range	
char y;							// y-axis in -127 to 127 range	
unsigned char abs_x;			// absolute value of x
unsigned char abs_y;			// absolute value of y
char untranslated_output_L;		// Left output in -127 to 127 range
char untranslated_output_R;		// Right output in -127 to 127 range
unsigned char case_number;		// Holding Value created by branches of nested if function tree used for later switch statement
int hold_data;					// Holding Value for reduction equations
int x_is_backwards_so_this_extra_variable_is_necessary;
	
	x_is_backwards_so_this_extra_variable_is_necessary = (((p1_x - 127)*-1) + 127);



	 /*For input one, are we in the dead zone? If we are, set the left value to 127, which is neutral*/
	    if((p1_x >= (127 - dead_zone)) && (p1_x <= (127 + dead_zone)))
	       hold_data = 127;
	
	       /*
	       This next set of code is the x reduction equation. This will reduce the input value by the amount
	       sent by the user from the parameter, reduction. This allows the driver to have greater control of
	       the robot without needing to worry about holding the joystick at other positions than full.
	       */
 		else if (x_is_backwards_so_this_extra_variable_is_necessary < 127)
			 {	
			     hold_data = -1 * x_is_backwards_so_this_extra_variable_is_necessary + 127;
			     hold_data = hold_data * reduction * -1;
			     hold_data /= 100;
	 	         hold_data += 127;
   	 		 }
		 else if (x_is_backwards_so_this_extra_variable_is_necessary > 127)
		      {
			       hold_data = x_is_backwards_so_this_extra_variable_is_necessary - 127;
			       hold_data *= reduction;
			       hold_data /= 100;
			       hold_data += 127;
	 	      }
	
	x = (hold_data - 127); //sets x to a -127 to +127 range
	    /*For input two, are we in the dead zone? If we are, set the left value to 127, which is neutral*/
	    if(p1_y >= 127 - dead_zone && p1_y <= 127 + dead_zone)
	       hold_data = 127;
	
	       /*
	       This next set of code is the y reduction equation. This will reduce the input value by the amount
	       sent by the user from the parameter, reduction. This allows the driver to have greater control of
	       the robot without needing to worry about holding the joystick at other positions than full.
	       */
	     else if (p1_y < 127)
		      {
		          hold_data = -1 * p1_y + 127;
		          hold_data = hold_data * reduction * -1;
		          hold_data /= 100;
		          hold_data += 127;
			  }
		 else if (p1_y > 127)
	 	      {
		          hold_data = p1_y - 127;
	 	          hold_data *= reduction;
	 	          hold_data /= 100;
	 	          hold_data += 127;
     	      }
	
	y = (hold_data - 127); //sets y to a -127 to +127 range

	abs_x = abs(x); //uses absolute value function to find variable for later mathematics
	abs_y = abs(y); //ditto


	if (y > 0)  // this nested if statement determines which eigth of the co-ordinate plane the joystick is in, and sets case_number accordingly
		{		// so that the correct case is used in the following switch statement. These eigth sections of the co-ordinate plane are found by
		if (x > 0)     // the quadrants in half diagonally through the orgin
			{
			   if (abs_x > abs_y)
			      case_number = 1;
			   else
			      case_number = 2;
			}
		else
			{
			   if (abs_y > abs_x)
			      case_number = 3;
			   else
			      case_number = 4;
			}
		}
	else
		{
		if (x < 0)
			{
			   if (abs_x > abs_y)
			      case_number = 5;
			   else
			      case_number = 6;
			}
		else
			{
			   if (abs_y > abs_x)
			      case_number = 7;
			   else
			      case_number = 8;
			}
		}

	switch(case_number)  // These cases are the various equations neccessary given the above determined position of the joystick.
	{
	
	case 1:	
		untranslated_output_L = abs_x;
		untranslated_output_R = -1*(abs_x-abs_y);
	break;	
	case 2:
		untranslated_output_L = abs_y;
		untranslated_output_R = abs_y-abs_x;
	break;
	case 3:
		untranslated_output_L = abs_y-abs_x;
		untranslated_output_R = abs_y;
	break;
	case 4:
		untranslated_output_L = -1*(abs_x-abs_y);
		untranslated_output_R = abs_x;
	break;
	case 5:	
		untranslated_output_L = -1*abs_x;
		untranslated_output_R = (abs_x-abs_y);
	break;
	case 6:
		untranslated_output_L = -1*abs_y;
		untranslated_output_R = -1*(abs_y-abs_x);
	break;	
	case 7:
		untranslated_output_L = -1*(abs_y-abs_x);
		untranslated_output_R = -1*abs_y;
	break;		
	case 8:
		untranslated_output_L = (abs_x-abs_y);
		untranslated_output_R = -1*abs_x;
	break;
	default:
		untranslated_output_L = 0;
		untranslated_output_R = 0;
	break;
	}

	*L_output = 127 + untranslated_output_L; //this converts the -127 to +127 range back to 0 to 254, and sets the pointer to it
	*R_output = 127 + untranslated_output_R; //ditto

	return;
}

/***********************************************************************
Created by:    Steven Shidlovsky - General coding and debugging
	       	   Sarah Judd		 - Revision to 1/4 of a second
			   Per - Mentor
Date modified: 2-10-06
Function name: initialize_timer1
Parameters:    none
Returns:       none
Purpose:       This enables timer1 on the FIRST PIC controller.
Notes:         This timer, timer1, will count in real time in increments of a second. This requires extra code to be put into the interrupt
	       handler. In the 2005 code, the function InterruptHandlerLow in user_routines_fast.c is where the interrupt handler is.
	       The following is what must be added to ensure that it will count in seconds.

 if (PIE1bits.TMR1IE && PIR1bits.TMR1IF)		//is the interrupt enabled and the flag triggered
		{		
		PIE1bits.TMR1IE = 0;					//turn interrupt off, this prevents calling it again while the following code is executed
		PIR1bits.TMR1IF = 0;					//flag reset
		timer_data(1, 0);
		PIE1bits.TMR1IE = 1;					//enable interrupt again
       }

The timer information is held in the function timer_data. This allows it to pass between the files without
issues. For more information on timer_data, see that function.
**********************************************************************/
void initialize_timer1(void)
{

    //next two lines set prescale to 1:4.  This is what makes it quarter seconds
	T1CONbits.T1CKPS1 = 1;		//sets prescale
    T1CONbits.T1CKPS0 = 0;		//sets prescale

    T1CONbits.TMR1CS = 0;		//use the internal clock

    T1CONbits.T1OSCEN = 0;		//oscillator is off

    T1CONbits.T1SYNC = 0;		//use internal clock, no external clock input

    T1CONbits.RD16 = 1;			//operating in 16 bit mode

    PIR1bits.TMR1IF = 0;		//interrupt flag set to 0

    PIE1bits.TMR1IE = 1;		//interrupt is on

    T1CONbits.TMR1ON = 1;		//timer1 is now on
    /*
    This is a debug statment to ensure that this function is working properly. If debugging needs to
    occur, the flag can be set in chopshop.h
    */
#if TIMER1_DEBUG
    printf("\rPrescale: T1CONbits.T1CKPS1 %d T1CONbits.T1CKPS0 %d\rInternal clock: T1CONbits.TMR1CS %d\rOscillator: T1CONbits.T1OSCEN %d\rBit mode: T1CONbits.RD16 %d\rInterrupt flag: PIR1bits.TMR1IF %d\rInterrupt state (on/off): PIE1bits.TMR1IE %d\rReady for interrupt: T1CONbits.TMR1ON %d\r", T1CONbits.T1CKPS1, T1CONbits.T1CKPS0, T1CONbits.T1OSCEN, T1CONbits.T1SYNC, T1CONbits.RD16, PIR1bits.TMR1IF, PIE1bits.TMR1IE, T1CONbits.TMR1ON );
#endif

}

/***********************************************************************
Created by:    Robert Harris - General coding and debugging
Date modified: 2-10-06
Function name: initialize_timer3
Parameters:    none
Returns:       none
Purpose:       This enables timer1 on the FIRST PIC controller.
Notes:         This timer, timer1, will count in real time in increments of a second. This requires extra code to be put into the interrupt
	       handler. In the 2005 code, the function InterruptHandlerLow in user_routines_fast.c is where the interrupt handler is.
	       The following is what must be added to ensure that it will count in seconds.

static char hold3 = 0;							//holds know of interrupt hits gotten

	else if(PIE2bits.TMR3IE && PIR2bits.TMR3IF)
	{
		PIE2bits.TMR3IE = 0; //turn interrupt off to prevent it from being recalled
		PIR2bits.TMR3IF = 0; //reset the flag to 0
		hold3++; 			 //adds one to the hold
		if (hold3 == 20)		 //is hold equal to 20
		{
			timer_data(3,0); //call timer_data function to add a second
			hold3 = 0;        //reset hold to 0 so the next second can be counted
		}
		PIE2bits.TMR3IE = 1; // enable interrupt
	}

The timer information is held in the function timer_data. This allows it to pass between the files without
issues. For more information on timer_data, see that function.
**********************************************************************/
void initialize_timer3(void)
{

    T3CONbits.RD16 = 1; 	//16 bit mode
    T3CONbits.T3CCP2 = 0;	// timer1 and timer2 is CCP source
    T3CONbits.T3CCP1 = 0; 	// ditto
    T3CONbits.T3CKPS1 = 1; 	// 1:8 prescale value
    T3CONbits.T3CKPS0 = 1; 	// ditto
    T3CONbits.T3SYNC = 0; 	//not using external clock
    T3CONbits.TMR3CS = 0; 	// ditto
    T3CONbits.TMR3ON = 1; 	//timer 3 enabled

    PIR2bits.TMR3IF = 0;	//set flag to 0
    PIE2bits.TMR3IE = 1;	//enable the interrupt

}

/***********************************************************************
Created by:     Steven Shidlovsky - General coding and debugging
				Per - Mentor
Function name:  timer_data
Parameters:     char timer = timer to be observed
		        char flag  = flag to determine option
Returns: The time held for the counter set in parameter
Purpose: To hold the current time recorded in seconds for the timers on the PIC controller.
Addition Information: This function allows the data from the timer to be used in any file of the program. To get the return, just set the flag to 1. If user does not seek to add to the timer counters, set timer to 0. This function is to be used with all timers.
**********************************************************************/

unsigned int timer_data(char timer, char flag)
{
static unsigned char hold1 = 0;			 	//hold for timer1	
static unsigned int quarter_seconds = 0;	//stores the amount of quarter seconds for timer1
static unsigned int timer3_seconds = 0; 	//stores the amount of seconds for timer3

    switch(timer)						//determine what timer the function will be working on
    {
    case 1:								//timer1 is selected.  This counts quarter seconds
		/*This nested if statement is used to determine the operation to be performed on the timerX seconds. It supports adding a second, returning the seconds stored,
 		or resetting the timer. The user decides the option by setting the flag to the correct value.*/ 
		if (!flag)
		{
 			//this piece has been called
			hold1++;
			//when its been called 10 times, a quarter second has passed
			if (hold1 == 10)
			{
				quarter_seconds++;
				hold1 = 0;          //reset hold
				//The next two lines are for debugging. Uncomment them if u need to debug the timer
			/*	if (quarter_seconds%4 == 0)
					printf("quarter seconds %d\r",quarter_seconds);
			*/
			}
		}
		else if (flag)						
  			{
 				return quarter_seconds;
			}
			else if (flag == 2)
				{
					quarter_seconds = 0;
				}
	break;
	case 3:
	/*This nested if statement is used to determine the operation to be performed on the timerX seconds. It supports adding a second, returning the seconds stored,
 		or resetting the timer. The user decides the option by setting the flag to the correct value.*/ 
		if(flag == 0)						
 		{
   		    timer3_seconds++;
		}
		else if (flag)
			 {
  			     return timer3_seconds;
			 }
			else if(flag == 2)
				 {
 				     timer3_seconds = 0;
				 } 	     
	break;
	}	
}

/********************distance_traveled*********************************
Created by:     Steven Shidlovsky - General coding and debugging
Function name:  distance_traveled
Parameters:     char flag  = flag to determine when to add a click
				char side  = determine the side to be added or recalled
Returns: The number of clicks that have occurred
Purpose: To hold the number of clicks that have occurred.
Addition Information: side R = 0 side L = 1
**********************************************************************/
int distance_traveled(char side, char flag)
{

static unsigned int clicksR = 0;	//used to store the number of clicks that have occurred
static unsigned int clicksL = 0;	//used to store the number of clicks for left side
   
   if(flag)							//is the flag true?
   {
      switch(side)					//determine the side being used
	  {
	  case 0:						//right side
	     clicksR++;					//add one to clicks for right
	  break;							//leave statement
	  case 1:						//left side
		 clicksL++;					//add to the left side clicks
	  break;							//leave statement
	  }
  }
  else								//else a return is needed
  {
     switch(side)					//determine the side being used
	 {
	 case 0:						//right side
	    return clicksR;				//return clicksR
	 break;							//leave statement
	 case 1:						//left side
		return clicksL;				//return clicksL
	 break;							//leave statement
	 }   
  }
}

/*******************pressure_control********
	Created by: 		Eric Finn
	Date modified: 		December 05, 2005
	Function name:  	pressure_control
	Parameters:     	pressure_sensor		= sensor that informs us when max pressure is reached
	Returns:		 	compressor          = turns compressor on and off
	Purpose: 			Keeps compressor from destroying itself
	Notes:				compressor returned as global and must be set in chopshop.h
******************************************/
void pressure_control(char pressure_sensor)
{
    if(pressure_sensor == 0) compressor = 1; 	//Turns compressor on when max pressure has not yet been reached.
    else compressor = 0;						//Turns compressor off when max pressure has been reached.
    /*Often, the teams finds that that compressor or related components do not work. The usual place to assign blame is the software team.
      For this reason, the following printf will solve any such problems by either proving the software is not working or that it is an
      electrical problem. To have the printf compiled, uncomment it and recompile*/
    //printf("\rsensor %d, compressor %d", pressure_sensor, compressor);

}
/********************  Robert's Linear Equation Finder Function  *********************** 
Created by:    Robert Harris - General Coding
	           Per - Mentor
Function name: find_linear_equation
Parameters: This function will return b as 42 and set m to 42 if it is given two points of 
            a vertical line.
Input:		   x1, y1, x2, y2, &m, robert_says_gimme_more_info	
Returns:       b, m (as a pointer)
Purpose:       This finds the constants for a linear equation. 
Notes:         1 point and either the slope or another point are required. The slope "m" is passed to this function as a pointer, and if two points are given, the slope will be edited by the function.
			   The y-intercept "b" will be the value returned by this function. The value "robert_says_gimme_more_info" should be passed "42" if two points are known, or "-42" if a point and the slope 
			   are known. If any other value is passed to "robert_says_gimme_more_info", then the function will not edit "m", and will return "42". If calculating the slope would cause division by zero,
			   This function will automatically return 42 as b and set m to 42.
*/
int find_linear_equation(int x1, int y1, int x2, int y2, int *m, int robert_says_gimme_more_info)
{

int b;  //y intercept of the linear equation

	/* based on what information we have (if we have two points: 42, when we have one point and slope: -42)
       figure out the rest of the numbers needed for the equation                                         */
    switch (robert_says_gimme_more_info)
	{
	case 42:    					  // two points

		if ((x1 - x2) == 0)	// avoid division by zero in slope formula
			{
			//set m to the default "this isn't really going to work" value
			*m = 42;
			// return this value
			return 42;
			//don't move on to the next case, which figures out what b is.  There is no m
			break;
			}
			
		*m = (y1 - y2) / (x1 - x2);

		//if we've gotten this far, it means we've found m, and want to go on to the next case, not break    

// NO BREAK HERE
          
	case -42:						// 1 point and slope
		b = y1 - (*m*x1);
		return b;
		break;

	default:
		return 42;
		break;
	}


}

/***********************************************************************
Created by: 	Robert Harris - General coding and debugging
Date modified:  1-27-2006
Function name:  abs
Parameters:     PLATYPUSES_ARE_L33T = number to find the absolute value of
Returns: 		absolute value of input
Purpose: 		This function finds absolute value
Notes:	 		None
**********************************************************************/

int abs(int PLATYPUSES_ARE_L33T) 
{
    if (PLATYPUSES_ARE_L33T < 0) 						//is input less than 0?
        PLATYPUSES_ARE_L33T = -1*PLATYPUSES_ARE_L33T;   //if so, make it positive

    return PLATYPUSES_ARE_L33T; 						// return the absolute value
}

/********************  Robert's Series of Randomly Useful Equation Macros  *********************** 
Created by:    Robert Harris - General Coding
	           Per - Mentor
Macro names:   macro_linear_equation(m, x, b), macro_quadratic_equation_standard_form(a, x, b, c), macro_quadratic_equation_vertex _form(a, x, h, k),
			   macro_quadratic_equation_intercept_form(a, x, p, q), macro_cubic_equation(a, x, b, c, d), macro_quartic_equation(a, x, b, c, d, e)
Parameters:    m, x, b, a, c, d, and e are assumed to be integers 	
Returns:       Solution to named equation
Purpose:       Simple way to decrease code space
*/

#define macro_linear_equation(m, x, b) ((m*x) + b)

#define macro_quadratic_equation_standard_form(a, x, b, c) (a*(x*x) + b*x + c)

#define macro_quadratic_equation_vertex _form(a, x, h, k) (a*(x-h)*(x-h) + k)

#define macro_quadratic_equation_intercept_form(a, x, p, q) (a*(x-p)*(x-q))

#define macro_cubic_equation(a, x, b, c, d) (a*(x*x*x) + b*(x*x) + c*x + d)

#define macro_quartic_equation(a, x, b, c, d, e) (a*(x*x*x*x) + b*(x*x*x) + c*(x*x) + d*x + e)
