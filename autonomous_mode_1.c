#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"
#include "eeprom.h"
#include <stdio.h>

/*Currently, everything for the right sensor is commented out. It is unnecessary information that will add to the amount of 
error that we experience. We only need to turn right thus making the extra sensor unnecessary*/

extern long temp_gyro_angle;
//this typedef defines the steps that we are going to take on the journey
typedef enum{MOVE_FOWARD_SLOW,
			 MOVE_FOWARD,
			 SLOW_DOWN,
			 TURN,
			 PREPARE_TO_SHOOT,	
			 FIRE,
		     STAND_BY
			}status_flags;

status_flags stat = MOVE_FOWARD_SLOW;		//set the stat flag to the beginning of the journey

/*
These tables are the amount of revlotions that are needed to move foward the correct distance. Each acceleration and
deacceleration is split into steps to prevent the wheels from sliding out
The journey steps are     MFS , MF, SD,  TURN, */
int left_journey[5] = {   5,    48,  58, 75,  120};
//int right_journey[5] = {5,    75,  82, 85,  170};
int JOURNEY_POINTER = 0;

void User_Autonomous_Code_1(void)
{
static char last_val_left = 0;				//records last value of the banner sensor on left drive wheel
static char last_val_right = 0;				//records last value of the banner sensor on right drive wheel
static int counter_L = 0;					//counts the number of clicks from the left banner sensor
static int counter_R = 0;					//counts the number of clicks from the right banner sensor
static int counter = 0;
static int last_val = 0;

//timer_data(1,2);
while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
		pressure_control(PRESSURE_SENSOR);
		
		//printf("rotations r: %d Rotations L: %d", counter_R, counter_L);
#if 0
	if (timer_data(1,1) > 38)
			{
				if(stat == FIRE)
				{
					shoot(0);
				}
				stat = STAND_BY;
			}
#endif
		User_Byte6 = counter_L; 
		/*the following code is used to determine if we have a count on the banner sensors*/
		if(L_encoder != last_val_left)		//has the value of the encoder changed?		
		{
			counter_L++;					//if so, add one to counter
			last_val_left = L_encoder;		//save the value
		}
/*
		if(R_encoder != last_val_right)		//has the value of the encoder changed?
		{
			counter_R++;					//if so, add one to the counter
			last_val_right = R_encoder;		//record the value of the encoder
		}
*/		
		if(stat == FIRE)
		{
			counter++;
		}
		flywheel = BASE_FIRE_SPEED;
		/*This determines where in the journey we are currently at. This will take us through each step and 
		stop the robot when we have reached the end of the journey*/
		switch(stat)
		{
		case MOVE_FOWARD_SLOW:
		//this step is to begin moving the robot forward. This is done to prevent the wheels from skidding
		two_stick_drive(170, 170, &pwm01, &pwm02, 100);			//set the drive motors to move foward	
		if(counter_L >= left_journey[JOURNEY_POINTER])			//have we reached our destination?								// || (counter_R >= right_journey[JOURNEY_POINTER]))
			{
				JOURNEY_POINTER++;									//add one to the journey pointer
				stat = MOVE_FOWARD;									//on to the next step of the journey
			}   
		break;
		case MOVE_FOWARD:
			two_stick_drive(190, 190, &pwm01, &pwm02, 100);			//power on to full strength
			if(counter_L >= left_journey[JOURNEY_POINTER])			//have we reached our destination?								// || (counter_R >= right_journey[JOURNEY_POINTER]))
			{
				JOURNEY_POINTER++;									//add one to the journey pointer
				stat = SLOW_DOWN;									//on to the next step of the journey
			}   
		break;
		case SLOW_DOWN:

			two_stick_drive(170, 170, &pwm01, &pwm02, 100);			//start slowing down
			if(counter_L >= left_journey[JOURNEY_POINTER])			//have we reached our destination?								// || (counter_R >= right_journey[JOURNEY_POINTER]))
			{
				JOURNEY_POINTER++;									//add one to the journey pointer
				stat = TURN;										//on to the next step of the journey
			}
		break;   
		case TURN:
			two_stick_drive(95, 170, &pwm01, &pwm02, 100);			//start slowing down
			if(counter_L >= left_journey[JOURNEY_POINTER])			//have we reached our destination?								// || (counter_R >= right_journey[JOURNEY_POINTER]))
			{
				JOURNEY_POINTER++;									//add one to the journey pointer
				stat = FIRE;										//on to the next step of the journey
			}
		break;
		case FIRE:
			two_stick_drive(127, 127, &pwm01, &pwm02, 100);			//start slowing down
			//counter++;
				flywheel = BASE_FIRE_SPEED;
			
			if(counter % 32 == 0)
			{	
				shoot(1);
				last_val = counter;
			}
			
			 if(!(counter % (10 + last_val)))
				{
					shoot(0);
					last_val += 2;
				}

			if(counter > 200)
			{
				shoot(0);
				stat = STAND_BY;
			}
		break;
		case STAND_BY:
			two_stick_drive(127, 127, &pwm01, &pwm02, 100);		//stop the robot
			flywheel = 127;
			shoot(0);
		break;
		}
		printf("\rcounter_l %d case %d number to get %d counter %d", counter_L, stat, left_journey[JOURNEY_POINTER], counter);


        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
//Add in the sperate autonomous files
        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}
