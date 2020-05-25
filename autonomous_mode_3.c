#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"
#include "eeprom.h"

/*
  this defensive autonomous mode moves to another robot, then self-destructs.
*/
void User_Autonomous_Code_3(void)
{
//counter for banner sensor
static int wheel_counter = 0;
static char last_val = 0;
//rotations
char wheel_rot = 0;
//which step it's on
char auto_step = 0;
static int counter = 0;
static char flag = 0;

while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
		pressure_control(PRESSURE_SENSOR);
        /* Add your own autonomous code here. */
        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
	    counter++;
		if(!flag)
		{
			gear_in = 1;
		}
		else
		{
			gear_in = 0;
			gear_out = 0;
		}
		if(counter > 0 && counter < 2000)
		{
			two_stick_drive(0,0,&pwm01,&pwm02,100);
		}
		else
		{
			//in means out. :)
			gatherer_motor = GATHER_MOTOR_IN;
		}
/*
	if(L_encoder != last_val)		//has the value of the encoder changed?		
		{
			wheel_counter++;					//if so, add one to counter
			last_val = L_encoder;		//save the value
		}
		switch(auto_step)
		{
			//step 0:
			case 0:
				//go forward, 75% of full speed
				two_stick_drive(0,0,&pwm01,&pwm02,75);
				if(wheel_counter >= 16)
				{
					wheel_rot++;
					wheel_counter = 0;
				}
				if(wheel_rot == 2)
				{
					auto_step = 1;
				}
				break;
			//step 1:	
			case 1:
				//go forward, 100% full speed
				two_stick_drive(0,0,&pwm01,&pwm02,100);
				if(wheel_counter >= 16)
				{
					wheel_rot++;
					wheel_counter = 0;
				}
				if(wheel_rot == 8)
				{
					auto_step = 2;
				}
				break;
			//step 2:
			case 2:
				//spit out balls
				gatherer_motor = 0;
				break;
		}
*/
		gatherer_motor = GATHER_MOTOR_OUT;
        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}
