#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"
#include "eeprom.h"
#include <stdio.h>

/*Currently, everything for the right sensor is commented out. It is unnecessary information that will add to the amount of 
error that we experience. We only need to turn right thus making the extra sensor unnecessary*/

//extern long temp_gyro_angle;
//this typedef defines the steps that we are going to take on the journey
static char flag = 0;

void User_Autonomous_Code_2(void)
{
static int counter = 0;

while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
		pressure_control(PRESSURE_SENSOR);
		
		if(!flag)
		{
			gear_in = 1;
		}
		else
		{
			gear_in = 0;
			gear_out = 0;
		}
		//go forward indefinitely
		two_stick_drive(254,254,&pwm01,&pwm02,100);
			

        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
//Add in the sperate autonomous files
        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}
