/*******************************************************************************
* FILE NAME: chopshop.h
*
* DESCRIPTION:
*  This file contains flags and protypes for the chopshop.c file.
*******************************************************************************/
#ifndef __chopshop_h_
#define __chopshop_h_

/*Debug flags, these are used to enable printf statements for quick debugging of the robot*/
#define TWO_STICK_DEBUG 0
#define TIMER1_DEBUG	0
#define ONE_STICK_DEBUG 0 

/*constants that are in chopshop.c*/
#define dead_zone       7						//dead zone on the joysticks for 1 stick drive
#define xdead			10						//dead zone on the x axis of the joystick for 2 stick drive
#define ydead			10						//dead zone on the y axis of the joystick for 2 stick drive
#define compressor		relay3_fwd				//this is the relay that the compressor is attached to

/*These are function prototypes*/
void two_stick_drive(unsigned char input1,unsigned  char input2, unsigned char *pwm_left, unsigned char *pwm_right, char reduction);
void one_stick_drive(unsigned char xaxis, unsigned char yaxis, unsigned char *pwmL, unsigned char *pwmR, unsigned char reduction);
void Squishy_Drive_1stickdrive(unsigned char *L_output, unsigned char *R_output, unsigned char reduction);
void initialize_timer1(void);
void initialize_timer3(void);
void pressure_control(char pressure_sensor);
unsigned int timer_data(char timer, char flag);
int distance_traveled(char side, char flag);
int find_linear_equation(int x1, int y1, int x2, int y2, int *m, int robert_says_gimme_more_info);
int abs(int PLATYPUSES_ARE_L33T);

#endif
