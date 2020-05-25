/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/*
 EXAMPLE CONSTANTS:
#define MAXIMUM_LOOPS   5
#define THE_ANSWER      42
#define TRUE            1
#define FALSE           0
#define PI_VAL          3.1415

 EXAMPLE ALIASES:
#define LIMIT_SWITCH_1  rc_dig_int1  (Points to another macro in ifi_aliases.h)
#define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
*/

/* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */

/*******************user defined macros and ailiases*******/
#define gear_out			relay1_fwd
#define gear_in				relay8_fwd
#define loader_in			relay8_fwd
#define loader_out			relay8_rev
//#define foot_up			relay5_fwd
//#define foot_down			relay5_rev

#define PRESSURE_SENSOR 	rc_dig_in03
#define L_encoder			rc_dig_in05
#define R_encoder			rc_dig_in06

#define flywheel			pwm04
#define warp_drive			pwm06
#define gatherer_motor		pwm11

#define WARP_DRIVE_PULSE	10	//THIS CONTROLS HOW MANY LOOPS THE WARP DRIVE WILL RUN FOR.


#define three_position_switch_for_gatherer_motor p4_x


#define GATHER_MOTOR_OUT	0
#define GATHER_MOTOR_IN		254

#define slide_pot		rc_ana_in05

#define FIRE_MODE		p4_y
																	//the flywheel speed for fine adjustments 
#define DEFAULT_DRIVE		    2									//the default drive setting, two stick drive

//the below aliases are for shooting 
#define PREPARE_TO_FIRE 		p4_sw_aux1  						//this says we are ready to prepare to fire
#define BASE_FIRE_SPEED			215									//minium speed to run the flywheel if we have no input from camera

#define ALLOWED_Y_ERROR			2									//maximum allowed rps error
#define ALLOWED_VERT_ERROR		5									//maxium allowed vertical speed error, this is a plus and minus value
#define	ALLOWED_X_ERROR			3									//allowed error for the horizontal aiming
#define X_CENTER				79									//x coordinate on teh camera that we are trying to get to
#define warp_drive_tolerance	10 									// auto-aim function tolerance
#define CENTER_OF_WINDOW		83
#define count_too_high		10
#define count_too_low		-10

#define SENSOR_DEAD 			0
#define FLYWHEEL_TRANS 			1
					
/****************end user defined macros and aliases********/
/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/
/* EXAMPLE DATA STRUCTURE */
/*
typedef struct
{
  unsigned int  NEW_CAPTURE_DATA:1;
  unsigned int  LAST_IN1:1;
  unsigned int  LAST_IN2:1;
  unsigned int  WHEEL_COUNTER_UP:1;
  unsigned int  :4;
  unsigned int wheel_left_counter;
  unsigned int wheel_right_counter;
} user_struct;
*/

/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Default_Routine(void);


/**********User created functions********************/
void User_Autonomous_Code_0(void);
void User_Autonomous_Code_1(void);
void User_Autonomous_Code_2(void);
void User_Autonomous_Code_3(void);
void User_Autonomous_Code_4(void);
char gearSwitch(void);
void aimed_shot(void);
void shoot(char flag);
float rps(char flag);
void acceleration(unsigned char, unsigned char);
void dist_from_acc(void);
//char feetControl(void);
void Six_Lines_of_Gatherer_Code(void);
void control_shooter_speed (void);
void warp_pulse(void);

/*********End user created functions****************/

/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
void Process_Data_From_Local_IO(void);
void Aim_High(int input_from_camera); //auto-aim function


#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
