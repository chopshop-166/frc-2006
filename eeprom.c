#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "eeprom.h"

unsigned char eeprom_queue_count = 0;    				//amount of writes that must be done
unsigned char eeprom_queue_full = 0;					//is the eeprom full? 0 is false and 1 is true
unsigned char eeprom_queue_empty = 1;					//is the eeprom empty? 0 is false and 1 is true
unsigned char eeprom_write_slot = 0;						//the slot 
unsigned char eeprom_prep_slot = 0;						//current position in the circular array in preperation function
unsigned char eeprom_data[EEPROM_QUEUE_SIZE];
unsigned int eeprom_address[EEPROM_QUEUE_SIZE];

/***********************************************************************
Created by: 	Steven Shidlovsky - General coding and debugging
				Per - Mentor
Date modified:  1-18-06
Function name:  EEPROM_read
Parameters:     unsigned int address = slot of EEPROM memory to be read
Returns: 		EEDATA               = the data stored in that piece of memory
Purpose: To allows the EEPROM memory to be read. 
Notes: Information on EEPROM is on pg 111 of the controller manual and register information 
	   can be found on pg 112 of the manual.
**********************************************************************/
unsigned char EEPROM_read(unsigned int address)
{
    EECON1bits.EEPGD = 0;										//access the data from EEPROM instead of flash										//access the EEPROM, DO NOT try to recalibrate them
    		 			
	EEADRH = ((unsigned char)(((unsigned int)(address)>>8)&0xFF));	//sets most significant byte to address
	EEADR  = ((unsigned char) (address));							//sets least signigicant byte to address

	EECON1bits.RD = 1;											//Begin reading at the set address, this will be cleared by hardware

	return(EEDATA);												//return the data
}


/***********************************************************************
Created by: 	Steven Shidlovsky - General coding and debugging
				Per - Mentor
Date modified:  1-18-06
Function name:  EEPROM_prep
Parameters:     unsigned int address = slot of EEPROM memory to be read
				unsigned char data   = data to be put into the EEPROM
Returns: 		pf                   = has the function passed or failed? 0 is failed 1 is passed
Purpose: To prepare data to be written to EEPROM 
Notes: Information on EEPROM is on pg 111 of the controller manual and register information 
	   can be found on pg 112 of the manual.
**********************************************************************/
unsigned char EEPROM_prep(unsigned int address, unsigned char data)
{
unsigned char pf;												//used to determine if the function has passed or failed

    if(eeprom_queue_full ==  0)									//is the eeprom_queue full?			
    {															//if not full, then beginning preparing the new data
        eeprom_data[eeprom_prep_slot] = data;					//move data into the eeprom data array
		eeprom_address[eeprom_prep_slot] = address;				//move address into the address slot of the array
 
		eeprom_queue_count++;									//the number of incoming writes in the queue has increased
	
		eeprom_prep_slot++;										//move foward one prep slot in the array so the next data entered will not overwrite the 
																//the previous data							
	
		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		eeprom_prep_slot &= EEPROM_QUEUE_INDEX_MASK;			//see documentation for full details
	
		// is the circular queue now full?
		if(eeprom_write_slot == eeprom_prep_slot)	//is the number of read slots the same as the number of write spots?
		{ 
				eeprom_queue_full = 1;							//then the queue is full 
		}
	
		eeprom_queue_empty = 0;									//it can't be empty now that we added something

	    pf = 1;													//the function has passed
    }
    else														//if the queue was full and had no room for more data
    {
	    pf = 0;													//the function has failed     
	}

   return pf;													//return to caller the result of this function

}

/***********************************************************************
Created by: 	Robert Harris - Debugging
				Sarah Judd - Debugging
				Steven Shidlovsky - General coding and debugging
				Per - Mentor
Date modified:  1-18-06
Function name:  EEPROM_write
Parameters:     none
Returns: 	    none
Purpose: To write data to EEPROM 
Notes: Information on EEPROM is on pg 111 of the controller manual and register information 
	   can be found on pg 112 of the manual.
**********************************************************************/
void EEPROM_write(void)
{

    unsigned char temp_GIEH;									//holds state of global interrupt activation
	unsigned char temp_GIEL;									//holds state of interrupt activation

	if(eeprom_queue_empty == 0)									//is the queue empty? if not, time to write
	{
	    temp_GIEH = INTCONbits.GIEH;							//save the current status of the global interrupt status
		temp_GIEL = INTCONbits.GIEL;							//sace the current status of the interrupt status
	
		EECON1bits.EEPGD = 0;									//Access EEPROM data memory instead of FLASH
		
		EECON1bits.FREE = 0;									//perform a write only

		PIR2bits.EEIF = 0;										//clear the write completion flag

		EEADR = ((unsigned char)(eeprom_address[eeprom_write_slot]));  //Sarah: placing the address in the place in EEPROM that will hold it
		EEADRH = ((unsigned char)(((unsigned int)(eeprom_address[eeprom_write_slot])>>8)&0xFF)); //Sarah: Places the address in the place in EEPROM to hold it                                                                                   //first must be moved back eight places

		EEDATA = eeprom_data[eeprom_write_slot]; 				//Move data to loading platform for EEPROM

		EECON1bits.WREN = 1;									//Allow a write. ONLY keep on during writethis write! IMPORTATNT 

		INTCONbits.GIEH = 0;									//turn off interrupts		
		INTCONbits.GIEL = 0;									//turn off interrupt

		/*The next three lines of code MUST be completed exactly in this order. This is why the interrupts are disabled for this section,
		  to ensure that nothing blocks these lines. */
		EECON2 = 0x55;											//pre-write sequence
		EECON2 = 0xAA;											//pre-write sequence
		EECON1bits.WR  = 1; 									//begin writing to the eeprom, will be cleared in hardware
	 
		INTCONbits.GIEH = temp_GIEH;							//restore global interrupts to their correct states
		INTCONbits.GIEL = temp_GIEL;							//restore interrupts to their correct states

		eeprom_queue_count--;									//the number of items to write is now 1 less
		eeprom_write_slot++;									//move to the next write slot

  
		eeprom_write_slot &= EEPROM_QUEUE_INDEX_MASK;  //Sarah: (gets this because Per explained it, but can't re-explain it)

	if(eeprom_write_slot == eeprom_prep_slot)  					//if the read slot has caught up with the write slot, stop
		{
			eeprom_queue_empty = 1;								//informs computer the queue is now empty
		}

	eeprom_queue_full = 0;										//a write has finished so the queue can not be full

	while(PIR2bits.EEIF == 0);									//wait for the write to be complete

	PIR2bits.EEIF = 0;											//clear the EEPROM complete flag										

	EECON1bits.WREN = 0;										//do not allow writes to EEPROM to continue
	}
}
