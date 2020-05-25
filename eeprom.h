#ifndef __eeprom_h_
#define __eeprom_h_


#define EEPROM_QUEUE_SIZE 32
#define EEPROM_QUEUE_INDEX_MASK EEPROM_QUEUE_SIZE - 1;
unsigned char EEPROM_read(unsigned int address);
unsigned char EEPROM_prep(unsigned int address, unsigned char data);
void EEPROM_write(void);


#endif
