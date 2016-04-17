#ifndef EEPROM_H_H
#define EEPROM_H_H

#include <EEPROM.h>

//#define USEEEPROM

#define START_ADDRESS 0x0A

struct EEpromData
{
  float KA_P;
  float KA_D;
  float KP_P;
  float KP_I;
  float K_Base;
};


void WritePIDintoEEPROM(struct EEpromData *SavingData)
{
  #ifdef USEEEPROM
  
  byte cnt = sizeof(struct EEpromData);
  byte *pt = (byte*)SavingData;
  int start = START_ADDRESS;
  for(int i = 0; i < cnt; i++)
  {
     EEPROM.write(start++,*pt);
     pt++;
     //Serial.println(*pt);
  } 
  #endif
}

void ReadFromEEprom(struct EEpromData *ReadingData)
{
    #ifdef USEEEPROM
  
    byte cnt = sizeof(struct EEpromData);
    byte *pt = (byte*)ReadingData;
    int start = START_ADDRESS;
    for(int i = 0; i < cnt; i++)
    {
       *pt = EEPROM.read(start++);
       pt++;
    } 
    #endif
}
#endif
