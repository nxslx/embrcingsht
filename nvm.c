/*
 * https://ioprog.com/2018/05/23/using-flash-memory-on-the-stm32f103/
 *
 * Some microcontrollers have a dedicated Non-Volatile-Memory (NVM) bank
 * for storing calibration data, program settings and so on. The
 * STM32F103C8T6 does not have NVM like this but it’s Flash program
 * memory can be used with care for the same purpose. The Flash memory
 * in this chip is divided into 1kiB sectors and there are 64 if them (0 to 63).
 * The code to erase, write and read a sector is shown below:
 */

#include <stdint.h>
#include <libopencmsis/core_cm3.h>
#include <STM32F103.h>
#include "nvm.h"

int  writeSector(uint32_t Address,void * values, uint16_t size) {
  uint16_t *AddressPtr;
  uint16_t *valuePtr;
  AddressPtr = (uint16_t *)Address;
  valuePtr=(uint16_t *)values;
  size = size / 2;  // incoming value is expressed in bytes, not 16 bit words
  while(size) {
    // unlock the flash 
    // Key 1 : 0x45670123
    // Key 2 : 0xCDEF89AB
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->CR &= ~BIT1; // ensure PER is low
    FLASH->CR |= BIT0;  // set the PG bit
    *(AddressPtr) = *(valuePtr);
    while(FLASH->SR & BIT0); // wait while busy
    if (FLASH->SR & BIT2)
      return -1; // flash not erased to begin with
    if (FLASH->SR & BIT4)
      return -2; // write protect error
    AddressPtr++;
    valuePtr++;
    size--;
  }
  return 0;
}
void eraseSector(uint32_t SectorStartAddress) {
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  FLASH->CR &= ~BIT0;  // Ensure PG bit is low
  FLASH->CR |= BIT1; // set the PER bit
  FLASH->AR = SectorStartAddress;
  FLASH->CR |= BIT6; // set the start bit 
  while(FLASH->SR & BIT0); // wait while busy
}
void readSector(uint32_t SectorStartAddress, void * values, uint16_t size) {
  uint16_t *AddressPtr;
  uint16_t *valuePtr;
  AddressPtr = (uint16_t *)SectorStartAddress;
  valuePtr=(uint16_t *)values;
  size = size/2; // incoming value is expressed in bytes, not 16 bit words
  while(size)
  {
    *((uint16_t *)valuePtr)=*((uint16_t *)AddressPtr);
    valuePtr++;
    AddressPtr++;
    size--;
  }
}
