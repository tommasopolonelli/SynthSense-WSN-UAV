
#ifndef FLASH_MNGM_H_
#define FLASH_MNGM_H_

void FlashMngm_Task (void const * argument);
void Flash_Write_Req (uint8_t *aTx, uint16_t size);
uint16_t Int_Sensors_Stream (uint8_t *out, uint16_t size);


#endif /* FLASH_MNGM_H_ */
