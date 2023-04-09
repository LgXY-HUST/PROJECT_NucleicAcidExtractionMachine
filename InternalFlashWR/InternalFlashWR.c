#include "InternalFlashWR.h"

uint32_t SectorUsed=ADDR_FLASH_SECTOR_7;
uint32_t NowDataPtr=ADDR_FLASH_SECTOR_7; //current pointer of data pointing to the address of internal flash 


/**
  * @brief  This function should be executed to unlock the flash when internal flash is to be writen.
  * @retval None
  */
void UnLockInternalFlash(void)
{
	HAL_FLASH_Unlock(); ;	//UnLock InternalFlash
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
 	
}

/**
  * @brief  This function is executed to lock the flash when flash had been written.
  * @retval None
  */
void LockInternalFlash()
{
	 HAL_FLASH_Lock();
}

/**
  * @brief  Given that flash could be write to '0' from '1' ,it is necessary to clear the flash 
						This fuction would clear the operated sector only.
  * @retval A uint8_t type to show wether the sector have been clear. Return '0' means failed while return '1' means OK  
  */
uint8_t Clear_InternalFlash()
{
		uint32_t SectorError=0;
		UnLockInternalFlash();
		FLASH_EraseInitTypeDef pEraseInit;
		pEraseInit.TypeErase = TYPEERASE_SECTORS;
		pEraseInit.Sector = SectorUsed;
		pEraseInit.NbSectors =1 ;
		pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

		if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
		{
				return (0);//Error Occured
		}
		NowDataPtr=SectorUsed;
		return (1);//Erase Done
	
}
/**
  * @brief  Write data into the flash
	* @para bufferptr :pointer to the data buffer array
	* @para num				:number of data to write
	* @para datasize  :size of each data , scaled by byte
  * @retval A uint8_t type to show wether the data have been writen successfuly. Return '0' means failed while return '1' means OK  
  */
uint8_t Write_InternalFlash(uint8_t *bufferptr , int num ,int datasize )
{
	UnLockInternalFlash();
	Clear_InternalFlash();
	for(int i=0 ; i<num ; i++){
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(NowDataPtr), *(uint32_t*)(bufferptr+datasize*i)) != HAL_OK)
			{	
				LockInternalFlash();
				return 0;//Error Happend;
			}
			NowDataPtr+=datasize;
	}
	LockInternalFlash();
	return 1;
}

/**
  * @brief  Write data into the flash
	* @para bufferptr :pointer to the data buffer array
	* @para num				:number of data to read
	* @para datasize  :size of each data , scaled by byte
  * @retval A uint8_t type to show wether the data have been writen successfuly. Return '0' means failed while return '1' means OK  
  */
uint8_t Read_InternalFlash(uint8_t *bufferptr , uint32_t * AddToRead , int num ,int datasize  )
{
		UnLockInternalFlash();
		return *AddToRead;
}

