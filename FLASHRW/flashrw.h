#ifndef __FLASHRW_h
#define __FLASHRW_h

#include "main.h"

extern uint16_t W25QXX_TYPE;					//定义W25QXX芯片型号		   

////////////////////////////////////////////////////////////////////////////////// 

//W25X系列/Q系列芯片列表	   
//W25Q80  ID  0XEF13
//W25Q16  ID  0XEF14
//W25Q32  ID  0XEF15
//W25Q64  ID  0XEF16	
//W25Q128 ID  0XEF17	
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17

////////////////////////////////////////////////////////////////////////////////// 
//指令表
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

//#define __STRUCTURE

#ifndef __STRUCTURE
#define __STRUCTURE
typedef struct 
{
  /* data */
	//47
    __IO uint8_t  run_state;                                
    __IO uint8_t  dir;                                      
    __IO uint8_t  number; 
  	__IO uint8_t  motion_sta; 
    __IO uint8_t  int_count; 
    __IO uint16_t last_accel_delay;  	
    __IO int32_t  step_position;                                                         
    __IO uint32_t g_add_pulse_count;                      
    __IO int32_t  step_delay;                              
    __IO uint32_t decel_start;                            
    __IO int32_t  decel_val;                               
    __IO int32_t  min_delay;                                
    __IO int32_t  accel_count;                             
		__IO int32_t  round_amp;																
		__IO int32_t  round_step;																                       
    __IO uint32_t step_count;                               
    __IO int32_t  rest;                                                                    
    TIM_HandleTypeDef *motor_tim;                           
    __IO uint32_t motor_tim_channel;                        
    
}motorData;
#endif




void W25QXX_Erase_Chip(void)  ;
void W25QXX_Erase_Sector(uint32_t Dst_Addr);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
void 	W25QXX_Init(void);
void 	SaveData(motorData* M1 ,motorData* M2 , motorData* M3 , motorData* M4 , motorData* M5 , uint8_t* buffer);
void  GetData(motorData* M1 ,motorData* M2 , motorData* M3 , motorData* M4 , motorData* M5 , uint8_t* buffer);



#endif