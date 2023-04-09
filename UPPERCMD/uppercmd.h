#ifndef __UPPERCMD_H
#define __UPPERCMD_H

#include "main.h"
#include "usart.h"

/***********************************************cmd definition************************************************/
//categoris define
#define		SINGLEMOVE									0xf0
#define		PARADELIVE									0xf1
#define		TESTTABLE										0xf2
#define		JOGMOVECATE									0xe0
#define		CONTINUEJOG									0xe1
#define		PAUSE												0xf4
#define		CONTINUE										0xf5
#define		TEST												0xf6
#define		START												0xf8
#define		MAGNETICSLEEVESAVE					0xfa
#define		MAGNETICRODSAVE							0xfb
#define		TABLEOFFSETSAVE							0xfc
#define		HEATPLATESAVE								0xfd
#define		BAFFLESAVING								0xfe





/**************************************************function definition****************************************/

#define  				SizeOfInfo_Max 				(uint8_t)33   //the size of information union
#define  				PosOfP 					      (uint8_t)32		//subscribe of data array pointer (to the operating position)
#define  				pFlag		 					(uint8_t)30	 	//subscribe of state flag 
#define 				pLenth						(uint8_t)31		//subscribe of parameter which define the effective length of data
#define					LenthOfBuffer					(uint8_t)29		//Size of data buffer



typedef struct UppBuferCPUCMD//CMD from UppBufer CPU that has been translated
{
	uint8_t jogmag;//magtitude of jog
	uint8_t Flag_New;//if new cmd is received (when new cmd is received was set and reseted by checking
										//set when new cmd was recieved and reset at the begining of cmd operating
	uint8_t CmdType;//the function type of movement to be invoked	
										//set when new cmd was recieved and reset at the begining of cmd operating	
	uint8_t CmdPara[3];//parameter with cmd 
												//reset when start excuting
	uint8_t StepPara[4][7];//the parameters of each step
}UppBuferCPUCMD;


typedef struct UppBuferCPUBuffer
{
	uint8_t type;//type of the message from UppBufer computer
	uint8_t UppBufer_Data[LenthOfBuffer]; //count from 1 to SizeOfInfo_Max
	uint8_t Flag;//0x0a表示开始接收，0x0b表示接收完成，0x0e表示还没有接收
	uint8_t Size;//数据大小
	uint8_t pointer;//point to next position recieving data
}UppBuferCPUBuffer;



typedef union UppBuf
{
	UppBuferCPUBuffer Data;
	uint8_t DArray[SizeOfInfo_Max];	
}UppBuf;

void UppBuferCPUInit(UppBuf* upcmd);
void Translator_UppBufCmd(UppBuf * buffer , UppBuferCPUCMD * cmd );
void UppBuferCPUInit(UppBuf* upcmd);
void UsartDataHandler(UppBuf *UppBuf)	;
void UpperInit(void);


#endif