#include "uppercmd.h"

UppBuf 								UppBuferCPU1;
UppBuferCPUCMD				UPPCMD1;

void UppBuferCPUInit(UppBuf* upcmd)
{
	for(int i=0;i<SizeOfInfo_Max;i++){
		upcmd->DArray[i]=0;
	}
	upcmd->DArray[pFlag]=0x0e;
}

void Translator_UppBufCmd(UppBuf * buffer , UppBuferCPUCMD * cmd ) //translate the data received
{
	if(buffer->Data.type==SINGLEMOVE){
		cmd->CmdType = buffer->Data.type; 
		cmd->CmdPara[0]= buffer->Data.UppBufer_Data [0] ; 
	}else if(buffer->Data.Flag==PARADELIVE){
		for(int i = 0 ; i < 28 ; i++){
			cmd->StepPara[i%7][i] =  buffer->Data.UppBufer_Data[i];
		}
	}else if(buffer->Data.type==JOGMOVECATE){
		cmd->CmdType = buffer->Data.type ; 
		cmd->CmdPara[0]=buffer->Data.UppBufer_Data [1];
	}else if(buffer->Data.type==CONTINUEJOG){
		cmd->CmdType = buffer->Data.type; 
		cmd->CmdPara[0]=buffer->Data.UppBufer_Data [0];
		cmd->CmdPara[1]=buffer->Data.UppBufer_Data [1];	
	}else if(buffer->Data.type==CONTINUE){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==START){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==MAGNETICSLEEVESAVE){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==MAGNETICRODSAVE){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==TABLEOFFSETSAVE){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==HEATPLATESAVE){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==BAFFLESAVING){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==PAUSE){
		cmd->CmdType=buffer->Data.type;
	}else if(buffer->Data.type==TEST){
		cmd->CmdType=buffer->Data.type;
		cmd->CmdPara[0]=buffer->Data.UppBufer_Data[0];
	}else if(buffer->Data.type==TESTTABLE){
		cmd->CmdType=buffer->Data.type;
		cmd->CmdPara[0]=buffer->Data.UppBufer_Data[0];
		cmd->CmdPara[1]=buffer->Data.UppBufer_Data[1];			
	}

}



/********************************************RECIEVE*******************************************/

//串口数据，0x0a
void UsartDataHandler(UppBuf *UppBuf)	
{
	if((UppBuf->DArray[pFlag]==0x0a))//还没有接收完
	{
		if(UppBuf->DArray[PosOfP]==0)//received the data tpye
		{
			if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==SINGLEMOVE){//single action cmd
				UppBuf->DArray[pLenth]=1;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==PARADELIVE) {//parameter delivering
				UppBuf->DArray[pLenth]=28;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==JOGMOVECATE) {//jog category
				UppBuf->DArray[pLenth]=2;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==CONTINUEJOG) {//continuous jog category
				UppBuf->DArray[pLenth]=3;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==PAUSE) {//PAUSE
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==CONTINUE) {//CONTINUE
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==MAGNETICSLEEVESAVE) {//Magnetic rod sleeve position Saving
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==MAGNETICRODSAVE) {//Magnetic rod position Saving
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==TABLEOFFSETSAVE) {//Turtable Origin Offset Saving
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==HEATPLATESAVE) {//Heating Plate Highest Place save
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==BAFFLESAVING) {//Baffle saving
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==START) {//START
				UppBuf->DArray[pLenth]=0;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==TEST) {//START
				UppBuf->DArray[pLenth]=1;
			}else if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==TESTTABLE) {//START
				UppBuf->DArray[pLenth]=2;
			}
			UppBuf->DArray[PosOfP]++;//update pointer
		}
		else if(UppBuf->DArray[PosOfP]<=UppBuf->DArray[pLenth])//havn`t finish recieving
		{
			UppBuf->DArray[PosOfP]++;//continue recieving
		}
		else if(UppBuf->DArray[PosOfP]==UppBuf->DArray[pLenth]+1)
		{
			if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==0xff)//receiving done
			{
				UppBuf->DArray[pFlag]=0x0b;//finish
			}
			else{
				UppBuf->DArray[pFlag]=0xee;//error			
			}
		}			
	}
	else if(UppBuf->DArray[pFlag]==0x0e)//recieving havn`t been started yet
	{
		if(UppBuf->DArray[UppBuf->DArray[PosOfP]]==0x00)//recieve start signal
		{
			UppBuf->DArray[pFlag]=0x0a;//start
		}
		else
		{
			UppBuf->DArray[pFlag]=0xee;//error
		}			
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{	
	if(huart->Instance==USART1)
	{
		UsartDataHandler(&UppBuferCPU1);
		if(UppBuferCPU1.DArray[pFlag]==0xee)
		{
			UppBuferCPU1.DArray[pFlag]=0x0e;
			UppBuferCPU1.DArray[pLenth]=0;
			UppBuferCPU1.DArray[PosOfP]=0;
		}else if(UppBuferCPU1.DArray[pFlag]==0x0b)
		{
			Translator_UppBufCmd(&UppBuferCPU1,&UPPCMD1);
			UppBuferCPU1.DArray[pFlag]=0x0e;			
			UppBuferCPU1.DArray[pLenth]=0;
			UppBuferCPU1.DArray[PosOfP]=0;			
		}
		HAL_UART_Receive_IT(&huart1,&(UppBuferCPU1.DArray[UppBuferCPU1.DArray[PosOfP]]),1);		
	}
}

void UpperInit(void)
{
	UppBuferCPUInit(&UppBuferCPU1);
	HAL_UART_Receive_IT(&huart1,&(UppBuferCPU1.DArray[UppBuferCPU1.DArray[PosOfP]]),1);			
	
}


/********************************************TRANSMIT*******************************************/
