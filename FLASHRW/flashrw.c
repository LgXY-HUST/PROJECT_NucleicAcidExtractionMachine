#include "flashrw.h"
#include "spi.h"
#include "usart.h"




//扇区
uint32_t 	W25QXXSector=0x00000000;
//上一次是否记录了
uint8_t		If_LastRecorded=1;

 




 
uint16_t W25QXX_TYPE=W25Q16;	//默认是W25Q16

//SPI2 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);
    return Rxdata;                      //返回收到的数据
}
//读取W25QXX的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
uint8_t W25QXX_ReadSR(void)   
{  
	uint8_t byte=0;   
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
	SPI1_ReadWriteByte(W25X_ReadStatusReg);    //发送读取状态寄存器命令    
	byte=SPI1_ReadWriteByte(0Xff);             //读取一个字节  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     
	return byte;   
} 
//等待空闲
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}  
//写W25QXX状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void W25QXX_Write_SR(uint8_t sr)   
{   
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
	SPI1_ReadWriteByte(W25X_WriteStatusReg);   //发送写取状态寄存器命令    
	SPI1_ReadWriteByte(sr);               //写入一个字节  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
}   
//W25QXX写使能	
//将WEL置位   
void W25QXX_Write_Enable(void)   
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_WriteEnable);      //发送写使能  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
} 
//W25QXX写禁止	
//将WEL清零  
void W25QXX_Write_Disable(void)   
{  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_WriteDisable);     //发送写禁止指令    
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
} 		
//读取芯片ID
//返回值如下:				   
//0XEF13,表示芯片型号为W25Q80  
//0XEF14,表示芯片型号为W25Q16    
//0XEF15,表示芯片型号为W25Q32  
//0XEF16,表示芯片型号为W25Q64 
//0XEF17,表示芯片型号为W25Q128 	  
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;	  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);				    
	SPI1_ReadWriteByte(0x90);//发送读取ID命令	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);				    
	return Temp;
}   		    
//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
 	uint16_t i;   										    
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_ReadData);         //发送读取命令   
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>24));  //发送24bit地址    
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>16));   
    SPI1_ReadWriteByte((uint8_t)ReadAddr>>8);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //循环读数  
    }
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);  				    	      
}  
//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
    W25QXX_Write_Enable();                  //SET WEL 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_PageProgram);      //发送写页命令   
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //发送24bit地址    
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((uint8_t)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//循环写数  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选 
	W25QXX_Wait_Busy();					   //等待写入结束
} 
//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 			 		 
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 
//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	uint8_t * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			W25QXX_Erase_Sector(secpos);//擦除这个扇区
			for(i=0;i<secremain;i++)	   //复制
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区  

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		   	pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		   	NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	};	 
}
//擦除整个芯片		  
//等待时间超长...
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_ChipErase);        //发送片擦除命令  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
	W25QXX_Wait_Busy();   				   //等待芯片擦除结束
}   
//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个山区的最少时间:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)   
{  
	//监视falsh擦除情况,测试用     
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_SectorErase);      //发送扇区擦除指令 
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //发送24bit地址    
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((uint8_t)Dst_Addr);  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
    W25QXX_Wait_Busy();   				   //等待擦除完成
}  
//进入掉电模式
void W25QXX_PowerDown(void)   
{ 
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_PowerDown);        //发送掉电命令  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
    for(int i=0 ; i <20 ; i++){}                                 //等待TPD  
}   
//唤醒
void W25QXX_WAKEUP(void)   
{  
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //取消片选     	      
    for(int i=0 ; i <20 ; i++){}                                //等待TRES1
}   



//DMA方式写一整页
uint8_t aaa1[256]={0};
void writepage(uint32_t addr  , uint8_t * a )
{
	W25QXX_Write_Enable();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);	
	SPI1_ReadWriteByte((uint8_t)W25X_PageProgram);
	SPI1_ReadWriteByte((uint8_t)addr>>16);
	SPI1_ReadWriteByte((uint8_t)addr>>8);	
	SPI1_ReadWriteByte((uint8_t)0x00);
	HAL_SPI_TransmitReceive_DMA(&hspi1 ,a ,aaa1 ,256 );
}

void DMA2_Stream3_IRQHandler()
{
	HAL_DMA_IRQHandler(&hdma_spi1_tx);		
	if(__HAL_DMA_GET_IT_SOURCE(&hdma_spi1_rx, DMA_IT_TC)){
		W25QXX_Wait_Busy();
		W25QXX_Erase_Sector(W25QXXSector);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  }
}

uint8_t* Thanslate(motorData* data1 , uint8_t* buffer ,uint8_t number)
{
	buffer[47*number+1]=data1->run_state;
	buffer[47*number+2]=data1->dir;
	buffer[47*number+3]=data1->number;

		buffer[47*number+4]=data1->step_position%0x100;
		buffer[47*number+5]=(data1->step_position%0x10000)>>8;
		buffer[47*number+6]=(data1->step_position%0x1000000)>>16;
		buffer[47*number+7]=(data1->step_position>>24);		
	
	buffer[47*number+8]=data1->motion_sta;

		buffer[47*number+9]=data1->g_add_pulse_count%0xff;
		buffer[47*number+10]=(data1->g_add_pulse_count%0xffff)>>8;
		buffer[47*number+11]=(data1->g_add_pulse_count%0x1000000)>>16;
		buffer[47*number+12]=(data1->g_add_pulse_count>>24);		

		buffer[47*number+13]=data1->step_delay%0x100;
		buffer[47*number+14]=(data1->step_delay%0x10000)>>8;
		buffer[47*number+15]=(data1->step_delay%0x1000000)>>16;
		buffer[47*number+16]=(data1->step_delay>>24);		

		buffer[47*number+17]=data1->decel_start%0x100;
		buffer[47*number+18]=(data1->decel_start%0x10000)>>8;
		buffer[47*number+19]=(data1->decel_start%0x1000000)>>16;
		buffer[47*number+20]=(data1->decel_start>>24);		

		buffer[47*number+21]=data1->decel_val%0x100;
		buffer[47*number+22]=(data1->decel_val%0x10000)>>8;
		buffer[47*number+23]=(data1->decel_val%0x1000000)>>16;
		buffer[47*number+24]=(data1->decel_val>>24);		

		buffer[47*number+25]=data1->accel_count%0x100;
		buffer[47*number+26]=(data1->accel_count%0x10000)>>8;
		buffer[47*number+27]=(data1->accel_count%0x1000000)>>16;
		buffer[47*number+28]=(data1->accel_count>>24);		

		buffer[47*number+29]=data1->round_amp%0x100;
		buffer[47*number+30]=(data1->round_amp%0x10000)>>8;
		buffer[47*number+31]=(data1->round_amp%0x1000000)>>16;
		buffer[47*number+32]=(data1->round_amp>>24);		

		buffer[47*number+33]=(data1->round_step%0x100);
		buffer[47*number+34]=(data1->round_step%0x10000)>>8;
		buffer[47*number+35]=(data1->round_step%0x1000000)>>16;
		buffer[47*number+36]=(data1->round_step>>24);		
			
	buffer[47*number+37]=data1->last_accel_delay%0x100;
	buffer[47*number+38]=data1->last_accel_delay/0xff;

		buffer[47*number+39]=data1->step_count%0x100;
		buffer[47*number+40]=(data1->step_count%0x10000)>>8;
		buffer[47*number+41]=(data1->step_count%0x1000000)>>16;
		buffer[47*number+42]=(data1->step_count>>24);		

		buffer[47*number+43]=data1->rest%0x100;
		buffer[47*number+44]=(data1->rest%0x10000)>>8;
		buffer[47*number+45]=(data1->rest%0x1000000)>>16;
		buffer[47*number+46]=(data1->rest>>24);			
				
	buffer[47*number+47]=data1->int_count;
	return buffer;
}
uint8_t* Translation(motorData* data1 , uint8_t* buffer ,uint8_t number)
{
	data1->run_state=buffer[47*number+1];
	data1->dir=buffer[47*number+2];
	data1->number=buffer[47*number+3];
	
		data1->step_position=buffer[47*number+4];
		data1->step_position+=buffer[47*number+5]<<8;
		data1->step_position+=buffer[47*number+6]<<16;
		data1->step_position+=buffer[47*number+7]<<24;		

	data1->motion_sta=buffer[47*number+8];

		data1->g_add_pulse_count=buffer[47*number+9];
		data1->g_add_pulse_count+=buffer[47*number+10]<<8;
		data1->g_add_pulse_count+=buffer[47*number+11]<<16;
		data1->g_add_pulse_count+=buffer[47*number+12]<<24;		


		data1->step_delay=buffer[47*number+13];
		data1->step_delay+=buffer[47*number+14]<<8;
		data1->step_delay+=buffer[47*number+15]<<16;
		data1->step_delay+=buffer[47*number+16]<<24;		

		data1->decel_start=buffer[47*number+17];
		data1->decel_start+=buffer[47*number+18]<<8;
		data1->decel_start+=buffer[47*number+19]<<16;
		data1->decel_start+=buffer[47*number+20]<<24;		

		data1->decel_val=buffer[47*number+21];
		data1->decel_val+=buffer[47*number+22]<<8;
		data1->decel_val+=buffer[47*number+23]<<16;
		data1->decel_val+=buffer[47*number+24]<<24;		

		data1->accel_count=buffer[47*number+25];
		data1->accel_count+=buffer[47*number+26]<<8;
		data1->accel_count+=buffer[47*number+27]<<16;
		data1->accel_count+=buffer[47*number+28]<<24;		

		data1->round_amp=buffer[47*number+29];
		data1->round_amp+=buffer[47*number+30]<<8;
		data1->round_amp+=buffer[47*number+31]<<16;
		data1->round_amp+=buffer[47*number+32]<<24;		
				
		data1->round_step=buffer[47*number+33];
		data1->round_step+=buffer[47*number+34]<<8;
		data1->round_step+=buffer[47*number+35]<<16;
		data1->round_step+=buffer[47*number+36]<<24;
	
	data1->last_accel_delay=buffer[47*number+37];
	data1->last_accel_delay+=buffer[47*number+38]<<8;

		data1->step_count=buffer[47*number+39];
		data1->step_count+=buffer[47*number+40]<<8;
		data1->step_count+=buffer[47*number+41]<<16;
		data1->step_count+=buffer[47*number+42]<<24;		

		data1->rest=buffer[47*number+43];
		data1->rest+=buffer[47*number+44]<<8;
		data1->rest+=buffer[47*number+45]<<16;
		data1->rest+=buffer[47*number+46]<<24;		
			
data1->int_count =	buffer[47*number+47];
	return buffer;
}



void SaveData(motorData* M1 ,motorData* M2 , motorData* M3 , motorData* M4 , motorData* M5 , uint8_t* buffer)
{
		Thanslate(M1 , buffer ,0);
		Thanslate(M2 , buffer ,1);
		Thanslate(M3 , buffer ,2);
		Thanslate(M4 , buffer ,3);
		Thanslate(M5 , buffer ,4);	
		buffer[255]=0x01;
		W25QXX_Wait_Busy();
		writepage(W25QXXSector , buffer);
		if(W25QXXSector)
		{
			W25QXXSector=0;
		}
		else
		{
			W25QXXSector=0x00000100;
		}
}

void  GetData(motorData* M1 ,motorData* M2 , motorData* M3 , motorData* M4 , motorData* M5 , uint8_t* buffer)
{
	W25QXX_Wait_Busy();	
	W25QXX_Read(buffer , 0x00000000 , 256);
	if(buffer[255]==0x01)
	{
		W25QXXSector=0x00000100;//从另一扇开始写
	}
	else
	{
		W25QXX_Read(buffer , 0x00000100 , 256);
		if(buffer[255]!=0x01)
		{
			If_LastRecorded=0;
		}
		W25QXXSector=0x00000000;//从这一扇开始写		
	}
	if(If_LastRecorded)//之前有记录，否则不会记录
	{
		Translation(M1 , buffer , 0);
		Translation(M2 , buffer , 1);
		Translation(M3 , buffer , 2);
		Translation(M4 , buffer , 3);
		Translation(M5 , buffer , 4);
	}
}








