#include "flashrw.h"
#include "spi.h"
#include "usart.h"




//����
uint32_t 	W25QXXSector=0x00000000;
//��һ���Ƿ��¼��
uint8_t		If_LastRecorded=1;

 




 
uint16_t W25QXX_TYPE=W25Q16;	//Ĭ����W25Q16

//SPI2 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);
    return Rxdata;                      //�����յ�������
}
//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
uint8_t W25QXX_ReadSR(void)   
{  
	uint8_t byte=0;   
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
	SPI1_ReadWriteByte(W25X_ReadStatusReg);    //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     
	return byte;   
} 
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(uint8_t sr)   
{   
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
	SPI1_ReadWriteByte(W25X_WriteStatusReg);   //����дȡ״̬�Ĵ�������    
	SPI1_ReadWriteByte(sr);               //д��һ���ֽ�  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteEnable);      //����дʹ��  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteDisable);     //����д��ָֹ��    
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
} 		
//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;	  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);				    
	SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);				    
	return Temp;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
 	uint16_t i;   										    
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����   
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>24));  //����24bit��ַ    
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>16));   
    SPI1_ReadWriteByte((uint8_t)ReadAddr>>8);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //ѭ������  
    }
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);  				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
    W25QXX_Write_Enable();                  //SET WEL 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PageProgram);      //����дҳ����   
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //����24bit��ַ    
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((uint8_t)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//ѭ��д��  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 			 		 
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	uint8_t * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			W25QXX_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)   
{  
	//����falsh�������,������     
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_SectorErase);      //������������ָ�� 
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((uint8_t)Dst_Addr);  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				   //�ȴ��������
}  
//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
    for(int i=0 ; i <20 ; i++){}                                 //�ȴ�TPD  
}   
//����
void W25QXX_WAKEUP(void)   
{  
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
    for(int i=0 ; i <20 ; i++){}                                //�ȴ�TRES1
}   



//DMA��ʽдһ��ҳ
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
		W25QXXSector=0x00000100;//����һ�ȿ�ʼд
	}
	else
	{
		W25QXX_Read(buffer , 0x00000100 , 256);
		if(buffer[255]!=0x01)
		{
			If_LastRecorded=0;
		}
		W25QXXSector=0x00000000;//����һ�ȿ�ʼд		
	}
	if(If_LastRecorded)//֮ǰ�м�¼�����򲻻��¼
	{
		Translation(M1 , buffer , 0);
		Translation(M2 , buffer , 1);
		Translation(M3 , buffer , 2);
		Translation(M4 , buffer , 3);
		Translation(M5 , buffer , 4);
	}
}








