/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25Q16.h" 
#include "stdio.h"
#include "flashrw.h"
#include "uppercmd.h"
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}
//int fputc(int ch, FILE *f)
//{
//	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
//	return ch;
//}



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t buffer1[256]={0};
uint8_t buffer2[256]={0};
motorData data1;
motorData data2;
motorData data3;
motorData data4;
motorData data5;


//uint8_t flag=0;
int ccc[256] ={0};
int ccc1[256] ={0};
uint16_t a=0;
uint8_t testbuffer[256]={0};
uint8_t testbuffer1[256]={0};
/*
void SPI1_DMA_TX_CpltCallback( struct __DMA_HandleTypeDef * hdma);
uint32_t 	W25QXXSector=0x00000000;
uint8_t		If_LastRecorded=1;
void writepage(uint32_t addr  , uint8_t * a )
{
	flag=0;
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



*/
//----------------------------------------------------------------------------//






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	data1.accel_count=10;
	data1.decel_start=1;
	data1.decel_val =2;
	data1.dir =3;
	data1.g_add_pulse_count =100;
	data1.int_count =10;
	data1.min_delay =30;
	data1.step_delay =22;
	data1.decel_val=-10;
	data1.int_count=0xf2;
	data1.round_amp=0xef;	
	data1.round_step=0xaf;
	data1.last_accel_delay=0xfa;
	data1.run_state=0x33;
	
	UpperInit();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
	float aa=1.4143;
	float bb=1.4423;
	float c=0;
	long int i=0;
	
	
	W25QXX_Erase_Chip();
//	W25QXX_Erase_Sector(0x00000000);
//W25QXX_Erase_Sector(0x00000100);


//	writepage(0,0,(uint8_t*)(&data1));
	
	SaveData((&data1) ,(&data1) ,(&data1) ,(&data1) ,(&data1) ,buffer1);
	
	HAL_Delay(1000);
	//W25QXX_Wait_Busy();
	GetData((&data1) ,(&data2) ,(&data3) ,(&data4) ,(&data5) ,buffer2);
	printf("\r\n");
	for(int i=0 ;i<5 ; i++)
	{
		for(int j=0 ; j<47 ;j++)
		{
			printf("%.2x" , *(buffer1+47*i+j));	printf("  ");	
		}
		printf("\r\n");
	}
		printf("\r\n");
		printf("\r\n");
		printf("\r\n");	
	for(int i=0 ;i<5 ; i++)
	{
		for(int j=0 ; j<47 ;j++)
		{
			printf("%.2x" , *(buffer2+47*i+j));printf("  ");	
			
		}
		printf("\r\n");
	}	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static unsigned char ledState = 0;
    if (htim == (&htim1))
    {
        if (ledState == 0)
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
        ledState = !ledState;
    }
}
//void DMA2_Stream3_IRQHandler()
//{
//	HAL_DMA_IRQHandler(&hdma_spi1_tx);		
//	if(__HAL_DMA_GET_IT_SOURCE(&hdma_spi1_rx, DMA_IT_TC)){
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//  }
//}

void SPI1_DMA_TX_CpltCallback( struct __DMA_HandleTypeDef * hdma)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
