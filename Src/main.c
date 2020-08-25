/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_CAN_MSG_SIZE               64

typedef union ui32_to_ui8 {
	uint32_t ui32;
	uint8_t ui8[4];
}ui32_to_ui8;



typedef struct
{
  uint8_t header[4];
  uint8_t id[2];
  uint8_t ml[2];
  uint8_t payload[56];
}naza_canmsg;

typedef struct
{
  uint8_t DaTa[60];

}gps_msg;

typedef struct
{
uint8_t msg[8];
}msg_8_byte;

typedef struct
{
	float GX[1];
	float GY[1];
}GX_GY;

typedef struct
{
uint8_t ID[2];
uint8_t ML[2];
uint8_t DT[4];
uint8_t LO[4];
uint8_t LA[4];
uint8_t AL[4];
uint8_t DATA[42];
}GPS_message;

typedef struct
{
	uint8_t ID[2];
	uint8_t ML[2];
	uint8_t LO[8];
	uint8_t LA[8];
	uint8_t AG[4];
	uint8_t AX[4];
	uint8_t AY[4];
	uint8_t AZ[4];
	uint8_t Gyro[12];
	uint8_t AB[4];
	uint8_t DATA[68];
}OSD_message;

typedef struct
{
	uint32_t GX[1];
	uint32_t GY[1];
	uint32_t GZ[1];
}Gyro_message;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * @brief hardware allocation
 */
#define CONF_COM_CAN_PERIPH            CAN1
#define CONF_COM_CAN_TxRxGPIO          GPIOB
#define CONF_COM_CAN_TxPIN             GPIO_PIN_9
#define CONF_COM_CAN_RxPIN             GPIO_PIN_8

/*
 * @brief bus parameters
 */
#define CONF_STATION_ID                0xAAAAAA
#define CONF_IRQLINE_COM_CANTxRx       CAN1_RX0_IRQn
#define CONF_IRQPRIO_COM_CANTxRx       0
#define CONF_IRQPRIO_SYSTICK           1

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
#define CAN_RX_BUFFERSIZE				  (COUNTOF(buffer) - 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t aTxBuffer[] = "HELLO WORLD!\r\n";

/*
 * @brief transmission medium configuration
 */
CAN_HandleTypeDef can_handle;
CAN_TxHeaderTypeDef tx_header;
CAN_TxHeaderTypeDef header_header;
CAN_TxHeaderTypeDef payload_header;
CAN_TxHeaderTypeDef footer_header;
CAN_RxHeaderTypeDef rx_header;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;




/*
 * @brief Rx message reception
 */
static bool rx_pending1 = false;
static bool rx_pending2 = false;
static bool rx_pending3 = false;
uint32_t tx_mailbox;
uint8_t buffer[8];
uint8_t TxData_header_payload[8] = {0x55, 0xAA, 0x55, 0xAA, 0x07, 0x10, 0x00, 0x00};
uint8_t TxData_footer[4] = {0x66, 0xCC, 0x66, 0xCC};
uint8_t c1, c2, c3, c4, c5, c6, c7, c8, c9, c10;
uint8_t msg_counter[3];
uint8_t OSD_msg[150];
uint8_t Gyro_msg[50];
uint8_t GPS_msg[80];
GPS_message GPS_m;
OSD_message OSD_m;
Gyro_message Gyro_m;
float lon, lat, latitude, longitude, alt, altitude ;
double longitude_osd, latitude_osd, a, b;
float altitude_osd;
int8_t  Gyro_X;
float rad, orientation;
uint8_t r =0, n=0, m=0;

bool OSD = false;

uint8_t Compass_data_idx = 0 ;
uint8_t GPS_data_idx = 0 ;
uint8_t OSD_data_idx = 0 ;
uint8_t gyro_data_idx = 0;
uint8_t data_length_GPS = 0;
uint8_t data_length_OSD = 0;
uint8_t data_length_gyro = 0;
uint8_t msg_length[4]={0, 0, 0, 0};
uint8_t canMsgByte[4]={0, 0, 0, 0};
uint8_t buffer_data_size = 8;
uint8_t header[4]={0, 0, 0, 0} ;
uint8_t footer[4]={0, 0, 0, 0} ;
uint8_t collectData[4]={0, 0, 0, 0} ;
uint8_t msg_indx[4]={0, 0, 0, 0};


naza_canmsg com1, com2, com3, gps1, osd_head, gyro_head;
gps_msg gps2, gps3, gps4, osd3, osd4, osd5, osd6, osd7, osd8, osd9, osd10;
msg_8_byte msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9, msg10, msg11, msg12, msg13, msg14, msg15, msg16, msg17, msg18, msg19, msg20;
GX_GY GXGY;


#pragma pack(1)



#pragma pack()


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void
error_critical (void)
{
  while (1)
    {
      ; // halt MCU
    }
}

uint8_t gyro[14];

int header_counter = 0;
bool header_found = false;
int footer_counter = 0;
bool footer_found= false;
uint32_t gyro_payload_size = 0;
uint32_t i,j;
uint32_t framesize;
uint8_t text[]="HEllo!\r\n";
uint32_t gyroX, gyroY, gyroZ, gyroX_old=0;

char destX[100];
char destY[16];
char destZ[16];
char dest[100];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buffer)==HAL_OK){
		HAL_UART_Transmit(&huart2, buffer, 8, 10);
		if (rx_header.StdId == 0x090){

			for(i=0;i<rx_header.DLC;i++){
				if (header_found){
					if (gyro_payload_size == (uint32_t)14){
						if (footer_counter == 0 && buffer[i]==0x66)
							footer_counter++;
						else if (footer_counter == 1 && buffer[i] == 0xcc)
							footer_counter++;
						else if (footer_counter==2 && buffer[i]==0x66)
							footer_counter++;
						else if (footer_counter==3 && buffer[i]==0xcc){
							for(j=0;j<14-4;j++){
								if (gyro[j]==0x55 && gyro[j+1]==0xaa&&gyro[j+2]==0x55&&gyro[j+3]==0xaa)
									HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
								if (gyro[j]==0x66 && gyro[j+1]==0xcc&&gyro[j+2]==0x66&&gyro[j+3]==0xcc)
									HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
							}


							gyroX = gyro[0] | (gyro[1] << 8) | (gyro[2] << 16) | (gyro[3] << 24);
							gyroY = gyro[4] | (gyro[5] << 8) | (gyro[6] << 16) | (gyro[7] << 24);
							gyroZ = gyro[8] | (gyro[9] << 8) | (gyro[10] << 16) | (gyro[11] << 24);


//							if (gyroX>100000)
//							gyroX=100000;

//							if (gyroY>100000)
//								gyroY=100000;

//							if (gyroZ>100000)
//								gyroZ=100000;
//
							if (gyroX<100000 && gyroY<100000 && gyroZ<100000){

								sprintf(dest, "%lu    %lu    %lu\r\n", (unsigned long)gyroX, (unsigned long) gyroY, (unsigned long)gyroZ);
								HAL_UART_Transmit_IT(&huart2, (uint8_t*)dest, 19);
							}
							if (gyroX<100000 && gyroY){
								sprintf(destX, "%lu\r\n", (unsigned long)gyroX);
								HAL_UART_Transmit_IT(&huart2, (uint8_t*)destX, 16);
							}

							sprintf(destY, "%lu\r\n",(unsigned long)gyroY);
							HAL_UART_Transmit_IT(&huart2, (uint8_t*)destY, 16);

							sprintf(destZ, "%lu\r\n",(unsigned long)gyroZ);
							HAL_UART_Transmit_IT(&huart2, (uint8_t*)destZ, 16);


							header_counter = 0;
							header_found = false;
							footer_counter = 0;
							footer_found = false;
							gyro_payload_size=0;
						}
						else{
							header_counter = 0;
							header_found = false;
							footer_counter = 0;
							footer_found = false;
							gyro_payload_size=0;
						}
					}
					else if (gyro_payload_size < (uint32_t)14){
						gyro[gyro_payload_size] = buffer[i];
						gyro_payload_size++;
					}
					else
						Error_Handler();
				}
				else{

					if (header_counter == 0){
						if (buffer[i] == 0x55)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 1){
						if (buffer[i] == 0xaa)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 2){
						if (buffer[i] == 0x55)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 3){
						if (buffer[i] == 0xaa)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 4){
						if (buffer[i] == 0x01)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 5){
						if (buffer[i] == 0x10)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 6){
						if (buffer[i] == 0x0e)
							header_counter++;
						else
							header_counter = 0;
					}
					else if (header_counter == 7){
						if (buffer[i] == 0x00){
							header_counter++;
							header_found = true;
						}
						else
							header_counter = 0;
					}
					else{
						header_counter = 0;
						header_found = false;
						footer_counter = 0;
						footer_found = false;
						gyro_payload_size=0;
					}
				}

			}*/
		}
	}
	return;

}




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
  MX_USART2_UART_Init();
  MX_GPIO_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */

  	  header_header.StdId = 0x108;
      header_header.ExtId = 0x01;
      header_header.RTR = CAN_RTR_DATA;
      header_header.DLC = sizeof(TxData_header_payload);
      header_header.IDE = CAN_ID_STD;
      header_header.TransmitGlobalTime = DISABLE;

      footer_header.StdId = 0x108;
      footer_header.ExtId = 0x01;
      footer_header.RTR = CAN_RTR_DATA;
      footer_header.DLC = sizeof(TxData_footer);
      footer_header.IDE = CAN_ID_STD;
      footer_header.TransmitGlobalTime = DISABLE;

      memset(buffer, 0, sizeof(buffer));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */
	//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_CAN_AddTxMessage (&can_handle, &header_header, (uint8_t*)TxData_header_payload, &tx_mailbox);
	  HAL_CAN_AddTxMessage (&can_handle, &footer_header, (uint8_t*)TxData_footer, &tx_mailbox);
  	  HAL_Delay(1000);


   	  /* USER CODE BEGIN 3 */



//	  if (HAL_CAN_GetRxMessage(&can_handle, CAN_RX_FIFO0, &rx_header, buffer)==HAL_OK)


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /* erase previous setting */
	  HAL_RCC_DeInit ();

	  /* SysClk = (PLL_N * (HSI_VALUE / PLL_M)) / PLL_R   is subject to:
	   * Freq(AHB)  < 168MHz
	   * Freq(APB)  < 42MHz
	   * Freq(APB2) < 84MHz */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;      // 16MHz
	//  RCC_OscInitStruct.MSIState = RCC_MSI_OFF;

	#if DCLOCK_USE_PLL
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLM = 8;                  // [1..8]
	  RCC_OscInitStruct.PLL.PLLN = 84;                 // [8..86]
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;      // SysClk = "
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
	#else
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
	#endif        // DCLOCK_USE_PLL
	  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
	    {
		  error_critical ();
	    }

	  /* initialize the SysClk and AHB and APB busses clocks */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	#if DCLOCK_USE_PLL
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	#else
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	#endif      // DCLOCK_USE_PLL
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	    {
		  error_critical ();
	    }

	  /* Configure the main internal regulator output voltage */
	  if (HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	    {
		  error_critical ();
	    }

	  /* Configure the Systick */
	  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

	#if 0        // portion causing clock inaccuracy
	  /* Configure the Systick interrupt time */
	  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);
	#endif    // 0

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority (SysTick_IRQn, CONF_IRQPRIO_SYSTICK, 0);
	}
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

	 CAN_FilterTypeDef  sFilterConfig;

	  can_handle = (CAN_HandleTypeDef){0};

	  /* peripheral clock enable */
	  __HAL_RCC_CAN1_CLK_ENABLE();

	  /* configure the CAN peripheral */
	  can_handle.Instance = CONF_COM_CAN_PERIPH;
	  can_handle.Init.Prescaler = 4;
	  can_handle.Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_LOOPBACK;//
	  can_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  can_handle.Init.TimeSeg1 = CAN_BS1_2TQ;
	  can_handle.Init.TimeSeg2 = CAN_BS2_1TQ;
	  can_handle.Init.TimeTriggeredMode = DISABLE;
	  can_handle.Init.AutoBusOff = DISABLE;
	  can_handle.Init.AutoWakeUp = DISABLE;
	  can_handle.Init.AutoRetransmission = DISABLE;
	  can_handle.Init.ReceiveFifoLocked = DISABLE;
	  can_handle.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&can_handle) != HAL_OK)
	    {
	      /* halt process */
		  error_critical ();
	    }

	  /* configure the CAN Filter */
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;        // unique CAN instance
	  if (HAL_CAN_ConfigFilter(&can_handle, &sFilterConfig) != HAL_OK)
	    {
	      /* halt process */
		  error_critical ();
	    }

	  /* start the CAN peripheral */
	  if (HAL_CAN_Start(&can_handle) != HAL_OK)
	    {
	      /* halt process */
		  error_critical ();
	    }

	  /* activate CAN Rx IRQ */
	  if (HAL_CAN_ActivateNotification(&can_handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	    {
	      /* halt process */
		  error_critical ();
	    }

	  /* configure transmission process */
	  tx_header.StdId =0x108 /*CONF_STATION_ID*/; /*CAN id = 0x108*/
	  tx_header.ExtId = 0x01;
	  tx_header.RTR = CAN_RTR_DATA;
	  tx_header.IDE = CAN_ID_STD;
	  tx_header.DLC = sizeof(TxData_header_payload);
	  tx_header.TransmitGlobalTime = DISABLE;

	  /* enable Rx IRQ */
	  HAL_NVIC_SetPriority (CONF_IRQLINE_COM_CANTxRx, CONF_IRQPRIO_COM_CANTxRx, 0);
	  HAL_NVIC_EnableIRQ (CONF_IRQLINE_COM_CANTxRx);

	  return;
	}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


/* USER CODE BEGIN 4 */
    /* Configure peripheral GPIO */
    GPIO_InitStruct.Pin = CONF_COM_CAN_RxPIN | CONF_COM_CAN_TxPIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate =  GPIO_AF9_CAN1;
    HAL_GPIO_Init(CONF_COM_CAN_TxRxGPIO, &GPIO_InitStruct);
/* USER CODE END 4 */
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
