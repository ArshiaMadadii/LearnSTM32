/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NMEA.h"
#include "uartRingBuffer.h"
#include "uart.h"
#include "stdio.h"
#include "string.h"
#include "LoRa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

				/*----------DHT22 Begin------------*/
#define DHT22_Pin GPIO_PIN_9
#define DHT22_GPIO_Port GPIOB
				/*----------DHT22 End------------*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
  /*---------------------------LoRa Begin----------------------*/

LoRa myLoRa;
uint16_t LoRa_stat;
uint8_t Rx_Buffer[128];
uint8_t Tx_Buffer[128];
LoRa myLoRa1;
uint16_t LoRa_stat1;
uint8_t Rx_Buffer1[128];
uint8_t Tx_Buffer1[128];
	char*  recieve_data;
	  /*---------------------------LoRa End----------------------*/

	/*----------DHT22 Begin------------*/

	char message1[16];
	char message2[16];
	uint8_t TOUT = 0, CheckSum, i;
	uint8_t T_Byte1, T_Byte2, RH_Byte1, RH_Byte2;
	float Voltage_mV = 0;
	float Temperature_C = 0;
	float Temperature_F = 0;
	char msg[32];
	char msg1[32];

	float temperature = 0.0f;
	float humidity = 0.0f;
	/*----------DHT22 End------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********************************** DHT22 Function ***************************/


void delay_us(uint16_t us)
{

	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < us);
}


void start_signal(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(30); // Wait 20-40μs

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

uint8_t check_response(void) {
    TOUT = 0;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (!HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) && (__HAL_TIM_GET_COUNTER(&htim1) < 100)) {};
    if (__HAL_TIM_GET_COUNTER(&htim1) >= 100)
        return 0;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) && (__HAL_TIM_GET_COUNTER(&htim1) < 100)) {};
    if (__HAL_TIM_GET_COUNTER(&htim1) >= 100)
        return 0;

    return 1;
}

uint8_t read_byte(void) {
    uint8_t num = 0;
    for (i = 0; i < 8; i++) {
        while (!HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)) {};
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)) {};
        if (__HAL_TIM_GET_COUNTER(&htim1) > 40) // 40μs threshold
            num |= (1 << (7 - i));
    }
    return num;
}

void send_uart(char *string) {
    HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}




/************************************ DHT22 ******************************************/

  	  /*--------------------------GPS Begin--------------------------*/

char GGA[100];
char RMC[100];

GPSSTRUCT gpsData;

int flagGGA = 0, flagRMC = 0;
char lcdBuffer [50];

int VCCTimeout = 5000;
  	  /*--------------------------GPS End--------------------------*/

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
  char tx_buffer[] = "No Function";
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	  /*--------------------------GPS Begin------------------------*/
  Ringbuf_init();
  HAL_Delay(500);
  uartx_write_text(&huart1, "Hello GPS...\r\n");
  HAL_Delay(500);
	  /*--------------------------GPS End--------------------------*/

  	  /*---------------------------LoRa Begin----------------------*/
  myLoRa = newLoRa();

  myLoRa.CS_port         = NSS_GPIO_Port;
  myLoRa.CS_pin          = NSS_Pin;
  myLoRa.reset_port      = RST_GPIO_Port;
  myLoRa.reset_pin       = RST_Pin;
  myLoRa.DIO0_port       = DIO0_GPIO_Port;
  myLoRa.DIO0_pin        = DIO0_Pin;
  myLoRa.hSPIx           = &hspi1;

  myLoRa.frequency             = 440;             // default = 433 MHz
  myLoRa.spredingFactor        = SF_7;            // default = SF_7
  myLoRa.bandWidth             = BW_31_25KHz;       // default = BW_125KHz
  myLoRa.crcRate               = CR_4_5;          // default = CR_4_5
  myLoRa.power                 = POWER_20db;      // default = 20db
  myLoRa.overCurrentProtection = 130;             // default = 100 mA
  myLoRa.preamble              = 9;              // default = 8;
	   /*---------------------------LoRa End---------------------------*/
  HAL_TIM_Base_Start(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*------------------------------DHT22 Begin--------------------------------*/

	  HAL_Delay(1000); // Delay for 1 second
	          start_signal(); // Send start signal to DHT22

	          uint8_t check = check_response();
	          if (!check) {
	              send_uart("No response from the sensor\r\n");
	          } else {
	              // Read bytes from DHT22
	              RH_Byte1 = read_byte();
	              RH_Byte2 = read_byte();
	              T_Byte1 = read_byte();
	              T_Byte2 = read_byte();
	              CheckSum = read_byte();

	              // Combine humidity and temperature bytes
	              uint16_t rh = (RH_Byte1 << 8) | RH_Byte2;
	              uint16_t temp = (T_Byte1 << 8) | T_Byte2;
	              uint8_t sign = 0;

	              // Check if temperature is negative
	              if (temp > 0x8000) {
	                  temp &= 0x7FFF; // Clear the sign bit
	                  sign = 1; // Indicate negative temperature
	              }

	              char rh_buf[8], temp_buf[8];
	              sprintf(rh_buf, "%2.2u", rh);
	              sprintf(temp_buf, "%2.2u", temp);

	              // Check checksum
	              if (CheckSum == ((RH_Byte1 + RH_Byte2 + T_Byte1 + T_Byte2) & 0xFF))
	              {
	            	  send_uart("Test Message\r\n");
	                  sprintf(msg1, "RH = %3u.%1u %% ", rh / 10, rh % 10);
	                  send_uart(msg1);
	                  sprintf(msg, "Temp = %c%3u.%1u C \r\n", sign ? '-' : ' ', temp / 10, temp % 10);
	                  send_uart(msg);
	              } else {
	                  send_uart("Checksum Error! Trying Again ...\r\n");
	              }
	          }
				/*------------------------------DHT22 End--------------------------------*/

	  	/*--------------------------GPS Begin--------------------------*/
	  uartx_write_text(&huart1, "Hello\r\n");

	 	  if (Wait_for("GGA") == 1)
	 	 	  {

	 	 		  VCCTimeout = 5000;  // Reset the VCC Timeout indicating the GGA is being received

	 	 		  Copy_upto("*", GGA);
	 	 		  if (decodeGGA(GGA, &gpsData.ggastruct) == 0) flagGGA = 2;  // 2 indicates the data is valid
	 	 		  else flagGGA = 1;  // 1 indicates the data is invalid
	 	 	  }

	 	 	  if (Wait_for("RMC") == 1)
	 	 	  {

	 	 		  VCCTimeout = 5000;  // Reset the VCC Timeout indicating the RMC is being received

	 	 		  Copy_upto("*", RMC);
	 	 		  if (decodeRMC(RMC, &gpsData.rmcstruct) == 0) flagRMC = 2;  // 2 indicates the data is valid
	 	 		  else flagRMC = 1;  // 1 indicates the data is invalid
	 	 	  }

	 	 	  if((flagGGA == 2 ) | (flagRMC == 2 )){
	 			  sprintf (lcdBuffer, " %02d:%02d:%02d, %02d%02d%02d", gpsData.ggastruct.tim.hour, \
	 			gpsData.ggastruct.tim.min, gpsData.ggastruct.tim.sec, gpsData.rmcstruct.date.Day, \

	 				gpsData.rmcstruct.date.Mon, gpsData.rmcstruct.date.Yr);

	 			  uartx_write_text(&huart1, lcdBuffer);
	 			  sprintf (lcdBuffer, " \r\n Latitude= %.2f%c, Longitude= %.2f%c  ", gpsData.ggastruct.lcation.latitude, gpsData.ggastruct.lcation.NS,\
	 	 					  gpsData.ggastruct.lcation.longitude, gpsData.ggastruct.lcation.EW);
	 			  uartx_write_text(&huart1, lcdBuffer);
	 	 		  }

	 	 		  else if ((flagGGA == 1) | (flagRMC == 1))
	 	 		  {
	 	 			  // Instead of clearing the display, it's better if we print spaces.
	 	 			  // This will avoid the "refreshing" part
	 				  uartx_write_text(&huart1, "   NO FIX YET   ");

	 				  uartx_write_text(&huart1, "   Please wait  ");

	 	 		  }

	 	 		  if (VCCTimeout <= 0)
	 	 		  {
	 	 			  VCCTimeout = 5000;  // Reset the timeout

	 	 			  //reset flags
	 	 			  flagGGA =flagRMC =0;

	 	 			  // You are here means the VCC is less, or maybe there is some connection issue
	 	 			  // Check the VCC, also you can try connecting to the external 5V


	 				  uartx_write_text(&huart1, "    VCC Issue   ");

	 				  uartx_write_text(&huart1, "Check Connection");

	 	 		  }


	 	 		  HAL_Delay(1000);

	 	 		 HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen(tx_buffer), 1000);
	 	 		  HAL_Delay(1000);
  	  	  	  	  /*--------------------------GPS Begin--------------------------*/

	 	  	  	  /*---------------------------LoRa Begin------------------------*/
	 	 		   Rx_Buffer1[128]=0;
	 	 		   Rx_Buffer[128]=0;
	 	 		  while (LoRa_stat == 0)
	 	 		  {
	 	 	  if(LoRa_init(&myLoRa)== LORA_OK )
	 	 	  {
	 	 		 LoRa_stat = 1;
	 	 	  }
	 	 		  }
	 	 	    // Calculate the total size of the combined buffer
	 	 	    uint16_t combinedSize = strlen(lcdBuffer) + strlen(msg) + strlen(msg1) + 1; // +1 for null terminator
	 	 	    char combinedBuffer[combinedSize];

	 	 	    // Clear the combined buffer
	 	 	    memset(combinedBuffer, 0, sizeof(combinedBuffer));

	 	 	    // Combine lcdBuffer, msg, and msg1 into the combined buffer
	 	 	    strcpy(combinedBuffer, lcdBuffer);          // Copy lcdBuffer
	 	 	    strcat(combinedBuffer, msg);               // Append msg
	 	 	    strcat(combinedBuffer, msg1);              // Append msg1

	 	 	    // Transmit the combined buffer using LoRa
	 	 	    LoRa_transmit(&myLoRa, (uint8_t *)combinedBuffer, strlen(combinedBuffer), 100);
	 	 	    HAL_Delay(5000);
//	 	 	  	  	LoRa_transmit(&myLoRa, (uint8_t*)lcdBuffer, 12, 100);
//	 	 	    	//LoRa_startReceiving(&myLoRa);
//	 	 	    	HAL_Delay(10);
//	 	 	  	  	LoRa_transmit(&myLoRa, (uint8_t*)msg, 12, 100);
//	 	 	    	HAL_Delay(10);
//	 	 	  	  	LoRa_transmit(&myLoRa, (uint8_t*)msg1, 12, 100);
	 	 		   /*---------------------------LoRa End---------------------------*/

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : NSS_Pin RST_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
