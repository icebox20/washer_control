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
#include "modbus_crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEX_DISPLAY_CS 			GPIO_PIN_6
#define HEX_DISPLAY_TEST 		0x0F
#define HEX_DISPLAY_INTENSITY 	0x0A
#define HEX_DISPLAY_SCANLIMIT 	0x0B
#define HEX_DISPLAY_BCDDECODE 	0x09
#define HEX_DISPLAY_SHUTDOWN 	0x0C

#define PRESSURE_PORT 			GPIOB
#define PRESSURE_MISO 			GPIO_PIN_1
#define PRESSURE_SCK 			GPIO_PIN_0
#define WATER_LEVEL_1			160000
#define WATER_LEVEL_2			170000
#define WATER_LEVEL_3			180000
#define WATER_LEVEL_4			190000
#define WATER_LEVEL_5			200000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t digit_address[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t char_b_font[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
uint8_t InverterRxData[32];
uint8_t InverterTxData[8];
uint16_t InverterData[10];
GPIO_TypeDef * keypad_ports[8] = {KEYPAD_0_GPIO_Port, KEYPAD_1_GPIO_Port, KEYPAD_2_GPIO_Port, KEYPAD_3_GPIO_Port, KEYPAD_4_GPIO_Port, KEYPAD_5_GPIO_Port, KEYPAD_6_GPIO_Port, KEYPAD_7_GPIO_Port};
const uint16_t keypad_pins[8] = {KEYPAD_0_Pin, KEYPAD_1_Pin, KEYPAD_2_Pin, KEYPAD_3_Pin, KEYPAD_4_Pin, KEYPAD_5_Pin, KEYPAD_6_Pin, KEYPAD_7_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void send_SPI_16bits_raw(SPI_HandleTypeDef *hspi, uint16_t GPIO_Pin, uint8_t address, uint8_t data);
void hex_display_init();
void hex_display_set_single(int digit, char c);
void hex_display_set(char word[8]);

int get_water_level();
int32_t read_hx710b();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_SPI_16bits_raw(SPI_HandleTypeDef *hspi, uint16_t GPIO_Pin, uint8_t address, uint8_t data){
	uint8_t data_16bits[2]={address, data};

	HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)data_16bits, 2, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_SET);
}

void hex_display_init(){
	  send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, HEX_DISPLAY_TEST, 0x01);
	  HAL_Delay(500);
	  send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, HEX_DISPLAY_TEST, 0x00);
	  send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, HEX_DISPLAY_INTENSITY, 0x0E);
	  send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, HEX_DISPLAY_SCANLIMIT, 0x0F);
	  send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, HEX_DISPLAY_BCDDECODE, 0xFF);
	  send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, HEX_DISPLAY_SHUTDOWN, 0x01);
}

void hex_display_set_single(int digit, char c){
	if(digit < 0 || digit > 7){
		return;
	}

	if(c >= 48 && c <=57){
		send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[(int)c - 48]);
		return;
	}

	switch (c) {
		case '-':
			send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[10]);
			break;
		case 'e':
			send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[11]);
			break;
		case 'h':
			send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[12]);
			break;
		case 'l':
			send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[13]);
			break;
		case 'p':
			send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[14]);
			break;
		case ' ':
			send_SPI_16bits_raw(&hspi1, HEX_DISPLAY_CS, digit_address[digit], char_b_font[15]);
			break;
		default:
			break;
	}
	return;
}

void hex_display_set(char word[8]){
	hex_display_set_single(7, word[0]);
	hex_display_set_single(6, word[1]);
	hex_display_set_single(5, word[2]);
	hex_display_set_single(4, word[3]);
	hex_display_set_single(3, word[4]);
	hex_display_set_single(2, word[5]);
	hex_display_set_single(1, word[6]);
	hex_display_set_single(0, word[7]);
}

int32_t read_hx710b(){
	int32_t output = 0;

	while(HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_MISO) == GPIO_PIN_SET);

	for (int i = 0; i < 24; ++i) {
		HAL_GPIO_WritePin(PRESSURE_PORT, PRESSURE_SCK, GPIO_PIN_SET);
		output <<= 1;
		output |= HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_MISO);
		HAL_GPIO_WritePin(PRESSURE_PORT, PRESSURE_SCK, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(PRESSURE_PORT, PRESSURE_SCK, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PRESSURE_PORT, PRESSURE_SCK, GPIO_PIN_RESET);

	if(output & 0x800000){
		output |= 0xFF000000;
	}

	return output;
}

int get_water_level(){
	int32_t pressure = read_hx710b();

	if(pressure < WATER_LEVEL_1 || pressure == 0x7FFFFF || pressure == 0x800000)
		return 0;
	else if(pressure >= WATER_LEVEL_5)
		return 5;
	else if(pressure >= WATER_LEVEL_4)
		return 4;
	else if(pressure >= WATER_LEVEL_3)
		return 3;
	else if(pressure >= WATER_LEVEL_2)
		return 2;
	else if(pressure >= WATER_LEVEL_1)
		return 1;

	return 0;

}

void sendData (uint8_t *data)
{
	HAL_GPIO_WritePin(INVERTER_EN_GPIO_Port, INVERTER_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, data, 8, 1000);
	HAL_GPIO_WritePin(INVERTER_EN_GPIO_Port, INVERTER_EN_Pin , GPIO_PIN_RESET);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	InverterData[0] = InverterRxData[3]<<8 | InverterRxData[4];
//	InverterData[1] = InverterRxData[5]<<8 | InverterRxData[6];
//	InverterData[2] = InverterRxData[7]<<8 | InverterRxData[8];
//	InverterData[3] = InverterRxData[9]<<8 | InverterRxData[10];
//	InverterData[4] = InverterRxData[11]<<8 | InverterRxData[12];
}

int read_keypad(){
	HAL_GPIO_WritePin(KEYPAD_0_GPIO_Port, KEYPAD_0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_1_GPIO_Port, KEYPAD_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_2_GPIO_Port, KEYPAD_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_3_GPIO_Port, KEYPAD_3_Pin, GPIO_PIN_RESET);

	for (int row = 0; row < 4; ++row) {
	  HAL_GPIO_WritePin(keypad_ports[row], keypad_pins[row], GPIO_PIN_SET);
	  for (int col = 0; col < 4; ++col) {
		  if(HAL_GPIO_ReadPin(keypad_ports[col+4], keypad_pins[col+4])){
			  return row * 4 + col;
		  }
	  }
	  HAL_GPIO_WritePin(keypad_ports[row], keypad_pins[row], GPIO_PIN_RESET);
	}
	return 16;
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
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  hex_display_init();

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, InverterRxData, 32);

  InverterTxData[0] = 0x05;  // slave address
  InverterTxData[1] = 0x03;  // Function code for Read Holding Registers

  InverterTxData[2] = 0;
  InverterTxData[3] = 0x04;
  //The Register address will be 00000000 00000100 = 4 + 40001 = 40005

  InverterTxData[4] = 0;
  InverterTxData[5] = 0x01;
  // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes

  uint16_t crc = crc16(InverterTxData, 6);
  InverterTxData[6] = crc&0xFF;   // CRC LOW
  InverterTxData[7] = (crc>>8)&0xFF;  // CRC HIGH

  sendData(InverterTxData);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //hex_display_set("        ");
	  int32_t pressure = read_hx710b();
	  char snum[8];
	  itoa(pressure/10, snum, 10);

	  //utoa((uint8_t)InverterData[0], snum, 10 );

	  hex_display_set(snum);
	  HAL_Delay(1000);

	  hex_display_set_single(1, (char)read_keypad()+48);

	  HAL_Delay(500);

//	  hex_display_set("01234567");
//	  HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HEX_CS_Pin|INVERTER_EN_Pin|KEYPAD_0_Pin|KEYPAD_1_Pin
                          |KEYPAD_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PRESSURE_SCK_Pin|KEYPAD_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HEX_CS_Pin INVERTER_EN_Pin KEYPAD_0_Pin KEYPAD_1_Pin
                           KEYPAD_2_Pin */
  GPIO_InitStruct.Pin = HEX_CS_Pin|INVERTER_EN_Pin|KEYPAD_0_Pin|KEYPAD_1_Pin
                          |KEYPAD_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PRESSURE_SCK_Pin KEYPAD_3_Pin */
  GPIO_InitStruct.Pin = PRESSURE_SCK_Pin|KEYPAD_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PRESSURE_MISO_Pin */
  GPIO_InitStruct.Pin = PRESSURE_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PRESSURE_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BALANCE_Pin ESTOP_Pin LOCK_Pin HINGE_Pin */
  GPIO_InitStruct.Pin = BALANCE_Pin|ESTOP_Pin|LOCK_Pin|HINGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_4_Pin KEYPAD_5_Pin KEYPAD_6_Pin KEYPAD_7_Pin */
  GPIO_InitStruct.Pin = KEYPAD_4_Pin|KEYPAD_5_Pin|KEYPAD_6_Pin|KEYPAD_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
