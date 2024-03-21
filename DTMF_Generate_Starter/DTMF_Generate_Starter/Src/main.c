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
#include <math.h>
#include "CS43L22.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Node
{
	struct Node* prev;
	float value;
	struct Node* next;
}Node;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FS 48000 //see also MX_I2S3_Init function

#define AUDIO_BUFFER_SIZE 180
//con 2*90 el plop se escucha cada tanto 90 samples por cada canal intercaladas

//opciones de compilacion

//#define USE_ADC  0
#define USE_LOOKUP_TABLE_COMPLEMENT_2 3

//#define ADC_OFFSET 0//2048  //Si uso LOOKUP TABLE offset = 0

// defino si quiero usar efectos en el camino de la senial

#define DSP

#define KEY 770
#define _USE_MATH_DEFINES
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t audioBufferA[AUDIO_BUFFER_SIZE];
uint16_t audioBufferB[AUDIO_BUFFER_SIZE];

/* %%%%%%%%%%%%%%%%% Declaro punteros para lectura y transmision de audio %%%%%%%%%%%%%%%%%%%*/

uint16_t *audioToSend = NULL;
uint16_t *audioToUpdate = NULL;

/* %%%%%%%%%%%%%%%%% Declaro variables globales %%%%%%%%%%%%%%%%%%%*/


flag_t flag = idle;

uint8_t adc_done = 0;

buffer_t  buffer_to_send = buffer_B;
buffer_t buffer_to_fill = buffer_A;

uint16_t sample;
volatile bool_t transferComplete = TRUE;

// initialize circular buffer pointers and global coefficient variables
Node* x_buff;
Node* y_buff;
float b1;
float a1;
float a2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t audio_read(void);
uint16_t* select_buffer_to_transmit(buffer_t);

void audio_buffer_init(void);   	//inicializo el audio buffer con ceros.
void fill_buffers();
void load_buffer(uint16_t* buff);

// creates a circular buffer of size 'size'
Node* create_circ_buffer(int size);
// updates the output
void update_output();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	audioToSend = audioBufferB;     //asigno punteros a c/u de los buffers
	audioToUpdate = audioBufferA;

	CS43L22_init();   				//configuro el DAC CS43L22

	audio_buffer_init();         	//inicializo los buffers con ceros

	HAL_TIM_Base_Start(&htim2); 	//activo el timer
	HAL_ADC_Start_IT(&hadc1); 		// y el ADC

	/* %%%%%%%%%%%%%%%%% compute filter coefficients %%%%%%%%%%%%%%%%%%%*/
	b1 = sin((float)KEY*M_PI/24000.0);
	a1 = 2*cos((float)KEY*M_PI/24000.0);
	a2 = -1;

	/* %%%%%%%%%%%%%%%%% Declare circular buffers %%%%%%%%%%%%%%%%%%%*/
	x_buff = create_circ_buffer(3);
	y_buff = create_circ_buffer(3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  audioToSend = select_buffer_to_transmit(buffer_to_send); //me devuelve el puntero al buffer listo para enviar

		if(transferComplete && (flag == data_ready_to_send)){

			flag = idle;

			CS43L22_AudioSend(audioToSend,AUDIO_BUFFER_SIZE); //Envio el buffer por I2S al codec

			transferComplete = FALSE;

		}

		if(adc_done){

			adc_done = 0;

			fill_buffers();
		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  //hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 175;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Sampling_Check_GPIO_Port, Sampling_Check_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Sampling_Check_Pin */
  GPIO_InitStruct.Pin = Sampling_Check_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Sampling_Check_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2s);

	//HAL_GPIO_TogglePin(Sampling_Check_GPIO_Port, Sampling_Check_Pin);

	transferComplete = TRUE;

}


void load_buffer(uint16_t *buff)
{

	static size_t i = 0;

#ifdef DSP
	//move to next sample in buffer
	y_buff = y_buff->next;
	x_buff = x_buff->next;

	// calculate new output value
	update_output();

	// cast to sample
	int16_t sample1 = (int16_t)(y_buff->value);
	sample = (uint16_t)sample1;
#endif

	buff[i] = sample;

	buff[i+1] =  buff[i];

	i = i+2;

	if( (i >= AUDIO_BUFFER_SIZE)){

		buffer_t aux = buffer_to_send;
		buffer_to_send = buffer_to_fill;
		buffer_to_fill = aux;

		flag = data_ready_to_send;
		i=0;
	}


}

void fill_buffers(){

	if((buffer_to_fill == buffer_A)){

		audioToUpdate = audioBufferA;
		load_buffer(audioToUpdate);

	}
	if((buffer_to_fill == buffer_B)){

		audioToUpdate = audioBufferB;
		load_buffer(audioToUpdate);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)

{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hadc);
	adc_done = 1;

}

uint16_t* select_buffer_to_transmit(buffer_t bf){

	uint16_t *ptr = NULL;

	if(buffer_to_send == buffer_A){

		ptr = audioBufferA;

	}else if(buffer_to_send == buffer_B){

		ptr = audioBufferB;

	}

	return ptr;

}

void audio_buffer_init(){

	size_t i;

	for(i = 0;i<AUDIO_BUFFER_SIZE;i++){

		audioBufferA[i] = 0;
		audioBufferB[i] = 0;

	}

}


/*uint16_t audio_read(void){

#if (RUN_OPT == USE_ADC)

	return HAL_ADC_GetValue(&hadc1);

#endif

#if (RUN_OPT == USE_LOOKUP_TABLE_COMPLEMENT_2)

	static size_t index = 0;

	if(index >= SINE_SAMPLES){

		index = 0;
	}

	return SINE_COMP[index++];

#endif

}*/

void CS43L22_EXTERNAL_DAC_I2S_transmit(uint16_t *buffer,uint16_t buffer_size){

	HAL_I2S_Transmit_IT(&hi2s3,buffer,buffer_size);

}


void CS43L22_EXTERNAL_DAC_I2C_write(uint8_t *iData, uint8_t len)
{ //maneja el periferico i2c para comunicaicon con el dac

	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, len, 100);

}

void CS43L22_EXTERNAL_DAC_I2C_recieve(uint8_t *iData){

	HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDRESS, iData, 1, 100);

}

void CS43L22_EXTERNAL_DAC_enable()
{
	//esta funcion se encarga de poner en alto o bajo el pin de reset del dac
	//recibe un 1 para poner el pin en alto y un 0 para ponerlo en bajo

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

}

void update_output()
{
	y_buff->value = b1*(x_buff->prev)->value + a1*(y_buff->prev)->value + a2*(y_buff->next->value);
	return;
}

Node* create_circ_buffer(int size)
{
	// create head and current node pointers
	Node* head = malloc(sizeof(Node));
	Node* current = malloc(sizeof(Node));
	Node* temp = malloc(sizeof(Node));

	// set current pointer to head
	current = head;

	//decrement size by 1
	size--;
	// create the rest of the nodes
	for (int i = 0; i < size; i++)
	{
		// create temporary node
		Node* temp = malloc(sizeof(Node));
		// point current to temp and vice versa
		current->next = temp;
		temp->prev = current;
		// set current node to next node
		current = temp;
	}
	// point head to tail and tail to head
	head->prev = current;
	current->next = head;
	free(temp);
	//return the start of the circular buffer
	return head;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
