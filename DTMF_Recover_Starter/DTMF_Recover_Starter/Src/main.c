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
#include <stdio.h>
//#include "sin256.h""

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float real, imag;
}COMPLEX;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AUDIO_BUFFER_SIZE 180
//con 2*90 el plop se escucha cada tanto 90 samples por cada canal intercaladas

//opciones de compilacion

#define USE_ADC  0


#define ADC_OFFSET 0//2048  //Si uso LOOKUP TABLE offset = 0

// defino si quiero usar efectos en el camino de la senial

#define DSP
#define PTS 1024

// defino la cantidad maxima de muestras que voy a reservar para el buffer del delay

//aca se define de donde levanto el audio para transferir al DAC externo

#define RUN_OPT USE_ADC
//#define RUN_OPT USE_LOOKUP_TABLE_COMPLEMENT_2
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

COMPLEX w[PTS];
COMPLEX samples[PTS];
int sample_index;
int peaks[PTS];
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
void load_buffer(uint16_t *);
void FFT(COMPLEX *, int);
void find_peaks(int *);
int __io_putchar(int ch);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int n=0;
int num=0;


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

	// calculate twiddle factors
	for (int i = 0; i < PTS; i++)
	{
		w[i].real = cos(2.0 * M_PI * (float)i / PTS);
		w[i].imag = sin(2.0 * M_PI * (float)i / PTS);
		samples[i].real = 0;
		samples[i].imag = 0;
	}

	sample_index = 0;

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


void load_buffer(uint16_t *buff){

	static size_t i = 0;


	sample = audio_read();

#ifdef DSP

	samples[sample_index].real = (float)sample;
	samples[sample_index].imag = 0.0;

	sample_index++;

	if(sample_index == PTS)
	{
		FFT(samples, PTS);
		sample_index = 0;

		int output[PTS];

		for (int i = 0; i< PTS; i++)
		{
			output[i] = sqrt(((int)((samples[i].real))^2) + ((int)((samples[i].imag))^2));
		}

		find_peaks(output);

		float freq1 = 0.0;
		float freq2 = 0.0;

		for (int i = 0; i < PTS; i ++)
		{
			if(peaks[i] == 0) {}
			else
			{
				printf("%d\n", output[i]);
			}
		}
	}



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


uint16_t audio_read(void){

#if (RUN_OPT == USE_ADC)

	return HAL_ADC_GetValue(&hadc1);

#endif

/*#if (RUN_OPT == USE_LOOKUP_TABLE_COMPLEMENT_2)

	static size_t index = 0;

	if(index >= SINE_SAMPLES){

		index = 0;
	}

	return SINE_COMP[index++];

#endif */

}

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

void find_peaks(int* spectrum)
{
	// create an edge detection kernel
	int kernel[3] = {0, -1, 1};

	// create a peaks array
	int value;
	int edges[PTS];

	// sum variable for average
	int sum = 0;

	for (int i = 2; i < PTS; i++)
	{
		// find edges
		value =
			spectrum[i]*kernel[2] +
			spectrum[i-1]*kernel[1] +
			spectrum[i-2]*kernel[0];

		// add
		edges[i-2]=value;
		sum += value;
	}

	// find peaks larger than average that are separated by more than 2 spots
	int average = sum/(PTS);
	int close_peaks=0;
	for (int i = 0; i < PTS; i++)
	{
		if((edges[i] > (average + 30)) & (close_peaks == 0))
		{
			peaks[i] = i;
			close_peaks = 2;
		}
		else
		{
			close_peaks <= 0 ? close_peaks = 0 : close_peaks--;
			peaks[i] = 0;
		}
	}
	return;
}

int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return 0;
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
