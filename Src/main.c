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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define GAIN_ADC 1.2182
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern uint16_t index_usb;
extern char usb_rs[50];
extern uint16_t tim_ovr;
extern uint16_t tam;
extern char data_tx[50];
uint16_t adc_buf[4], adc_atv=0, pwm_v=0;
uint8_t status_mod=0;
uint32_t tim3_prs=71, tim2_prs=71;
extern uint16_t sen_pwm[100];
extern uint8_t sen_act;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void module_start()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	status_mod=1;
}
void module_stop()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
	HAL_TIM_Base_Stop_IT(&htim3);
	tim_ovr=0;
	status_mod=0;
}

void active_ch(uint16_t chanel_atv)
{
	module_stop();
	adc_atv=chanel_atv;
}

void change_freq()
{
	module_stop();
	if(usb_rs[4]=='1' && usb_rs[5]=='0' && usb_rs[6]=='0')
	{
		sen_act=1;
		tim3_prs = 7;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='1' && usb_rs[5]=='0')
	{
		sen_act=1;
		tim3_prs = 71;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='1')
	{
		sen_act=1;
		tim3_prs = 719;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='5' && usb_rs[5]=='0')
	{
		sen_act=1;
		tim3_prs = 13;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='5')
	{
		sen_act=1;
		tim3_prs = 143;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='3' && usb_rs[5]=='0')
	{
		sen_act=1;
		tim3_prs = 23;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='2' && usb_rs[5]=='0')
	{
		sen_act=1;
		tim3_prs = 35;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='2')
	{
		sen_act=1;
		tim3_prs = 359;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='0' && usb_rs[5]=='5')
	{
		sen_act=1;
		tim3_prs = 1439;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='0' && usb_rs[5]=='2')
	{
		sen_act=1;
		tim3_prs = 3599;
		MX_TIM3_Init();
	}
	else if(usb_rs[4]=='0')
	{
		sen_act=0;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,pwm_v);
	}
}

void change_freq_aq()
{
	module_stop();
	if(usb_rs[7]=='2' && usb_rs[8]=='0' && usb_rs[9]=='0')
	{
		tim2_prs = 71;
		MX_TIM2_Init();
	}
	else if(usb_rs[7]=='1' && usb_rs[8]=='0' && usb_rs[9]=='0')
	{
		tim2_prs = 143;
		MX_TIM2_Init();
	}
	else if(usb_rs[7]=='5' && usb_rs[8]=='0')
	{
		tim2_prs = 287;
		MX_TIM2_Init();
	}
	else if(usb_rs[7]=='2' && usb_rs[8]=='0')
	{
		tim2_prs = 719;
		MX_TIM2_Init();
	}
	else if(usb_rs[7]=='9')
	{
		tim2_prs = 1514;
		MX_TIM2_Init();
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(adc_atv==0)
	{
		tam=sprintf(data_tx, "%.1f\n",(float)(adc_buf[0]/GAIN_ADC));
	}
	else if(adc_atv==1)
	{
		tam=sprintf(data_tx, "%.1f\t%.1f\n",(float)(adc_buf[0]/GAIN_ADC),(float)(adc_buf[1]/GAIN_ADC));
	}
	else if(adc_atv==2)
	{
		tam=sprintf(data_tx, "%.1f\t%.1f\t%.1f\n",(float)(adc_buf[0]/GAIN_ADC),(float)(adc_buf[1]/GAIN_ADC),(float)(adc_buf[2]/GAIN_ADC));
	}
	else if(adc_atv==3)
	{
		tam=sprintf(data_tx, "%.1f\t%.1f\t%.1f\t%.1f\n",(float)(adc_buf[0]/GAIN_ADC),(float)(adc_buf[1]/GAIN_ADC),(float)(adc_buf[2]/GAIN_ADC),(float)(adc_buf[3]/GAIN_ADC));
	}

//	if(adc_atv==0)
//		{
//			tam=sprintf(data_tx, "%d\n",(adc_buf[0]/4));
//		}
//		else if(adc_atv==1)
//		{
//			tam=sprintf(data_tx, "%d\t%d\n",(adc_buf[0]/4),(adc_buf[1]/4));
//		}
//		else if(adc_atv==2)
//		{
//			tam=sprintf(data_tx, "%d\t%d\t%d\n",(adc_buf[0]/4),(adc_buf[1]/4),(adc_buf[2]/4));
//		}
//		else if(adc_atv==3)
//		{
//			tam=sprintf(data_tx, "%d\t%d\t%d\t%d\n",(adc_buf[0]/4),(adc_buf[1]/4),(adc_buf[2]/4),(adc_buf[3]/4));
//		}
	CDC_Transmit_FS(data_tx, tam);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i;
	double ang, sin_tmp;
	char char_aux;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start_DMA(&hadc1,(uint16_t*)adc_buf, 4);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
	module_stop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(strncmp(usb_rs, "start", 5) == 0)
		{
			module_start();
			index_usb = 0;
			usb_rs[0]= '\0';
		}
		else if(strncmp(usb_rs, "stop", 4) == 0)
		{
			module_stop();
			index_usb = 0;
			usb_rs[0]= '\0';
		}
		else if(strncmp(usb_rs, "act_ch", 6) == 0)
		{
			char_aux=usb_rs[6];
			if(char_aux=='0')
				active_ch(0);
			else if(char_aux=='1')
				active_ch(1);
			else if(char_aux=='2')
				active_ch(2);
			else if(char_aux=='3')
				active_ch(3);
			index_usb = 0;
			usb_rs[0]= '\0';
		}
		else if(strncmp(usb_rs, "freq_aq", 7) == 0)
		{
			change_freq_aq();
		}
		else if(strncmp(usb_rs, "freq", 4) == 0)
		{
			change_freq();
		}
		else if(strncmp(usb_rs, "pwm_v", 5) == 0)
				{
					pwm_v=(usb_rs[5]-48)*1000 + (usb_rs[6]-48)*100 + (usb_rs[7]-48)*10 + usb_rs[8]-48;
					if(pwm_v > 1000)
						pwm_v=1000;
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,pwm_v);
				}
		else if(index_usb > 2)
			index_usb=0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 4;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
