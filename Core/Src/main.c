/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "atraso.h"
#include "defPrincipais.h"
#include "NOKIA5110_fb.h"
#include "figuras.h"
#include "PRNG_LFSR.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

//osThreadId defaultTaskHandle;
//uint32_t defaultTaskBuffer[ 128 ];
//osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */

struct pontos_t _carro;
struct pontos_t _pedra[4];

uint16_t pontuacao = 0;
uint8_t vidas = 3;

uint32_t ADC_buffer[2];
uint32_t valor_ADC[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);

void vTask_LCD_Print(void *pvParameters);
void vTask_Carro(void *pvParameters);
void vTask_Pista(void *pvParameters);
void vTask_Pontuacao(void *pvParameters);
void vTask_Obstaculo(void *pvParameters);
void vTask_GameOver(void *pvParameters);

xTaskHandle xTask_GameOver_Handle;
xTaskHandle xTask_Print_Handle;
xTaskHandle xTask_Carro_Handle;
xTaskHandle xTask_Pista_Handle;
xTaskHandle xTask_Pontuacao_Handle;
xTaskHandle xTask_Obstaculo_Handle;

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		valor_ADC[0]=ADC_buffer[0];
		valor_ADC[1]=ADC_buffer[1];
	}
}

//---------------------------------------------------------------------------------------------------
// Tarefa para atualizar periodicamente o LCD
void vTask_LCD_Print(void *pvParameters)
{
	while(1)
	{
		imprime_LCD();
	}

}
//---------------------------------------------------------------------------------------------------

void vTask_Carro(void *pvParameters)
{
	static uint8_t x = 40, y = 19;
	static int32_t dif_eixoX, dif_eixoY;

	while(1)
	{
		dif_eixoX = 2048 - valor_ADC[0]; // media_eixoX;
		dif_eixoY = 2048 - valor_ADC[1]; //media_eixoY;

		// Le o ADC e converte
		if(dif_eixoX < -200)
		{
			if(x<65) x++;
		}
		else if(dif_eixoX > 200)
		{
			if(x>0) x--;
		}
		// ------------------------------------------------------------------------
		if(dif_eixoY < -200)
		{
			if(y<35) y++;
		}
		else if(dif_eixoY > 200)
		{
			if(y>7) y--;
		}

		_carro.x1 = x;  _carro.y1 = y;

		//Desenha o carro no Display
		desenha_fig(&_carro,&carro);
		vTaskDelay(40);
		desenha_fig(&_carro,&apaga_carro);
	}
}

void vTask_Pista(void *pvParameters)
{
	struct pontos_t _pistaS;
	struct pontos_t _pistaI;

	//Define as posicoes das linhas da pista
	_pistaS.x1 = 11;
	_pistaS.y1 = 5;
	_pistaI.x1 = 11;
	_pistaI.y1 = 46;

	while(1)
	{
		//Realiza o deslocamento no display
		_pistaS.x1--;
		_pistaI.x1--;

		if(_pistaS.x1 == 0){
			_pistaS.x1 = 11;
			_pistaI.x1 = 11;
		}

		desenha_fig(&_pistaS, &pista);
		desenha_fig(&_pistaI, &pista);

		vTaskDelay(200);
	}
}

void vTask_Pontuacao(void *pvParameters)
{
	struct pontos_t _vidas;
	uint8_t i=0,j=0;
	_vidas.y1 = 0;

	while(1)
	{
		vTaskDelay(50);

		// Verifica se ouve colisao
		for(i=0;i<4;i++)
		{
			if(	(_carro.x1 <= _pedra[i].x1+3) && ((_carro.x1+19) >= (_pedra[i].x1+3)) &&
				(_carro.y1 <= _pedra[i].y1+3) && ((_carro.y1+11) >= (_pedra[i].y1+3)))
			{
				desenha_fig(&_pedra[i],&apaga_pedra);
				_pedra[i].x1=200;
				_pedra[i].y1=0;
				vidas--;
			}
		}

		// Desenha as vidas no display
		_vidas.x1 = 12;
		if(vidas == 3){ desenha_fig(&_vidas, &coracao); }
		else { desenha_fig(&_vidas, &apaga_coracao); }

		_vidas.x1 = 6;
		if(vidas >= 2){ desenha_fig(&_vidas, &coracao); }
		else { desenha_fig(&_vidas, &apaga_coracao); }

		_vidas.x1 = 0;
		if(vidas >= 1){ desenha_fig(&_vidas, &coracao); }
		else { desenha_fig(&_vidas, &apaga_coracao); }

		// Finaliza o jogo caso:
		if(vidas <= 0)
		{
			xTaskCreate(vTask_GameOver, "Game Over", 128, NULL, 2, &xTask_GameOver_Handle);
		}

		// Encrementa e imprime a pontuacao
		j++;
		if(j==10)
		{
			pontuacao ++;
			escreve_Nr_Peq(36,0,pontuacao,3);
			j=0;
		}
	}
}

void vTask_Obstaculo(void *pvParameters)
{
	uint8_t i=0, k=0;

	while(1)
	{
		// Gera as pedras aleatoriamente
		if(i==0)
		{
			_pedra[k].x1 = 84;
			_pedra[k].y1 = prng_LFSR() % 32 + 7;
			k++;
			i=20;
			if(k >= 4) k=0;
		}
		i--;

		// Desloca as pedras no display
		_pedra[0].x1--;
		_pedra[1].x1--;
		_pedra[2].x1--;
		_pedra[3].x1--;

		desenha_fig(&_pedra[0], &pedra);
		desenha_fig(&_pedra[1], &pedra);
		desenha_fig(&_pedra[2], &pedra);
		desenha_fig(&_pedra[3], &pedra);

		vTaskDelay(200);

		desenha_fig(&_pedra[0], &apaga_pedra);
		desenha_fig(&_pedra[1], &apaga_pedra);
		desenha_fig(&_pedra[2], &apaga_pedra);
		desenha_fig(&_pedra[3], &apaga_pedra);
	}
}

void vTask_GameOver(void *pvParameters)
{
	while(1)
	{
		vTaskPrioritySet(xTask_Print_Handle, 2);
		limpa_LCD();
		escreve2fb((unsigned char *)tela_final);
		goto_XY(45, 4);
		string_LCD("Pontos");
		goto_XY(50, 3);
		string_LCD_Nr("", pontuacao, 3);

		vTaskDelay(14); // Delay para a impressao

		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))// quando pressionar a tecla reinicia o jogo
		{

		}

		// Reinicia o game
		limpa_LCD();
		vidas = 3;
		pontuacao = 0;

		vTaskPrioritySet(xTask_Print_Handle, osPriorityNormal);
		vTaskDelete(xTask_GameOver_Handle);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t semente_PRNG=1;
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
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_buffer,2);
	HAL_ADC_Start_IT(&hadc1);

	// inicializa LCD 5110
	inic_LCD();
	limpa_LCD();

	// --------------------------------------------------------------------------------------
	// inicializa tela do jogo
	escreve2fb((unsigned char *)tela_inicial);
	HAL_Delay(1000);

	goto_XY(50, 3);
	string_LCD("Click");
	imprime_LCD();

	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))// quando pressionar a tecla come�a o jogo
	{
		semente_PRNG++;
	}

	init_LFSR(semente_PRNG);	// inicializacao para geracao de numeros pseudoaleatorios
	limpa_LCD();

	// -----------------------------------------

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	xTaskCreate(vTask_LCD_Print, "Task_Print", 128, NULL, osPriorityNormal,xTask_Print_Handle);
	xTaskCreate(vTask_Carro, "Task_Carro", 128, NULL, osPriorityNormal,xTask_Carro_Handle);
	xTaskCreate(vTask_Pista, "Task_Pista", 128, NULL, osPriorityNormal,xTask_Pista_Handle);
	xTaskCreate(vTask_Pontuacao, "Task_Pontuacao", 128, NULL, osPriorityAboveNormal,xTask_Pontuacao_Handle);
	xTaskCreate(vTask_Obstaculo, "Task_Obstaculo", 128, NULL, osPriorityNormal,xTask_Obstaculo_Handle);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

