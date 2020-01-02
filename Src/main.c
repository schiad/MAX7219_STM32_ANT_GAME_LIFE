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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
struct St_Ant ant1;
char buff[11];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void to_bin(uint8_t nb, char* str, uint8_t byte) {
	static uint8_t b;

	b = 0;
	while (b < byte) {
		str[b] = (((nb >> b) & 1) + '0');
		b++;
	}
	str[b] = ' ';
	b++;
	str[b] = ' ';
	b++;
	str[b] = '\n';
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		ant1.x = 0;
	}
}

void MAXIM(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t comm) {
	static uint8_t data[2];
	data[0] = addr;
	data[1] = comm;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	HAL_SPI_Transmit(hspi, &data[0], 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}

void MAXIM_Init(SPI_HandleTypeDef *hspi) {
	uint8_t i;
	i = 0;
	MAXIM(hspi, 0xA, 0x0); // LED INTENSITY
	while (i < 8) {
		MAXIM(hspi, 0xA, 0x0);
		MAXIM(hspi, i + 1, 0x0);
		i++;
	}
	MAXIM(hspi, 0xC, 0x1);
	MAXIM(hspi, 0xB, 0x7);
	MAXIM(hspi, 0xA, 0x0); // LED INTENSITY
	MAXIM(hspi, 0xF, 0x1);
	MAXIM(hspi, 0xF, 0x0);
	MAXIM(hspi, 0x9, 0x0);
	i = 0;
}

void Maxim_Map(SPI_HandleTypeDef *hspi, uint8_t *map) {
	static uint8_t i;
	i = 0;
	while (i < 8) {
		MAXIM(hspi, i + 1, map[i]);
		i++;
	}
}

void init_map(struct St_Ant* ant, uint8_t init) {
	ant->y = 0;
	switch (init) {
	case 0:			// Create a blank map
		while ((ant->y) < 8) {
			ant->map[ant->y] = 0;
			ant->y++;
		}
		break;
	case 1:			//Create zebra map
		while ((ant->y) < 8) {
			ant->map[ant->y] = 0b10101010;
			ant->y++;
		}
		break;
	case 5:			// create a randomness map
		while ((ant->y) < 8) {
			HAL_ADC_Start(&hadc1);
			while (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)) {
				;
			}
			srand(HAL_ADC_GetValue(&hadc1));
			ant->map[ant->y] = rand() & 0b11111111;
			ant->y++;
		}
		break;
	case 2:			// Le crapeau
		ant->map[0] = 0b00000000;
		ant->map[1] = 0b00000000;
		ant->map[2] = 0b00000000;
		ant->map[3] = 0b01110000;
		ant->map[4] = 0b00111000;
		ant->map[5] = 0b00000000;
		ant->map[6] = 0b00000000;
		ant->map[7] = 0b00000000;
		break;
	case 3:			// Le planneur
		ant->map[4] = 0b01000000;
		ant->map[5] = 0b00100000;
		ant->map[6] = 0b11100000;
		ant->map[7] = 0b00000000;
		ant->map[0] = 0b00000000;
		ant->map[1] = 0b00000000;
		ant->map[2] = 0b00000000;
		ant->map[3] = 0b00000000;
		break;
	case 4:			// 2 Clignotants
		ant->map[0] = 0b01000000;
		ant->map[1] = 0b01000000;
		ant->map[2] = 0b01000000;
		ant->map[3] = 0b00000000;
		ant->map[4] = 0b00000000;
		ant->map[5] = 0b00001110;
		ant->map[6] = 0b00000000;
		ant->map[7] = 0b00000000;
		break;

	case 6:			// La balise
		ant->map[0] = 0b00000000;
		ant->map[1] = 0b00000000;
		ant->map[2] = 0b00000000;
		ant->map[3] = 0b00001100;
		ant->map[4] = 0b00001100;
		ant->map[5] = 0b00110000;
		ant->map[6] = 0b00110000;
		ant->map[7] = 0b00000000;
		break;
	case 7:			// Vaisseau
		ant->map[0] = 0b00000000;
		ant->map[1] = 0b00000000;
		ant->map[2] = 0b00111100;
		ant->map[3] = 0b01000100;
		ant->map[4] = 0b00000100;
		ant->map[5] = 0b01001000;
		ant->map[6] = 0b00000000;
		ant->map[7] = 0b00000000;
		break;
	default:
		break;
	}
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	MAXIM_Init(&hspi1);
}

void Ant(struct St_Ant* ant) {
	if ((ant->map[ant->y] >> (ant->x)) & 1) {
		ant->dir += 1;
		ant->map[ant->y] = ant->map[ant->y] - (1 << ant->x);
	} else {
		ant->dir -= 1;
		ant->map[ant->y] = ant->map[ant->y] + (1 << ant->x);
	}

	ant->dir = ant->dir % 4;

	switch (ant->dir) {
	case 0:
		ant->y += 1;
		break;
	case 1:
		ant->x -= 1;
		break;
	case 2:
		ant->y -= 1;
		break;
	case 3:
		ant->x += 1;
		break;
	default:
		break;
	}
	ant->x = ant->x % 8;
	ant->y = ant->y % 8;
	Maxim_Map(&hspi1, ant->map);
}

int8_t mod(int8_t x) {
	static int ret;
	ret = x % 8;

	return ret;
}

int8_t cell_near(uint8_t *mapi, uint8_t ay, uint8_t ax) {
	static int8_t dy, cx, dx, cy;
	static int8_t nearby;

	nearby = 0;

	dy = 7;
	while (dy <= 9) {
		dx = 7;
		while (dx <= 9) {
			cy = mod(ay + dy);
			cx = mod(ax + dx);
			if (((mapi[cy] >> cx) & 1)) {
				if (dy == 8 && dx == 8) {
					;
				} else {
					nearby++;
				}
			}
			dx++;
		}
		dy++;
	}

	return nearby;
}

void life_game(struct St_Ant* ant) {
	uint8_t tmp_map[8];
	uint8_t x;
	uint8_t nearby;

	ant->y = 0;

	while (ant->y < 8) {
		x = 0;
		tmp_map[ant->y] = ant->map[ant->y];
		while (x < 8) {
			if ((tmp_map[ant->y] >> x) & 1) {
			} else {
			}
			x++;
		}
		ant->y++;
	}

	ant->y = 0;
	x = 0;

	while (ant->y < 8) {
		x = 0;
		while (x < 8) {
			nearby = cell_near(&tmp_map[0], ant->y, x);
			if (nearby == 3) {
				ant->map[ant->y] |= (1 << x);
				if (tmp_map[ant->y] && (1 << x)) {
					;
				} else {
					ant->x = 10;
				}
			}
			if ((nearby > 3) || (nearby < 2)) {
				ant->map[ant->y] &= ~(1 << x);
			}
			x++;
		}
		ant->y++;
	}
	ant->x--;
	if (!ant->x) {
		init_map(ant, 5);
		ant->x = 10;
	}
	Maxim_Map(&hspi1, ant->map);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	uint8_t i;
	uint8_t map;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	init_map(&ant1, 7);
	HAL_TIM_Base_Start_IT(&htim1);
	ant1.x = 7;
	ant1.y = 7;
	ant1.dir = 0;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
	MAXIM_Init(&hspi1);
	Maxim_Map(&hspi1, ant1.map);
	HAL_Delay(1000);

	while (1) {
		if (!ant1.x) {
			i = 0;
			while (i < 8) {
				map = ant1.map[i];
				to_bin(map, buff, 8);
				CDC_Transmit_FS(buff, 11);
				HAL_Delay(1);
				i++;
			}
			init_map(&ant1.map, 5);
			__HAL_TIM_SET_COUNTER(&htim1, 0);
			//__HAL_TIM_GET_COUNTER(&htim1);
			CDC_Transmit_FS("\r\n", 2);
		}
		//Ant(&ant1);
		life_game(&ant1);
		HAL_Delay(100);

		// dir 0 = N;  1 = E;  2 = S; 3 = O;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 35999;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 35999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
