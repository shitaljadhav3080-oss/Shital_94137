/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
//-------------------------------------------------------------------------------------------------------
#include<stdio.h>
#include "my_lcd.h"
//-------------------------------------------------------------------------------------------------------
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

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//-----------------------------------------------------------------------------------------------------------------------

#define DHT11_PORT GPIOB
#define DHT11_PIN  GPIO_PIN_0

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


void DWT_Delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}
//-----------------------------------
/* ---------- GPIO direction control ---------- */
static void DHT11_Pin_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

static void DHT11_Pin_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}


/* ---------- Main Read Function ---------- */

uint8_t DHT11_Read(uint8_t *temperature, uint8_t *humidity)
{
    uint8_t data[5] = {0};
    uint32_t timeout;

    /* ===== MCU start signal ===== */
    DHT11_Pin_Output();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    DWT_Delay_us(30);

    DHT11_Pin_Input();
    DWT_Delay_us(10);   // <-- IMPORTANT

    /* ===== DHT11 response ===== */
    // LOW (80us)
    timeout = 100;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    {
        if (--timeout == 0)
        	return 1;
        DWT_Delay_us(1);
    }

    // HIGH (80us)
    timeout = 100;
    while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    {
        if (--timeout == 0)
        	return 1;
        DWT_Delay_us(1);
    }

    // LOW before data
    timeout = 100;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    {
        if (--timeout == 0) return 1;
        DWT_Delay_us(1);
    }

    /* ===== Read 5 bytes ===== */
    for (uint8_t byte = 0; byte < 5; byte++)
    {
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            timeout = 100;
            while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
            {
                if (--timeout == 0)
                	return 2;
                DWT_Delay_us(1);
            }

            DWT_Delay_us(30);

            if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
                data[byte] |= (1 << (7 - bit));

            timeout = 100;
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
            {
                if (--timeout == 0)
                	return 2;
                DWT_Delay_us(1);
            }
        }
    }

    /* ===== Checksum ===== */
//    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
//        return 3;

    *humidity    = data[0];
    *temperature = data[2];

    return 0;
}
//---------------------------------------------------------------------------------------------------------------------
uint16_t MQ135_Read_ADC(void)
{
    uint16_t adc_value = 0;

    HAL_ADC_Start(&hadc1);                      // Start ADC
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversion
    adc_value = HAL_ADC_GetValue(&hadc1);       // Read ADC value
    HAL_ADC_Stop(&hadc1);                       // Stop ADC

    return adc_value;                           // 0 to 4095
}

/***************************
Note : MQ-22 => Connect ADC1 => Pin PA0
       DHT-11 => GPIO I/P    => Pin PB0

//*******************************/


//------------------------------------------------------------------------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//----------------------------------------------------------------------------------------------------------------------------
	char str[20];
	uint8_t temp,humdt;
//----------------------------------------------------------------------------------------------------------------------------
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //--------------------------------------------------------------------------------------------
  DWT_Init();
  if ( !lcd16x2_i2c_init(&hi2c1))
    {
        // LCD not detected
        while(1);
    }

    lcd16x2_i2c_clear();

    //Print on 1st line
    lcd16x2_i2c_setCursor(0, 0);   // Row 0, Col 0
    lcd16x2_i2c_printf("sunbeam");
    DWT_Delay_us(2000000);
    uint8_t test;
    uint16_t mq135_adc;
//---------------------------------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd16x2_i2c_clear();
	  lcd16x2_i2c_printf("loop");
	  DWT_Delay_us(2000000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  test = DHT11_Read(&temp,&humdt);
	  sprintf(str,"retun = %u ",test);
	  lcd16x2_i2c_clear();
	  lcd16x2_i2c_printf(str);
	  DWT_Delay_us(2000000);

	  if(test == 0 )
	  {
		  sprintf(str,"Temp = %u ",temp);
		  lcd16x2_i2c_clear();
		  lcd16x2_i2c_printf(str);
		  lcd16x2_i2c_setCursor(1, 0);
		  sprintf(str,"Humi = %u",humdt);
		  lcd16x2_i2c_printf(str);
		  DWT_Delay_us(2000000);
	  }
	  else
	  {
		  lcd16x2_i2c_clear();
		  lcd16x2_i2c_printf("else");
		  DWT_Delay_us(2000000);
	  }
	  //-----------------------------------------------------------
	  	  lcd16x2_i2c_clear();
	  	  mq135_adc = MQ135_Read_ADC();

	  	  sprintf(str,"Gas: %u",mq135_adc);
	  	  lcd16x2_i2c_printf(str);
	  	DWT_Delay_us(2000000);
	  //-----------------------------------------------------------
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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
#ifdef USE_FULL_ASSERT
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
