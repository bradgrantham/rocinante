/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include <string.h>
#include <stdio.h>

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

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart2;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void panic()
{
    while(1) {
        HAL_Delay(200);
        HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
    }
}

uint32_t getMicros(void)
{
    uint32_t micros = HAL_RCC_GetSysClockFreq() / 1000000;

    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());
    return (ms * 1000) + (micros * 1000 - cycle_cnt) / micros;
}

void delayMicros(uint16_t micros)
{
    uint32_t start = getMicros();
    while (getMicros()-start < (uint32_t) micros) {
        asm("nop");
    }
}

void delayNanos(uint16_t nanos)
{
    uint32_t then = SysTick->VAL;
    uint32_t nanosPerCycle = 1000000000 / HAL_RCC_GetSysClockFreq();
    uint32_t cycles = nanos / nanosPerCycle;
    uint32_t elapsedCycles;
    do {
        uint32_t now = SysTick->VAL;
        elapsedCycles = (now - then);
    } while(elapsedCycles < cycles);
}

const int WS2812B_bitlength_nanos = 400;
const int WS2812B_reset_micros = 50;

// This one expands every bit into 3 , LSBit first
void one_byte_to_three_bytes(uint8_t byte, uint32_t *pattern)
{
    *pattern = 0;
    for(int i = 0; i < 8; i++) {
        int bits[3];
        if(byte & (1 << i)) {
            bits[0] = 1; bits[1] = 1; bits[2] = 1;
        } else {
            bits[0] = 0; bits[1] = 0; bits[2] = 0;
        }
        for(int j = 0; j < 3; j++) {
            int bit = (i * 3 + j);
            *pattern |= (bits[j] << bit);
        }
    }
}

// This one expands every bit into WS2812's weird modulation, MSBit first.
void byte_to_WS2812_bit_pattern(uint8_t byte, uint32_t *pattern)
{
    static int bits[3];
    *pattern = 0;
    for(int i = 0; i < 8; i++) {
        if(byte & (1 << (7 - i))) {
            bits[0] = 1; bits[1] = 1; bits[2] = 0;
        } else {
            bits[0] = 1; bits[1] = 0; bits[2] = 0;
        }
        for(int j = 0; j < 3; j++) {
            int bit = (i * 3 + j);
            *pattern |= (bits[j] << bit);
        }
    }
}

void write24Bits(uint32_t bits)
{
#if 0
    for(int i = 0; i < 24; i++) {
        int bit = bits & (1 << i);
        HAL_GPIO_WritePin(RGBLED_GPIO_Port, RGBLED_Pin, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
        // This could be hosed by an interrupt
        delayNanos(WS2812B_bitlength_nanos);
        if(0) {
            if(bit) {
                HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
                HAL_Delay(900);
                HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
                HAL_Delay(100);
            } else {
                HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
                HAL_Delay(900);
            }
        }
    }
#else
    int result = HAL_SPI_Transmit_IT(&hspi4, (unsigned char *)&bits, 3);
    if(result != HAL_OK){
        static char message[512];
        sprintf(message, "SPI_Transmit_IT error %d\n", result);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message));
        panic();
    }
#endif
}

void threebytes_to_ninebytes(uint32_t bitpattern, uint8_t *bytes)
{
    uint32_t bitpattern2;
    one_byte_to_three_bytes((bitpattern >> 0) & 0xFF, &bitpattern2);
    bytes[0] = (bitpattern2 >> 0) & 0xFF;
    bytes[1] = (bitpattern2 >> 8) & 0xFF;
    bytes[2] = (bitpattern2 >> 16) & 0xFF;
    one_byte_to_three_bytes((bitpattern >> 8) & 0xFF, &bitpattern2);
    bytes[3] = (bitpattern2 >> 0) & 0xFF;
    bytes[4] = (bitpattern2 >> 8) & 0xFF;
    bytes[5] = (bitpattern2 >> 16) & 0xFF;
    one_byte_to_three_bytes((bitpattern >> 16) & 0xFF, &bitpattern2);
    bytes[6] = (bitpattern2 >> 0) & 0xFF;
    bytes[7] = (bitpattern2 >> 8) & 0xFF;
    bytes[8] = (bitpattern2 >> 16) & 0xFF;
}

size_t rgb_to_WS2812_pattern(uint8_t r, uint8_t g, uint8_t b, uint8_t *buffer)
{
    uint8_t *p = buffer;
    uint32_t bitpattern;
    byte_to_WS2812_bit_pattern(g, &bitpattern);
    threebytes_to_ninebytes(bitpattern, p); p += 9;
    byte_to_WS2812_bit_pattern(r, &bitpattern);
    threebytes_to_ninebytes(bitpattern, p); p += 9;
    byte_to_WS2812_bit_pattern(b, &bitpattern);
    threebytes_to_ninebytes(bitpattern, p); p += 9;
    return p - buffer;
}

void writeColor(uint8_t r, uint8_t g, uint8_t b)
{
    static uint8_t buffer[28];
    static uint8_t *p = buffer;

    if(1) {
        // If I don't first send 0's, I get a weird long spike for the first bit.
        *p++ = 0x0;
    }
    p += rgb_to_WS2812_pattern(r, g, b, p);

    int result = HAL_SPI_Transmit(&hspi4, (unsigned char *)buffer, p - buffer, 1000000);
    if(result != HAL_OK){
        static char message[512];
        sprintf(message, "SPI_Transmit error %d\n", result);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message));
        panic();
    }
    HAL_GPIO_WritePin(RGBLED_SPI_GPIO_Port, RGBLED_SPI_Pin, GPIO_PIN_RESET);
}

void resetLEDstring()
{
    static uint8_t buffer[50];
    memset(buffer, 0, sizeof(buffer));
    int result = HAL_SPI_Transmit(&hspi4, (unsigned char *)buffer, sizeof(buffer), 1000000);
    if(result != HAL_OK){
        static char message[512];
        sprintf(message, "SPI_Transmit error %d\n", result);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message));
        panic();
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
  MX_USART2_UART_Init();
  MX_FMC_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(RGBLED_SPI_GPIO_Port, RGBLED_SPI_Pin, GPIO_PIN_RESET);

    static char message[512];
    sprintf(message, "Hello World\n");
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
        panic();
    }
    HAL_Delay(100);

    sprintf(message, "System clock is %lu\n", HAL_RCC_GetSysClockFreq());
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
        panic();
    }
    HAL_Delay(100);

    uint32_t bits;
    byte_to_WS2812_bit_pattern(0xA5, &bits);
    // 6592CB -> 0110 0101 1001 0010 1100 1011
    // -> 110100110100100110100110
    // A5 is 10100101, should be 110100110100100110100110
    sprintf(message, "bits would become %08lX\n", bits);
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
        panic();
    }
    HAL_Delay(100);
    if(0){
        static uint8_t bytes[27];
        rgb_to_WS2812_pattern(0xFF, 0xFF, 0xFF, bytes);
        for(int i = 0; i < 27; i++) {
            sprintf(message, "%d%d%d%d%d%d%d%d\n",
                (bytes[i] >> 7) & 1,
                (bytes[i] >> 6) & 1,
                (bytes[i] >> 5) & 1,
                (bytes[i] >> 4) & 1,
                (bytes[i] >> 3) & 1,
                (bytes[i] >> 2) & 1,
                (bytes[i] >> 1) & 1,
                (bytes[i] >> 0) & 1);
            if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
                panic();
            }
            HAL_Delay(100);
        }
    }

    while(0) {
        for(int i = 0; i < 1000000; i++) {
            delayNanos(1000);
        }
        HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET );
        for(int i = 0; i < 1000000; i++) {
            delayNanos(1000);
        }
        HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET );
    }

  int LEDvalue = 127;
  resetLEDstring();
  writeColor(0x0F, 0, 0);
  writeColor(0, 0x0F, 0);
  writeColor(0, 0, 0x0F);
  while (1)
  {
      int lightLED = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 0
    if(HAL_GPIO_ReadPin(USER1_GPIO_Port, USER1_Pin)) {
        // sprintf("user 1\n");
        // if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
            // panic();
        // }
        // HAL_Delay(100);
        lightLED = 1;
    }
#endif
    if(HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin)) {
        // sprintf("user 2\n");
        // if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
            // panic();
        // }
        // HAL_Delay(100);
        lightLED = 1;
    }
    if(HAL_GPIO_ReadPin(USER3_GPIO_Port, USER3_Pin)) {
        // sprintf("user 2\n");
        // if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
            // panic();
        // }
        // HAL_Delay(100);
        lightLED = 1;
    }
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, lightLED ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if(lightLED) {
        resetLEDstring();
        writeColor(LEDvalue, 0, 0);
        writeColor(0, LEDvalue, 0);
        writeColor(0, 0, LEDvalue);
        LEDvalue = (LEDvalue + 1) % 256;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_3;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_2;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER2_Pin USER3_Pin */
  GPIO_InitStruct.Pin = USER2_Pin|USER3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
