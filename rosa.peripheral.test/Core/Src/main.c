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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbh_hid.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

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

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch2;

UART_HandleTypeDef huart2;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
extern const unsigned char testNTSCImage_bytes[];
extern unsigned int testNTSCImage_length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

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

char sprintfBuffer[512];

void msgprintf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vsnprintf(sprintfBuffer, sizeof(sprintfBuffer), fmt, args);
    va_end(args);
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)sprintfBuffer, strlen(sprintfBuffer)) != HAL_OK) {
        panic();
    }
    HAL_Delay(10);
}

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD) {
        HID_KEYBD_Info_TypeDef *kbd = USBH_HID_GetKeybdInfo(phost);
        char key = USBH_HID_GetASCIICode(kbd);
        if(key != 0) {
            static char message[512];
            // sprintf(message, "keyboard: %02X : '%c'\n", key, key);
            sprintf(message, "%c", key);
            if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
                panic();
            }
            HAL_Delay(100);
        }
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

enum FirstBit {
    MSB, LSB
};

void set_bit(enum FirstBit firstBit, int bitIndex, uint8_t *bitmap, int value)
{
    int bitInByte = (bitIndex % 8);
    int byteIndex = bitIndex / 8;
    uint8_t mask;
    if(firstBit == MSB) {
        mask = 0x80U >> bitInByte;
    } else {
        mask = 0x01U << bitInByte;
    }
    uint8_t cleared = bitmap[byteIndex] & ~mask;
    uint8_t newBit = value ? mask : 0;
    bitmap[byteIndex] = cleared | newBit;
}

int get_bit(enum FirstBit firstBit, int bitIndex, uint8_t *bitmap)
{
    int bitInByte = (bitIndex % 8);
    int byteIndex = bitIndex / 8;
    uint8_t mask;
    if(firstBit == MSB) {
        mask = 0x80 >> bitInByte;
    } else {
        mask = 0x01 << bitInByte;
    }
    return (bitmap[byteIndex] & mask) != 0;
}

const int spi_bits_per_ws2812_bit = 3;

void expand_bit_pattern_lsb(int srcBitCount, int srcBitOffset, uint8_t *srcPattern, int dstBitsPerSrcBit, int dstBitOffset, uint8_t *dstPattern)
{
    for(int i = 0; i < srcBitCount; i++) {
        int srcBit = get_bit(LSB, i + srcBitOffset, srcPattern);
        for(int j = 0; j < dstBitsPerSrcBit; j++) {
            set_bit(LSB, i * dstBitsPerSrcBit + j + dstBitOffset, dstPattern, srcBit);
        }
    }
}

// Assumes MSB-first bits in src will be streamed as LSB-first from dst
void expand_bits_to_WS2812(int bitCount, uint8_t *srcPattern, uint8_t *dstPattern)
{
    for(int i = 0; i < bitCount; i++) {
        int bit = get_bit(MSB, i, srcPattern);
        if(bit) {
            set_bit(LSB, i * 3 + 0, dstPattern, 1);
            set_bit(LSB, i * 3 + 1, dstPattern, 1);
            set_bit(LSB, i * 3 + 2, dstPattern, 0);
        } else {
            set_bit(LSB, i * 3 + 0, dstPattern, 1);
            set_bit(LSB, i * 3 + 1, dstPattern, 0);
            set_bit(LSB, i * 3 + 2, dstPattern, 0);
        }
    }
}

size_t rgb_to_WS2812_NZR(uint8_t r, uint8_t g, uint8_t b, uint8_t *buffer)
{
    uint8_t *p = buffer;
    uint8_t expanded[9];

    // 3 bytes (24 bits) will be expanded to 9 bytes (72 bits)
    expand_bits_to_WS2812(8, &g, expanded + 0);
    expand_bits_to_WS2812(8, &r, expanded + 3);
    expand_bits_to_WS2812(8, &b, expanded + 6);

    // 9 bytes (72 bits) will be expanded to 27 bytes (216 bits)
    expand_bit_pattern_lsb(72, 0, expanded, spi_bits_per_ws2812_bit, 0, p); p += 27;

    return p - buffer;
}

// 47 bytes at 7.5Mbit/s yields a touch more than 50uS
// But in practice I needed this many bytes to reliably latch/reset.
#define reset_bytes 150

void write3LEDString(uint8_t colors[3][3])
{
    static uint8_t buffer[reset_bytes + 1 + 27 * 3 + reset_bytes];
    uint8_t *p = buffer;

    // Wait for SPI to finish transmitting the previous color
    while (hspi4.State != HAL_SPI_STATE_READY);

    *p++ = 0;
    for(int i = 0; i < 3; i++) {
        // Write colors to SPI buffer
        p += rgb_to_WS2812_NZR(colors[i][0], colors[i][1], colors[i][2], p);
    }
    // Write latch / reset to SPI buffer
    memset(p, 0, reset_bytes); p += reset_bytes;

    int result = HAL_SPI_Transmit_IT(&hspi4, (unsigned char *)buffer, p - buffer); // , 2);
    if(result != HAL_OK){
        static char message[512];
        sprintf(message, "SPI_Transmit error %d, error code %08lX, status %08lX\n", result, hspi4.ErrorCode, SPI4->SR);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message));
        panic();
    }
    HAL_GPIO_WritePin(RGBLED_SPI_GPIO_Port, RGBLED_SPI_Pin, GPIO_PIN_RESET);
}

void HSVToRGB3f(float h, float s, float v, float *r, float *g, float *b)
{
    if(s < .00001) {
        *r = v; *g = v; *b = v;
    } else {
    	int i;
	float p, q, t, f;

	h = fmodf(h, M_PI * 2);	/* wrap just in case */

        i = floorf(h / (M_PI / 3));

	/*
	 * would have used "f = fmod(h, M_PI / 3);", but fmod seems to have
	 * a bug under Linux.
	 */

	f = h / (M_PI / 3) - floorf(h / (M_PI / 3));

	p = v * (1 - s);
	q = v * (1 - s * f);
	t = v * (1 - s * (1 - f));
	switch(i) {
	    case 0: *r = v; *g = t; *b = p; break;
	    case 1: *r = q; *g = v; *b = p; break;
	    case 2: *r = p; *g = v; *b = t; break;
	    case 3: *r = p; *g = q; *b = v; break;
	    case 4: *r = t; *g = p; *b = v; break;
	    case 5: *r = v; *g = p; *b = q; break;
	}
    }
}

uint8_t __attribute__((section (".ram_d1"))) rowDoubleBuffer[910 * 2];
int rowNumber = 0;
int frameNumber = 0;

int why;

void DMA2_Stream1_IRQHandler(void)
{
    rowNumber = (rowNumber + 1) % 262;
    if(rowNumber == 0) {
        frameNumber ++;
    }
    uint8_t *rowDest;
    if(DMA2->LISR & DMA_FLAG_HTIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
        rowDest = rowDoubleBuffer + 0;
    } else if(DMA2->LISR & DMA_FLAG_TCIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
        rowDest = rowDoubleBuffer + 910;
    } else {
        panic();
    }
    memcpy(rowDest, testNTSCImage_bytes + rowNumber * 910, 910);

    if(DMA2->LISR) {
        // oldLISR = DMA2->LISR;
        DMA2->LIFCR = 0xFFFF;
    }
}

#if 0
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET );
    rowNumber = (rowNumber + 1) % 262;
    if(rowNumber == 0) {
        frameNumber ++;
    }
    uint8_t *rowDest = rowDoubleBuffer + 0;
    memcpy(rowDest, testNTSCImage_bytes + rowNumber * 910, 910);
    memset(rowDest, 0, 910); // XXX
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET );
    rowNumber = (rowNumber + 1) % 262;
    if(rowNumber == 0) {
        frameNumber ++;
    }
    uint8_t *rowDest = rowDoubleBuffer + 910;
    memcpy(rowDest, testNTSCImage_bytes + rowNumber * 910, 910);
    memset(rowDest, 0xFF, 910); // XXX
}
#endif

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    sprintf(sprintfBuffer, "error code %08lX, state %08X, line %d\n", hdma_tim1_ch2.ErrorCode, hdma_tim1_ch2.State, why);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)sprintfBuffer, strlen(sprintfBuffer));
    // panic();
}

void startNTSCScanout()
{
    HAL_StatusTypeDef status;
    // memcpy(testNTSCImage_RAM, testNTSCImage_bytes, sizeof(testNTSCImage_bytes));
    memcpy(rowDoubleBuffer + 0, testNTSCImage_bytes + 0, 910);
    memcpy(rowDoubleBuffer + 910, testNTSCImage_bytes + 910, 910);
    rowNumber = 1; // the previous row that was filled in

    // Set DMA request on capture-compare channel 1
    DMA2_Stream1->NDTR = sizeof(rowDoubleBuffer);
    DMA2_Stream1->M0AR = (uint32_t)rowDoubleBuffer;  // Destination address
    DMA2_Stream1->PAR = (uint32_t)&GPIOI->ODR;  // Destination address
    DMA2_Stream1->FCR = DMA_FIFOMODE_ENABLE |   // Enable FIFO to improve stutter
        DMA_FIFO_THRESHOLD_FULL;        
    DMA2_Stream1->CR |= DMA_SxCR_TCIE;    /* enable transfer complete interrupt */
    DMA2_Stream1->CR |= DMA_SxCR_HTIE;    /* enable half transfer interrupt */
    DMA2_Stream1->CR |= DMA_SxCR_EN;    /* enable DMA */
    TIM1->DIER |= TIM_DIER_CC2DE;

    status = HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);

    msgprintf("DMA2_Stream1->CR = %08lX\n", DMA2_Stream1->CR);
    msgprintf("DMA2_Stream1->NDTR = %08lX\n", DMA2_Stream1->NDTR);
    msgprintf("DMA2_Stream1->PAR = %08lX\n", DMA2_Stream1->PAR);
    msgprintf("DMA2_Stream1->M0AR = %08lX\n", DMA2_Stream1->M0AR);
    msgprintf("DMA2_Stream1->M1AR = %08lX\n", DMA2_Stream1->M1AR);
    msgprintf("DMA2_Stream1->FCR = %08lX\n", DMA2_Stream1->FCR);

    if(status != HAL_OK){
        sprintf(sprintfBuffer, "DMA error %08d, error code %08lX, line %d\n", status, hdma_tim1_ch2.ErrorCode, why);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)sprintfBuffer, strlen(sprintfBuffer));
        panic();
    }
}

int hey;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_USART2_UART_Init();
  MX_FMC_Init();
  MX_SPI4_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

    sprintf(sprintfBuffer, "Hey %d\n", hey);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)sprintfBuffer, strlen(sprintfBuffer));
    HAL_Delay(100);

  if(0){
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = USER1_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
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

    if(0){
        static uint8_t bytes[27];
        rgb_to_WS2812_NZR(0xFF, 0xFF, 0xFF, bytes);
        char *p = message;
        for(int i = 0; i < 27 * 8; i++) {
            if(i % 3 == 0)
                *p++ = ' ';
            if(i % 9 == 0)
                *p++ = ' ';
            *p++ = get_bit(LSB, i, bytes) ? '1' : '0';
        }
        *p++ = '\n';
        *p++ = '\0';
        if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
            panic();
        }
        HAL_Delay(100);
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

  uint8_t colors[3][3];

  colors[0][0] = 0x10; colors[0][1] = 0; colors[0][2] = 0;
  colors[1][0] = 0; colors[1][1] = 0x10; colors[1][2] = 0;
  colors[2][0] = 0; colors[2][1] = 0; colors[2][2] = 0x10;
  write3LEDString(colors);

  startNTSCScanout();

  int DACvalue = 0;
  while (1) {

      float now = HAL_GetTick() / 1000.0f;
      int lightLED = 0;

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // ----- USER button test
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
    // ----- DEBUG LED test
    // HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, lightLED ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // ----- WS2812B RGB LED test
    int halves = (int)(now * 2);
    if(halves % 2) {
        colors[0][0] = 0; colors[0][1] = 0; colors[0][2] = 0;
    } else {
        colors[0][0] = 0; colors[0][1] = 32; colors[0][2] = 0;
    }

    float h = now / 2.0f * 3.14159;
    float s = 1.0f;
    float v = 1.0f;
    float r, g, b;
    HSVToRGB3f(h, s, v, &r, &g, &b);
    colors[1][0] = r * 32; colors[1][1] = g * 32; colors[1][2] = b * 32;

    int phase = (int)now % 2;
    float value = phase ? (now - (int)now) : (1 - (now - (int)now));
    colors[2][0] = value * 32; colors[2][1] = value * 32; colors[2][2] = value * 32;

    write3LEDString(colors);

    // ----- Composite DAC test
    // DACvalue = (DACvalue + 1) % 256;
    // GPIOI->ODR = (GPIOI->ODR & 0xFFFFFF00) | DACvalue;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PI0 PI1 PI2 PI3
                           PI4 PI5 PI6 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
