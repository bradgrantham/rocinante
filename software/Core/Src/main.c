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
#include "fatfs.h"
#include "libjpeg.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbh_hid.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "rocinante.h"
#include "hid.h"
#include "events.h"
#include "text-mode.h"
#include "ui.h"
#include "8x16.h"

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

DAC_HandleTypeDef hdac1;

JPEG_HandleTypeDef hjpeg;

SD_HandleTypeDef hsd2;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim1_ch2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

FATFS gFATVolume;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_FMC_Init(void);
static void MX_UART4_Init(void);
static void MX_JPEG_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----------------------------------------------------------------------------
// From: https://kbiva.wordpress.com/2013/03/25/microsecond-delay-function-for-stm32/

void delay_init(void) 
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) 
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}
 
uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}
 
static __inline
uint8_t DWT_Compare(int32_t tp)
{
  return (((int32_t)DWT_Get() - tp) < 0);
}
 
void delay_us(uint32_t us) // microseconds
{
  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}

// End of from: https://kbiva.wordpress.com/2013/03/25/microsecond-delay-function-for-stm32/

enum { SDRAM_TEST_STEP_SIZE = 65536 };
enum { SDRAM_START = 0x70000000 };

void memcpy_fast_16byte_multiple(void* dst_, const void* src_, size_t size)
{
    const uint32_t* src = (uint32_t*)src_;
    uint32_t* dst = (uint32_t*)dst_;
#pragma GCC unroll 8
    for(size_t i = 0; i < size / sizeof(*dst); i += 4 /* i++ */) {
        uint32_t t0 = src[0];
        uint32_t t1 = src[1];
        uint32_t t2 = src[2];
        uint32_t t3 = src[3];
        dst[0] = t0;
        dst[1] = t1;
        dst[2] = t2;
        dst[3] = t3;
        src += 4;
        dst += 4;
        // *dst++ = *src++;
    }
}


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Assumed to be raw mode - returns next character, not buffering until newline
int __io_getchar(void)
{
#if 0
    while(1) {
#ifdef USE_PS2KBD
        int c = KBD_process_queue(gDumpKeyboardData);
        if(c >= 0) {
            return c;
        }
#endif
        SERIAL_poll_continue();
        unsigned char isEmpty = queue_isempty(&mon_queue);
        if(!isEmpty) {
            unsigned char c = queue_deq(&mon_queue);
            return c;
        }
        HAL_Delay(10);
    }
#endif
    return -1;
} 

volatile int USARTBusy = 0;

static int serial_which = 0;
static int serial_where = 0;
static char serial_buffer[2][256];

void console_flush()
{
    while(USARTBusy);
    USARTBusy = 1;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&serial_buffer[serial_which], serial_where);
    // HAL_UART_Transmit(&huart2, (uint8_t *)&serial_buffer[serial_which], serial_where, 100);
    serial_which = (serial_which + 1) % 2;
    serial_where = 0;
}


void __io_putchar( char c )
{
    serial_buffer[serial_which][serial_where++] = c;
    if(/* !USARTBusy || */ (c == '\n') || (serial_where >= sizeof(serial_buffer[0]))) {
        console_flush();
    }
}  

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    USARTBusy = 0;
}

#ifdef __cplusplus
};
#endif /* __cplusplus */

void panic()
{
    while(1) {
        HAL_Delay(200);
        HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
    }
}

void RoDelayMillis(uint32_t millis)
{
    HAL_Delay(millis);
}

uint32_t RoGetMillis()
{
    return HAL_GetTick();
}

void RoPanic()
{
    panic();
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

enum { SDRAM_TIMEOUT = 0xFFFF };

void SDRAMInit()
{
    int result;
    static FMC_SDRAM_CommandTypeDef cmd;

    cmd.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
    cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber = 1;
    cmd.ModeRegisterDefinition = 0;
    result = HAL_SDRAM_SendCommand(&hsdram1, &cmd, SDRAM_TIMEOUT);
    if(result != HAL_OK){
        printf("HAL_SDRAM_SendCommand(CLK_ENABLE) error %d\n", result);
        panic();
    }

    HAL_Delay(1);

    cmd.CommandMode = FMC_SDRAM_CMD_PALL;
    cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber = 1;
    cmd.ModeRegisterDefinition = 0;
    result = HAL_SDRAM_SendCommand(&hsdram1, &cmd, SDRAM_TIMEOUT);
    if(result != HAL_OK){
        printf("HAL_SDRAM_SendCommand(CLK_PALL) error %d\n", result);
        panic();
    }

    cmd.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
    cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber = 1;
    cmd.ModeRegisterDefinition = 0x0223; // SDRAM_MODEREG_BURST_LENGTH_8 |
         // SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |
         // SDRAM_MODEREG_CAS_LATENCY_2 |
         // SDRAM_MODEREG_OPERATING_MODE_STANDARD |
         // SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
    result = HAL_SDRAM_SendCommand(&hsdram1, &cmd, SDRAM_TIMEOUT);
    if(result != HAL_OK){
        printf("HAL_SDRAM_SendCommand(CLK_ENABLE) error %d\n", result);
        panic();
    }

    cmd.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber = 8;
    cmd.ModeRegisterDefinition = 0;
    result = HAL_SDRAM_SendCommand(&hsdram1, &cmd, SDRAM_TIMEOUT);
    if(result != HAL_OK){
        printf("HAL_SDRAM_SendCommand(AUTOREFRESH_MODE) error %d\n", result);
        panic();
    }

    HAL_Delay(1);

    cmd.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber = 8;
    cmd.ModeRegisterDefinition = 0;
    result = HAL_SDRAM_SendCommand(&hsdram1, &cmd, SDRAM_TIMEOUT);
    if(result != HAL_OK){
        printf("HAL_SDRAM_SendCommand(AUTOREFRESH_MODE) error %d\n", result);
        panic();
    }

    HAL_Delay(1);

    result = HAL_SDRAM_ProgramRefreshRate(&hsdram1, 1250);
    if(result != HAL_OK){
        printf("HAL_SDRAM_ProgramRefreshRate error %d\n", result);
        panic();
    }
}

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD) {

        HID_KEYBD_Info_TypeDef *kbd = USBH_HID_GetKeybdInfo(phost);
        if(0) printf("%02X %c%c%c%c%c%c%c%c [%d, %d, %d, %d, %d, %d]\n",
            kbd->state,
            kbd->lctrl ? '+' : '_',
            kbd->lshift ? '+' : '_',
            kbd->lalt ? '+' : '_',
            kbd->lgui ? '+' : '_',
            kbd->rctrl ? '+' : '_',
            kbd->rshift ? '+' : '_',
            kbd->ralt ? '+' : '_',
            kbd->rgui ? '+' : '_',
            kbd->keys[0], kbd->keys[1], kbd->keys[2], kbd->keys[3], kbd->keys[4], kbd->keys[5]);

        int modifiers[8];
        modifiers[0] = kbd->lctrl;
        modifiers[1] = kbd->lshift;
        modifiers[2] = kbd->lalt;
        modifiers[3] = kbd->lgui;
        modifiers[4] = kbd->rctrl;
        modifiers[5] = kbd->rshift;
        modifiers[6] = kbd->ralt;
        modifiers[7] = kbd->rgui;
        ConvertUSBModifiersToKeyEvent(modifiers);

        int keys[6];
        for(int i = 0; i < 6; i++) {
            keys[i] = kbd->keys[i];
        }
        ConvertUSBKeysToKeyEvent(keys);

    } else if(USBH_HID_GetDeviceType(phost) == HID_MOUSE) {

        HID_MOUSE_Info_TypeDef *mouse = USBH_HID_GetMouseInfo(phost);
        int dx = (mouse->x >= 128) ? (mouse->x - 256) : mouse->x;
        int dy = (mouse->y >= 128) ? (mouse->y - 256) : mouse->y;
        int buttons[3];
        for(int i = 0; i < 3; i++) {
            buttons[i] = mouse->buttons[i];
        }
        ConvertUSBMouseToMouseEvent(dx, dy, buttons);
        if(0)printf("mouse %d, %d, %d, %d, %d\n", mouse->x, mouse->y,
            mouse->buttons[0], mouse->buttons[1], mouse->buttons[2]);
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

int LEDBusy = 0;

void write3LEDString(uint8_t colors[3][3])
{
    static uint8_t buffer[reset_bytes + 1 + 27 * 3 + reset_bytes];
    uint8_t *p = buffer;

    // Wait for SPI to finish transmitting the previous color
    // while (hspi4.State != HAL_SPI_STATE_READY);
    HAL_Delay(1); // XXX if I don't do this the LED write never happens??
    while(LEDBusy);

    *p++ = 0;
    for(int i = 0; i < 3; i++) {
        // Write colors to SPI buffer
        p += rgb_to_WS2812_NZR(colors[i][0], colors[i][1], colors[i][2], p);
    }
    // Write latch / reset to SPI buffer
    memset(p, 0, reset_bytes); p += reset_bytes;

    LEDBusy = 1;
    int result = HAL_SPI_Transmit_DMA(&hspi4, (unsigned char *)buffer, p - buffer); // , 2);
    if(result != HAL_OK){
        printf("SPI_Transmit error %d, error code %08lX, status %08lX\n", result, hspi4.ErrorCode, SPI4->SR);
        panic();
    }
    // HAL_GPIO_WritePin(RGBLED_SPI_GPIO_Port, RGBLED_SPI_Pin, GPIO_PIN_RESET);
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

//----------------------------------------------------------------------------
// Controller Code from alice3cv that needs to be cleaned up and pared down

static inline void set_GPIO_value(GPIO_TypeDef* gpio, int mask, int value)
{
    unsigned long int data = value ? mask : 0;
    gpio->ODR = (gpio->ODR & ~mask) | data;
    __asm__ volatile("" ::: "memory"); // Force all memory operations before to come before and all after to come after.
}

static inline void set_GPIO_iotype(GPIO_TypeDef* gpio, int pin, unsigned int iotype)
{
    long unsigned int mask = ~(3U << (pin * 2));
    long unsigned int value = iotype << (pin * 2);
    gpio->MODER = (gpio->MODER & mask) | value;
    __asm__ volatile("" ::: "memory"); // Force all memory operations before to come before and all after to come after.
}

typedef struct gpio
{
    GPIO_TypeDef* port;
    int pin;
    unsigned int mask;
#if 0
    gpio(GPIO_TypeDef* port_, int pin_) :
        port(port_),
        pin(pin_)
    {
        mask = 1U << pin;
    }
#endif
} gpio;

const gpio select_joystick = { GPIOB, 13, 1U << 13};
const gpio select_keypad_1 = { GPIOF, 7, 1U << 7};
const gpio select_keypad_2 = { GPIOF, 8, 1U << 8};

const gpio joystick_1_north = { GPIOH, 6, 1U << 6};
const gpio joystick_1_south = { GPIOH, 7, 1U << 7};
const gpio joystick_1_west = { GPIOH, 8, 1U << 8};
const gpio joystick_1_east = { GPIOH, 9, 1U << 9};
const gpio joystick_1_fire = { GPIOI, 10, 1U << 10};

const gpio joystick_2_north = { GPIOH, 12, 1U << 12};
const gpio joystick_2_south = { GPIOH, 13, 1U << 13};
const gpio joystick_2_west = { GPIOH, 14, 1U << 14};
const gpio joystick_2_east = { GPIOH, 15, 1U << 15};
const gpio joystick_2_fire = { GPIOI, 11, 1U << 11};

void InitializeControllers(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    // Joystick directions and fire are hooked to select_joystick
    // and select_keyboard within the controller.

    // Set all joystick directions and fire button to inputs

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = joystick_1_north.mask;
    HAL_GPIO_Init(joystick_1_north.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_south.mask;
    HAL_GPIO_Init(joystick_1_south.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_west.mask;
    HAL_GPIO_Init(joystick_1_west.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_east.mask;
    HAL_GPIO_Init(joystick_1_east.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_fire.mask;
    HAL_GPIO_Init(joystick_1_fire.port, &GPIO_InitStruct);

    // Set select signals to high-impedance

    GPIO_InitStruct.Pin = select_joystick.mask;
    HAL_GPIO_Init(select_joystick.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_2.mask;
    HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 

    // Joystick directions and fire are hooked to select_joystick
    // and select_keyboard within the controller.

    // Set all joystick directions and fire button to inputs

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = joystick_2_north.mask;
    HAL_GPIO_Init(joystick_2_north.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_south.mask;
    HAL_GPIO_Init(joystick_2_south.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_west.mask;
    HAL_GPIO_Init(joystick_2_west.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_east.mask;
    HAL_GPIO_Init(joystick_2_east.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_fire.mask;
    HAL_GPIO_Init(joystick_2_fire.port, &GPIO_InitStruct);
}

uint8_t RoGetJoystickState(RoControllerIndex which)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    // Set select_joystick to RESET, which grounds joystick and fire-left switches
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick.mask;
    HAL_GPIO_WritePin(select_joystick.port, select_joystick.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_joystick.port, &GPIO_InitStruct); 

    delay_us(10);
    // HAL_Delay(1);

    // read joystick and fire-left
    unsigned int joystick_value = 0;
    
    switch(which) {
        case CONTROLLER_1:
            joystick_value = (
                (HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
                (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
                (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
                (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3) |
                (HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask) << 6)
                ) ^ 0x4F;
            break;

        case CONTROLLER_2:
            joystick_value = (
                (HAL_GPIO_ReadPin(joystick_2_north.port, joystick_2_north.mask) << 0) | 
                (HAL_GPIO_ReadPin(joystick_2_east.port, joystick_2_east.mask) << 1) | 
                (HAL_GPIO_ReadPin(joystick_2_south.port, joystick_2_south.mask) << 2) | 
                (HAL_GPIO_ReadPin(joystick_2_west.port, joystick_2_west.mask) << 3) |
                (HAL_GPIO_ReadPin(joystick_2_fire.port, joystick_2_fire.mask) << 6)
                ) ^ 0x4F;
            break;
    }

    // set select_joystick to high-impedance
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_joystick.mask;
    HAL_GPIO_Init(select_joystick.port, &GPIO_InitStruct); 

    // HAL_Delay(1);
    return joystick_value;
}

uint8_t RoGetKeypadState(RoControllerIndex which)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    // Set select_keypad to RESET, which grounds keypad and fire-right switches
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    switch(which) {
        case CONTROLLER_1:
            GPIO_InitStruct.Pin = select_keypad_1.mask;
            HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_RESET);
            HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 
            break;
        case CONTROLLER_2:
            GPIO_InitStruct.Pin = select_keypad_2.mask;
            HAL_GPIO_WritePin(select_keypad_2.port, select_keypad_2.mask, GPIO_PIN_RESET);
            HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 
            break;
    }

    delay_us(10);
    // HAL_Delay(1);

    // read keypad and fire-right
    unsigned int keypad_value = 0;

    switch(which) {
        case CONTROLLER_1:
            keypad_value =  
                ((HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
                (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
                (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
                (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3) |
                (HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask) << 6)
                ) ^ 0x4F;
            break;
        case CONTROLLER_2:
            keypad_value =  
                ((HAL_GPIO_ReadPin(joystick_2_north.port, joystick_2_north.mask) << 0) | 
                (HAL_GPIO_ReadPin(joystick_2_east.port, joystick_2_east.mask) << 1) | 
                (HAL_GPIO_ReadPin(joystick_2_south.port, joystick_2_south.mask) << 2) | 
                (HAL_GPIO_ReadPin(joystick_2_west.port, joystick_2_west.mask) << 3) |
                (HAL_GPIO_ReadPin(joystick_2_fire.port, joystick_2_fire.mask) << 6)
                ) ^ 0x4F;
            break;
    }

    // set select_keypad to high-impedance
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    switch(which) {
        case CONTROLLER_1:
            GPIO_InitStruct.Pin = select_keypad_1.mask;
            HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 
            break;
        case CONTROLLER_2:
            GPIO_InitStruct.Pin = select_keypad_2.mask;
            HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 
            break;
    }

    // HAL_Delay(1);

    return keypad_value;
}

char ColecoKeypadToCharacter(uint8_t value)
{
    switch(value) {
        case 0: return '-';
        case CONTROLLER_KEYPAD_0: return '0';
        case CONTROLLER_KEYPAD_1: return '1';
        case CONTROLLER_KEYPAD_2: return '2';
        case CONTROLLER_KEYPAD_3: return '3';
        case CONTROLLER_KEYPAD_4: return '4';
        case CONTROLLER_KEYPAD_5: return '5';
        case CONTROLLER_KEYPAD_6: return '6';
        case CONTROLLER_KEYPAD_7: return '7';
        case CONTROLLER_KEYPAD_8: return '8';
        case CONTROLLER_KEYPAD_9: return '9';
        case CONTROLLER_KEYPAD_asterisk: return '*';
        case CONTROLLER_KEYPAD_pound: return '#';
        default: return '?';
    }
}


// Audio subsystem =========================================================

// This was 15700 / 60 * 2 * 2, // 261 samples per channel per
// buffer.  But that seemed unreliable under WAV playback for some
// reason; maybe the overhead of reading plus the read ended up being
// longer than the duration of one buffer, so it missed.

#define AUDIO_CHUNK_SIZE (256 * 2)
#define AUDIO_CHUNK_COUNT 4
#define AUDIO_BUFFER_SIZE (AUDIO_CHUNK_SIZE * AUDIO_CHUNK_COUNT)
static uint8_t audioBuffer[AUDIO_BUFFER_SIZE];

volatile size_t audioReadNext = 0;
volatile size_t audioWriteNext = AUDIO_CHUNK_SIZE * AUDIO_CHUNK_COUNT / 2;
volatile size_t missedAudioSamples = 0;

void RoAudioGetSamplingInfo(float *rate, size_t *recommendedChunkSize)
{
    // If NTSC line ISR is providing audio, we will have a sampling rate of 15.6998 KHz
    *rate = 15699.76074561403508;
    *recommendedChunkSize = AUDIO_CHUNK_SIZE;
}

size_t WriteOverlapsRead(size_t audioWriteNext, size_t writeSize, size_t audioReadNext)
{
    size_t testReadPosition = (audioReadNext < audioWriteNext) ? (audioReadNext + AUDIO_BUFFER_SIZE) : audioReadNext;
    // printf("audioReadNext %zd, testReadPosition %zd\n", audioReadNext, testReadPosition);
    if((testReadPosition > audioWriteNext) && (audioWriteNext + writeSize >= testReadPosition)) {
        return audioWriteNext + writeSize - testReadPosition;
    } else {
        return 0;
    }
}

size_t RoAudioEnqueueSamplesBlocking(size_t writeSize /* in bytes */, uint8_t* buffer)
{
#if 0
    static size_t missedPreviously = 0;
    if(missedPreviously != missedAudioSamples) {
        RoDebugOverlayPrintf("Missed %ld\n", missedAudioSamples - missedPreviously);
        missedPreviously = missedAudioSamples;
    }
#endif

    size_t waitSampleCount;

    if(writeSize > AUDIO_BUFFER_SIZE) {
        return SIZE_MAX;
    }

    waitSampleCount = WriteOverlapsRead(audioWriteNext, writeSize, audioReadNext);

    while(WriteOverlapsRead(audioWriteNext, writeSize, audioReadNext) != 0) {
        // printf("Wait... %zd %zd %zd\n", testReadPosition, audioWriteNext, writeSize);
    }

    size_t toCopy = (writeSize < (AUDIO_BUFFER_SIZE - audioWriteNext)) ? writeSize : (AUDIO_BUFFER_SIZE - audioWriteNext);
    memcpy(audioBuffer + audioWriteNext, buffer, toCopy);

    size_t remaining = writeSize - toCopy;
    if(remaining > 0) {
        memcpy(audioBuffer, buffer + toCopy, remaining);
    }

    if(audioReadNext == audioWriteNext) {
        audioReadNext = (audioReadNext + 2) % sizeof(audioBuffer);
    }

    audioWriteNext = (audioWriteNext + writeSize) % AUDIO_BUFFER_SIZE;

    return waitSampleCount;
}

void RoAudioClear()
{
    memset(audioBuffer, 128, sizeof(audioBuffer));
    audioReadNext = 0;
    audioWriteNext = AUDIO_CHUNK_SIZE * AUDIO_CHUNK_COUNT / 2;
}

void AudioStart()
{
    RoAudioClear();
    DAC1->CR |= DAC_CR_EN1;
    DAC1->CR |= DAC_CR_EN2;
}

//============================================================================
// NTSC output

//----------------------------------------------------------------------------
// VGA timing 

#if 0 // 800x600

#define VGA_CLOCK_DIVIDER 7

#define VGA_VSYNC_BACK_PORCH 22
#define VGA_TOP_BORDER_ROWS 0
#define VGA_VISIBLE_ROWS 600
#define VGA_BOTTOM_BORDER_ROWS 0
#define VGA_VSYNC_FRONT_PORCH 1
#define VGA_VSYNC_ROWS 2

#define VGA_HSYNC_BACK_PORCH 120
#define VGA_LEFT_BORDER_COLUMNS 8
#define VGA_VISIBLE_COLUMNS 800
#define VGA_RIGHT_BORDER_COLUMNS 8
#define VGA_HSYNC_FRONT_PORCH 16
#define VGA_HSYNC_CLOCKS 72

#else // 640x480

#define VGA_CLOCK_DIVIDER       10

#define VGA_VSYNC_BACK_PORCH    25
#define VGA_TOP_BORDER_ROWS     8
#define VGA_VISIBLE_ROWS        480
#define VGA_BOTTOM_BORDER_ROWS  8
#define VGA_VSYNC_FRONT_PORCH   2
#define VGA_VSYNC_ROWS          2

#define VGA_HSYNC_BACK_PORCH            40
#define VGA_LEFT_BORDER_COLUMNS         8
#define VGA_VISIBLE_COLUMNS             640
#define VGA_RIGHT_BORDER_COLUMNS        8
#define VGA_HSYNC_FRONT_PORCH           8
#define VGA_HSYNC_CLOCKS                96

#endif


#define VGA_TOTAL_COLUMNS               (VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS + VGA_VISIBLE_COLUMNS + VGA_RIGHT_BORDER_COLUMNS + VGA_HSYNC_FRONT_PORCH + VGA_HSYNC_CLOCKS)
#define VGA_DMA_COLUMNS                 VGA_TOTAL_COLUMNS

#define VGA_TOTAL_ROWS          (VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS + VGA_VISIBLE_ROWS + VGA_BOTTOM_BORDER_ROWS + VGA_VSYNC_FRONT_PORCH + VGA_VSYNC_ROWS)


#define SECTION_CCMRAM

// These should be in CCM to reduce contention with SRAM1 during DMA 
unsigned char SECTION_CCMRAM NTSCEqSyncPulseLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCVSyncLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCBlankLineBW[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCBlankLineColor[ROW_SAMPLES];

unsigned char SECTION_CCMRAM NTSCSyncTip;
unsigned char SECTION_CCMRAM NTSCSyncPorch;
unsigned char SECTION_CCMRAM NTSCBlack;
unsigned char SECTION_CCMRAM NTSCWhite;

int NTSCEqPulseClocks;
int NTSCVSyncClocks;
int NTSCHSyncClocks;
int NTSCLineClocks;
int NTSCFrontPorchClocks;
int NTSCBackPorchClocks;

unsigned char NTSCColorburst0;
unsigned char NTSCColorburst90;
unsigned char NTSCColorburst180;
unsigned char NTSCColorburst270;


void NTSCCalculateParameters()
{
    // Calculate values for a scanline
    NTSCLineClocks = ROW_SAMPLES;
    NTSCHSyncClocks = floorf(NTSCLineClocks * NTSC_HOR_SYNC_DUR + 0.5);

    NTSCFrontPorchClocks = NTSCLineClocks * NTSC_FRONTPORCH;
    NTSCBackPorchClocks = NTSCLineClocks * NTSC_BACKPORCH;
    NTSCEqPulseClocks = NTSCLineClocks * NTSC_EQ_PULSE_INTERVAL;
    NTSCVSyncClocks = NTSCLineClocks * NTSC_VSYNC_BLANK_INTERVAL;

    NTSCSyncTip = RoDACVoltageToValue(NTSC_SYNC_TIP_VOLTAGE);
    NTSCSyncPorch = RoDACVoltageToValue(NTSC_SYNC_PORCH_VOLTAGE);
    NTSCBlack = RoDACVoltageToValue(NTSC_SYNC_BLACK_VOLTAGE);
    NTSCWhite = RoDACVoltageToValue(NTSC_SYNC_WHITE_VOLTAGE);

    // Calculate the four values for the colorburst that we'll repeat to make a wave
    // The waveform is defined as sine in the FCC broadcast doc, but for
    // composite the voltages are reversed, so the waveform becomes -sine.
    NTSCColorburst0 = NTSCSyncPorch;
    NTSCColorburst90 = NTSCSyncPorch - .6 * NTSCSyncPorch;
    NTSCColorburst180 = NTSCSyncPorch;
    NTSCColorburst270 = NTSCSyncPorch + .6 * NTSCSyncPorch;
}

void NTSCFillEqPulseLine(unsigned char *rowBuffer)
{
    for (int col = 0; col < NTSCLineClocks; col++) {
        if (col < NTSCEqPulseClocks || (col > NTSCLineClocks/2 && col < NTSCLineClocks/2 + NTSCEqPulseClocks)) {
            rowBuffer[col] = NTSCSyncTip;
        } else {
            rowBuffer[col] = NTSCSyncPorch;
        }
    }
}

void NTSCFillVSyncLine(unsigned char *rowBuffer)
{
    for (int col = 0; col < NTSCLineClocks; col++) {
        if (col < NTSCVSyncClocks || (col > NTSCLineClocks/2 && col < NTSCLineClocks/2 + NTSCVSyncClocks)) {
            rowBuffer[col] = NTSCSyncTip;
        } else {
            rowBuffer[col] = NTSCSyncPorch;
        }
    }
}

// Haven't accelerated because not yet done in scan ISR
void NTSCAddColorburst(unsigned char *rowBuffer, int row)
{
    static const int startOfColorburstClocks = 76; // 80 - 3 * 4; // XXX magic number for current clock

    int rowCBOffset = (row * ROW_SAMPLES) % 4;
    for(int col = startOfColorburstClocks; col < startOfColorburstClocks + NTSC_COLORBURST_CYCLES * 4; col++) {
        switch((col - startOfColorburstClocks + rowCBOffset) % 4) {
            case 0: rowBuffer[col] = NTSCColorburst0; break;
            case 1: rowBuffer[col] = NTSCColorburst90; break;
            case 2: rowBuffer[col] = NTSCColorburst180; break;
            case 3: rowBuffer[col] = NTSCColorburst270; break;
        }
    }
}

void NTSCFillBlankLine(unsigned char *rowBuffer, int withColorburst)
{
    memset(rowBuffer, NTSCBlack, ROW_SAMPLES);
    for (int col = 0; col < NTSCLineClocks; col++) {
        if (col < NTSCHSyncClocks) {
            rowBuffer[col] = NTSCSyncTip;
        } else if(col < NTSCHSyncClocks + NTSCBackPorchClocks) {
            rowBuffer[col] = NTSCSyncPorch;
        } else if(col >= NTSCLineClocks - NTSCFrontPorchClocks) {
            rowBuffer[col] = NTSCSyncPorch;
        } else {
            rowBuffer[col] = NTSCBlack;
        }
    }
    if(withColorburst) {
        NTSCAddColorburst(rowBuffer, 0);
    }
}

void NTSCGenerateLineBuffers()
{
    // one line = (1 / 3579545) * (455/2)

    // front porch is (.165) * (1 / 15734) / (1 / 3579545) = 37.53812921062726565701 cycles (37.5)
    //     74 cycles at double clock
    // pixels is (1 - .165) * (1 / 15734) / (1 / 3579545) = 189.96568418711380557696 cycles (190)
    //     280 cycles at double clock

    NTSCFillEqPulseLine(NTSCEqSyncPulseLine);
    NTSCFillVSyncLine(NTSCVSyncLine);
    NTSCFillBlankLine(NTSCBlankLineBW, 0);
    NTSCFillBlankLine(NTSCBlankLineColor, 1);
}

int DefaultNeedsColorburst()
{
    return 1;
}

void DefaultFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    if((rowNumber > 60) && (rowNumber < 60 + 192 * 2)) {
        memset(rowBuffer + 72, (NTSCBlack + NTSCWhite) / 2, 560);
    }
}

uint8_t NTSCVideoMemory[65536];
uint8_t /*  __attribute__((section (".ram_d1"))) */ NTSCRowDoubleBuffer[ROW_SAMPLES * 2];
volatile int NTSCRowNumber = 0;
volatile int NTSCFrameNumber = 0;
volatile int markHandlerInSamples = 0;
int NTSCModeFuncsValid = 0;
int NTSCModeInterlaced = 1;
RoNTSCModeFillRowBufferFunc NTSCModeFillRowBuffer = DefaultFillRowBuffer;
RoNTSCModeInitVideoMemoryFunc NTSCModeInitVideoMemory = NULL;
RoNTSCModeNeedsColorburstFunc NTSCModeNeedsColorburst = DefaultNeedsColorburst;

void RoNTSCSetMode(int interlaced, RoNTSCModeInitVideoMemoryFunc initFunc, RoNTSCModeFillRowBufferFunc fillBufferFunc, RoNTSCModeNeedsColorburstFunc needsColorBurstFunc)
{
    if((NTSCModeInterlaced == interlaced) &&
        (initFunc == NTSCModeInitVideoMemory) &&
        (fillBufferFunc == NTSCModeFillRowBuffer) &&
        (needsColorBurstFunc == NTSCModeNeedsColorburst))
    {
        return;
    }

    NTSCModeFuncsValid = 0;
    NTSCModeNeedsColorburst = needsColorBurstFunc;
    NTSCModeInitVideoMemory = initFunc;
    NTSCModeFillRowBuffer = fillBufferFunc;
    NTSCModeInterlaced = interlaced;
    initFunc(NTSCVideoMemory, sizeof(NTSCVideoMemory), NTSCBlack, NTSCWhite);
    NTSCRowNumber = 0;
    NTSCFrameNumber = 0;
    NTSCModeFuncsValid = 1;
}

// NTSC interlaced is made up of "odd" and "even" fields.  For NTSC, the first field is
// odd, which means it's #1.

void NTSCFillRowBuffer(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    // XXX could optimize these by having one branch be lines < 21

    /*
     * Rows 0 through 8 are equalizing pulse, then vsync, then equalizing pulse
     */

    if(lineNumber < NTSC_EQPULSE_LINES) {

        // odd field equalizing pulse
        memcpy(rowBuffer, NTSCEqSyncPulseLine, sizeof(NTSCEqSyncPulseLine));

    } else if(lineNumber - NTSC_EQPULSE_LINES < NTSC_VSYNC_LINES) {

        // odd field VSYNC
        memcpy(rowBuffer, NTSCVSyncLine, sizeof(NTSCVSyncLine));

    } else if(lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES) < NTSC_EQPULSE_LINES) {

        // odd field equalizing pulse
        memcpy(rowBuffer, NTSCEqSyncPulseLine, sizeof(NTSCEqSyncPulseLine));

    } else if(lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES) < NTSC_VBLANK_LINES) {

        /*
         * Rows 9 through 2X are other part of vertical blank
         */

        // odd field vblank
        if(NTSCModeFuncsValid) {
            memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, ROW_SAMPLES);
        } else {
            memcpy(rowBuffer, NTSCBlankLineBW, ROW_SAMPLES);
        }

    } else if(lineNumber >= 263 && lineNumber <= 271) {

        // Handle interlace half line and vertical retrace and sync.
        if(lineNumber <= 264) {
            // lines 263, 264 - last 405 of even field eq pulse then first 405 of eq pulse
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 265) {
            // line 265 - last 405 of even field eq pulse then first 405 of vsync
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCVSyncLine, ROW_SAMPLES / 2);
        } else if(lineNumber <= 267) {
            // lines 266, 267 - last 405 of even field vsync then first 405 of vsync
            memcpy(rowBuffer, NTSCVSyncLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCVSyncLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 268) {
            // even vield lines 268 - last 405 of even field vsync then first 405 of eq pulse
            memcpy(rowBuffer, NTSCVSyncLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber <= 270) {
            // lines 269, 270 - last 405 of even field eq pulse then first 405 of eq pulse
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 271) {
            // line 271 - last 405 of even field eq pulse then 405 of SyncPorch
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memset(rowBuffer + ROW_SAMPLES / 2, NTSCSyncPorch, ROW_SAMPLES / 2);
        }

    } else if((lineNumber >= 272) && (lineNumber <= 281)) { // XXX half line at 282

        /*
         * Rows 272 through 2XX are other part of vertical blank
         */

        // even field vertical safe area
        if(NTSCModeFuncsValid) {
            memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, ROW_SAMPLES);
        } else {
            memcpy(rowBuffer, NTSCBlankLineBW, ROW_SAMPLES);
        }

    } else {

        if(NTSCModeFuncsValid) {
            memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, ROW_SAMPLES);
        } else {
            memcpy(rowBuffer, NTSCBlankLineBW, ROW_SAMPLES);
        }

        int rowWithinFrame;
        if(NTSCModeInterlaced) {
            rowWithinFrame = (lineNumber % 263) * 2 + lineNumber / 263 - 22;
        } else {
            rowWithinFrame = lineNumber % 263 - 22;
        }
        if(NTSCModeFuncsValid) {
            NTSCModeFillRowBuffer(frameNumber, rowWithinFrame, 704, rowBuffer + 164);
        }

        if((lineNumber == 262) && NTSCModeInterlaced) {
            // interlacing, line 262 - overwrite last 405 samples with first 405 samples of EQ pulse
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if((lineNumber == 282) && NTSCModeInterlaced) {
            // interlacing, special line 282 - write SyncPorch from BackPorch to middle of line after mode's fillRow()
            memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCSyncPorch, ROW_SAMPLES / 2 - (NTSCHSyncClocks + NTSCBackPorchClocks));
        }
    }
}

// debug overlay scanout

// There should be some kind of a way to reserve a "slot" (not
// always the same line on screen) for debug output for system or other
// continuous output, and then a way to scroll debug output logging

int debugOverlayEnabled = 0;

#define debugDisplayLeftTick (NTSCHSyncClocks + NTSCBackPorchClocks + 64)
#define debugDisplayTopTick (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES + 18)
/* debugFontWidthScale != 4 looks terrible in a color field because of adjacent color columns; probably need to ensure 0s around any 1 text column */
#define debugFontWidthScale 2
#define debugCharGapPixels 1
#define debugFontHeightScale 1
#define debugDisplayWidth (604 / (font8x16Width * debugFontWidthScale + debugCharGapPixels))
#define debugDisplayHeight ((218 - 6) / font8x16Height)

char debugDisplay[debugDisplayHeight][debugDisplayWidth];

void RoDebugOverlayPrintf(const char *fmt, ...)
{
    static int debugLine = 0;
    static char buffer[debugDisplayWidth + 1];
    va_list args;

    debugOverlayEnabled = 1;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    char *line;
    char *inputstring = buffer;
    while((line = strsep(&inputstring, "\n")) != NULL) {

        if(debugLine == (debugDisplayHeight - 1)) {
            for(int i = 0; i < debugDisplayHeight - 1; i++) {
                memcpy(debugDisplay[i], debugDisplay[i + 1], debugDisplayWidth);
            }
        }

        memset(debugDisplay[debugLine] + strlen(buffer), 0, debugDisplayWidth - strlen(buffer));
        memcpy(debugDisplay[debugLine], buffer, strlen(buffer));

        if(debugLine < (debugDisplayHeight - 1)) {
            debugLine ++;
        }
    }
}

void RoDebugOverlaySetLine(int line, const char *str, size_t size)
{
    size_t toWrite = (size > debugDisplayWidth) ? debugDisplayWidth : size;
    memcpy(debugDisplay[line], str, toWrite);
}

void NTSCFillRowDebugOverlay(int frameNumber, int lineNumber, unsigned char* nextRowBuffer)
{
#if debugFontWidthScale == 4 && font8x16Width == 8
    ntsc_wave_t NTSCWhiteLong =
        (NTSCWhite <<  0) |
        (NTSCWhite <<  8) |
        (NTSCWhite << 16) |
        (NTSCWhite << 24);
#endif
    int debugFontScanlineHeight = font8x16Height * debugFontHeightScale;

    int rowWithinDebugArea = (lineNumber % 263) - debugDisplayTopTick;
    int charRow = rowWithinDebugArea / debugFontScanlineHeight;
    int charPixelY = (rowWithinDebugArea % debugFontScanlineHeight) / debugFontHeightScale;

// XXX this code assumes font width <= 8 and each row padded out to a byte
    if((rowWithinDebugArea >= 0) && (charRow < debugDisplayHeight)) {
        for(int charCol = 0; charCol < debugDisplayWidth; charCol++) {
            unsigned char debugChar = debugDisplay[charRow][charCol];
            if(debugChar != 0) {
                unsigned char charRowBits = font8x16Bits[debugChar * font8x16Height + charPixelY];
#if debugFontWidthScale == 4 && font8x16Width == 8
                unsigned char *charPixels = nextRowBuffer + debugDisplayLeftTick + (charCol * (font8x16Width + debugCharGapPixels)) * debugFontWidthScale;
                if(charRowBits & 0x80) { ((ntsc_wave_t*)charPixels)[0] = NTSCWhiteLong; } 
                if(charRowBits & 0x40) { ((ntsc_wave_t*)charPixels)[1] = NTSCWhiteLong; }
                if(charRowBits & 0x20) { ((ntsc_wave_t*)charPixels)[2] = NTSCWhiteLong; }
                if(charRowBits & 0x10) { ((ntsc_wave_t*)charPixels)[3] = NTSCWhiteLong; }
                if(charRowBits & 0x08) { ((ntsc_wave_t*)charPixels)[4] = NTSCWhiteLong; }
                if(charRowBits & 0x04) { ((ntsc_wave_t*)charPixels)[5] = NTSCWhiteLong; }
                if(charRowBits & 0x02) { ((ntsc_wave_t*)charPixels)[6] = NTSCWhiteLong; }
                if(charRowBits & 0x01) { ((ntsc_wave_t*)charPixels)[7] = NTSCWhiteLong; }
#else
                for(int charPixelX = 0; charPixelX < font8x16Width; charPixelX++) {
                    int pixel = charRowBits & (0x80 >> charPixelX);
                    if(pixel) {
                        unsigned char *charPixels = nextRowBuffer + debugDisplayLeftTick + (charCol * (font8x16Width + debugCharGapPixels) + charPixelX) * debugFontWidthScale;
#if debugFontWidthScale == 4
                        *(ntsc_wave_t *)charPixels = NTSCWhiteLong; 
#else
                        for(int col = 0; col < debugFontWidthScale; col++) {
                            charPixels[col] = NTSCWhite;
                            charPixels[col + 1] = NTSCBlack;
                            charPixels[col + 2] = NTSCBlack;
                            charPixels[col + 3] = NTSCBlack;
                        }
#endif
                    }
                }
#endif
            }
        }
    }
}

extern const uint16_t compiled_image[640 * 480];

uint16_t* SDRAM_image = (uint16_t *)SDRAM_START;

void VGAFillRowBuffer(int frameNumber, int lineNumber, uint16_t *rowBuffer)
{
    if(lineNumber < (VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS)) {
        memset(rowBuffer, 0, VGA_DMA_COLUMNS * sizeof(uint16_t));
    } else if(lineNumber >= (VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS + VGA_VISIBLE_ROWS)) {
        memset(rowBuffer, 0, VGA_DMA_COLUMNS * sizeof(uint16_t));
    } else {
        int visibleLineNumber = lineNumber - (VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS);
        // memcpy_fast_16byte_multiple(rowBuffer + VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS, compiled_image + VGA_VISIBLE_COLUMNS * visibleLineNumber, VGA_VISIBLE_COLUMNS * sizeof(uint16_t));
        memcpy_fast_16byte_multiple(rowBuffer + VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS, SDRAM_image + VGA_VISIBLE_COLUMNS * visibleLineNumber, VGA_VISIBLE_COLUMNS * sizeof(uint16_t));
    }
}

int why = 0;

enum DisplayMode { DISPLAY_MODE_NONE, DISPLAY_MODE_NTSC, DISPLAY_MODE_VGA } currentDisplayMode = DISPLAY_MODE_NONE;

void RoNTSCWaitFrame()
{
    // NTSC won't actually go lineNumber >= 525...
    int field0_vblank;
    int field1_vblank;
    do {
        field0_vblank = (NTSCRowNumber > 257) && (NTSCRowNumber < 262);
        field1_vblank = (NTSCRowNumber > 520) && (NTSCRowNumber < NTSC_FRAME_LINES);
    } while(!field0_vblank && !field1_vblank); // Wait for VBLANK; should do something smarter
}

#define CONSOLE_BUTTON_START_REPEAT_DELAY 600
#define CONSOLE_BUTTON_REPEAT_DELTA 20
#define CONSOLE_BUTTON_LONGPRESS_DELAY 400
#define CONSOLE_BUTTON_PRESS_DEBOUNCE 10

void CheckConsoleButtons()
{
    static int button_pressed[3] = {0, 0, 0};
    static uint32_t button_pressed_millis[3];
    static GPIO_TypeDef* button_ports[3] = {USER1_GPIO_Port, USER2_GPIO_Port, USER3_GPIO_Port};
    static const int button_pins[3] = {USER1_Pin, USER2_Pin, USER3_Pin};

    uint32_t now = HAL_GetTick();

    for(int b = 0; b < 3; b++) {
        if(HAL_GPIO_ReadPin(button_ports[b], button_pins[b])) {
            if(!button_pressed[b]) { 
                // Initial press; record the start
                button_pressed[b] = 1;
                button_pressed_millis[b] = now;
            } else {
                // If has exceeded the repeat time, issue a repeated press
                if(now - button_pressed_millis[b] > CONSOLE_BUTTON_START_REPEAT_DELAY) {
                    // Do something KEWL
                    RoDebugOverlayPrintf("Repeat button %d\n", b);
                    button_pressed_millis[b] += CONSOLE_BUTTON_REPEAT_DELTA;
                }
            }
        } else {
            if(button_pressed[b]) {
                if(now - button_pressed_millis[b] > CONSOLE_BUTTON_START_REPEAT_DELAY) {
                    RoDebugOverlayPrintf("Stop Repeat %d\n", b);
                    // Nothing
                } else if(now - button_pressed_millis[b] > CONSOLE_BUTTON_LONGPRESS_DELAY) {
                    // Do something KEWL
                    RoDebugOverlayPrintf("Long press %d\n", b);
                } else if(now - button_pressed_millis[b] > CONSOLE_BUTTON_PRESS_DEBOUNCE) {
                    ConvertConsoleButtonPressToEvent(b);
                } else {
                    // Nothing
                }
                button_pressed[b] = 0;
            }
        }
    }
}

void NTSCRowHandler(void)
{
    NTSCRowNumber = NTSCRowNumber + 1;
    if(NTSCModeInterlaced) {
        if(NTSCRowNumber == 525) {
            NTSCRowNumber = 0;
        }
    } else {
        if(NTSCRowNumber == 262) {
            NTSCRowNumber = 0;
        }
    }

    if(NTSCRowNumber == 0) {
        NTSCFrameNumber ++;
    }

    uint8_t *rowDest;
    if(DMA2->LISR & DMA_FLAG_HTIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
        rowDest = NTSCRowDoubleBuffer + 0;
    } else if(DMA2->LISR & DMA_FLAG_TCIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
        rowDest = NTSCRowDoubleBuffer + ROW_SAMPLES;
    } else {
        panic();
    }

// per-line, 15KHz
// XXX this could be a looping DMA with or without its own interrupt handler?  But needs two outputs
    // don't write next audio samples if the ring buffer head caught up with the tail.
    if(audioReadNext != audioWriteNext) {
        DAC1->DHR8R1 = audioBuffer[audioReadNext + 0];
        DAC1->DHR8R2 = audioBuffer[audioReadNext + 1];
        audioReadNext = (audioReadNext + 2) % sizeof(audioBuffer);
    } else {
        missedAudioSamples++;
    }

    NTSCFillRowBuffer(NTSCFrameNumber, NTSCRowNumber, rowDest);
    if(debugOverlayEnabled) {
        NTSCFillRowDebugOverlay(NTSCFrameNumber, NTSCRowNumber, rowDest);
    }
    SCB_CleanDCache();

    CheckConsoleButtons();

    // A little pulse so we know where we are on the line when we finished
    if(markHandlerInSamples) {
        for(int i = 0; i < 5; i++) { GPIOI->ODR = (GPIOI->ODR & 0xFFFFFF00) | 0xFFFFFFE8; }
    }
}

uint16_t /*  __attribute__((section (".ram_d1"))) */ VGARowDoubleBuffer[VGA_DMA_COLUMNS * 2];
int VGARowNumber = 0;
int VGAFrameNumber = 0;

void VGARowHandler()
{
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
    VGARowNumber = (VGARowNumber + 1) % VGA_TOTAL_ROWS;
    if(VGARowNumber == 0) {
        VGAFrameNumber ++;
    }

    uint16_t *rowDest;
    if(DMA2->LISR & DMA_FLAG_HTIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
        rowDest = VGARowDoubleBuffer + 0;
    } else if(DMA2->LISR & DMA_FLAG_TCIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
        rowDest = VGARowDoubleBuffer + VGA_DMA_COLUMNS;
    } else {
        panic();
    }

    VGAFillRowBuffer(VGAFrameNumber, VGARowNumber, rowDest);

    if(debugOverlayEnabled) {
        // VGAFillRowDebugOverlay(frameNumber, rowNumber, rowDest); - comment out for now?
    }
    SCB_CleanDCache();
}

void ROSA_DMA2_Stream1_IRQHandler(void)
{
    if(currentDisplayMode == DISPLAY_MODE_NTSC) {
        NTSCRowHandler();
    } else if(currentDisplayMode == DISPLAY_MODE_VGA) {
        VGARowHandler();
    }

    if(DMA2->LISR) {
        // oldLISR = DMA2->LISR;
        DMA2->LIFCR = 0xFFFF;
    }
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    printf("error code %08lX, state %08X, line %d\n", hdma_tim1_ch2.ErrorCode, hdma_tim1_ch2.State, why);
}

void startNTSCScanout()
{
    AudioStart();

    currentDisplayMode = DISPLAY_MODE_NTSC;

    NTSCCalculateParameters();
    NTSCGenerateLineBuffers();

    HAL_StatusTypeDef status;
    memcpy(NTSCRowDoubleBuffer + 0, NTSCEqSyncPulseLine, ROW_SAMPLES);
    memcpy(NTSCRowDoubleBuffer + ROW_SAMPLES, NTSCEqSyncPulseLine, ROW_SAMPLES);
    NTSCRowNumber = 1; // the previous row that was filled in

    // Set DMA request on capture-compare channel 1
    DMA2_Stream1->NDTR = sizeof(NTSCRowDoubleBuffer);
    DMA2_Stream1->M0AR = (uint32_t)NTSCRowDoubleBuffer;  // Destination address
    DMA2_Stream1->PAR = (uint32_t)&GPIOI->ODR;  // Destination address
    DMA2_Stream1->FCR = DMA_FIFOMODE_ENABLE |   // Enable FIFO to improve stutter
        DMA_FIFO_THRESHOLD_FULL;        
    DMA2_Stream1->CR |= DMA_SxCR_TCIE;    /* enable transfer complete interrupt */
    DMA2_Stream1->CR |= DMA_SxCR_HTIE;    /* enable half transfer interrupt */
    DMA2_Stream1->CR |= DMA_SxCR_EN;    /* enable DMA */
    TIM1->DIER |= TIM_DIER_CC2DE;

    status = HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);

    if(0) {
        msgprintf("DMA2_Stream1->CR = %08lX\n", DMA2_Stream1->CR);
        msgprintf("DMA2_Stream1->NDTR = %08lX\n", DMA2_Stream1->NDTR);
        msgprintf("DMA2_Stream1->PAR = %08lX\n", DMA2_Stream1->PAR);
        msgprintf("DMA2_Stream1->M0AR = %08lX\n", DMA2_Stream1->M0AR);
        msgprintf("DMA2_Stream1->M1AR = %08lX\n", DMA2_Stream1->M1AR);
        msgprintf("DMA2_Stream1->FCR = %08lX\n", DMA2_Stream1->FCR);
    }

    if(status != HAL_OK){
        printf("DMA error %08d, error code %08lX, line %d\n", status, hdma_tim1_ch2.ErrorCode, why);
        panic();
    }
}

//----------------------------------------------------------------------------
// FillRowBuffer tests

void ImageFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    for(int col = 0; col < maxSamples; col++) {
        int checker = (col / 35 + rowNumber / 20) % 2;
        rowBuffer[col] = checker ? NTSCWhite : NTSCBlack;
    }
}

int launcher_main(int argc, const char **argv);

int writeLEDColors = 1;
// set these no higher than 32, maybe after getting a case and light
// diffuser we could go higher.
uint8_t LEDColors[3][3];
int LEDSPIBusy = 0;

int RoLEDSet(int which, uint8_t red, uint8_t green, uint8_t blue)
{
    if((which < 0) || (which > 2)) {
        return -1;
    }

    LEDColors[which][0] = (red + 7) / 8;
    LEDColors[which][1] = (green + 7) / 8;
    LEDColors[which][2] = (blue + 7) / 8;

    writeLEDColors = 1;

    return 0;
}

// ----- WS2812B RGB LED test
void LEDTestIterate()
{
    static float then = 0.0f;
    float now = HAL_GetTick() / 1000.0f;

    if(!LEDBusy && (now - then > .01666667f)) {
        int halves = (int)(now * 2);
        if(halves % 2) {
            RoLEDSet(0, 0, 0, 0);
        } else {
            RoLEDSet(0, 0, 255, 0);
        }

        float h = now / 2.0f * 3.14159;
        float s = 1.0f;
        float v = 1.0f;
        float r, g, b;
        HSVToRGB3f(h, s, v, &r, &g, &b);
        if(1) RoLEDSet(1, r * 255, g * 255, b * 255);

        int phase = (int)now % 2;
        float value = phase ? (now - (int)now) : (1 - (now - (int)now));
        if(1)RoLEDSet(2, value * 255, value * 255, value * 255);

        writeLEDColors = 1;
    }
}
  
int RoDoHousekeeping(void)
{
    MX_USB_HOST_Process();

    // LEDTestIterate();

    if(writeLEDColors) {
        write3LEDString(LEDColors);
        writeLEDColors = 0;
    }

    return 0;
}

//----------------------------------------------------------------------------

Status RoFillFilenameList(const char* dirName, uint32_t flags, const char* optionalFilterSuffix, size_t maxNames, char **filenames, size_t* filenamesSize)
{
    Status status;

    FRESULT res;
    DIR dir;
    static FILINFO fno;

    res = f_opendir(&dir, dirName);                       /* Open the directory */
    if (res == FR_OK) {

        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if(res != FR_OK) {
                printf("failed to readdir - %d\n", res);
                break;
            }
            if (fno.fname[0] == 0) break;  /* Break on end of dir */

            // if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                // printf("%s/\n", fno.fname);
            // }

            int addToList = 1;

            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                // XXX Really should have a way to descend into directories.
                addToList = 0;
            } else if((fno.fname[0] == '.') && (flags & CHOOSE_FILE_IGNORE_DOTFILES)) {
                addToList = 0;
            } else if(optionalFilterSuffix && (strcmp(optionalFilterSuffix, fno.fname + strlen(fno.fname) - strlen(optionalFilterSuffix)) != 0)) {
                addToList = 0;
            }

            if(addToList) {
                if(*filenamesSize > maxNames - 1) {
                    return RO_RESOURCE_EXHAUSTED;
                }
                filenames[(*filenamesSize)++] = strdup(fno.fname);
            }
        }
        f_closedir(&dir);
        status = RO_SUCCESS;

    } else {

        if(*filenamesSize > maxNames - 1) {
            return RO_RESOURCE_EXHAUSTED;
        }
        filenames[(*filenamesSize)++] = strdup("failed to f_opendir");
        status = RO_RESOURCE_NOT_FOUND;

    }

    return status;
}


int doCommandLS(int wordCount, char **words)
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;

    res = f_opendir(&dir, "/");                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if(res != FR_OK) {
                printf("failed to readdir - %d\n", res);
                break;
            }
            if (fno.fname[0] == 0) break;  /* Break on end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                printf("%s/\n", fno.fname);
            } else {                                       /* It is a file. */
                printf("%s\n", fno.fname);
            }
        }
        f_closedir(&dir);
    } else {
        printf("failed to f_opendir - %d\n", res);
    }
    return 0;
}

// ----------------------------------------------------------------------------


static void VGA_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = VGA_CLOCK_DIVIDER - 1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_TIM1_CLK_ENABLE();

}

void startVGAScanout()
{
    memset(SDRAM_image, 0, VGA_VISIBLE_ROWS * VGA_VISIBLE_COLUMNS);
    for(int y = 0; y < 480; y++) {
        memcpy(SDRAM_image + VGA_VISIBLE_COLUMNS * y, compiled_image + 640 * y, 640 * sizeof(compiled_image[0]));
    }

    HAL_StatusTypeDef status;
    HAL_StatusTypeDef status1, status2;

    VGARowNumber = -1; // the previous row that was filled in
    currentDisplayMode = DISPLAY_MODE_VGA;

    for(int i = 0; i < VGA_VISIBLE_COLUMNS; i++) {
        VGARowDoubleBuffer[VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS + i]  = 0x07E0;
        VGARowDoubleBuffer[VGA_DMA_COLUMNS + VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS + i]  = 0xF81F;
    }

    // Set DMA request on capture-compare channel 1
    DMA2_Stream1->NDTR = VGA_DMA_COLUMNS * 2;
    DMA2_Stream1->M0AR = (uint32_t)VGARowDoubleBuffer;  // Destination address
    DMA2_Stream1->PAR = (uint32_t)&GPIOC->ODR;  // Destination address
    DMA2_Stream1->CR |= DMA_MDATAALIGN_HALFWORD;  // Memory data size
    DMA2_Stream1->CR |= DMA_PDATAALIGN_HALFWORD;  // Peripheral size
    DMA2_Stream1->CR |= DMA_PRIORITY_VERY_HIGH;  // Priority
    DMA2_Stream1->FCR = DMA_FIFOMODE_ENABLE |   // Enable FIFO to improve stutter
        DMA_FIFO_THRESHOLD_HALFFULL;        
    DMA2_Stream1->CR |= DMA_MBURST_INC4;  /* enable bursts from memory source */
    DMA2_Stream1->CR |= DMA_SxCR_TCIE;    /* enable transfer complete interrupt */
    DMA2_Stream1->CR |= DMA_SxCR_HTIE;    /* enable half transfer interrupt */

    VGA_TIM1_Init();

    status1 = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    status2 = HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

    if(status1 != HAL_OK) {
        printf("HAL_TIM_Base_Start(TIM4) failed with status %u\n", status1);
        panic();
    }

    if(status2 != HAL_OK) {
        printf("HAL_TIM_Base_Start(TIM5) failed with status %u\n", status2);
        panic();
    }

    uint32_t maxtim4 = 0;
    uint32_t oldv = 0, v;
    while((v = __HAL_TIM_GetCounter(&htim4)) >= oldv) {
        if(v > maxtim4) {
            maxtim4 = v;
        }
        oldv = v;
    }
    printf("max tim4 seen: %lu\n", maxtim4); HAL_Delay(100);

    uint32_t maxtim5 = 0;
    oldv = 0;
    while((v = __HAL_TIM_GetCounter(&htim5)) >= oldv) {
        if(v > maxtim5) {
            maxtim5 = v;
        }
        oldv = v;
    }
    printf("max tim5 seen: %lu\n", maxtim5); HAL_Delay(100);


    if(1) {

        while(__HAL_TIM_GetCounter(&htim5) > 0); // wait for end of first vertical sync and timer restart
        DMA2_Stream1->CR |= DMA_SxCR_EN;    /* enable DMA */
        TIM1->DIER |= TIM_DIER_CC2DE;

        status = HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
        if(status != HAL_OK){
            printf("VGA DMA error %08d, error code %08lX, line %d\n", status, hdma_tim1_ch2.ErrorCode, why);
            panic();
        }

    } else {
        static uint16_t scanline[VGA_VISIBLE_COLUMNS];

        // __disable_irq(); // XXX rock solid if IRQs disabled

        while(__HAL_TIM_GetCounter(&htim5) < ((VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS + VGA_VISIBLE_ROWS) * VGA_TOTAL_COLUMNS)); // wait until first vertical sync front porch 

        while(__HAL_TIM_GetCounter(&htim5) > 0); // wait for end of first vertical sync and timer restart

        while(1) {
            while(__HAL_TIM_GetCounter(&htim5) < ((VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS) * VGA_TOTAL_COLUMNS)); // wait for last line of top margin

            // while in visible lines
            while(__HAL_TIM_GetCounter(&htim5) < ((VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS + VGA_VISIBLE_ROWS) * VGA_TOTAL_COLUMNS)) {
                uint32_t line = __HAL_TIM_GetCounter(&htim5) / VGA_TOTAL_COLUMNS - (VGA_VSYNC_BACK_PORCH + VGA_TOP_BORDER_ROWS);

                while(__HAL_TIM_GetCounter(&htim4) < (VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS + VGA_VISIBLE_COLUMNS + VGA_RIGHT_BORDER_COLUMNS)); // wait until hsync

                if(line < VGA_VISIBLE_ROWS) {
                    // memcpy(scanline, compiled_image + line * 640, sizeof(scanline));
                    memcpy_fast_16byte_multiple(scanline, compiled_image + line * 640, 640 * sizeof(compiled_image[0]));
                }

                // GPIOC->ODR = 0xFFFF; // little pulse to show us where we are on screen
                // GPIOC->ODR = 0xFFFF;
                // GPIOC->ODR = 0;

                const uint16_t *pattern_pixel = scanline;

                while(__HAL_TIM_GetCounter(&htim4) > VGA_HSYNC_BACK_PORCH); // wait for end of horizontal sync and timer to restart

                uint32_t pixel = VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS;
                uint32_t prevPixel = pixel;

                // wait for visible region of line
                while(__HAL_TIM_GetCounter(&htim4) < (VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS));

#pragma GCC unroll 16
                do {
                    pattern_pixel += (pixel - prevPixel);
                    GPIOC->ODR = *pattern_pixel;
                    prevPixel = pixel;
                    pixel = __HAL_TIM_GetCounter(&htim4);
                } while(pixel < (VGA_VISIBLE_COLUMNS + VGA_HSYNC_BACK_PORCH + VGA_LEFT_BORDER_COLUMNS)); // through visible pixels

                GPIOC->ODR = (GPIOC->ODR & 0xFFFF0000);
            }
        }
    }

    // GPIO_0 .059V blue
    // GPIO_1 .114V blue
    // GPIO_2 .193V blue
    // GPIO_3 .327V blue
    // GPIO_4 .893V blue
    // GPIO_5 .029V green
    // GPIO_6 .057V green
    // GPIO_7 .110V green
    // GPIO_8 .217V green
    // GPIO_9 .421V green
    // GPIO_10 .852V green
    // GPIO_11 .057V red
    // GPIO_12 .110V red
    // GPIO_13 .214V red
    // GPIO_14 .407V red
    // GPIO_15 .804V red
}

//----------------------------------------------------------------------------
// Drawing routines - bad, should delete them - provide better,
// format-independent, clipped, and optimized functions

void SetPixel(int x, int y, int c)
{
    SDRAM_image[x + y * VGA_VISIBLE_COLUMNS] = c;
}

void DrawFilledCircle(int cx, int cy, int r, int c)
{
    /* should clip here */
    for(int y = cy - r - 1; y < cy + r + 1; y++) {
        if((y < 0) || (y >= VGA_VISIBLE_ROWS)) continue;
        for(int x = cx - r - 1; x < cx + r + 1; x++) {
            if((x < 0) || (x >= VGA_VISIBLE_COLUMNS)) continue;
            int dx = (x - cx);
            int dy = (y - cy);
            int distsquared = dx * dx + dy * dy;
            if(distsquared < r * r) {
                SetPixel(x, y, c);
            }
        }
    }
}

void DrawLine(int x0, int y0, int x1, int y1, int c)
{
    int dx = x1 - x0;
    int dy = y1 - y0;

    /* should clip here */
    if(abs(dx) > abs(dy)) {
        if(x1 < x0) {
            int tx = x1; x1 = x0; x0 = tx;
            int ty = y1; y1 = y0; y0 = ty;
        }
        int y = y0 * 65536;
        int d = dy * 65536 / dx;
        for(int x = x0; x < x1; x++) {
            SetPixel(x, y / 65536, c);
            y += d;
        }
    } else {
        if(y1 < y0) {
            int tx = x1; x1 = x0; x0 = tx;
            int ty = y1; y1 = y0; y0 = ty;
        }
        int x = x0 * 65536;
        int d = dx * 65536 / dy;
        for(int y = y0; y < y1; y++) {
            SetPixel(x / 65536, y, c);
            x += d;
        }
    }
}

//----------------------------------------------------------------------------

void TestControllers()
{
    while(1) {
        uint8_t joystick_1_state = RoGetJoystickState(CONTROLLER_1);
        uint8_t keypad_1_state = RoGetKeypadState(CONTROLLER_1);
        uint8_t joystick_2_state = RoGetJoystickState(CONTROLLER_2);
        uint8_t keypad_2_state = RoGetKeypadState(CONTROLLER_2);
        printf("joy 1 %c %c %c %c %c %c %c     ",
            (joystick_1_state & CONTROLLER_NORTH_BIT) ? 'N' : '-',
            (joystick_1_state & CONTROLLER_SOUTH_BIT) ? 'S' : '-',
            (joystick_1_state & CONTROLLER_WEST_BIT) ? 'W' : '-',
            (joystick_1_state & CONTROLLER_EAST_BIT) ? 'E' : '-',
            (joystick_1_state & CONTROLLER_FIRE_BIT) ? '1' : '-',
            (keypad_1_state & CONTROLLER_FIRE_BIT) ? '2' : '-',
            ColecoKeypadToCharacter(keypad_1_state & CONTROLLER_KEYPAD_MASK));
        printf("joy 2 %c %c %c %c %c %c %c\n",
            (joystick_2_state & CONTROLLER_NORTH_BIT) ? 'N' : '-',
            (joystick_2_state & CONTROLLER_SOUTH_BIT) ? 'S' : '-',
            (joystick_2_state & CONTROLLER_WEST_BIT) ? 'W' : '-',
            (joystick_2_state & CONTROLLER_EAST_BIT) ? 'E' : '-',
            (joystick_2_state & CONTROLLER_FIRE_BIT) ? '1' : '-',
            (keypad_2_state & CONTROLLER_FIRE_BIT) ? '2' : '-',
            ColecoKeypadToCharacter(keypad_2_state & CONTROLLER_KEYPAD_MASK));
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    SCB->CACR |= SCB_CACR_FORCEWT_Msk;
    SCB_EnableDCache();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI4_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_SDMMC2_SD_Init();
  MX_FATFS_Init();
  MX_DAC1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_FMC_Init();
  MX_UART4_Init();
  MX_JPEG_Init();
  MX_LIBJPEG_Init();
  /* USER CODE BEGIN 2 */

    delay_init();
    SDRAMInit();
    InitializeControllers();

    if(0){
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = USER1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    }

    printf("Rocinante Firmware -------------------------------------\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    if(0) {
        uint16_t *sdram16 = (uint16_t*)SDRAM_START;

        uint16_t begin = 0;
        uint16_t end = 16; // C++ end, one past actual end
        printf("SDRAM U16 write %u through %u\n", begin, end);
        for(int i = begin; i < end; i++) {
            uint16_t expected = ((i & 0xFF) << 8) | (i & 0xFF);
            sdram16[i] = expected;
        }
        int succeeded = 1;
        for(int i = begin; i < end; i++) {
            uint16_t expected = ((i & 0xFF) << 8) | (i & 0xFF);
            uint16_t result = sdram16[i];
            if(result != expected) {
                printf("SDRAM U16 read %5d failed.  Expected 0x%04X, got 0x%04X\n", i, expected, result);
                succeeded = 0;
            }
        }
        if(succeeded) {
            printf("SDRAM U16 test from %u to %u succeeded!\n", begin, end);
        } else {
            printf("SDRAM U16 test from %u to %u failed.\n", begin, end);
            panic();
        }
    }

    if(0) {
        unsigned char* sdram = (unsigned char *)SDRAM_START;

        printf("SDRAM write test...\n");
        uint32_t begin = 0;
        uint32_t end = 1023; // C++ end, one past actual end
        for(int i = begin; i < end; i++) {
            uint8_t expected = i & 0xFF;
            sdram[i] = expected;
        }
        for(int i = 0; i < 15; i++) {
            printf("letting SDRAM decay, %d seconds to go...\n", 15 - i);
            HAL_Delay(1000);
        }
        int succeeded = 1;
        for(int i = begin; i < end; i++) {
            uint8_t expected = i & 0xFF;
            uint8_t result = sdram[i];
            if(result != expected) {
                printf("SDRAM read %5d failed.  Expected %02X, read is %02X\n", i, expected, result);
                succeeded = 0;
                HAL_Delay(10);
            }
        }
        if(succeeded) {
            printf("SDRAM U16 test from %lu to %lu succeeded!\n", begin, end);
        } else {
            printf("SDRAM U16 test from %lu to %lu failed.\n", begin, end);
            panic();
        }
    }

    if(0) {
        unsigned char* sdram = (unsigned char *)SDRAM_START;

        for(size_t a = 0; a < 16 * 1024 * 1024; a++) {
            if(a % SDRAM_TEST_STEP_SIZE == 0) {
                printf("memory test 0x%08X - 0x%08X...\n", SDRAM_START + a, SDRAM_START + a + (SDRAM_TEST_STEP_SIZE - 1));
                HAL_Delay(10);
            }
            sdram[a] = 0x0;
            if(sdram[a] != 0x0) {
                printf("0x%08X failed setting to 0\n", SDRAM_START + a);
                panic();
            }
            sdram[a] = a & 0xFF;
            if(sdram[a] != (a & 0xFF)) {
                printf("0x%08X failed setting to %d\n", SDRAM_START + a, a & 0xFF);
                panic();
            }
            sdram[a] = 0xFF;
            if(sdram[a] != 0xFF) {
                printf("0x%08X failed setting to FF\n", SDRAM_START + a);
                panic();
            }
        }
        printf("sdram fill test...\n");
        HAL_Delay(100);
        uint32_t* sdram32 = (uint32_t*)sdram;
        for(int i = 0; i < 16 * 1024 * 1024 / 4; i++) {
            uint32_t val = i | (i << 24);
            sdram32[i] = val;
        }
        for(int i = 0; i < 16 * 1024 * 1024 / 4; i++) {
            uint32_t val = i | (i << 24);
            if(sdram32[i] != val) {
                printf("%p (%08X; %d) failed setting to 0x%lX\n", sdram32 + i, i, i, val);
                panic();
            }
        }
        printf("successful\n");
        HAL_Delay(100);
    }

    FRESULT result = f_mount(&gFATVolume, "0:", 1);
    if(result != FR_OK) {
        printf("ERROR: FATFS mount result is %d\n", result);
        panic();
    } else {
        printf("Mounted FATFS from SD card successfully.\n");
    }
    // printf("# ls 0:\n");
    // doCommandLS(0, NULL);

    HAL_GPIO_WritePin(RGBLED_SPI_GPIO_Port, RGBLED_SPI_Pin, GPIO_PIN_RESET);

    printf("Hello World\n");
    printf("System clock is %lu\n", HAL_RCC_GetSysClockFreq());

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

    LEDColors[0][0] = 0x10; LEDColors[0][1] = 0; LEDColors[0][2] = 0;
    LEDColors[1][0] = 0; LEDColors[1][1] = 0x10; LEDColors[1][2] = 0;
    LEDColors[2][0] = 0; LEDColors[2][1] = 0; LEDColors[2][2] = 0x10;
    write3LEDString(LEDColors);

    // Serial Terminal to ESP-01
    if(0) {
        HAL_GPIO_WritePin(GPIOG, ESP_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, ESP_RESET_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOG, ESP_RESET_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        while(1) {
            HAL_StatusTypeDef status;
            uint8_t input;

            if(HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin)) {
                HAL_GPIO_WritePin(GPIOG, ESP_RESET_Pin, GPIO_PIN_RESET);
                HAL_Delay(10);
                while(HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin));
                HAL_GPIO_WritePin(GPIOG, ESP_RESET_Pin, GPIO_PIN_SET);
            }

            do {
                status = HAL_UART_Receive(&huart4, &input, 1, 0);
                if(UART4->ISR & USART_ISR_ORE) {
                    printf("ORE\n"); HAL_Delay(1);
                    UART4->ICR = USART_ICR_ORECF;
                }
                if(UART4->ISR & USART_ISR_FE) {
                    printf("FE\n"); HAL_Delay(1);
                    UART4->ICR = USART_ICR_FECF;
                }
                if(UART4->ISR & USART_ISR_NE) {
                    printf("NE\n"); HAL_Delay(1);
                    UART4->ICR = USART_ICR_NECF;
                }
                if(UART4->ISR & USART_ISR_CMF) {
                    printf("CMF\n"); HAL_Delay(1);
                    UART4->ICR = USART_ICR_CMCF;
                }
                if(status == HAL_OK) {
                    HAL_UART_Transmit_IT(&huart2, &input, 1);
                } else if(status != HAL_TIMEOUT) {
                    printf("Status was %u\n", status);
                    HAL_Delay(100);
                    panic();
                }
            } while(status == HAL_OK);

            status = HAL_UART_Receive(&huart2, &input, 1, 0);
            if(status == HAL_OK) {
                if(0 && (input == 13)) {
                    static uint8_t crnl[2] = {13, 10};
                    HAL_UART_Transmit_IT(&huart4, crnl, 2);
                } else {
                    HAL_UART_Transmit_IT(&huart4, &input, 1);
                }
                // HAL_UART_Transmit_IT(&huart2, &input, 1);
            }
        }
    }

    if(0) {
        startVGAScanout();
        printf("VGA Scanout Started\n"); HAL_Delay(100);

        while(1) {
            if(HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin)) {
                int cx = 20 + rand() % (VGA_VISIBLE_COLUMNS - 40);
                int cy = 20 + rand() % (VGA_VISIBLE_ROWS - 40);
                int cr = 5 + rand() % 15;
                int c = rand() % 65536;
                // printf("%d %d %d\n", cx, cy , cr); HAL_Delay(10);
                DrawFilledCircle(cx, cy, cr, c);

                int x0 = 10 + rand() % (VGA_VISIBLE_COLUMNS - 20);
                int y0 = 10 + rand() % (VGA_VISIBLE_ROWS - 20);
                int x1 = 10 + rand() % (VGA_VISIBLE_COLUMNS - 20);
                int y1 = 10 + rand() % (VGA_VISIBLE_ROWS - 20);
                c = rand() % 65536;
                DrawLine(x0, y0, x1, y1, c);
            }
        }

    } else {

        startNTSCScanout();
    }

    if(0) {
        TestControllers();
    }

    if(1) {
        const char *args[] = {
            "launcher",
        };
        launcher_main(sizeof(args) / sizeof(args[0]), args);
    }

    [[maybe_unused]] int user1 = 0;
    int user2 = 0;
    int user3 = 0;

    while (1) {

#if 1
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
        // ----- USER button test
#if 0
        if(HAL_GPIO_ReadPin(USER1_GPIO_Port, USER1_Pin)) {
            if(!user1) {
                printf("user 1\n");
            }
            user1 = 1;
        } else {
            user1 = 0;
        }
#endif
        if(HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin)) {
            if(!user2) {
                printf("user 2\n");
            }
            user2 = 1;
        } else {
            user2 = 0;
        }
        if(HAL_GPIO_ReadPin(USER3_GPIO_Port, USER3_Pin)) {
            if(!user3) {
                printf("user 3\n");
            }
            printf("user 3\n");
            user3 = 1;
        } else {
            user3 = 0;
        }

#endif
        LEDTestIterate();
        if(writeLEDColors) {
            write3LEDString(LEDColors);
            writeLEDColors = 0;
        }

    }

    return 0;

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
  RCC_OscInitStruct.PLL.PLLQ = 20;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 5;
  hsd2.Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = VGA_CLOCK_DIVIDER - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = VGA_TOTAL_COLUMNS - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (VGA_TOTAL_COLUMNS - VGA_HSYNC_CLOCKS);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = VGA_CLOCK_DIVIDER - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = (VGA_TOTAL_COLUMNS * VGA_TOTAL_ROWS) - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = VGA_TOTAL_COLUMNS * (VGA_TOTAL_ROWS - VGA_VSYNC_ROWS);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 5;
  SdramTiming.RowCycleDelay = 5;
  SdramTiming.WriteRecoveryTime = 5;
  SdramTiming.RPDelay = 1;
  SdramTiming.RCDDelay = 2;

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ESP_RESET_Pin|ESP_EN_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CD_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDIO_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP_RESET_Pin ESP_EN_Pin */
  GPIO_InitStruct.Pin = ESP_RESET_Pin|ESP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PI0 PI1 PI2 PI3
                           PI4 PI5 PI6 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

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
