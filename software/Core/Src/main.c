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

#include "rocinante.h" // XXX Should not include, should be target API
#include "hid.h" // XXX Should not include, should be target API
#include "events.h" // XXX Should Not Include, should be target API

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
        delay_ms(10);
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


// Audio subsystem =========================================================

static uint8_t audioBuffer[15700 / 60 * 2 * 2]; // Two 16.667ms buffers of stereo u8 
static volatile size_t audioBufferPosition = 0;

void RoAudioGetSamplingInfo(float *rate, size_t *bufferLength, uint8_t **stereoBufferU8)
{
    // If NTSC line ISR is providing audio, we would have a sampling rate of 15.6998 KHz
    *rate = 15699.76074561403508;
    *bufferLength = sizeof(audioBuffer);
    *stereoBufferU8 = audioBuffer;
}

// I'd prefer an API where:
//
// 1) apps could allocate a real-time buffer that is composited with
// other apps' buffers, at the system rate, and blocking would occur on
// the buffer, or
//
// 2) apps could submit a one-time clip for play, and the system
// would block until the clip is played.

// Returns where one should write a half-buffer's worth of data to
// stay ahead of the audio streamer
size_t RoAudioBlockToHalfBuffer()
{
    if(audioBufferPosition > sizeof(audioBuffer) / 2) {
        size_t oldPosition = audioBufferPosition;
        while(audioBufferPosition >= oldPosition);
        return sizeof(audioBuffer) / 2;
    } else {
        while(audioBufferPosition < sizeof(audioBuffer) / 2);
        return 0;
    }
}

Status RoAudioSetHalfBufferMonoSamples(size_t where, const uint8_t *monoBufferU8)
{
    for(size_t i = 0; i < sizeof(audioBuffer) / 4; i++) {
        audioBuffer[where + i * 2 + 0] = monoBufferU8[i];
        audioBuffer[where + i * 2 + 1] = monoBufferU8[i];
    }
    return RO_SUCCESS;
}

void RoAudioClear()
{
    memset(audioBuffer, 128, sizeof(audioBuffer));
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
// DAC

#define DAC_VALUE_LIMIT 0xFF

#define MAX_DAC_VOLTAGE 1.32f
#define MAX_DAC_VOLTAGE_F16 (132 * 65536 / 100)

#define INLINE 
//inline

INLINE unsigned char voltageToDACValue(float voltage)
{
    if(voltage < 0.0f) {
        return 0x0;
    }
    uint32_t value = (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
    if(value >= DAC_VALUE_LIMIT) {
        return DAC_VALUE_LIMIT;
    }
    return value;
}

INLINE unsigned char voltageToDACValueNoBounds(float voltage)
{
    return (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
}

INLINE int voltageToDACValueFixed16NoBounds(int voltage)
{
    return (uint32_t)(voltage * 65535 / MAX_DAC_VOLTAGE_F16) * 256;
}

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


//----------------------------------------------------------------------------
// NTSC timing and voltage levels

#define NTSC_COLORBURST_FREQUENCY       3579545

// Number of samples we target; if we're doing 4x colorburst at 228 cycles, that's 912 samples at 14.318180MHz

#define ROW_SAMPLES        912
#define NTSC_EQPULSE_LINES	3
#define NTSC_VSYNC_LINES	3
#define NTSC_VBLANK_LINES	11
#define NTSC_FRAME_LINES	525

/* these are in units of one scanline */
#define NTSC_EQ_PULSE_INTERVAL	.04
#define NTSC_VSYNC_BLANK_INTERVAL	.43
#define NTSC_HOR_SYNC_DUR	.075
#define NTSC_FRONTPORCH		.02
/* BACKPORCH including COLORBURST */
#define NTSC_BACKPORCH		.075

#define NTSC_COLORBURST_CYCLES  9

#define NTSC_FRAMES		(59.94 / 2)

#define NTSC_SYNC_TIP_VOLTAGE   0.0f
#define NTSC_SYNC_PORCH_VOLTAGE   .285f
#define NTSC_SYNC_BLACK_VOLTAGE   .339f
#define NTSC_SYNC_WHITE_VOLTAGE   1.0f  /* VCR had .912v */

#define NTSC_SYNC_BLACK_VOLTAGE_F16   22217
#define NTSC_SYNC_WHITE_VOLTAGE_F16   65536

typedef uint32_t ntsc_wave_t;

INLINE unsigned char NTSCYIQToDAC(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float w_t = tcycles * M_PI * 2;
    float sine = sinf(w_t + 33.0f / 180.0f * M_PI);
    float cosine = cosf(w_t + 33.0f / 180.0f * M_PI);
    float signal = y + q * sine + i * cosine;
// end of transcription

    return voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

INLINE unsigned char NTSCYIQDegreesToDAC(float y, float i, float q, int degrees)
{
    float sine, cosine;
    if(degrees == 0) {
        sine = 0.544638f;
        cosine = 0.838670f;
    } else if(degrees == 90) {
        sine = 0.838670f;
        cosine = -0.544638f;
    } else if(degrees == 1160) {
        sine = -0.544638f;
        cosine = -0.838670f;
    } else if(degrees == 270) {
        sine = -0.838670f;
        cosine = 0.544638f;
    } else {
        sine = 0;
        cosine = 0;
    }
    float signal = y + q * sine + i * cosine;

    return voltageToDACValueNoBounds(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

INLINE ntsc_wave_t NTSCYIQToWave(float y, float i, float q)
{
    unsigned char b0 = NTSCYIQToDAC(y, i, q,  .0f);
    unsigned char b1 = NTSCYIQToDAC(y, i, q, .25f);
    unsigned char b2 = NTSCYIQToDAC(y, i, q, .50f);
    unsigned char b3 = NTSCYIQToDAC(y, i, q, .75f);

    return (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

// This is transcribed from the NTSC spec, double-checked.
INLINE void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
{
    *y = .30f * r + .59f * g + .11f * b;
    *i = -.27f * (b - *y) + .74f * (r - *y);
    *q = .41f * (b - *y) + .48f * (r - *y);
}

// Alternatively, a 3x3 matrix transforming [r g b] to [y i q] is:
// (untested - computed from equation above)
// 0.300000 0.590000 0.110000
// 0.599000 -0.277300 -0.321700
// 0.213000 -0.525100 0.312100

// A 3x3 matrix transforming [y i q] back to [r g b] is:
// (untested - inverse of 3x3 matrix above)
// 1.000000 0.946882 0.623557
// 1.000000 -0.274788 -0.635691
// 1.000000 -1.108545 1.709007

// Using inverse 3x3 matrix above.  Tested numerically to be the inverse of RGBToYIQ
INLINE void YIQToRGB(float y, float i, float q, float *r, float *g, float *b)
{
    *r = 1.0f * y + .946882f * i + 0.623557f * q;
    *g = 1.000000f * y + -0.274788f * i + -0.635691f * q;
    *b = 1.000000f * y + -1.108545f * i + 1.709007f * q;
}

INLINE ntsc_wave_t NTSCRGBToWave(float r, float g, float b)
{
    float y, i, q;
    RGBToYIQ(r, g, b, &y, &i, &q);
    return NTSCYIQToWave(y, i, q);
}

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

    NTSCSyncTip = voltageToDACValue(NTSC_SYNC_TIP_VOLTAGE);
    NTSCSyncPorch = voltageToDACValue(NTSC_SYNC_PORCH_VOLTAGE);
    NTSCBlack = voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE);
    NTSCWhite = voltageToDACValue(NTSC_SYNC_WHITE_VOLTAGE);

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

typedef void (*NTSCModeFillRowBufferFunc)(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer);
typedef int (*NTSCModeNeedsColorburstFunc)();
int NTSCModeFuncsValid = 0;
NTSCModeFillRowBufferFunc NTSCModeFillRowBuffer = DefaultFillRowBuffer;
NTSCModeNeedsColorburstFunc NTSCModeNeedsColorburst = DefaultNeedsColorburst;

void NTSCSwitchModeFuncs(NTSCModeFillRowBufferFunc fillBufferFunc, NTSCModeNeedsColorburstFunc needsColorBurstFunc)
{
    NTSCModeFuncsValid = 0;
    NTSCModeNeedsColorburst = needsColorBurstFunc;
    NTSCModeFillRowBuffer = fillBufferFunc;
    NTSCModeFuncsValid = 1;
}

void NTSCFillRowBuffer(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    // XXX could optimize these by having one branch be lines < 21
    /*
     * Rows 0 through 8 are equalizing pulse, then vsync, then equalizing pulse
     */
    if(lineNumber < NTSC_EQPULSE_LINES) {

        memcpy(rowBuffer, NTSCEqSyncPulseLine, sizeof(NTSCEqSyncPulseLine));

    } else if(lineNumber - NTSC_EQPULSE_LINES < NTSC_VSYNC_LINES) {

        memcpy(rowBuffer, NTSCVSyncLine, sizeof(NTSCVSyncLine));

    } else if(lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES) < NTSC_EQPULSE_LINES) {

        memcpy(rowBuffer, NTSCEqSyncPulseLine, sizeof(NTSCEqSyncPulseLine));

    } else if(lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES) < NTSC_VBLANK_LINES) {

        /*
         * Rows 9 through 2X are other part of vertical blank
         */

        if(NTSCModeFuncsValid) {
            memcpy(rowBuffer, NTSCModeNeedsColorburst() ? NTSCBlankLineColor : NTSCBlankLineBW, ROW_SAMPLES);
        } else {
            memcpy(rowBuffer, NTSCBlankLineBW, ROW_SAMPLES);
        }


    } else if(lineNumber >= 263 && lineNumber <= 271) {
        // Interlacing handling weird lines
        if(lineNumber <= 264) {
            //lines 263, 264 - last 405 of eq pulse then first 405 of eq pulse
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 265) {
            //line 265 - last 405 of eq pulse then first 405 of vsync
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCVSyncLine, ROW_SAMPLES / 2);
        } else if(lineNumber <= 267) {
            //lines 266, 267 - last 405 of vsync then first 405 of vsync
            memcpy(rowBuffer, NTSCVSyncLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCVSyncLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 268) {
            //lines 268 - last 405 of vsync then first 405 of eq pulse
            memcpy(rowBuffer, NTSCVSyncLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber <= 270) {
            //lines 269, 270 - last 405 of eq pulse then first 405 of eq pulse
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 271) {
            //line 271 - last 405 of eq pulse then 405 of SyncPorch
            memcpy(rowBuffer, NTSCEqSyncPulseLine + ROW_SAMPLES / 2, ROW_SAMPLES / 2);
            memset(rowBuffer + ROW_SAMPLES / 2, NTSCSyncPorch, ROW_SAMPLES / 2);
        }

    } else if((lineNumber >= 272) && (lineNumber <= 281)) { // XXX half line at 282

        /*
         * Rows 272 through 2XX are other part of vertical blank
         */

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

        int rowWithinFrame = (lineNumber % 263) * 2 + lineNumber / 263 - 22;
        if(NTSCModeFuncsValid) {
            NTSCModeFillRowBuffer(frameNumber, rowWithinFrame, 704, rowBuffer + 164);
        }

        if(lineNumber == 262) {
            //line 262 - overwrite last 405 samples with first 405 samples of EQ pulse
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 282) {
            //special line 282 - write SyncPorch from BackPorch to middle of line after mode's fillRow()
            memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCSyncPorch, ROW_SAMPLES / 2 - (NTSCHSyncClocks + NTSCBackPorchClocks));
        }
    }
}

// debug overlay scanout

// There should be some kind of a way to reserve a "slot" (not
// always the same line on screen) for debug output for system or other
// continuous output, and then a way to scroll debug output logging


#include "8x16.h"
// static int font8x16Width = 8, font8x16Height = 16;
// static unsigned char font8x16Bits[] = /* was a bracket here */

int debugOverlayEnabled = 0;

#define debugDisplayLeftTick (NTSCHSyncClocks + NTSCBackPorchClocks + 56)
#define debugDisplayTopTick (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES + 10)
/* debugFontWidthScale != 4 looks terrible in a color field because of adjacent color columns; probably need to ensure 0s around any 1 text column */
#define debugFontWidthScale 2
#define debugCharGapPixels 1
#define debugFontHeightScale 1
#define debugDisplayWidth (604 / (font8x16Width * debugFontWidthScale + debugCharGapPixels))
#define debugDisplayHeight ((218 + 10) / font8x16Height)

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

uint8_t /*  __attribute__((section (".ram_d1"))) */ NTSCRowDoubleBuffer[ROW_SAMPLES * 2];
int NTSCRowNumber = 0;
int NTSCFrameNumber = 0;

int why = 0;

enum DisplayMode { DISPLAY_MODE_NONE, DISPLAY_MODE_NTSC, DISPLAY_MODE_VGA } currentDisplayMode = DISPLAY_MODE_NONE;

void NTSCRowHandler(void)
{
    NTSCRowNumber = (NTSCRowNumber + 1) % 525;
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
    DAC1->DHR8R1 = audioBuffer[audioBufferPosition + 0];
    DAC1->DHR8R2 = audioBuffer[audioBufferPosition + 1];
    audioBufferPosition = (audioBufferPosition + 2) % sizeof(audioBuffer);

    NTSCFillRowBuffer(NTSCFrameNumber, NTSCRowNumber, rowDest);
    if(debugOverlayEnabled) {
        NTSCFillRowDebugOverlay(NTSCFrameNumber, NTSCRowNumber, rowDest);
    }
    SCB_CleanDCache();

    // A little pulse so we know where we are on the line when we finished
    if(0 /* markHandlerInSamples */) {
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
// Woz (Apple ][ graphics) mode

#define WOZ_MODE_LEFT 72 
#define WOZ_MODE_WIDTH 560 
#define WOZ_MODE_TOP 60 
#define WOZ_MODE_HEIGHT 192 
#define WOZ_MODE_MIXED_TEXT_ROWS 4
#define WOZ_MODE_FONT_HEIGHT 8
#define WOZ_MODE_FONT_WIDTH 7

enum WozDisplayMode {TEXT, LORES, HIRES};
enum WozDisplayMode WozModeDisplayMode = TEXT;
int WozModeAux = 0;
int WozModeMixed = 0;
int WozModePage = 0;
int WozModeVid80 = 0;
int WozModeDHGR = 0;

void WozModeClearFlags()
{
    WozModeDisplayMode = TEXT;
    WozModeAux = 0;
    WozModeMixed = 0;
    WozModePage = 0;
    WozModeVid80 = 0;
    WozModeDHGR = 0;
}
// indexed by aux, then by page, then by buffer address in scan order, not in Apple ][ memory order
uint8_t WozModeHGRBuffers[2][2][7680];
uint8_t WozModeTextBuffers[2][2][960];

__attribute__((hot,flatten)) void WozModeFillRowBufferHGR(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    uint8_t darker = NTSCBlack + (NTSCWhite - NTSCBlack) / 4; // XXX debug
        // Serial Terminal to ESP-01
    if((rowIndex >= 0) && (rowIndex < 192)) {
        const uint8_t *rowSrc = WozModeHGRBuffers[WozModeAux][WozModePage] + rowIndex * 40; // row - ...?

        for(int byteIndex = 0; byteIndex < 40; byteIndex++) {

            uint8_t byte = *rowSrc++;

            uint8_t colorShift = byte >> 7;
            uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT + byteIndex * 14 + colorShift;

            for(int bitIndex = 0; bitIndex < 7; bitIndex++) {
                if(byte & 0x1) {
                    if(0) {
                        // XXX debug
                        *rowDst++ = darker;
                        *rowDst++ = darker;
                    } else {
                        *rowDst++ = NTSCWhite;
                        *rowDst++ = NTSCWhite;
                    }
                } else {
                    if(0) {
                        *rowDst++ = NTSCBlack;
                        *rowDst++ = NTSCBlack;
                    } else {
                        rowDst += 2;
                    }
                }
                byte = byte >> 1;
            }
        }
    }
}

__attribute__((hot,flatten)) void WozModeFillRowBufferDHGR(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    if((rowIndex >= 0) && (rowIndex < 192)) {
        const uint8_t *rowSrc;
        
        /* Even is bytes from AUX DHGR */
        rowSrc = WozModeHGRBuffers[1][WozModePage] + rowIndex * 40; // row - ...?
        for(int byteIndex = 0; byteIndex < 40; byteIndex++) {

            uint8_t byte = *rowSrc++;

            /* For whatever reason, DHGR starts one clock late. */
            uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT + byteIndex * 14 + 1;

            for(int bitIndex = 0; bitIndex < 7; bitIndex++) {
                *rowDst++ = (byte & 0x1) ? NTSCWhite : NTSCBlack;
                byte = byte >> 1;
            }
        }

        /* Odd is bytes from MAIN DHGR */
        rowSrc = WozModeHGRBuffers[0][WozModePage] + rowIndex * 40; // row - ...?
        for(int byteIndex = 0; byteIndex < 40; byteIndex++) {

            uint8_t byte = *rowSrc++;

            /* For whatever reason, DHGR starts one clock late. */
            uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT + byteIndex * 14 + 1 + 7;

            for(int bitIndex = 0; bitIndex < 7; bitIndex++) {
                *rowDst++ = (byte & 0x1) ? NTSCWhite : NTSCBlack;
                byte = byte >> 1;
            }
        }
    }
}

int WozModeFontOffset = 32;
const unsigned char WozModeFontBytes[96 * WOZ_MODE_FONT_HEIGHT] = {
    // 32 :  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    // 33 : !
    0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00, 
    // 34 : "
    0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 
    // 35 : #
    0x14, 0x14, 0x3E, 0x14, 0x3E, 0x14, 0x14, 0x00, 
    // 36 : $
    0x08, 0x3C, 0x0A, 0x1C, 0x28, 0x1E, 0x08, 0x00, 
    // 37 : %
    0x06, 0x26, 0x10, 0x08, 0x04, 0x32, 0x30, 0x00, 
    // 38 : &
    0x04, 0x0A, 0x0A, 0x04, 0x2A, 0x12, 0x2C, 0x00, 
    // 39 : '
    0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
    // 40 : (
    0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08, 0x00, 
    // 41 : )
    0x08, 0x10, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, 
    // 42 : *
    0x08, 0x2A, 0x1C, 0x08, 0x1C, 0x2A, 0x08, 0x00, 
    // 43 : +
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 
    // 44 : ,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x04, 0x00, 
    // 45 : -
    0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 
    // 46 : .
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 
    // 47 : /
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 
    // 48 : 0
    0x1C, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x1C, 0x00, 
    // 49 : 1
    0x08, 0x0C, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 
    // 50 : 2
    0x1C, 0x22, 0x20, 0x18, 0x04, 0x02, 0x3E, 0x00, 
    // 51 : 3
    0x3E, 0x20, 0x10, 0x18, 0x20, 0x22, 0x1C, 0x00, 
    // 52 : 4
    0x10, 0x18, 0x14, 0x12, 0x3E, 0x10, 0x10, 0x00, 
    // 53 : 5
    0x3E, 0x02, 0x1E, 0x20, 0x20, 0x22, 0x1C, 0x00, 
    // 54 : 6
    0x38, 0x04, 0x02, 0x1E, 0x22, 0x22, 0x1C, 0x00, 
    // 55 : 7
    0x3E, 0x20, 0x10, 0x08, 0x04, 0x04, 0x04, 0x00, 
    // 56 : 8
    0x1C, 0x22, 0x22, 0x1C, 0x22, 0x22, 0x1C, 0x00, 
    // 57 : 9
    0x1C, 0x22, 0x22, 0x3C, 0x20, 0x10, 0x0E, 0x00, 
    // 58 : :
    0x00, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 
    // 59 : ;
    0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x04, 0x00, 
    // 60 : <
    0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x00, 
    // 61 : =
    0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00, 0x00, 
    // 62 : >
    0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04, 0x00, 
    // 63 : ?
    0x1C, 0x22, 0x10, 0x08, 0x08, 0x00, 0x08, 0x00, 
    // 64 : @
    0x1C, 0x22, 0x2A, 0x3A, 0x1A, 0x02, 0x3C, 0x00, 
    // 65 : A
    0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x00, 
    // 66 : B
    0x1E, 0x22, 0x22, 0x1E, 0x22, 0x22, 0x1E, 0x00, 
    // 67 : C
    0x1C, 0x22, 0x02, 0x02, 0x02, 0x22, 0x1C, 0x00, 
    // 68 : D
    0x1E, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1E, 0x00, 
    // 69 : E
    0x3E, 0x02, 0x02, 0x1E, 0x02, 0x02, 0x3E, 0x00, 
    // 70 : F
    0x3E, 0x02, 0x02, 0x1E, 0x02, 0x02, 0x02, 0x00, 
    // 71 : G
    0x3C, 0x02, 0x02, 0x02, 0x32, 0x22, 0x3C, 0x00, 
    // 72 : H
    0x22, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22, 0x00, 
    // 73 : I
    0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 
    // 74 : J
    0x20, 0x20, 0x20, 0x20, 0x20, 0x22, 0x1C, 0x00, 
    // 75 : K
    0x22, 0x12, 0x0A, 0x06, 0x0A, 0x12, 0x22, 0x00, 
    // 76 : L
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x3E, 0x00, 
    // 77 : M
    0x22, 0x36, 0x2A, 0x2A, 0x22, 0x22, 0x22, 0x00, 
    // 78 : N
    0x22, 0x22, 0x26, 0x2A, 0x32, 0x22, 0x22, 0x00, 
    // 79 : O
    0x1C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 
    // 80 : P
    0x1E, 0x22, 0x22, 0x1E, 0x02, 0x02, 0x02, 0x00, 
    // 81 : Q
    0x1C, 0x22, 0x22, 0x22, 0x2A, 0x12, 0x2C, 0x00, 
    // 82 : R
    0x1E, 0x22, 0x22, 0x1E, 0x0A, 0x12, 0x22, 0x00, 
    // 83 : S
    0x1C, 0x22, 0x02, 0x1C, 0x20, 0x22, 0x1C, 0x00, 
    // 84 : T
    0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 
    // 85 : U
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 
    // 86 : V
    0x22, 0x22, 0x22, 0x22, 0x22, 0x14, 0x08, 0x00, 
    // 87 : W
    0x22, 0x22, 0x22, 0x2A, 0x2A, 0x36, 0x22, 0x00, 
    // 88 : X
    0x22, 0x22, 0x14, 0x08, 0x14, 0x22, 0x22, 0x00, 
    // 89 : Y
    0x22, 0x22, 0x14, 0x08, 0x08, 0x08, 0x08, 0x00, 
    // 90 : Z
    0x3E, 0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x00, 
    // 91 : [
    0x3E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x3E, 0x00, 
    // 92 : backslash
    0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 
    // 93 : ]
    0x3E, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3E, 0x00, 
    // 94 : ^
    0x00, 0x00, 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 
    // 95 : _
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 
    // 96 : `
    0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 
    // 97 : a
    0x00, 0x00, 0x1C, 0x20, 0x3C, 0x22, 0x3C, 0x00, 
    // 98 : b
    0x02, 0x02, 0x1E, 0x22, 0x22, 0x22, 0x1E, 0x00, 
    // 99 : c
    0x00, 0x00, 0x3C, 0x02, 0x02, 0x02, 0x3C, 0x00, 
    // 100 : d
    0x20, 0x20, 0x3C, 0x22, 0x22, 0x22, 0x3C, 0x00, 
    // 101 : e
    0x00, 0x00, 0x1C, 0x22, 0x3E, 0x02, 0x3C, 0x00, 
    // 102 : f
    0x18, 0x24, 0x04, 0x1E, 0x04, 0x04, 0x04, 0x00, 
    // 103 : g
    0x00, 0x00, 0x1C, 0x22, 0x22, 0x3C, 0x20, 0x1C, 
    // 104 : h
    0x02, 0x02, 0x1E, 0x22, 0x22, 0x22, 0x22, 0x00, 
    // 105 : i
    0x08, 0x00, 0x0C, 0x08, 0x08, 0x08, 0x1C, 0x00, 
    // 106 : j
    0x10, 0x00, 0x18, 0x10, 0x10, 0x10, 0x12, 0x0C, 
    // 107 : k
    0x02, 0x02, 0x22, 0x12, 0x0E, 0x12, 0x22, 0x00, 
    // 108 : l
    0x0C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 
    // 109 : m
    0x00, 0x00, 0x36, 0x2A, 0x2A, 0x2A, 0x22, 0x00, 
    // 110 : n
    0x00, 0x00, 0x1E, 0x22, 0x22, 0x22, 0x22, 0x00, 
    // 111 : o
    0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x00, 
    // 112 : p
    0x00, 0x00, 0x1E, 0x22, 0x22, 0x1E, 0x02, 0x02, 
    // 113 : q
    0x00, 0x00, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x20, 
    // 114 : r
    0x00, 0x00, 0x3A, 0x06, 0x02, 0x02, 0x02, 0x00, 
    // 115 : s
    0x00, 0x00, 0x3C, 0x02, 0x1C, 0x20, 0x1E, 0x00, 
    // 116 : t
    0x04, 0x04, 0x1E, 0x04, 0x04, 0x24, 0x18, 0x00, 
    // 117 : u
    0x00, 0x00, 0x22, 0x22, 0x22, 0x32, 0x2C, 0x00, 
    // 118 : v
    0x00, 0x00, 0x22, 0x22, 0x22, 0x14, 0x08, 0x00, 
    // 119 : w
    0x00, 0x00, 0x22, 0x22, 0x2A, 0x2A, 0x36, 0x00, 
    // 120 : x
    0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 
    // 121 : y
    0x00, 0x00, 0x22, 0x22, 0x22, 0x3C, 0x20, 0x1C, 
    // 122 : z
    0x00, 0x00, 0x3E, 0x10, 0x08, 0x04, 0x3E, 0x00, 
    // 123 : {
    0x38, 0x0C, 0x0C, 0x06, 0x0C, 0x0C, 0x38, 0x00, 
    // 124 : |
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 
    // 125 : }
    0x0E, 0x18, 0x18, 0x30, 0x18, 0x18, 0x0E, 0x00, 
    // 126 : ~
    0x2C, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    // 127 : 
    0x00, 0x2A, 0x14, 0x2A, 0x14, 0x2A, 0x00, 0x00, 
};

void WozMemoryByteToFontIndex(int byte, int *fontIndex, int *inverse)
{
    *inverse = 0;

    if(byte >= 0 && byte <= 31) {
        *fontIndex = byte - 0 + 32;
        *inverse = 1;
    } else if(byte >= 32 && byte <= 63) {
        *fontIndex = byte - 32 + 0;
        *inverse = 1;
    } else if(byte >= 64 && byte <= 95) {
        *fontIndex = byte - 64 + 32; // XXX BLINK 
        *inverse = 1;
    } else if(byte >= 96 && byte <= 127){
        *fontIndex = byte - 96 + 0; // XXX BLINK 
        *inverse = 1;
    } else if(byte >= 128 && byte <= 159)
        *fontIndex = byte - 128 + 32;
    else if(byte >= 160 && byte <= 191)
        *fontIndex = byte - 160 + 0;
    else if(byte >= 192 && byte <= 223)
        *fontIndex = byte - 192 + 32;
    else if(byte >= 224 && byte <= 255)
        *fontIndex = byte - 224 + 64;
    else 
        *fontIndex = 33;
}

void WozModeFillRowBuffer80Text(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    if((rowIndex >= 0) && (rowIndex < 192)) {
        memset(rowBuffer + WOZ_MODE_LEFT, NTSCBlack, WOZ_MODE_WIDTH);
        int rowInText = rowIndex / 8;
        int rowInGlyph = rowIndex % 8;

        const uint8_t *rowSrc;
        uint8_t *rowDst;

        rowSrc = WozModeTextBuffers[0][WozModePage] + rowInText * 40;
        rowDst = rowBuffer + WOZ_MODE_LEFT + 7;
        for(int textColumn = 0; textColumn < 40; textColumn++) {
            uint8_t byte = rowSrc[textColumn];
            int fontIndex, inverse;
            WozMemoryByteToFontIndex(byte, &fontIndex, &inverse);
            int fontRowByte = WozModeFontBytes[fontIndex * 8 + rowInGlyph];
            for(int column = 0; column < 7; column ++) {
                *rowDst++ = ((fontRowByte & 0x01) ^ inverse) ? NTSCWhite : NTSCBlack;
                fontRowByte = fontRowByte >> 1;
            }
            rowDst += 7;
        }

        rowSrc = WozModeTextBuffers[1][WozModePage] + rowInText * 40;
        rowDst = rowBuffer + WOZ_MODE_LEFT + 0;
        for(int textColumn = 0; textColumn < 40; textColumn++) {
            uint8_t byte = rowSrc[textColumn];
            int fontIndex, inverse;
            WozMemoryByteToFontIndex(byte, &fontIndex, &inverse);
            int fontRowByte = WozModeFontBytes[fontIndex * 8 + rowInGlyph];
            for(int column = 0; column < 7; column ++) {
                *rowDst++ = ((fontRowByte & 0x01) ^ inverse) ? NTSCWhite : NTSCBlack;
                fontRowByte = fontRowByte >> 1;
            }
            rowDst += 7;
        }
    }
}

void WozModeFillRowBuffer40Text(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    if((rowIndex >= 0) && (rowIndex < 192)) {
        memset(rowBuffer + WOZ_MODE_LEFT, NTSCBlack, WOZ_MODE_WIDTH);
        int rowInText = rowIndex / 8;
        int rowInGlyph = rowIndex % 8;
        const uint8_t *rowSrc = WozModeTextBuffers[WozModeAux][WozModePage] + rowInText * 40;
        uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT;
        for(int textColumn = 0; textColumn < 40; textColumn++) {
            uint8_t byte = rowSrc[textColumn];
            int fontIndex, inverse;
            WozMemoryByteToFontIndex(byte, &fontIndex, &inverse);
            int fontRowByte = WozModeFontBytes[fontIndex * 8 + rowInGlyph];
            for(int column = 0; column < 7; column ++) {
                *rowDst++ = ((fontRowByte & 0x01) ^ inverse) ? NTSCWhite : NTSCBlack;
                *rowDst++ = ((fontRowByte & 0x01) ^ inverse) ? NTSCWhite : NTSCBlack;
                fontRowByte = fontRowByte >> 1;
            }
        }
    }
}

void WozModeFillRowBufferLGR(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    if((rowIndex >= 0) && (rowIndex < 192)) {

        memset(rowBuffer + WOZ_MODE_LEFT, NTSCBlack, WOZ_MODE_WIDTH);

        int rowInText = rowIndex / 8;
        int rowInGlyph = rowIndex % 8;

        const uint8_t *rowSrc = WozModeTextBuffers[WozModeAux][WozModePage] + rowInText * 40;

        uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT;

        for(int column = 0; column < 560;) {
            uint8_t byte = rowSrc[column / 14];
            uint8_t nybble = ((rowInGlyph < 4) ? (byte) : (byte >> 4)) & 0xF;

            for(int i = 0; i < 14; i++, column++) {
                int bitInNybble = column % 4;
                int bit = nybble & (1 << bitInNybble);
                *rowDst++ = bit ? NTSCWhite : NTSCBlack;
            }
        }
    }
}

int WozModeNeedsColorburst()
{
    return (WozModeDisplayMode != TEXT);
}

void WozModeFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    enum WozDisplayMode mode = WozModeDisplayMode;
    if(WozModeMixed && (rowIndex >= WOZ_MODE_HEIGHT - WOZ_MODE_MIXED_TEXT_ROWS * WOZ_MODE_FONT_HEIGHT)) {
        mode = TEXT;
    }
    switch(mode) {
        case TEXT: 
            if(WozModeVid80) {
                WozModeFillRowBuffer80Text(frameIndex, rowNumber, maxSamples, rowBuffer);
            } else {
                WozModeFillRowBuffer40Text(frameIndex, rowNumber, maxSamples, rowBuffer);
            }
            break;
        case HIRES: 
            if(WozModeDHGR) {
                WozModeFillRowBufferDHGR(frameIndex, rowNumber, maxSamples, rowBuffer);
            } else {
                WozModeFillRowBufferHGR(frameIndex, rowNumber, maxSamples, rowBuffer);
            }
            break;
        case LORES: 
            WozModeFillRowBufferLGR(frameIndex, rowNumber, maxSamples, rowBuffer);
            break;
    }
}

int AlwaysColorburst()
{
    return 1;
}

//----------------------------------------------------------------------------
// Text Mode

#define TextModeLeftTick 56
#define TextModeTopTick 18
#define TextModeFontWidthScale 2
#define TextModeCharGapPixels 1
#define TextModeFontHeightScale 1
#define TextModeWidth (604 / (font8x16Width * TextModeFontWidthScale + TextModeCharGapPixels))
#define TextModeHeight ((218 + 10) / font8x16Height)

enum {
    TEXT_NO_ATTRIBUTES = 0x00,
    TEXT_INVERSE       = 0x01,
};
uint8_t TextModeAttributes[debugDisplayHeight * debugDisplayWidth];
char TextModeBuffer[debugDisplayHeight * debugDisplayWidth];

__attribute__((hot,flatten)) void TextModeFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int fontScanlineHeight = font8x16Height * TextModeFontHeightScale;

    int rowWithinTextArea = rowNumber / 2 - TextModeTopTick;

    int charRow = rowWithinTextArea / fontScanlineHeight;
    int charPixelY = (rowWithinTextArea % fontScanlineHeight) / TextModeFontHeightScale;

// XXX this code assumes font width <= 8 and each row padded out to a byte
    if((rowWithinTextArea >= 0) && (charRow < TextModeHeight)) {

        uint8_t *rowDst = rowBuffer + TextModeLeftTick;

        for(int charCol = 0; charCol < TextModeWidth; charCol++) {

            uint8_t character = TextModeBuffer[charRow * TextModeWidth + charCol];
            int inverse = (TextModeAttributes[charRow * TextModeWidth + charCol] & TEXT_INVERSE) ? 0xFF : 0;
            uint8_t charRowBits = font8x16Bits[character * font8x16Height + charPixelY] ^ inverse;

            for(int charPixelX = 0; charPixelX < font8x16Width; charPixelX++) {

                int pixel = (charRowBits & (0x80 >> charPixelX));

                for(int col = 0; col < TextModeFontWidthScale; col++) {
                    *rowDst++ = pixel ? NTSCWhite : NTSCBlack;
                }
            }
        }
    }
}

void TextModeClearDisplay()
{
    memset(TextModeBuffer, ' ', sizeof(TextModeBuffer));
}

void TextModeGetSize(int *w, int *h)
{
    *w = TextModeWidth;
    *h = TextModeHeight;
}

int TextModeNeedsColorburst()
{
    return 0;
}

void TextModeClearArea(int column, int w, int row, int h, uint8_t attributes)
{
    for(int y = row; y < row + h; y++) {
        memset(TextModeBuffer + y * TextModeWidth + column, ' ', w);
        memset(TextModeAttributes + y * TextModeWidth + column, attributes, w);
    }
}

void TextModeSetLine(int row, int column, uint8_t attributes, const char *string)
{
    int stringExceedsWidth = strlen(string) > TextModeWidth - column;
    size_t toCopy = stringExceedsWidth ? TextModeWidth - column : strlen(string);
    memcpy(TextModeBuffer + row * TextModeWidth + column, string, toCopy);
    memset(TextModeAttributes + row * TextModeWidth + column, attributes, toCopy);
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

void ImageFillRowBuffer2(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    // times 2 because we are given interlaced height rows, 0 to 238
    if((rowNumber > WOZ_MODE_TOP) && (rowNumber < WOZ_MODE_TOP + WOZ_MODE_HEIGHT * 2)) {
        for(int col = 0; col < WOZ_MODE_WIDTH; col++) {
            int checker = (col / 35 + rowNumber / 20) % 2;
            rowBuffer[WOZ_MODE_LEFT + col] = checker ? NTSCWhite : NTSCBlack;
        }
    }
}

int apple2_main(int argc, const char **argv);

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
  
int main_iterate(void)
{
    static int q = 0;
    if(++q > 1000) {
        q = 0;
        // printf(".");
    }

    MX_USB_HOST_Process();

    // LEDTestIterate();

    if(writeLEDColors) {
        write3LEDString(LEDColors);
        writeLEDColors = 0;
    }

    return 0;
}

void HGRModeTest()
{
    printf("HGR Mode Test\n");

    memset(WozModeHGRBuffers, 0, sizeof(WozModeHGRBuffers));
    WozModeDisplayMode = HIRES;
    NTSCSwitchModeFuncs(WozModeFillRowBuffer, WozModeNeedsColorburst);

    static int x = 140;
    static int y = 96;

    struct Event ev;
    int done = 0;
    int palette = 0x00;

    KeyRepeatManager keyRepeat;

    while(!done) {
        int haveEvent = RoEventPoll(&ev);
        
        haveEvent = KeyRepeatUpdate(&keyRepeat, haveEvent, &ev);

        if(haveEvent) {
            int draw = 0;
            int clear = 0;
            switch(ev.eventType) {
                case MOUSE_MOVE: {
                    const struct MouseMoveEvent move = ev.u.mouseMove;
                    x += move.x;
                    y += move.y;
                    draw = 1;
                    break;
                }
                case MOUSE_BUTTONPRESS: {
                    const struct MouseButtonPressEvent press = ev.u.mouseButtonPress;
                    if(press.button == 0) {
                        clear = 1;
                    } else {
                        palette ^= 0x80;
                    }
                }
                case KEYBOARD_RAW: {
                    const struct KeyboardRawEvent raw = ev.u.keyboardRaw;
                    if(raw.isPress) {

                        if(raw.key == KEYCAP_UP) {
                            y -= 1;
                            draw = 1;
                        } else if(raw.key == KEYCAP_DOWN) {
                            y += 1;
                            draw = 1;
                        } else if(raw.key == KEYCAP_LEFT) {
                            x -= 1;
                            draw = 1;
                        } else if(raw.key == KEYCAP_RIGHT) {
                            x += 1;
                            draw = 1;
                        } else if(raw.key == KEYCAP_1_EXCLAMATION) {
                            palette = 0x00;
                            draw = 1;
                        } else if(raw.key == KEYCAP_2_AT) {
                            palette = 0x80;
                            draw = 1;
                        } else if(raw.key == KEYCAP_END) {
                            clear = 1;
                        }
                    }
                    break;
                }
                default:
                    // pass;
                    break;
            }
            if(draw) {
                if((x >= 0) && (y >= 0) && (x < 280) && (y < 192)) {
                    WozModeHGRBuffers[0][0][y * 40 + x / 7] |= (1 << (x % 7)) | palette;
                }
            } 
            if(clear) {
                memset(WozModeHGRBuffers, 0, sizeof(WozModeHGRBuffers));
            }
        }

        main_iterate();
    }
}

void TextModeTest()
{
    printf("Text Mode Text\n");

    memset(WozModeTextBuffers, 248, sizeof(WozModeTextBuffers));
    NTSCSwitchModeFuncs(WozModeFillRowBuffer, WozModeNeedsColorburst);
    // fill with ONE TWO THREE FOUR over and over again

    const char *str = "ONE TWO FREE FOUR ";
    int stringIndex = 0;

    char *buffer = malloc(sizeof(WozModeTextBuffers[0][0]) * 2);
    for(size_t s = 0; s < sizeof(WozModeTextBuffers[0][0]) * 2; s++) {
        buffer[s] = str[stringIndex];
        stringIndex = (stringIndex + 1) % strlen(str);
    }

    int bufferIndex = 0;
    while(1) {
        memcpy(WozModeTextBuffers[0][0], buffer + bufferIndex, sizeof(WozModeTextBuffers[0][0]));
        bufferIndex = (bufferIndex + 1) % sizeof(WozModeTextBuffers[0][0]);
        char *foobar = malloc(bufferIndex * 667 % 513);
        free(foobar);
        main_iterate();
    }
}

extern void enqueue_ascii(int s);

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

extern int errno;

int playAudio(int argc, const char **argv)
{
    float rate;
    size_t bufferLength;
    uint8_t *stereoBuffer;

    RoAudioGetSamplingInfo(&rate, &bufferLength, &stereoBuffer);
    size_t halfBufferSamples = bufferLength / 2;
    size_t halfBufferMonoSamples = halfBufferSamples / 2;
    uint8_t *monoTrack = malloc(halfBufferMonoSamples);

    const char *filename = argv[1];
    FILE *fp = fopen (filename, "rb");
    if(fp == NULL) {
        printf("ERROR: couldn't open \"%s\" for reading, errno %d\n", filename, errno);
        return 1;
    }

    size_t where;
    size_t samplesRead = 0;
    int quit = 0;
    do {
        // Wait for audio to get at least half a buffer past us
        samplesRead = fread(monoTrack, 1, halfBufferMonoSamples, fp);
        if(samplesRead < 1) {
            printf("ERROR: couldn't read block of audio from \"%s\", read %zd\n", filename, samplesRead);
            return 1;
        }
        where = RoAudioBlockToHalfBuffer();
        for(int i = 0; i < samplesRead; i++) {
            int v = monoTrack[i];
            audioBuffer[where + i * 2 + 0] = v; // 128 + (v - 128) * a;
            audioBuffer[where + i * 2 + 1] = v; // 128 + (v - 128) * (1 - a);
        }

        Event ev;
        int haveEvent = RoEventPoll(&ev);

        if(haveEvent) {
            switch(ev.eventType) {
                case KEYBOARD_RAW: {
                    const struct KeyboardRawEvent raw = ev.u.keyboardRaw;
                    if(raw.isPress) {
                        quit = 1;
                    }
                    break;
                }
                default:
                    // pass;
                    break;
            }
        }
        main_iterate(); // XXX
    } while(!quit && (samplesRead == halfBufferMonoSamples));

    // fill any part not read from file to silence
    for(int i = samplesRead; i < halfBufferMonoSamples; i++) {
        audioBuffer[where + i * 2 + 0] = 128;
        audioBuffer[where + i * 2 + 1] = 128;
    }

    // Wait until audio is playing our final half and clear the other half
    where = RoAudioBlockToHalfBuffer();
    memset(audioBuffer + where, 128, halfBufferSamples);

    // Wait until audio is done with our final half then clear it
    where = RoAudioBlockToHalfBuffer();
    memset(audioBuffer + where, 128, halfBufferSamples);

    free(monoTrack);

    fclose(fp);

    return 0;
}

// File chooser and UI routines

enum {
    CHOOSE_FILE_NO_FLAGS = 0,
    CHOOSE_FILE_IGNORE_DOTFILES = 0x01,
};

Status FillFilenameList(const char* dirName, uint32_t flags, const char* optionalFilterSuffix, size_t maxNames, char **filenames, size_t* filenamesSize)
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

void ShowListOfItems(const char *title, char **items, size_t itemsSize, int whichAtTop, int whichSelected)
{
    int w, h;
    TextModeGetSize(&w, &h);
    TextModeClearDisplay();

    int titleIndent = (w - strlen(title)) / 2;
    TextModeSetLine(0, titleIndent, TEXT_NO_ATTRIBUTES, title);

    int tooManyRows = itemsSize - whichAtTop > (h - 4);
    int rowsToDisplay = tooManyRows ? (h - 4) : (itemsSize - whichAtTop);
    for(int i = 0; i < rowsToDisplay; i++) {
        if(i == whichSelected - whichAtTop) {
            TextModeClearArea(0, w, i + 2, 1, TEXT_INVERSE);
            TextModeSetLine(i + 2, 0, TEXT_INVERSE, items[whichAtTop + i]);
        } else {
            TextModeClearArea(0, w, i + 2, 1, TEXT_NO_ATTRIBUTES);
            TextModeSetLine(i + 2, 0, TEXT_NO_ATTRIBUTES, items[whichAtTop + i]);
        }
    }

    static const char* prompt = "ESC - Cancel, ENTER - Choose";
    int promptIndent = (w - strlen(prompt)) / 2;
    TextModeSetLine(h - 1, promptIndent, TEXT_NO_ATTRIBUTES, prompt);
}

void DisplayStringAndWaitForEnter(const char *message)
{
    int w, h;
    NTSCSwitchModeFuncs(TextModeFillRowBuffer, TextModeNeedsColorburst);
    TextModeGetSize(&w, &h);
    TextModeClearDisplay();

    TextModeSetLine(h / 2, (w - strlen(message)) / 2, TEXT_NO_ATTRIBUTES, message);

    int done = 0;
    while(!done) {
        Event ev;
        int haveEvent = RoEventPoll(&ev);
        
        if(haveEvent) {
            switch(ev.eventType) {
                case KEYBOARD_RAW: {
                    // const struct KeyboardRawEvent raw = ev.u.keyboardRaw;
                    done = 1;
                    break;
                }
                default:
                    // pass;
                    break;
            }
        }
        main_iterate(); // XXX
    }
}

Status PromptUserToChooseFile(const char *title, const char *dirName, uint32_t flags, const char *optionalFilterSuffix, char** fileChosen)
{
    NTSCSwitchModeFuncs(TextModeFillRowBuffer, TextModeNeedsColorburst);

    char *filenames[256];
    size_t filenamesSize = 0;
    int whichFilenameAtTop = 0;
    int whichFilenameSelected = 0;

    Status result = FillFilenameList(dirName, flags, optionalFilterSuffix, 256, filenames, &filenamesSize);
    if(RO_FAILURE(result)) {
        // XXX show some kind of failure
        DisplayStringAndWaitForEnter("Filename chooser failure!");
        return RO_RESOURCE_NOT_FOUND;
    }

    int done = 0;
    int redraw = 1;
    Status status = RO_SUCCESS;

    KeyRepeatManager keyRepeat;

    while(!done) {

        if(redraw) {
            ShowListOfItems(title, filenames, filenamesSize, whichFilenameAtTop, whichFilenameSelected);
            redraw = 0;
        }

        Event ev;
        int haveEvent = RoEventPoll(&ev);

        haveEvent = KeyRepeatUpdate(&keyRepeat, haveEvent, &ev);

        if(haveEvent) {
            switch(ev.eventType) {
                case KEYBOARD_RAW: {
                    const struct KeyboardRawEvent raw = ev.u.keyboardRaw;

                    if(raw.isPress) {

                        if(raw.key == KEYCAP_ESCAPE) {

                            *fileChosen = strdup("");
                            status = RO_USER_DECLINED;
                            done = 1;

                        } else if(raw.key == KEYCAP_ENTER) {

                            *fileChosen = strdup(filenames[whichFilenameSelected]);
                            status = RO_SUCCESS;
                            done = 1;

                        } else if(raw.key == KEYCAP_UP) {

                            whichFilenameSelected = (whichFilenameSelected - 1 < 0) ? 0 : (whichFilenameSelected - 1);
                            if(whichFilenameAtTop > whichFilenameSelected) {
                                whichFilenameAtTop = whichFilenameSelected;
                            }
                            redraw = 1;

                        } else if(raw.key == KEYCAP_DOWN) {

                            int w, h;
                            TextModeGetSize(&w, &h);
                            int availableLines = h - 4;
                            whichFilenameSelected = whichFilenameSelected + 1;
                            if((whichFilenameSelected + 1) > filenamesSize - 1) {
                                whichFilenameSelected = filenamesSize - 1;
                            }
                            if(whichFilenameSelected > whichFilenameAtTop + (availableLines - 1)) {
                                whichFilenameAtTop = whichFilenameSelected - (availableLines - 1);
                            }
                            redraw = 1;
                        }
                    }
                    break;
                }
                default:
                    // pass;
                    break;
            }
        }
        main_iterate(); // XXX
    }

    for(size_t i = 0; i < filenamesSize; i++) {
        free(filenames[i]);
    }

    return status;
}

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

    SDRAMInit();

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
    // RoDebugOverlayPrintf("Hello World\n");

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

    if(0) TextModeTest();

    if(0) HGRModeTest();

    if(0) {
        const char *args[] = {
            "play",
            // "inside-out.u8",
            "deeper_understanding.u8",
        };
        playAudio(sizeof(args) / sizeof(args[0]), args);
    }

    // DisplayStringAndWaitForEnter("Press ENTER.");

    if(1) {

        printf("Starting Apple ][ emulation menu\n");

        Status status;
        char *fileChosenInDir;
        static char fileChosen[512];

        status = PromptUserToChooseFile("Choose an Apple ][ boot disk", "/floppies", CHOOSE_FILE_IGNORE_DOTFILES, NULL /* ".dsk" */, &fileChosenInDir);
        sprintf(fileChosen, "/floppies/%s", fileChosenInDir);
        if(status == RO_SUCCESS) {
            const char *args[] = {
                "apple2e",
                // "-fast",
                "-diskII", "diskII.c600.c6ff.bin", fileChosen, "none",
                "apple2e.rom",
            };
            NTSCSwitchModeFuncs(WozModeFillRowBuffer, WozModeNeedsColorburst);
            const char* programString = R"(
10  HGR : POKE  - 16302,0
11 MX = 280
12 MY = 192
20  FOR X = 0 TO MX - 1
30 CX = X / MX * 3 - 2
40  FOR Y = 0 TO MY / 2 - 1
50 CY = Y / MY * 3 - 1.5
60 C = 0
70 XX = 0:YY = 0
80  IF XX * XX + YY * YY > 4 OR C > 15 THEN 200
90 OX = XX
100 XX = XX * XX - YY * YY + CX
110 YY = 2 * OX * YY + CY
120 C = C + 1
130  GOTO 80
200  IF C > 7 THEN C = C - 8: GOTO 200
205  HCOLOR= C
210  HPLOT X,Y
215  HPLOT X,MY - 1 - Y
220  NEXT Y
230  NEXT X
REM 5 FOR X = 1 TO 100 : PRINT X : NEXT X
REM 5 GOTO 5
REM 6 END
RUN
)";
#if 0
            programString = R"(
CALL -151
70: 2C 50 C0 2C 52 C0 20 70 FC A9 FF 91 2A 88 10 FB A9 16 85 25 20 22 FC A0 27 68 F0 05 0A F0 04 90 02 49 1D 48 30 09 B1 28 29 07 AA B5 A8 91 28 88 10 E7 C6 25 10 DE 30 CE 00 BB 00 AA 00 99 00 DD
70G
)";
#endif
            if(0) {
                for(size_t s = 0; s < strlen(programString); s++) {
                    enqueue_ascii(programString[s]);
                }
            }

            apple2_main(sizeof(args) / sizeof(args[0]), args); /* doesn't return */

        } else {

            // declined or error
        }
    }

    int user1 = 0;
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
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
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
