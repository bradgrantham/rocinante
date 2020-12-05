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

void __io_putchar( char c )
{
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&c, 1);
    HAL_Delay(1);
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

int fromUSB = -1;

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    fromUSB = 'F';
    if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD) {
        HID_KEYBD_Info_TypeDef *kbd = USBH_HID_GetKeybdInfo(phost);
        char key = USBH_HID_GetASCIICode(kbd);
        if(key != 0) {
            fromUSB = key;
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

//============================================================================
// NTSC output

//----------------------------------------------------------------------------
// DAC

#define DAC_VALUE_LIMIT 0xFF

#define MAX_DAC_VOLTAGE 1.32f
#define MAX_DAC_VOLTAGE_F16 (132 * 65536 / 100)

inline unsigned char voltageToDACValue(float voltage)
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

inline unsigned char voltageToDACValueNoBounds(float voltage)
{
    return (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
}

inline int voltageToDACValueFixed16NoBounds(int voltage)
{
    return (uint32_t)(voltage * 65535 / MAX_DAC_VOLTAGE_F16) * 256;
}

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

inline unsigned char NTSCYIQToDAC(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float w_t = tcycles * M_PI * 2;
    float sine = sinf(w_t + 33.0f / 180.0f * M_PI);
    float cosine = cosf(w_t + 33.0f / 180.0f * M_PI);
    float signal = y + q * sine + i * cosine;
// end of transcription

    return voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

inline unsigned char NTSCYIQDegreesToDAC(float y, float i, float q, int degrees)
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

inline ntsc_wave_t NTSCYIQToWave(float y, float i, float q)
{
    unsigned char b0 = NTSCYIQToDAC(y, i, q,  .0f);
    unsigned char b1 = NTSCYIQToDAC(y, i, q, .25f);
    unsigned char b2 = NTSCYIQToDAC(y, i, q, .50f);
    unsigned char b3 = NTSCYIQToDAC(y, i, q, .75f);

    return (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

// This is transcribed from the NTSC spec, double-checked.
inline void RGBToYIQ(float r, float g, float b, float *y, float *i, float *q)
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
inline void YIQToRGB(float y, float i, float q, float *r, float *g, float *b)
{
    *r = 1.0f * y + .946882f * i + 0.623557f * q;
    *g = 1.000000f * y + -0.274788f * i + -0.635691f * q;
    *b = 1.000000f * y + -1.108545f * i + 1.709007f * q;
}

inline ntsc_wave_t NTSCRGBToWave(float r, float g, float b)
{
    float y, i, q;
    RGBToYIQ(r, g, b, &y, &i, &q);
    return NTSCYIQToWave(y, i, q);
}

#define SECTION_CCMRAM

// These are in CCM to reduce contention with SRAM1 during DMA 
unsigned char SECTION_CCMRAM NTSCEqSyncPulseLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCVSyncLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCBlankLine[ROW_SAMPLES];

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
    NTSCFillBlankLine(NTSCBlankLine, 1);
}

void DefaultFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    if((rowNumber > 60) && (rowNumber < 60 + 192 * 2)) {
        memset(rowBuffer + 72, (NTSCBlack + NTSCWhite) / 2, 560);
    }
}

typedef void (*NTSCModeFillRowBufferFunc)(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer);
NTSCModeFillRowBufferFunc NTSCModeFillRowBuffer = DefaultFillRowBuffer;

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

        // XXX should just change DMA source address, then this needs to be in SRAM2
        memcpy(rowBuffer, NTSCBlankLine, sizeof(NTSCBlankLine));

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

        // XXX should just change DMA source address, then this needs to be in SRAM2
        memcpy(rowBuffer, NTSCBlankLine, sizeof(NTSCBlankLine));

    } else {

        // Don't need to do these because did both buffers at some point during vertical blank?
        // memcpy(rowBuffer, NTSCBlankLine, NTSCHSyncClocks + NTSCBackPorchClocks);
        // memcpy(rowBuffer + ROW_SAMPLES - NTSCFrontPorchClocks, NTSCBlankLine + ROW_SAMPLES - NTSCFrontPorchClocks, NTSCFrontPorchClocks);
        memcpy(rowBuffer, NTSCBlankLine, ROW_SAMPLES);

        int rowWithinFrame = (lineNumber % 263) * 2 + lineNumber / 263 - 22;
        NTSCModeFillRowBuffer(frameNumber, rowWithinFrame, 704, rowBuffer + 164);

        if(lineNumber == 262) {
            //line 262 - overwrite last 405 samples with first 405 samples of EQ pulse
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 282) {
            //special line 282 - write SyncPorch from BackPorch to middle of line after mode's fillRow()
            memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCSyncPorch, ROW_SAMPLES / 2 - (NTSCHSyncClocks + NTSCBackPorchClocks));
        }
    }
}

uint8_t /*  __attribute__((section (".ram_d1"))) */ rowDoubleBuffer[ROW_SAMPLES * 2];
int rowNumber = 0;
int frameNumber = 0;

int why;

void DMA2_Stream1_IRQHandler(void)
{
    rowNumber = (rowNumber + 1) % 525;
    if(rowNumber == 0) {
        frameNumber ++;
    }

    uint8_t *rowDest;
    if(DMA2->LISR & DMA_FLAG_HTIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
        rowDest = rowDoubleBuffer + 0;
    } else if(DMA2->LISR & DMA_FLAG_TCIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
        rowDest = rowDoubleBuffer + ROW_SAMPLES;
    } else {
        panic();
    }

    NTSCFillRowBuffer(frameNumber, rowNumber, rowDest);

    // A little pulse so we know where we are on the line when we finished
    if(1 /* markHandlerInSamples */) {
        for(int i = 0; i < 14; i++) { GPIOI->ODR = (GPIOI->ODR & 0xFFFFFF00) | 0xFFFFFFE8; }
    }

    if(DMA2->LISR) {
        // oldLISR = DMA2->LISR;
        DMA2->LIFCR = 0xFFFF;
    }
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    sprintf(sprintfBuffer, "error code %08lX, state %08X, line %d\n", hdma_tim1_ch2.ErrorCode, hdma_tim1_ch2.State, why);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)sprintfBuffer, strlen(sprintfBuffer));
    // panic();
}

void startNTSCScanout()
{
    NTSCCalculateParameters();
    NTSCGenerateLineBuffers();

    HAL_StatusTypeDef status;
    // memcpy(testNTSCImage_RAM, testNTSCImage_bytes, sizeof(testNTSCImage_bytes));
    memcpy(rowDoubleBuffer + 0, testNTSCImage_bytes + 0, ROW_SAMPLES);
    memcpy(rowDoubleBuffer + ROW_SAMPLES, testNTSCImage_bytes + ROW_SAMPLES, ROW_SAMPLES);
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

//----------------------------------------------------------------------------
// Woz (Apple ][ graphics) mode

#define WOZ_MODE_LEFT 72 
#define WOZ_MODE_WIDTH 560 
#define WOZ_MODE_TOP 60 
#define WOZ_MODE_HEIGHT 192 
#define WOZ_MODE_MIXED_TEXT_ROWS 4
#define WOZ_MODE_FONT_HEIGHT 8
#define WOZ_MODE_FONT_WIDTH 7

enum DisplayMode {TEXT, LORES, HIRES};
enum DisplayMode WozModeDisplayMode = TEXT;
int WozModeMixed = 0;
int WozModePage = 0;
uint8_t WozModeHGRBuffers[2][8192];
uint8_t WozModeTextBuffers[2][1024];

const int WozModeTextRowOffsets[24] =
{
    0x000,
    0x080,
    0x100,
    0x180,
    0x200,
    0x280,
    0x300,
    0x380,
    0x028,
    0x0A8,
    0x128,
    0x1A8,
    0x228,
    0x2A8,
    0x328,
    0x3A8,
    0x050,
    0x0D0,
    0x150,
    0x1D0,
    0x250,
    0x2D0,
    0x350,
    0x3D0,
};

const int WozModeHGRRowOffsets[192] =
{
     0x0000,  0x0400,  0x0800,  0x0C00,  0x1000,  0x1400,  0x1800,  0x1C00,
     0x0080,  0x0480,  0x0880,  0x0C80,  0x1080,  0x1480,  0x1880,  0x1C80,
     0x0100,  0x0500,  0x0900,  0x0D00,  0x1100,  0x1500,  0x1900,  0x1D00,
     0x0180,  0x0580,  0x0980,  0x0D80,  0x1180,  0x1580,  0x1980,  0x1D80,
     0x0200,  0x0600,  0x0A00,  0x0E00,  0x1200,  0x1600,  0x1A00,  0x1E00,
     0x0280,  0x0680,  0x0A80,  0x0E80,  0x1280,  0x1680,  0x1A80,  0x1E80,
     0x0300,  0x0700,  0x0B00,  0x0F00,  0x1300,  0x1700,  0x1B00,  0x1F00,
     0x0380,  0x0780,  0x0B80,  0x0F80,  0x1380,  0x1780,  0x1B80,  0x1F80,
     0x0028,  0x0428,  0x0828,  0x0C28,  0x1028,  0x1428,  0x1828,  0x1C28,
     0x00A8,  0x04A8,  0x08A8,  0x0CA8,  0x10A8,  0x14A8,  0x18A8,  0x1CA8,
     0x0128,  0x0528,  0x0928,  0x0D28,  0x1128,  0x1528,  0x1928,  0x1D28,
     0x01A8,  0x05A8,  0x09A8,  0x0DA8,  0x11A8,  0x15A8,  0x19A8,  0x1DA8,
     0x0228,  0x0628,  0x0A28,  0x0E28,  0x1228,  0x1628,  0x1A28,  0x1E28,
     0x02A8,  0x06A8,  0x0AA8,  0x0EA8,  0x12A8,  0x16A8,  0x1AA8,  0x1EA8,
     0x0328,  0x0728,  0x0B28,  0x0F28,  0x1328,  0x1728,  0x1B28,  0x1F28,
     0x03A8,  0x07A8,  0x0BA8,  0x0FA8,  0x13A8,  0x17A8,  0x1BA8,  0x1FA8,
     0x0050,  0x0450,  0x0850,  0x0C50,  0x1050,  0x1450,  0x1850,  0x1C50,
     0x00D0,  0x04D0,  0x08D0,  0x0CD0,  0x10D0,  0x14D0,  0x18D0,  0x1CD0,
     0x0150,  0x0550,  0x0950,  0x0D50,  0x1150,  0x1550,  0x1950,  0x1D50,
     0x01D0,  0x05D0,  0x09D0,  0x0DD0,  0x11D0,  0x15D0,  0x19D0,  0x1DD0,
     0x0250,  0x0650,  0x0A50,  0x0E50,  0x1250,  0x1650,  0x1A50,  0x1E50,
     0x02D0,  0x06D0,  0x0AD0,  0x0ED0,  0x12D0,  0x16D0,  0x1AD0,  0x1ED0,
     0x0350,  0x0750,  0x0B50,  0x0F50,  0x1350,  0x1750,  0x1B50,  0x1F50,
     0x03D0,  0x07D0,  0x0BD0,  0x0FD0,  0x13D0,  0x17D0,  0x1BD0,  0x1FD0,
};

void WozModeFillRowBufferHGR(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    if((rowIndex >= 0) && (rowIndex < 192)) {
        const uint8_t *rowSrc = WozModeHGRBuffers[WozModePage] + WozModeHGRRowOffsets[rowIndex]; // row - ...?
        memset(rowBuffer + WOZ_MODE_LEFT, NTSCBlack, WOZ_MODE_WIDTH);
        for(int byteIndex = 0; byteIndex < 40; byteIndex++) {
            uint8_t byte = rowSrc[byteIndex];
            int colorShift = (byte & 0x80) ? 1 : 0;
            uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT + byteIndex * 14 + colorShift;
            for(int bitIndex = 0; bitIndex < 7; bitIndex++) {
                if(byte & (1 << bitIndex)) {
                    rowDst[bitIndex * 2 + 0] = NTSCWhite; // or DAC max?
                    rowDst[bitIndex * 2 + 1] = NTSCWhite; // or DAC max?
                }
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

void WozModeFillRowBuffer40Text(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    if((rowIndex >= 0) && (rowIndex < 192)) {
        memset(rowBuffer + WOZ_MODE_LEFT, NTSCBlack, WOZ_MODE_WIDTH);
        int rowInText = rowIndex / 8;
        int rowInGlyph = rowIndex % 8;
        const uint8_t *rowSrc = WozModeTextBuffers[WozModePage] + WozModeTextRowOffsets[rowInText]; // row - ...?
        uint8_t *rowDst = rowBuffer + WOZ_MODE_LEFT;
        for(int textColumn = 0; textColumn < 40; textColumn++) {
            uint8_t byte = rowSrc[textColumn];
            int fontIndex, inverse;
            WozMemoryByteToFontIndex(byte, &fontIndex, &inverse);
            int fontRowByte = WozModeFontBytes[fontIndex * 8 + rowInGlyph];
            for(int column = 0; column < 7; column ++) {
                *rowDst++ = (fontRowByte & 0x01) ? NTSCWhite : NTSCBlack;
                *rowDst++ = (fontRowByte & 0x01) ? NTSCWhite : NTSCBlack;
                fontRowByte = fontRowByte >> 1;
            }
        }
    }
}

void WozModeFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    int rowIndex = (rowNumber - WOZ_MODE_TOP) / 2;
    enum DisplayMode mode = WozModeDisplayMode;
    if(WozModeMixed && (rowIndex >= WOZ_MODE_HEIGHT - WOZ_MODE_MIXED_TEXT_ROWS * WOZ_MODE_FONT_HEIGHT)) {
        mode = TEXT;
    }
    switch(mode) {
        case TEXT: 
            WozModeFillRowBuffer40Text(frameIndex, rowNumber, maxSamples, rowBuffer);
            break;
        case HIRES: 
            WozModeFillRowBufferHGR(frameIndex, rowNumber, maxSamples, rowBuffer);
            break;
        case LORES: 
            // WozModeFillRowBufferGR(frameIndex, rowNumber, maxSamples, rowBuffer);
            break;
    }
}

//----------------------------------------------------------------------------
// FillRowBuffer tests

void ImageFillRowBuffer(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    //memcpy(rowBuffer, testNTSCImage_bytes + 164 + rowNumber * ROW_SAMPLES, maxSamples);

    for(int col = 0; col < maxSamples; col++) {
        int checker = (col / 35 + rowNumber / 20) % 2;
        rowBuffer[col] = checker ? NTSCWhite : NTSCBlack;
    }
}

void ImageFillRowBuffer2(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer)
{
    //memcpy(rowBuffer, testNTSCImage_bytes + 164 + rowNumber * ROW_SAMPLES, maxSamples);

    // times 2 because we are given interlaced height rows, 0 to 238
    if((rowNumber > WOZ_MODE_TOP) && (rowNumber < WOZ_MODE_TOP + WOZ_MODE_HEIGHT * 2)) {
        for(int col = 0; col < WOZ_MODE_WIDTH; col++) {
            int checker = (col / 35 + rowNumber / 20) % 2;
            rowBuffer[WOZ_MODE_LEFT + col] = checker ? NTSCWhite : NTSCBlack;
        }
    }
}

int RowModeIndex = 1;
NTSCModeFillRowBufferFunc RowModeFunctions[] = {
    DefaultFillRowBuffer,
    WozModeFillRowBuffer,
};
size_t RowModeFunctionCount = sizeof(RowModeFunctions) / sizeof(RowModeFunctions[0]);

int apple2_main(int argc, char **argv);

int main_iterate(void)
{
    MX_USB_HOST_Process();

    if(fromUSB != -1) {
        printf("key: %c\n", fromUSB);
        fromUSB = -1;
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
  while (HAL_GetTick() < 5000) {
    main_iterate();
  }

    const char *args[] = {
        "apple2e",
        "apple2e.rom",
    };
    printf("here we go\n");
    NTSCModeFillRowBuffer = WozModeFillRowBuffer;
    apple2_main(2, args); /* doesn't return */

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
        NTSCModeFillRowBuffer = RowModeFunctions[(++RowModeIndex) % RowModeFunctionCount];
        HAL_Delay(500);
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
