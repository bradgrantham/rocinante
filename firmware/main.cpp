#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <cmath>

#undef USE_PS2KBD

#include <stm32f7xx_hal.h>

#include "ff.h"

#include "defs.h"
#include "delay.h"
#include "leds.h"
#include "byte_queue.h"
#include "gpio_helpers.h"
#include "utility.h"
#include "crc7.h"
#include "logprintf.h"

#include "monitor_queue.h"
#include "console_queue.h"
#include "uart.h"
#include "sd_spi.h"
#include "keyboard.h"
#include "reset_button.h"

// System APIs
#include "videomode.h"
#include "graphics.h"
#include "textport.h"
#include "rocinante.h"
#include "commandline.h"
#include "segment_utility.h"

// System driver internal definitions
#include "videomodeinternal.h"

TIM_HandleTypeDef htim1;

static int gDumpKeyboardData = 0;

#define IOBOARD_FIRMWARE_VERSION_STRING XSTR(IOBOARD_FIRMWARE_VERSION)

void panic_worse()
{
    LED_set_panic(1);
    for(;;);
}

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void panic(void)
{
    static int entered = 0;

    LED_set_panic(1);

    int pin = 0;
    for(;;) {
        if(!entered) {
            // SERIAL_flush() can itself panic(), so stop reentry here
            entered = 1;
            SERIAL_flush();
            entered = 0;
        }

        LED_set_panic(pin);
        pin = pin ? 0 : 1;
        delay_ms(100);
    }
}

#ifdef __cplusplus
};
#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void FPU_IRQHandler(void)
{
    SERIAL_enqueue_one_char('f');
    SERIAL_enqueue_one_char('p');
    SERIAL_enqueue_one_char('u');
    SERIAL_enqueue_one_char('\n');
    SERIAL_flush();
    panic();
}

#ifdef __cplusplus
};
#endif /* __cplusplus */

//----------------------------------------------------------------------------
// System Initialization Goop

typedef struct {
    float CPUMHz;      // not used; computed
    uint32_t HSOSC;       // not used; fixed
    uint32_t PLL_M;
    uint32_t PLL_N;
    uint32_t PLL_P;
    uint32_t DMA_TIM_CLOCKS;
    uint32_t HCLK_DIV;
    uint32_t APB1_DIV;
    uint32_t APB2_DIV;
    float colorburstClock;      // not used; computed
    float error;      // not used; computed
} ClockConfiguration;

unsigned int whichConfig = 0;
static const ClockConfiguration clockConfigs[] =
{
    // Base mode we know works on 415 and 746
    // { 229.14, 8000000, 7, 401, 2, 8, RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV4, 3.580357, 0.000227 }, // can't get SD to lock in
    // { 200.50, 8000000, 4, 401, 4, 7, RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV4, 3.580357, 0.000227},

    // Know this one works at HSI 16MHz on 746, but jittery, no color:
    // { 200.50, 16000000, 8, 401, 4, 7, RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV4, 3.580357, 0.000227},
    // (16 / 8 * 401 / 4) / (7 * 2)

// PLL_OUTPUT MHz, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, HCLK_DIV, APB1_DIV, APB2_DIV, actual color clock MHz, error
    // {204.54, 20004300, 20, 409, 2, 2, RCC_SYSCLK_DIV1, RCC_HCLK_DIV4, RCC_HCLK_DIV2, 102.271984, 6.142805},

{ 215.93, 20004300, 17, 367, 2, 29, RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV4, 3.722910, -0.739987 },
};
static const int clockConfigCount = sizeof(clockConfigs) / sizeof(clockConfigs[0]);

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  const ClockConfiguration *clk = clockConfigs + whichConfig;

#if 1 // HSE - high speed external oscillator; on the NUCLEO-144 it's a 20MHz clock I installed

  unsigned int PLL_Q = (HSE_VALUE / clk->PLL_M * clk->PLL_N / clk->PLL_P / 48 + 999999) / 1000000;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = clk->PLL_M; // Divide PLL source by this
  RCC_OscInitStruct.PLL.PLLN = clk->PLL_N; // Then multiply by this 
  RCC_OscInitStruct.PLL.PLLP = clk->PLL_P; // Then divide by this
  RCC_OscInitStruct.PLL.PLLQ = PLL_Q; // Divide by this for SD, USB OTG FS, and some other peripherals
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    panic();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = clk->HCLK_DIV;
  RCC_ClkInitStruct.APB1CLKDivider = clk->APB1_DIV; // APB1 clock
  RCC_ClkInitStruct.APB2CLKDivider = clk->APB2_DIV; // APB2 clock
  // grantham - 5 cycles for 168MHz is stated in Table 10 in the STM32F4 reference manual
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    panic();
  }

#else

  unsigned int PLL_Q = (HSI_VALUE / clk->PLL_M * clk->PLL_N / clk->PLL_P / 48 + 999999) / 1000000;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = clk->PLL_M; // Divide PLL source by this
  RCC_OscInitStruct.PLL.PLLN = clk->PLL_N; // Then multiply by this 
  RCC_OscInitStruct.PLL.PLLP = clk->PLL_P; // Then divide by this
  RCC_OscInitStruct.PLL.PLLQ = PLL_Q; // Divide by this for SD, USB OTG FS, and some other peripherals
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    panic();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = clk->HCLK_DIV;
  RCC_ClkInitStruct.APB1CLKDivider = clk->APB1_DIV; // APB1 clock
  RCC_ClkInitStruct.APB2CLKDivider = clk->APB2_DIV; // APB2 clock

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    panic();
  }

#endif

  /* Enable other CLKs */
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  __HAL_RCC_TIM6_CLK_ENABLE();
  __HAL_RCC_TIM7_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DAC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

    // Since this relies solely on the clock 
    delay_init();
}

// Was in the 417's header but I could not find in the 746's headers
#ifndef RCC_PLLCFGR_RST_VALUE 
#define RCC_PLLCFGR_RST_VALUE 0x24003010
#endif

void DeInitRCCAndPLL()
{
    // Unset the RCC and reset it
    RCC->CR |= RCC_HSI_ON;
    RCC->CFGR = 0x00000000U;
    uint32_t vl_mask = 0xFFFFFFFFU;
    /* Reset HSEON, PLLSYSON bits */
    CLEAR_BIT(vl_mask, (RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_PLLON | RCC_CR_CSSON));
    RCC->CR &= vl_mask;
    RCC->CR = (RCC->CR & ~RCC_CR_HSITRIM_Msk) | (0x10 << RCC_CR_HSITRIM_Pos); /* Set HSITRIM bits to the reset value*/
    RCC->PLLCFGR = RCC_PLLCFGR_RST_VALUE; /* Reset PLLCFGR register */
    RCC->CR &= ~RCC_CR_HSEBYP; /* Reset HSEBYP bit */
  /* Disable all interrupts */
    RCC->CIR = 0x00000000U;
// *            - HSI ON and used as system clock source
// *            - HSE and PLL OFF
// *            - AHB, APB1 and APB2 prescaler set to 1.
// *            - CSS, MCO1 and MCO2 OFF
// *            - All interrupts disabled
}

//----------------------------------------------------------------------------
// stdio

#define OUTPUT_TO_SERIAL        0x01
#define OUTPUT_TO_TEXTPORT         0x02

int gOutputDevices = OUTPUT_TO_SERIAL;

int InputGetChar(void)
{
    SERIAL_poll_continue();
    unsigned char isEmpty = queue_isempty(&mon_queue);
    if(!isEmpty) {
        unsigned char c = queue_deq(&mon_queue);
        return c;
    }
    return -1;
}

int InputWaitChar(void)
{
    int c;
    do { 
        c = InputGetChar();
    } while(c < 0);
    return c;
}

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Assumed to be raw mode - returns next character, not buffering until newline
int __io_getchar(void)
{
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
}

void __io_putchar( char c )
{
    if(gOutputDevices & OUTPUT_TO_SERIAL) {
        SERIAL_enqueue_one_char(c);
        if(c == '\n') {
            SERIAL_flush();
        }
    }

    if(gOutputDevices & OUTPUT_TO_TEXTPORT) {
        TextportPutchar(c);
    }
}

#ifdef __cplusplus
};
#endif /* __cplusplus */

//----------------------------------------------------------------------------
// File operations

void check_exceptional_conditions()
{
    if(gConsoleOverflowed) {
        logprintf(DEBUG_WARNINGS, "WARNING: Console input queue overflow\n");
        gConsoleOverflowed = 0;
    }

#ifdef USE_PS2KBD
    if(gKeyboardOverflowed) {
        logprintf(DEBUG_WARNINGS, "WARNING: Keyboard data queue overflow\n");
        gKeyboardOverflowed = 0;
    }

    if(gKeyboardParityError) {
        logprintf(DEBUG_WARNINGS, "WARNING: Keyboard data parity error, received 0x%02X\n", gKeyboardParityError - 256);
        gKeyboardParityError = 0;
    }

    if(gKeyboardBATBadParity) {
        logprintf(DEBUG_EVENTS, "EVENT: Received initial BAT with parity error from PS/2 keyboard\n");
        gKeyboardBATBadParity = 0;
    }
#endif
}

void ProcessAnyInput()
{
    unsigned char isEmpty = queue_isempty(&mon_queue);

    if(!isEmpty) {
        unsigned char c = queue_deq(&mon_queue);
        ProcessKey(c);
    }
}

void uart_received(char c)
{
    queue_enq(&mon_queue, c);
}

void console_queue_init()
{
    queue_init(&con_queue, QUEUE_CAPACITY);
}

//----------------------------------------------------------------------------
// DAC

#define DAC_VALUE_LIMIT 0xFF

#define MAX_DAC_VOLTAGE 1.32f
#define MAX_DAC_VOLTAGE_F16 (132 * 65536 / 100)

unsigned char voltageToDACValue(float voltage)
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

unsigned char voltageToDACValueNoBounds(float voltage)
{
    return (uint32_t)(voltage / MAX_DAC_VOLTAGE * 255);
}

int voltageToDACValueFixed16NoBounds(int voltage)
{
    return (uint32_t)(voltage * 65535 / MAX_DAC_VOLTAGE_F16) * 256;
}

//----------------------------------------------------------------------------
// NTSC Video goop

// Number of samples we target, 4x colorburst yields 227.5 cycles, or 910 samples at 14.318180MHz
#define ROW_SAMPLES        910

#define NTSC_COLORBURST_FREQUENCY       3579545
#define NTSC_EQPULSE_LINES	3
#define NTSC_VSYNC_LINES	3
#define NTSC_VBLANK_LINES	11
#define NTSC_FRAME_LINES	525
#define NTSC_EQ_PULSE_INTERVAL	.04
#define NTSC_VSYNC_BLANK_INTERVAL	.43
#define NTSC_HOR_SYNC_DUR	.075
#define NTSC_FRAMES		(59.94 / 2)
#define NTSC_FRONTPORCH		.02
/* BACKPORCH including COLORBURST */
#define NTSC_BACKPORCH		.075
#define NTSC_COLORBURST_CYCLES  14 /* 12 */

#define NTSC_SYNC_TIP_VOLTAGE   0.0f
#define NTSC_SYNC_PORCH_VOLTAGE   .285f
#define NTSC_SYNC_BLACK_VOLTAGE   .339f
#define NTSC_SYNC_WHITE_VOLTAGE   1.0f  /* VCR had .912v */

#define NTSC_SYNC_BLACK_VOLTAGE_F16   22217
#define NTSC_SYNC_WHITE_VOLTAGE_F16   65536

int SECTION_CCMRAM markHandlerInSamples = 0;

volatile int16_t SECTION_CCMRAM rowDMARemained[NTSC_FRAME_LINES];
volatile uint32_t SECTION_CCMRAM DMAFIFOUnderruns = 0;
volatile uint32_t SECTION_CCMRAM DMATransferErrors = 0;
typedef enum { NTSC_SOLID_FILL, NTSC_COLOR_TEST, NTSC_SCAN_TEST, NTSC_DISPLAY_BLACK, NTSC_USE_VIDEO_MODE } VideoMode;
volatile VideoMode SECTION_CCMRAM NTSCMode = NTSC_COLOR_TEST;
volatile int SECTION_CCMRAM videoScanTestLeft = 200;
volatile int SECTION_CCMRAM videoScanTestRight = 700;
volatile int SECTION_CCMRAM videoScanTestTop = 50;
volatile int SECTION_CCMRAM videoScanTestBottom = 200;

// These are in CCM to reduce contention with SRAM1 during DMA 
unsigned char SECTION_CCMRAM NTSCEqSyncPulseLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCVSyncLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCBlankLine[ROW_SAMPLES];

unsigned char SECTION_CCMRAM NTSCSyncTip;
unsigned char SECTION_CCMRAM NTSCSyncPorch;
unsigned char SECTION_CCMRAM NTSCBlack;
unsigned char SECTION_CCMRAM NTSCWhite;
unsigned char SECTION_CCMRAM NTSCMaxAllowed;

int NTSCEqPulseClocks;
int NTSCVSyncClocks;
int NTSCHSyncClocks;
int NTSCLineClocks;
int NTSCFrameClocks;
int NTSCFrontPorchClocks;
int NTSCBackPorchClocks;

typedef uint32_t ntsc_wave_t;

// Essentially the remainder of CCMRAM, will need to be shrunk if more goes into CCM
#define VRAM_SIZE  51725

// (Following was copied from paper chicken scratches)
// On Orion TV, one 14MHz clock is .0225 inches, one 240p row is .052 inches.
// So to find a close value for width, width = sqrt(4 / 3 * .052 / .0225 * VRAM_SIZE)
// Then, height should be no more than width * 3 * .0225 / (4 / .052)
// So for 53248, a reasonable 4:3 framebuffer is 400x128
// 4:3 aspect would be 1.333
// 400 wide would be 9 inches, and 128 high would be 6.656 inches, and that's 1.352, so it's not too bad

unsigned char SECTION_CCMRAM VRAM[VRAM_SIZE];

// XXX these are in SRAM2 to reduce contention with SRAM1 during DMA
unsigned char __attribute__((section (".sram2"))) row0[ROW_SAMPLES];
unsigned char __attribute__((section (".sram2"))) row1[ROW_SAMPLES];
unsigned char __attribute__((section (".sram2"))) audio0[512];
unsigned char __attribute__((section (".sram2"))) audio1[512];

unsigned char NTSCYIQToDAC(float y, float i, float q, float tcycles)
{
// This is transcribed from the NTSC spec, double-checked.
    float wt = tcycles * M_PI * 2;
    float sine = sinf(wt + 33.0f / 180.0f * M_PI);
    float cosine = cosf(wt + 33.0f / 180.0f * M_PI);
    float signal = y + q * sine + i * cosine;
// end of transcription

    return voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE + signal * (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE));
}

unsigned char NTSCYIQDegreesToDAC(float y, float i, float q, int degrees)
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

ntsc_wave_t NTSCYIQToWave(float y, float i, float q)
{
    unsigned char b0 = NTSCYIQToDAC(y, i, q,  .0f);
    unsigned char b1 = NTSCYIQToDAC(y, i, q, .25f);
    unsigned char b2 = NTSCYIQToDAC(y, i, q, .50f);
    unsigned char b3 = NTSCYIQToDAC(y, i, q, .75f);

    return (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

ntsc_wave_t NTSCRGBToWave(float r, float g, float b)
{
    float y, i, q;
    RGBToYIQ(r, g, b, &y, &i, &q);
    return NTSCYIQToWave(y, i, q);
}

unsigned char NTSCColorburst0;
unsigned char NTSCColorburst90;
unsigned char NTSCColorburst180;
unsigned char NTSCColorburst270;

void NTSCCalculateParameters(float clock)
{
    // Calculate values for a scanline
    NTSCFrameClocks = floorf(clock * 1000000.0 / NTSC_FRAMES + 0.5);
    // NTSCLineClocks = floorf((double)NTSCFrameClocks / NTSC_FRAME_LINES + 0.5);
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
    NTSCMaxAllowed = 255;

    // Calculate the four values for the colorburst that we'll repeat to make a wave
    // The waveform is defined as sine in the FCC broadcast doc, but for
    // composite the voltages are reversed, so the waveform becomes -sine.
    NTSCColorburst0 = NTSCSyncPorch;
    NTSCColorburst90 = NTSCSyncPorch - .6 * NTSCSyncPorch;
    NTSCColorburst180 = NTSCSyncPorch;
    NTSCColorburst270 = NTSCSyncPorch + .6 * NTSCSyncPorch;

    memset(row0, NTSCSyncPorch, sizeof(row0));
    memset(row1, NTSCSyncPorch, sizeof(row1));
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
void NTSCAddColorburst(unsigned char *rowBuffer)
{
    static const int startOfColorburstClocks = 80 - 3 * 4; // XXX magic number for current clock

    for(int col = startOfColorburstClocks; col < startOfColorburstClocks + NTSC_COLORBURST_CYCLES * 4; col++) {
        switch((col - startOfColorburstClocks) % 4) {
            case 0: rowBuffer[col] = NTSCColorburst0; break;
            case 1: rowBuffer[col] = NTSCColorburst90; break;
            case 2: rowBuffer[col] = NTSCColorburst180; break;
            case 3: rowBuffer[col] = NTSCColorburst270; break;
        }
    }
}

void NTSCFillBlankLine(unsigned char *rowBuffer, int withColorburst)
{
    for (int col = 0; col < NTSCLineClocks; col++) {
        if (col < NTSCHSyncClocks) {
            rowBuffer[col] = NTSCSyncTip;
        } else {
            rowBuffer[col] = NTSCSyncPorch;
        }
    }
    if(withColorburst) {
        NTSCAddColorburst(rowBuffer);
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

#define NTSC_NUM_PALETTES 2
#define NTSC_WAVE_SIZE 4
#define NTSC_PALETTE_INDEX_SIZE 1

// #define VRAM_PIXMAP_AVAIL (VRAM_SIZE - NTSC_NUM_PALETTES * sizeof(uint32_t) * 256 - sizeof(unsigned char) * NTSC_FRAME_LINES)
#define VRAM_PIXMAP_AVAIL (VRAM_SIZE - NTSC_NUM_PALETTES * NTSC_WAVE_SIZE * 256 - NTSC_PALETTE_INDEX_SIZE * NTSC_FRAME_LINES)

ntsc_wave_t *NTSCGetPalettePointer(int palette)
{
    return (ntsc_wave_t*)(VRAM + VRAM_SIZE - palette * NTSC_WAVE_SIZE * 256);
}

unsigned char *NTSCGetPaletteForRowPointer()
{
    return (unsigned char*)(VRAM + VRAM_SIZE - NTSC_NUM_PALETTES * NTSC_WAVE_SIZE * 256 - NTSC_PALETTE_INDEX_SIZE * NTSC_FRAME_LINES);
}

ntsc_wave_t *NTSCGetPaletteForRow(int row)
{
    int palette = NTSCGetPaletteForRowPointer()[row];
    return NTSCGetPalettePointer(palette);
}

ntsc_wave_t NTSCGetPaletteEntry(int palette, int which)
{
    return NTSCGetPalettePointer(palette)[which];
}

ntsc_wave_t NTSCGetPaletteEntryForRow(int row, int which)
{
    return NTSCGetPaletteForRow(row)[which];
}

int NTSCSetPaletteEntry(int palette, int which, float r, float g, float b)
{
    if(palette < 0) {
        return 2; // palette number invalid
    }
    if(palette > 1) {
        return 1; // palette number out of range
    }
    ntsc_wave_t *paletteToWave = NTSCGetPalettePointer(palette);
    paletteToWave[which] = NTSCRGBToWave(r, g, b);
    return 0;
}

int NTSCSetPaletteForRow(int row, int palette)
{
    unsigned char *palettesByRow = NTSCGetPaletteForRowPointer();
    palettesByRow[row] = palette;
    return 0;
}

VideoFillRowFunc SECTION_CCMRAM VideoCurrentFillRow;
ntsc_wave_t SECTION_CCMRAM solidFillWave;

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
        memcpy(rowBuffer, NTSCBlankLine, NTSCHSyncClocks + NTSCBackPorchClocks);
        memcpy(rowBuffer + ROW_SAMPLES - NTSCFrontPorchClocks, NTSCBlankLine + ROW_SAMPLES - NTSCFrontPorchClocks, NTSCFrontPorchClocks);

        // 244 lines
        // 189 columns @ 4 per pixel
        int y = lineNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES);

        switch(NTSCMode) {
            case NTSC_COLOR_TEST: {
                if((y > 20 && y < 230) || (y > 283 && y < 493)) {
                    
                    // Clear pixels outside of text area 
                    memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, 80);
                    int clocksToRightEdge = 169 * 4;
                    memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

                    unsigned char *rowOut = rowBuffer + (NTSCHSyncClocks + NTSCBackPorchClocks + 3) / 4 * 4 + 20 * 4;
                    ntsc_wave_t* rowWords = (ntsc_wave_t*)rowOut;
                    unsigned char v = NTSCBlack + (NTSCWhite - NTSCBlack) / 2;
                    unsigned char a = NTSCWhite;
                    unsigned char c = NTSCBlack;
                    ntsc_wave_t colorWord = (v << 0) | (v << 8) | (v << 16) | (v << 24); // little endian
                    for(int col = 20; col < 50; col++) {
                        *rowWords++ = colorWord;
                    }
                    colorWord = (v << 0) | (a << 8) | (v << 16) | (c << 24); // little endian
                    for(int col = 50; col < 169; col++) {
                        *rowWords++ = colorWord;
                    }
                }
                break;
            }
            case NTSC_SOLID_FILL: {
                if((y % 263) > 20 && (y % 263) < 230) {
                    unsigned char *rowOut = rowBuffer + (NTSCHSyncClocks + NTSCBackPorchClocks + 3) / 4 * 4 + 20 * 4;
                    ntsc_wave_t* rowWords = (ntsc_wave_t*)rowOut;
                    for(int col = 20; col < 169; col++) {
                        *rowWords++ = solidFillWave;
                    }
                }
                break;
            }
            case NTSC_SCAN_TEST: {
                if((lineNumber % 263) >= videoScanTestTop && (lineNumber % 263) <= videoScanTestBottom) {
                    for(int row = videoScanTestLeft; row < videoScanTestRight; row++) {
                        rowBuffer[row] = 200;
                    }
                }
                break;
            }
            case NTSC_DISPLAY_BLACK: {
                memcpy(rowBuffer, NTSCBlankLine, ROW_SAMPLES);
                break;
            }
            case NTSC_USE_VIDEO_MODE: {
                VideoCurrentFillRow(frameNumber, lineNumber, rowBuffer);
                break;
            }
        }
        if(lineNumber == 262) {
            //line 262 - overwrite last 405 samples with first 405 samples of EQ pulse
            memcpy(rowBuffer + ROW_SAMPLES / 2, NTSCEqSyncPulseLine, ROW_SAMPLES / 2);
        } else if(lineNumber == 282) {
            //special line 282 - write SyncPorch from BackPorch to middle of line after mode's fillRow()
            memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCSyncPorch, ROW_SAMPLES / 2 - (NTSCHSyncClocks + NTSCBackPorchClocks));
        }
    }
    // if(lineNumber == 0) { rowBuffer[0] = 255;}
}

// debug overlay scanout

int SECTION_CCMRAM debugOverlayEnabled = 0;

#define debugDisplayWidth 19
#define debugDisplayHeight 13
#define debugDisplayLeftTick (NTSCHSyncClocks + NTSCBackPorchClocks + 48)
#define debugDisplayTopTick (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES + 20)
/* debugFontWidthScale != 4 looks terrible in a color field because of adjacent color columns; probably need to ensure 0s around any 1 text column */
#define debugFontWidthScale 4
#define debugCharGapPixels 1
#define debugFontHeightScale 1

char SECTION_CCMRAM debugDisplay[debugDisplayHeight][debugDisplayWidth];

#include "8x16.h"
// static int font8x16Width = 8, font8x16Height = 16;
// static unsigned char font8x16Bits[] = /* was a bracket here */

void fillRowDebugOverlay(int frameNumber, int lineNumber, unsigned char* nextRowBuffer)
{
    ntsc_wave_t NTSCWhiteLong =
        (NTSCWhite <<  0) |
        (NTSCWhite <<  8) |
        (NTSCWhite << 16) |
        (NTSCWhite << 24);
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
                        }
#endif
                    }
                }
#endif
            }
        }
    }
}

volatile int SECTION_CCMRAM lineNumber = 0;
volatile int SECTION_CCMRAM frameNumber = 0;

// unused at the present, was no faster than memcpy
void MemoryCopyDMA(unsigned char* dst, unsigned char* src, size_t size)
{
    // XXX wait on previous DMA
    // Configure DMA to copy tmp row
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;       /* disable DMA2_1 */
    DMA2->LIFCR = 0xF00;                        /* clear flags */
    DMA2_Stream1->NDTR = size / 4;
    DMA2_Stream1->PAR = (uint32_t)src;        // Source buffer address 0 in Memory-to-Memory mode
    DMA2_Stream1->M0AR = (uint32_t)dst;        // Dest buffer address in Memory-to-Memory mode
    DMA2_Stream1->FCR = DMA_FIFOMODE_ENABLE |   // Enable FIFO to improve stutter
        DMA_FIFO_THRESHOLD_FULL;        
    DMA2_Stream1->CR =
        DMA_CHANNEL_1 |                         
        DMA_MEMORY_TO_MEMORY |                  // Memory to Memory
        DMA_PDATAALIGN_WORD | // DMA_PBURST_INC4 |
        DMA_MDATAALIGN_WORD | // DMA_MBURST_INC4 |
        DMA_PRIORITY_LOW |
        DMA_PINC_ENABLE |                       // Increment memory address
        DMA_MINC_ENABLE |                       // Increment memory address
        0;
    DMA2_Stream1->CR |= DMA_SxCR_EN;    /* enable DMA */
} 

// XXX audio experiment
// In this mode we would have a sampling rate of 15.6998 KHz
unsigned char SECTION_CCMRAM audioBuffer[256];
volatile size_t audioBufferPosition = 0;

int doPlay(int argc, char **argv)
{
    size_t bufferHalfSize = sizeof(audioBuffer) / 2;

    char *filename = argv[1];
    FIL file;
    FRESULT result = f_open (&file, filename, FA_READ | FA_OPEN_EXISTING);
    if(result) {
        logprintf(DEBUG_ERRORS, "ERROR: couldn't open \"%s\" for reading, FatFS result %d\n", filename, result);
        return 1;
    }

    int where = 0;

    UINT wasread;
    int quit = 0;
    do {
        // Wait for audio to get past the buffer we read
        while((audioBufferPosition - where + sizeof(audioBuffer)) % sizeof(audioBuffer) < bufferHalfSize);
        result = f_read(&file, audioBuffer + where, bufferHalfSize, &wasread);
        if(result) {
            logprintf(DEBUG_ERRORS, "ERROR: couldn't read block of audio from \"%s\", result %d\n", filename, result);
            memset(audioBuffer, 128, sizeof(audioBuffer));
            return COMMAND_FAILED;
        }
        where = (where + bufferHalfSize) % sizeof(audioBuffer);
        LED_beat_heart();
        quit = (InputGetChar() != -1);
    } while(!quit && (wasread == bufferHalfSize));

    f_close(&file);

    while((audioBufferPosition - where + sizeof(audioBuffer)) % sizeof(audioBuffer) < bufferHalfSize);
    memset(audioBuffer, 128, sizeof(audioBuffer));

    return COMMAND_CONTINUE;
}

static void RegisterCommandPlay(void) __attribute__((constructor));
static void RegisterCommandPlay(void)
{
    RegisterApp("play", 2, doPlay, "filename",
        "play 15.7KHz u8 mono audio stream"
        );
}

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

uint32_t oldLISR = 0;

void DMA2_Stream1_IRQHandler(void)
{
    // Configure timer TIM2 for performance measurement
    TIM9->CNT = 0;
    TIM9->CR1 = TIM_CR1_CEN;            /* enable the timer */

    // Clear interrupt flag
    DMA2->LIFCR |= DMA_LIFCR_CTCIF1; // XXX this bit (and other LISR/LIFCR HISR/HIFCR) are dependent on the stream 

    if(markHandlerInSamples) {
        for(int i = 0; i < 28; i++) { GPIOC->ODR = (GPIOC->ODR & 0xFFFFFF00) | 0xFFFFFFF8; }
    } else {
        // Do a little loop here since FIFO may have starved a little on interrupt
        // And hope we're early enough in the line that values aren't changing
        //uint32_t GPIOJ_ODR = GPIOJ->ODR;
        //for(int i = 0; i < 15; i++) { GPIOJ->ODR = GPIOJ_ODR; }
    }

    if(DMA2->LISR & DMA_FLAG_FEIF1_5) { // XXX 1_5 is "1 or 5"
        DMA2->LIFCR |= DMA_LIFCR_CFEIF1;
        DMAFIFOUnderruns++;
    }
    if(DMA2->LISR & DMA_FLAG_TEIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CTEIF1;
        DMATransferErrors++;
    }
    if(DMA2->LISR & DMA_FLAG_HTIF1_5) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
    }

    if(DMA2->LISR) {
        oldLISR = DMA2->LISR;
        DMA2->LIFCR = 0xFFFF;
    }

    // XXX audio experiment
    DAC1->DHR8R1 = audioBuffer[audioBufferPosition];
    audioBufferPosition = (audioBufferPosition + 1) % sizeof(audioBuffer);

    int whichIsScanning = (DMA2_Stream1->CR & DMA_SxCR_CT) ? 1 : 0;

    unsigned char *nextRowBuffer = (whichIsScanning == 1) ? row0 : row1;

    NTSCFillRowBuffer(frameNumber, lineNumber, nextRowBuffer);
    if(debugOverlayEnabled) {
        fillRowDebugOverlay(frameNumber, lineNumber, nextRowBuffer);
    }

    int thisLineNumber = lineNumber;
    lineNumber = lineNumber + 1;
    if(lineNumber == NTSC_FRAME_LINES) {
        lineNumber = 0;
        frameNumber++;
    }

    // A little pulse so we know where we are on the line when we finished
    if(markHandlerInSamples) {
        for(int i = 0; i < 28; i++) { GPIOC->ODR = (GPIOC->ODR & 0xFFFFFF00) | 0xFFFFFFE8; }
    }

    if(0) {
        int withinVisiblePartOfEvenFrame = (lineNumber > 27) && (lineNumber < 257);
        int withinVisiblePartOfOddFrame = (lineNumber > (262 + 27)) && (lineNumber < (262 + 257));

        if(withinVisiblePartOfEvenFrame || withinVisiblePartOfOddFrame) {
            while(DMA2_Stream1->NDTR > 100);
        }
    }

#if 1
    TIM9->CR1 = 0;            /* stop the timer */
    // rowCyclesSpent[lineNumber] = TIM9->CNT;     // TIM9 CK_INT is RCC on 415xxx, APB1 on 746xxx
    rowDMARemained[thisLineNumber] = 912 - TIM9->CNT / 7;
#else
    rowDMARemained[thisLineNumber] = *(uint16_t*)&DMA2_Stream1->NDTR;
#endif
}

#ifdef __cplusplus
};
#endif /* __cplusplus */


void DMAStartScanout(uint32_t dmaCount)
{
    HAL_StatusTypeDef status;

    // Configure DAC
    GPIO_InitTypeDef  GPIO_InitStruct = {0};
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = 0xFF;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

    // Configure E9 as TI1_CH1 
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // Enable DMA interrupt handler
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    // Configure DMA
    DMA2_Stream1->NDTR = ROW_SAMPLES;
    DMA2_Stream1->M0AR = (uint32_t)row0;        // Source buffer address 0
    DMA2_Stream1->M1AR = (uint32_t)row1;        // Source buffer address 1 
    // DMA2_Stream1->PAR = (uint32_t)&DAC1->DHR8R1;  // Destination address
    DMA2_Stream1->PAR = (uint32_t)&GPIOC->ODR;  // Destination address
    DMA2_Stream1->FCR = DMA_FIFOMODE_ENABLE |   // Enable FIFO to improve stutter
        DMA_FIFO_THRESHOLD_FULL;        
    DMA2_Stream1->CR =
        DMA_CHANNEL_6 |                         // which channel is driven by which timer to which peripheral is limited
        DMA_MEMORY_TO_PERIPH |                  // Memory to Peripheral
        DMA_PDATAALIGN_BYTE |                   // BYTES to peripheral
        DMA_MDATAALIGN_HALFWORD |
        DMA_SxCR_DBM |                          // double buffer
        DMA_PRIORITY_VERY_HIGH |                // Video data must be highest priority, can't stutter
        DMA_MINC_ENABLE |                       // Increment memory address
        DMA_IT_TC |                             // Interrupt on transfer complete of each buffer
        // DMA_IT_HT |                          // Interrupt on transfer complete of half a buffer 
        // DMA_IT_TE |                             // XXX Interrupt on transfer error
        // DMA_IT_DME |                             // XXX Interrupt on DMA error
        0;

    // Clear FIFO and transfer error flags
    DMA2->LIFCR |= DMA_LIFCR_CFEIF2;
    DMA2->LIFCR |= DMA_LIFCR_CTEIF2;

    DMA2_Stream1->CR |= DMA_SxCR_EN;    /* enable DMA */

    // Configure TIM1 to clock from input capture for channel 1
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if ((status = HAL_TIM_IC_Init(&htim1)) != HAL_OK)
    {
        printf("HAL_TIM_IC_Init failed, status %d\n", status);
        panic();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        printf("HAL_TIMEx_MasterConfigSynchronization failed, status %d\n", status);
        panic();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        printf("HAL_TIM_IC_ConfigChannel failed, status %d\n", status);
        panic();
    }

    // Set DMA request on capture-compare channel 1
    TIM1->DIER |= TIM_DIER_CC1DE;

    lineNumber = 1; // Next up is row 1
    frameNumber = 0; 

    printf("TIM1->CR1 = %08lX\n", TIM1->CR1);
    printf("TIM1->CR2 = %08lX\n", TIM1->CR2);
    printf("TIM1->SMCR = %08lX\n", TIM1->SMCR);
    printf("TIM1->DIER = %08lX\n", TIM1->DIER);
    printf("TIM1->SR = %08lX\n", TIM1->SR);
    printf("TIM1->EGR = %08lX\n", TIM1->EGR);
    printf("TIM1->CCMR1 = %08lX\n", TIM1->CCMR1);
    printf("TIM1->CCMR2 = %08lX\n", TIM1->CCMR2);
    printf("TIM1->CCER = %08lX\n", TIM1->CCER);
    printf("TIM1->CNT = %08lX\n", TIM1->CNT);
    printf("TIM1->PSC = %08lX\n", TIM1->PSC);
    printf("TIM1->ARR = %08lX\n", TIM1->ARR);
    printf("TIM1->RCR = %08lX\n", TIM1->RCR);
    printf("TIM1->CCR1 = %08lX\n", TIM1->CCR1);
    printf("TIM1->CCR2 = %08lX\n", TIM1->CCR2);
    printf("TIM1->CCR3 = %08lX\n", TIM1->CCR3);
    printf("TIM1->CCR4 = %08lX\n", TIM1->CCR4);
    printf("TIM1->BDTR = %08lX\n", TIM1->BDTR);
    printf("TIM1->DCR = %08lX\n", TIM1->DCR);
    printf("TIM1->DMAR = %08lX\n", TIM1->DMAR);
    printf("TIM1->OR = %08lX\n", TIM1->OR);
    printf("TIM1->CCMR3 = %08lX\n", TIM1->CCMR3);
    printf("TIM1->CCR5 = %08lX\n", TIM1->CCR5);
    printf("TIM1->CCR6 = %08lX\n", TIM1->CCR6);

    printf("DMA2->LISR = %08lX\n", DMA2->LISR);
    printf("DMA2->HISR = %08lX\n", DMA2->HISR);

    printf("DMA2_Stream1->CR = %08lX\n", DMA2_Stream1->CR);
    printf("DMA2_Stream1->NDTR = %08lX\n", DMA2_Stream1->NDTR);
    printf("DMA2_Stream1->PAR = %08lX\n", DMA2_Stream1->PAR);
    printf("DMA2_Stream1->M0AR = %08lX\n", DMA2_Stream1->M0AR);
    printf("DMA2_Stream1->M1AR = %08lX\n", DMA2_Stream1->M1AR);
    printf("DMA2_Stream1->FCR = %08lX\n", DMA2_Stream1->FCR);

    status = HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
    if(status != HAL_OK) {
        printf("HAL_TIM_IC_Start status %d\n", status);
        panic();
    }
}

void DMAStopScanout()
{
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;       /* disable DMA */
    // TIM1->CR1 &= ~TIM_CR1_CEN;            /* disable the timer... what's the HAL function? */
    HAL_TIM_Base_Stop(&htim1);
}

//----------------------------------------------------------------------------
// Video modes 

#define font8x8Width 8
#define font8x8Height 8

unsigned char SECTION_CCMRAM font8x8Bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x12, 0x10,
    0x7C, 0x10, 0x12, 0x7C, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x00, 0x04, 0x08, 0x3C, 0x42, 0x7E, 0x40, 0x3C, 0x00, 0x24,
    0x00, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x08, 0x14, 0x08, 0x14,
    0x22, 0x3E, 0x22, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x02, 0x02, 0x00,
    0x00, 0x14, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x1D, 0x22,
    0x26, 0x2A, 0x32, 0x22, 0x5C, 0x00, 0x10, 0x08, 0x42, 0x42, 0x42,
    0x46, 0x3A, 0x00, 0x32, 0x4C, 0x00, 0x2C, 0x32, 0x22, 0x22, 0x00,
    0x08, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x38,
    0x04, 0x3C, 0x44, 0x3A, 0x00, 0x3E, 0x7A, 0x44, 0x44, 0x78, 0x48,
    0x44, 0x00, 0x14, 0x00, 0x08, 0x14, 0x22, 0x3E, 0x22, 0x00, 0x32,
    0x4C, 0x08, 0x14, 0x22, 0x3E, 0x22, 0x00, 0x32, 0x4C, 0x22, 0x32,
    0x2A, 0x26, 0x22, 0x00, 0x00, 0x14, 0x1C, 0x22, 0x22, 0x22, 0x1C,
    0x00, 0x09, 0x16, 0x26, 0x2A, 0x32, 0x34, 0x48, 0x00, 0x32, 0x4C,
    0x00, 0x3C, 0x42, 0x42, 0x3C, 0x00, 0x3C, 0x22, 0x22, 0x3C, 0x22,
    0x22, 0x7C, 0x00, 0x24, 0x00, 0x42, 0x42, 0x42, 0x46, 0x3A, 0x00,
    0x32, 0x4C, 0x00, 0x18, 0x24, 0x24, 0x18, 0x00, 0x1C, 0x2A, 0x0A,
    0x1C, 0x28, 0x2A, 0x1C, 0x00, 0x28, 0x00, 0x38, 0x04, 0x3C, 0x44,
    0x3A, 0x00, 0x20, 0x10, 0x38, 0x04, 0x3C, 0x44, 0x3A, 0x00, 0x10,
    0x00, 0x38, 0x04, 0x3C, 0x44, 0x3A, 0x00, 0x3C, 0x40, 0x7C, 0x42,
    0x3E, 0x02, 0x3C, 0x00, 0x04, 0x08, 0x3E, 0x20, 0x3E, 0x20, 0x3E,
    0x00, 0x08, 0x1E, 0x24, 0x26, 0x3C, 0x24, 0x26, 0x00, 0x1C, 0x22,
    0x20, 0x20, 0x22, 0x1C, 0x08, 0x10, 0x00, 0x00, 0x32, 0x4C, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00, 0x24, 0x24, 0x24,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x24, 0x7E, 0x24, 0x7E, 0x24,
    0x24, 0x00, 0x08, 0x1E, 0x28, 0x1C, 0x0A, 0x3C, 0x08, 0x00, 0x00,
    0x62, 0x64, 0x08, 0x10, 0x26, 0x46, 0x00, 0x30, 0x48, 0x48, 0x30,
    0x4A, 0x44, 0x3A, 0x00, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, 0x04, 0x00, 0x20, 0x10,
    0x08, 0x08, 0x08, 0x10, 0x20, 0x00, 0x08, 0x2A, 0x1C, 0x3E, 0x1C,
    0x2A, 0x08, 0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x10, 0x00, 0x00, 0x00,
    0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x00, 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x00, 0x3C,
    0x42, 0x46, 0x5A, 0x62, 0x42, 0x3C, 0x00, 0x08, 0x18, 0x28, 0x08,
    0x08, 0x08, 0x3E, 0x00, 0x3C, 0x42, 0x02, 0x0C, 0x30, 0x40, 0x7E,
    0x00, 0x3C, 0x42, 0x02, 0x1C, 0x02, 0x42, 0x3C, 0x00, 0x04, 0x0C,
    0x14, 0x24, 0x7E, 0x04, 0x04, 0x00, 0x7E, 0x40, 0x78, 0x04, 0x02,
    0x44, 0x38, 0x00, 0x1C, 0x20, 0x40, 0x7C, 0x42, 0x42, 0x3C, 0x00,
    0x7E, 0x42, 0x04, 0x08, 0x10, 0x10, 0x10, 0x00, 0x3C, 0x42, 0x42,
    0x3C, 0x42, 0x42, 0x3C, 0x00, 0x3C, 0x42, 0x42, 0x3E, 0x02, 0x04,
    0x38, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x00, 0x00, 0x08, 0x08, 0x10, 0x06, 0x0C, 0x18, 0x30,
    0x18, 0x0C, 0x06, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00,
    0x00, 0x60, 0x30, 0x18, 0x0C, 0x18, 0x30, 0x60, 0x00, 0x3C, 0x42,
    0x02, 0x0C, 0x10, 0x00, 0x10, 0x00, 0x1C, 0x22, 0x4A, 0x56, 0x4C,
    0x20, 0x1E, 0x00, 0x18, 0x24, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x00,
    0x7C, 0x22, 0x22, 0x3C, 0x22, 0x22, 0x7C, 0x00, 0x1C, 0x22, 0x40,
    0x40, 0x40, 0x22, 0x1C, 0x00, 0x78, 0x24, 0x22, 0x22, 0x22, 0x24,
    0x78, 0x00, 0x7E, 0x40, 0x40, 0x78, 0x40, 0x40, 0x7E, 0x00, 0x7E,
    0x40, 0x40, 0x78, 0x40, 0x40, 0x40, 0x00, 0x1C, 0x22, 0x40, 0x4E,
    0x42, 0x22, 0x1C, 0x00, 0x42, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42,
    0x00, 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x0E, 0x04,
    0x04, 0x04, 0x04, 0x44, 0x38, 0x00, 0x42, 0x44, 0x48, 0x70, 0x48,
    0x44, 0x42, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7E, 0x00,
    0x42, 0x66, 0x5A, 0x5A, 0x42, 0x42, 0x42, 0x00, 0x42, 0x62, 0x52,
    0x4A, 0x46, 0x42, 0x42, 0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42,
    0x3C, 0x00, 0x7C, 0x42, 0x42, 0x7C, 0x40, 0x40, 0x40, 0x00, 0x3C,
    0x42, 0x42, 0x42, 0x4A, 0x44, 0x3A, 0x00, 0x7C, 0x42, 0x42, 0x7C,
    0x48, 0x44, 0x42, 0x00, 0x3C, 0x42, 0x40, 0x3C, 0x02, 0x42, 0x3C,
    0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x42, 0x42,
    0x42, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x42, 0x42, 0x42, 0x24, 0x24,
    0x18, 0x18, 0x00, 0x42, 0x42, 0x42, 0x5A, 0x5A, 0x66, 0x42, 0x00,
    0x42, 0x42, 0x24, 0x18, 0x24, 0x42, 0x42, 0x00, 0x22, 0x22, 0x22,
    0x1C, 0x08, 0x08, 0x08, 0x00, 0x7E, 0x02, 0x04, 0x18, 0x20, 0x40,
    0x7E, 0x00, 0x3C, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x00, 0x00,
    0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x3C, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x3C, 0x00, 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x10, 0x08,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x04, 0x3C,
    0x44, 0x3A, 0x00, 0x40, 0x40, 0x5C, 0x62, 0x42, 0x62, 0x5C, 0x00,
    0x00, 0x00, 0x3C, 0x42, 0x40, 0x42, 0x3C, 0x00, 0x02, 0x02, 0x3A,
    0x46, 0x42, 0x46, 0x3A, 0x00, 0x00, 0x00, 0x3C, 0x42, 0x7E, 0x40,
    0x3C, 0x00, 0x0C, 0x12, 0x10, 0x7C, 0x10, 0x10, 0x10, 0x00, 0x00,
    0x00, 0x3A, 0x46, 0x46, 0x3A, 0x02, 0x3C, 0x40, 0x40, 0x5C, 0x62,
    0x42, 0x42, 0x42, 0x00, 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x1C,
    0x00, 0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x44, 0x38, 0x40, 0x40,
    0x44, 0x48, 0x50, 0x68, 0x44, 0x00, 0x18, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x1C, 0x00, 0x00, 0x00, 0x76, 0x49, 0x49, 0x49, 0x49, 0x00,
    0x00, 0x00, 0x5C, 0x62, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00, 0x3C,
    0x42, 0x42, 0x42, 0x3C, 0x00, 0x00, 0x00, 0x5C, 0x62, 0x62, 0x5C,
    0x40, 0x40, 0x00, 0x00, 0x3A, 0x46, 0x46, 0x3A, 0x02, 0x02, 0x00,
    0x00, 0x5C, 0x62, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x3E, 0x40,
    0x3C, 0x02, 0x7C, 0x00, 0x10, 0x10, 0x7C, 0x10, 0x10, 0x12, 0x0C,
    0x00, 0x00, 0x00, 0x42, 0x42, 0x42, 0x46, 0x3A, 0x00, 0x00, 0x00,
    0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00, 0x00, 0x41, 0x49, 0x49,
    0x49, 0x36, 0x00, 0x00, 0x00, 0x42, 0x24, 0x18, 0x24, 0x42, 0x00,
    0x00, 0x00, 0x42, 0x42, 0x46, 0x3A, 0x02, 0x3C, 0x00, 0x00, 0x7E,
    0x04, 0x18, 0x20, 0x7E, 0x00, 0x0C, 0x10, 0x10, 0x20, 0x10, 0x10,
    0x0C, 0x00, 0x08, 0x08, 0x08, 0x00, 0x08, 0x08, 0x08, 0x00, 0x30,
    0x08, 0x08, 0x04, 0x08, 0x08, 0x30, 0x00, 0x30, 0x49, 0x06, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x3E,
    0x00, 0x08, 0x1C, 0x3E, 0x7F, 0x7F, 0x3E, 0x08, 0x08, 0x00, 0x36,
    0x7F, 0x7F, 0x3E, 0x1C, 0x08, 0x00, 0x08, 0x1C, 0x3E, 0x7F, 0x3E,
    0x1C, 0x08, 0x00, 0x1C, 0x1C, 0x08, 0x6B, 0x7F, 0x6B, 0x08, 0x1C,
    0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C, 0x3C, 0x42, 0xA5,
    0x81, 0x99, 0xA5, 0x42, 0x3C, 0x04, 0x08, 0x10, 0x20, 0x10, 0x08,
    0x04, 0x3C, 0x20, 0x10, 0x08, 0x04, 0x08, 0x10, 0x20, 0x3C, 0x00,
    0x00, 0x39, 0x46, 0x46, 0x39, 0x00, 0x00, 0x3C, 0x22, 0x3C, 0x22,
    0x22, 0x3C, 0x20, 0x40, 0x61, 0x12, 0x14, 0x18, 0x10, 0x30, 0x30,
    0x00, 0x0C, 0x12, 0x10, 0x0C, 0x0A, 0x12, 0x0C, 0x00, 0x06, 0x08,
    0x10, 0x3E, 0x10, 0x08, 0x06, 0x00, 0x16, 0x06, 0x08, 0x10, 0x1C,
    0x02, 0x0C, 0x00, 0x2C, 0x52, 0x12, 0x12, 0x02, 0x02, 0x02, 0x00,
    0x08, 0x14, 0x22, 0x3E, 0x22, 0x14, 0x08, 0x00, 0x00, 0x20, 0x20,
    0x20, 0x22, 0x22, 0x1C, 0x00, 0x40, 0x48, 0x50, 0x60, 0x50, 0x4A,
    0x44, 0x00, 0x20, 0x10, 0x10, 0x10, 0x18, 0x24, 0x42, 0x00, 0x24,
    0x24, 0x24, 0x24, 0x3A, 0x20, 0x20, 0x00, 0x00, 0x00, 0x32, 0x12,
    0x14, 0x18, 0x10, 0x00, 0x08, 0x1C, 0x20, 0x18, 0x20, 0x1C, 0x02,
    0x0C, 0x00, 0x18, 0x24, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00, 0x00,
    0x3E, 0x54, 0x14, 0x14, 0x14, 0x00, 0x00, 0x18, 0x24, 0x24, 0x38,
    0x20, 0x20, 0x00, 0x00, 0x00, 0x3E, 0x48, 0x48, 0x30, 0x00, 0x00,
    0x00, 0x00, 0x3E, 0x48, 0x08, 0x08, 0x08, 0x08, 0x00, 0x02, 0x64,
    0x24, 0x24, 0x24, 0x18, 0x00, 0x08, 0x1C, 0x2A, 0x2A, 0x2A, 0x1C,
    0x08, 0x00, 0x00, 0x00, 0x62, 0x14, 0x08, 0x14, 0x23, 0x00, 0x49,
    0x2A, 0x2A, 0x1C, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x22, 0x41,
    0x49, 0x49, 0x36, 0x00, 0x1C, 0x22, 0x41, 0x41, 0x63, 0x22, 0x63,
    0x00, 0x1E, 0x10, 0x10, 0x10, 0x50, 0x30, 0x10, 0x00, 0x00, 0x08,
    0x00, 0x3E, 0x00, 0x08, 0x00, 0x00, 0x7E, 0x20, 0x10, 0x0C, 0x10,
    0x20, 0x7E, 0x00, 0x00, 0x32, 0x4C, 0x00, 0x32, 0x4C, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x14, 0x22, 0x7F, 0x00, 0x00, 0x04, 0x08, 0x10,
    0x10, 0x08, 0x08, 0x10, 0x20, 0x01, 0x02, 0x7F, 0x08, 0x7F, 0x20,
    0x40, 0x00, 0x10, 0x08, 0x04, 0x3E, 0x10, 0x08, 0x04, 0x00, 0x3F,
    0x52, 0x24, 0x08, 0x12, 0x25, 0x42, 0x00, 0x1C, 0x22, 0x41, 0x41,
    0x7F, 0x22, 0x22, 0x63, 0x00, 0x00, 0x36, 0x49, 0x49, 0x36, 0x00,
    0x00, 0x00, 0x02, 0x04, 0x48, 0x50, 0x60, 0x40, 0x00, 0x1E, 0x20,
    0x1C, 0x22, 0x1C, 0x02, 0x3C, 0x00, 0x22, 0x55, 0x2A, 0x14, 0x2A,
    0x55, 0x22, 0x00, 0x3C, 0x42, 0x9D, 0xA1, 0xA1, 0x9D, 0x42, 0x3C,
    0x42, 0x24, 0x18, 0x24, 0x18, 0x24, 0x42, 0x00, 0x3E, 0x4A, 0x4A,
    0x3A, 0x0A, 0x0A, 0x0A, 0x0A, 0x08, 0x1C, 0x2A, 0x28, 0x2A, 0x1C,
    0x08, 0x00, 0x3C, 0x7A, 0xA5, 0xA5, 0xB9, 0xA9, 0x66, 0x3C, 0x5F,
    0x60, 0x63, 0x62, 0x64, 0x7B, 0x60, 0x5F, 0xFF, 0x04, 0x03, 0xFC,
    0x02, 0xFC, 0x04, 0xF8, 0xFC, 0x02, 0xFC, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x78, 0x44, 0x44, 0x78, 0x4A, 0x44, 0x4B, 0x00, 0x61, 0x82,
    0x84, 0x68, 0x16, 0x29, 0x49, 0x86, 0x0E, 0x06, 0x0A, 0x70, 0x90,
    0x90, 0x60, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x08, 0x1C, 0x08,
    0x0E, 0x08, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE3, 0xDD,
    0xF3, 0xF7, 0xFF, 0xF7, 0xFF, 0x08, 0x14, 0x08, 0x1C, 0x2A, 0x08,
    0x14, 0x22, 0x08, 0x14, 0x08, 0x1C, 0x2A, 0x14, 0x3E, 0x14, 0x08,
    0x14, 0x22, 0x22, 0x22, 0x2A, 0x36, 0x22, 0x22, 0x14, 0x08, 0x3E,
    0x08, 0x3E, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x50, 0x20,
    0x00, 0x1C, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x04, 0x04, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00,
    0x00, 0x7E, 0x02, 0x7E, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x3E,
    0x02, 0x0C, 0x08, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x0C, 0x14,
    0x04, 0x00, 0x00, 0x00, 0x08, 0x3E, 0x22, 0x04, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x3E, 0x08, 0x08, 0x3E, 0x00, 0x00, 0x00, 0x04, 0x3E,
    0x0C, 0x14, 0x24, 0x00, 0x00, 0x00, 0x10, 0x3E, 0x12, 0x14, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x1C, 0x04, 0x04, 0x3E, 0x00, 0x00, 0x00,
    0x3C, 0x04, 0x1C, 0x04, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x2A,
    0x02, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x7E, 0x02, 0x16, 0x18, 0x10, 0x20, 0x00, 0x02, 0x04, 0x08,
    0x18, 0x28, 0x48, 0x08, 0x00, 0x10, 0x7E, 0x42, 0x02, 0x02, 0x04,
    0x08, 0x00, 0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x3E, 0x00, 0x04,
    0x7E, 0x0C, 0x14, 0x24, 0x44, 0x04, 0x00, 0x10, 0x7E, 0x12, 0x12,
    0x12, 0x22, 0x44, 0x00, 0x08, 0x7F, 0x08, 0x7F, 0x08, 0x08, 0x08,
    0x00, 0x20, 0x3E, 0x22, 0x42, 0x04, 0x08, 0x10, 0x00, 0x20, 0x3E,
    0x48, 0x08, 0x08, 0x08, 0x10, 0x00, 0x00, 0x7E, 0x02, 0x02, 0x02,
    0x02, 0x7E, 0x00, 0x24, 0x7E, 0x24, 0x24, 0x04, 0x08, 0x10, 0x00,
    0x00, 0x60, 0x02, 0x62, 0x02, 0x04, 0x38, 0x00, 0x00, 0x3E, 0x02,
    0x04, 0x08, 0x14, 0x22, 0x00, 0x10, 0x7E, 0x11, 0x12, 0x10, 0x10,
    0x0E, 0x00, 0x00, 0x42, 0x22, 0x02, 0x04, 0x08, 0x10, 0x00, 0x20,
    0x3E, 0x22, 0x52, 0x0C, 0x08, 0x10, 0x00, 0x04, 0x38, 0x08, 0x7E,
    0x08, 0x08, 0x10, 0x00, 0x00, 0x52, 0x52, 0x52, 0x02, 0x04, 0x08,
    0x00, 0x00, 0x3C, 0x00, 0x7E, 0x08, 0x08, 0x10, 0x00, 0x10, 0x10,
    0x10, 0x18, 0x14, 0x12, 0x10, 0x00, 0x08, 0x08, 0x7E, 0x08, 0x08,
    0x08, 0x10, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00,
    0x00, 0x3E, 0x02, 0x14, 0x08, 0x14, 0x22, 0x00, 0x10, 0x7E, 0x04,
    0x08, 0x14, 0x32, 0x50, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x10,
    0x20, 0x00, 0x00, 0x10, 0x08, 0x44, 0x42, 0x42, 0x42, 0x00, 0x40,
    0x40, 0x7E, 0x40, 0x40, 0x40, 0x3E, 0x00, 0x7E, 0x02, 0x02, 0x02,
    0x02, 0x04, 0x18, 0x00, 0x00, 0x10, 0x28, 0x44, 0x02, 0x01, 0x01,
    0x00, 0x08, 0x7F, 0x08, 0x08, 0x49, 0x49, 0x08, 0x00, 0x7E, 0x02,
    0x02, 0x24, 0x18, 0x08, 0x04, 0x00, 0x40, 0x3C, 0x00, 0x3C, 0x00,
    0x7C, 0x02, 0x00, 0x08, 0x10, 0x20, 0x40, 0x42, 0x7E, 0x02, 0x00,
    0x02, 0x02, 0x14, 0x08, 0x14, 0x20, 0x40, 0x00, 0x00, 0x7E, 0x10,
    0x7E, 0x10, 0x10, 0x0E, 0x00, 0x10, 0x10, 0x7E, 0x12, 0x14, 0x10,
    0x10, 0x00, 0x00, 0x3C, 0x04, 0x04, 0x04, 0x04, 0x7F, 0x00, 0x3E,
    0x02, 0x02, 0x3E, 0x02, 0x02, 0x3E, 0x00, 0x3C, 0x00, 0x7E, 0x02,
    0x02, 0x04, 0x08, 0x00, 0x44, 0x44, 0x44, 0x44, 0x04, 0x08, 0x10,
    0x00, 0x10, 0x50, 0x50, 0x50, 0x52, 0x54, 0x58, 0x00, 0x40, 0x40,
    0x40, 0x44, 0x48, 0x50, 0x60, 0x00, 0x00, 0x7E, 0x42, 0x42, 0x42,
    0x42, 0x7E, 0x00, 0x00, 0x7E, 0x42, 0x42, 0x02, 0x04, 0x08, 0x00,
    0x00, 0x60, 0x00, 0x02, 0x02, 0x04, 0x78, 0x00, 0x10, 0x48, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x50, 0x20, 0x00, 0x00, 0x00,
    0x00, 0x00,
};

// Video mode specifics follow

int SetPaletteEntryAlwaysFails(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    return 0;
}

void SetPaletteEntry(int palette, int which, float r, float g, float b)
{
    NTSCSetPaletteEntry(palette, which, r, g, b);
}

int SetRowPalette(const struct VideoModeEntry* modeEntry, int row, int palette)
{
    return NTSCSetPaletteForRow(row, palette);
}

const static float HSVFor4BitPalette[][3] = {
    { M_PI / 3 * 0, 1.0, 1.0 },
    { M_PI / 3 * 1, 1.0, 1.0 },
    { M_PI / 3 * 2, 1.0, 1.0 },
    { M_PI / 3 * 3, 1.0, 1.0 },
    { M_PI / 3 * 4, 1.0, 1.0 },
    { M_PI / 3 * 5, 1.0, 1.0 },
    { M_PI / 3 * 0, 0.25, 0.5 },
    { M_PI / 3 * 1, 0.25, 0.5 },
    { M_PI / 3 * 2, 0.25, 0.5 },
    { M_PI / 3 * 3, 0.25, 0.5 },
    { M_PI / 3 * 4, 0.25, 0.5 },
    { M_PI / 3 * 5, 0.25, 0.5 },
    { 0.0, 0.0, 1.0},
    { 0.0, 0.0, .66},
    { 0.0, 0.0, .33},
    { 0.0, 0.0, .0},
};

static void MakeDefaultPalette(int whichPalette, int paletteSize)
{
    switch(paletteSize) {
        case -1: break; /* no palette */
        case 16:  {
            for(int entry = 0; entry < 16; entry++) {
                const float *hsv = HSVFor4BitPalette[entry];
                float r, g, b;
                HSVToRGB3f(hsv[0], hsv[1], hsv[2], &r, &g, &b);
                SetPaletteEntry(whichPalette, entry, r, g, b);
            }
            break;
        }
        case 256: {
            // H3S2V3
            for(unsigned int entry = 0; entry < 256; entry++) {
                float h = ((entry >> 5) & 7) / 7.0f * M_PI * 2;
                float s = ((entry >> 3) & 3) / 3.0f;
                float v = ((entry >> 0) & 7) / 7.0f;
                float r, g, b;
                HSVToRGB3f(h, s, v, &r, &g, &b);
                SetPaletteEntry(whichPalette, entry, r, g, b);
            }
        }
    }
}


// ----------------------------------------
// plain 40x24 black-on-white textport

#define TextportPlain40x24Width 40
#define TextportPlain40x24Height 24
#define TextportPlain40x24TopTick 44
#define TextportPlain40x24LeftTick 196

int TextportPlain40x24CursorX = 0;
int TextportPlain40x24CursorY = 0;
int TextportPlain40x24CursorFlash = 1;

// Textport video mode structs
const static VideoTextportInfo TextportPlain40x24Info = {
    TextportPlain40x24Width, TextportPlain40x24Height,
    TEXT_8BIT_BLACK_ON_WHITE,
};

void TextportPlain40x24Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 0);        /* monochrome text */
}

void TextportPlain40x24GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoTextportInfo *info = (VideoTextportInfo*)modeEntry->info;
    VideoTextportParameters *params = (VideoTextportParameters*)params_;
    params->base = VRAM;
    params->cursorX = &TextportPlain40x24CursorX;
    params->cursorY = &TextportPlain40x24CursorY;
    params->rowSize = info->width;
}

void TextportPlain40x24FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithinArea = (lineNumber % 263) - TextportPlain40x24TopTick;
    int charRow = rowWithinArea / font8x8Height;
    int charPixelY = (rowWithinArea % font8x8Height);

    int invert = (TextportPlain40x24CursorFlash == 0) || (frameNumber / 15 % 2 == 0);

// XXX this code assumes font width <= 8 and each row padded out to a byte
    if((rowWithinArea >= 0) && (charRow < TextportPlain40x24Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCWhite, TextportPlain40x24LeftTick - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightTextEdge = TextportPlain40x24LeftTick + TextportPlain40x24Width * font8x8Width * 2;
        memset(rowBuffer + clocksToRightTextEdge, NTSCWhite, ROW_SAMPLES - clocksToRightTextEdge - NTSCFrontPorchClocks);

        for(int charCol = 0; charCol < TextportPlain40x24Width; charCol++) {
            unsigned char whichChar = VRAM[charRow * TextportPlain40x24Width + charCol];
            unsigned char charRowBits = font8x8Bits[whichChar * font8x8Height + charPixelY];
            unsigned char *charPixels = rowBuffer + TextportPlain40x24LeftTick + charCol * font8x8Width * 2;
            int invertThis = (charCol == TextportPlain40x24CursorX && charRow == TextportPlain40x24CursorY && invert);
            for(int charPixelX = 0; charPixelX < font8x8Width; charPixelX++) {
                int pixel = (charRowBits & (0x80 >> charPixelX)) ^ invertThis;
                charPixels[charPixelX * 2 + 0] = pixel ? NTSCBlack : NTSCWhite;
                charPixels[charPixelX * 2 + 1] = pixel ? NTSCBlack : NTSCWhite;
            }
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCWhite, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// plain 80x24 white-on-black textport

// 196, 832, 44, 236
#define TextportPlain80x24Width 80
#define TextportPlain80x24Height 24
#define TextportPlain80x24TopTick 44
#define TextportPlain80x24LeftTick 196

int TextportPlain80x24CursorX = 0;
int TextportPlain80x24CursorY = 0;
int TextportPlain80x24CursorFlash = 0;

// Textport video mode structs
const static VideoTextportInfo TextportPlain80x24Info = {
    TextportPlain80x24Width, TextportPlain80x24Height,
    TEXT_8BIT_WHITE_ON_BLACK,
};

void TextportPlain80x24Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 0);        /* monochrome text */
}

void TextportPlain80x24GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoTextportInfo *info = (VideoTextportInfo*)modeEntry->info;
    VideoTextportParameters *params = (VideoTextportParameters*)params_;
    params->base = VRAM;
    params->cursorX = &TextportPlain80x24CursorX;
    params->cursorY = &TextportPlain80x24CursorY;
    params->rowSize = info->width;
}

int TextportPlain80x24SetPaletteEntry(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    return 0;
}

void TextportPlain80x24FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithinArea = (lineNumber % 263) - TextportPlain80x24TopTick;
    int charRow = rowWithinArea / font8x8Height;
    int charPixelY = (rowWithinArea % font8x8Height);

    int invert = (TextportPlain80x24CursorFlash == 0) || (frameNumber / 15 % 2 == 0);

// XXX this code assumes font width <= 8 and each row padded out to a byte
    if((rowWithinArea >= 0) && (charRow < TextportPlain80x24Height)) {
        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, TextportPlain80x24LeftTick - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightTextEdge = TextportPlain80x24LeftTick + TextportPlain80x24Width * font8x8Width;
        memset(rowBuffer + clocksToRightTextEdge, NTSCBlack, ROW_SAMPLES - clocksToRightTextEdge - NTSCFrontPorchClocks);

        for(int charCol = 0; charCol < TextportPlain80x24Width; charCol++) {
            unsigned char whichChar = VRAM[charRow * TextportPlain80x24Width + charCol];
            unsigned char charRowBits = font8x8Bits[whichChar * font8x8Height + charPixelY];
            unsigned char *charPixels = rowBuffer + TextportPlain80x24LeftTick + charCol * font8x8Width;
            int invertThis = (charCol == TextportPlain80x24CursorX && charRow == TextportPlain80x24CursorY && invert);
            if(invertThis) {
                for(int charPixelX = 0; charPixelX < font8x8Width; charPixelX++) {
                    int pixel = charRowBits & (0x80 >> charPixelX);
                    charPixels[charPixelX] = pixel ? NTSCBlack : NTSCWhite;
                }
            } else {
                for(int charPixelX = 0; charPixelX < font8x8Width; charPixelX++) {
                    int pixel = charRowBits & (0x80 >> charPixelX);
                    charPixels[charPixelX] = pixel ? NTSCWhite : NTSCBlack;
                }
            }
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// highest-horizontal-resolution 8-bit framebuffer

#define MAX_RES_MODE_WIDTH 384
#define MAX_RES_MODE_HEIGHT 128

#if VRAM_PIXMAP_AVAIL < MAX_RES_MODE_WIDTH * MAX_RES_MODE_HEIGHT
#error Available VRAM has become too small for MaxRes mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo ColorMaxResInfo = {
    MAX_RES_MODE_WIDTH, MAX_RES_MODE_HEIGHT,
    VideoPixmapFormat::PALETTE_8BIT,
    256,
    1,
    1,
    225, 225 / 4 * 3 * MAX_RES_MODE_WIDTH / MAX_RES_MODE_HEIGHT,
};

void ColorMaxResSetup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 1);
    MakeDefaultPalette(0, 256);
    for(int y = 0; y < MAX_RES_MODE_HEIGHT; y++) {
        NTSCGetPaletteForRowPointer()[y] = 0;
    }
}

void ColorMaxResGetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width;
}

// 510 and 142 are magic numbers measured for middle of screen
#define ColorMaxResLeft (512 - MAX_RES_MODE_WIDTH / 2) 
#define ColorMaxResWidth MAX_RES_MODE_WIDTH
#define ColorMaxResTop (142 - MAX_RES_MODE_HEIGHT / 2)
#define ColorMaxResHeight MAX_RES_MODE_HEIGHT

int ColorMaxResSetPaletteEntry(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    if(which >= 256) {
        return 1;
    }
    SetPaletteEntry(palette, which, r, g, b);
    return 0;
}

// XXX assumes ColorMaxResLeft is /4 and ColorMaxResWidth /4
void ColorMaxResFillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - ColorMaxResTop;

    if((rowWithin >= 0) && (rowWithin < ColorMaxResHeight)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ColorMaxResLeft - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = ColorMaxResLeft + ColorMaxResWidth;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *srcPixels = VRAM + rowWithin * ColorMaxResWidth;
        ntsc_wave_t *dstWords = (ntsc_wave_t*)(rowBuffer + ColorMaxResLeft);

        ntsc_wave_t *palette = NTSCGetPaletteForRow(rowWithin);

        for(int i = 0; i < ColorMaxResWidth; i += 4) {
            uint32_t waveformPart0 = palette[srcPixels[i + 0]] & 0x000000FF;
            uint32_t waveformPart1 = palette[srcPixels[i + 1]] & 0x0000FF00;
            uint32_t waveformPart2 = palette[srcPixels[i + 2]] & 0x00FF0000;
            uint32_t waveformPart3 = palette[srcPixels[i + 3]] & 0xFF000000;
            *dstWords++ = waveformPart0 | waveformPart1 | waveformPart2 | waveformPart3;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}


// ----------------------------------------
// 175 x 230 8-bit pixmap display
// to be /4, 164, 864, 27, 257

#define Color175x230Left 164
#define Color175x230WidthSamples 700
#define Color175x230WidthPixels (Color175x230WidthSamples / 4)
#define Color175x230PixelRowBytes Color175x230WidthPixels
#define Color175x230Top 27
#define Color175x230Height 230

#if VRAM_PIXMAP_AVAIL < Color175x230PixelRowBytes * Color175x230Height
#error Available VRAM has become too small for Color175x230 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Color175x230Info = {
    Color175x230WidthSamples / 4, Color175x230Height,
    VideoPixmapFormat::PALETTE_8BIT,
    256,
    1,
    1,
    900, 520,
};

void Color175x230Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 1);
    MakeDefaultPalette(0, 256);
    for(int y = 0; y < Color175x230Height; y++) {
        NTSCGetPaletteForRowPointer()[y] = 0;
    }
}

void Color175x230GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width;
}

int Color175x230SetPaletteEntry(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    if(which >= 256) {
        return 1;
    }
    SetPaletteEntry(palette, which, r, g, b);
    return 0;
}

void Color175x230FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Color175x230Top;

    if((rowWithin >= 0) && (rowWithin < Color175x230Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Color175x230WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Color175x230Left + Color175x230WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *srcPixels = VRAM + rowWithin * (Color175x230WidthSamples / 4);
        unsigned char *dstBytes = rowBuffer + Color175x230Left;

        ntsc_wave_t *palette = NTSCGetPaletteForRow(rowWithin);

        for(int i = 0; i < Color175x230WidthSamples; i += 4) {
            int p = srcPixels[i / 4];
            ntsc_wave_t word = palette[p];
            *(ntsc_wave_t*)(dstBytes + i) = word;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// 160 x 192 8-bit bitmap display
// 196, 832, 44, 236

#define Color160x192Left 196
#define Color160x192WidthSamples 640
#define Color160x192WidthPixels (Color160x192WidthSamples / 4)
#define Color160x192PixelRowBytes Color160x192WidthPixels
#define Color160x192Top 44
#define Color160x192Height 192

#if VRAM_PIXMAP_AVAIL < Color160x192PixelRowBytes * Color160x192Height
#error Available VRAM has become too small for Color160x192 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Color160x192Info = {
    Color160x192WidthSamples / 4, Color160x192Height,
    VideoPixmapFormat::PALETTE_8BIT,
    256,
    1,
    0,
    900, 520,
};

void Color160x192Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 1);
    MakeDefaultPalette(0, 256);
    for(int y = 0; y < Color160x192Height; y++) {
        NTSCGetPaletteForRowPointer()[y] = 0;
    }
}

void Color160x192GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width;
}

int Color160x192SetPaletteEntry(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    if(which >= 256) {
        return 1;
    }
    SetPaletteEntry(palette, which, r, g, b);
    return 0;
}

void Color160x192FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Color160x192Top;

    if((rowWithin >= 0) && (rowWithin < Color160x192Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Color160x192WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Color160x192Left + Color160x192WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *srcPixels = VRAM + rowWithin * (Color160x192WidthSamples / 4);
        unsigned char *dstBytes = rowBuffer + Color160x192Left;

        ntsc_wave_t *palette = NTSCGetPaletteForRow(rowWithin);

        for(int i = 0; i < Color160x192WidthSamples; i += 4) {
            int p = srcPixels[i / 4];
            ntsc_wave_t word = palette[p];
            *(ntsc_wave_t*)(dstBytes + i) = word;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// 350 x 230 4-bit bitmap display
// to be /4, 164, 864, 27, 257

#define Color350x230Left 164
#define Color350x230WidthSamples 700
#define Color350x230WidthPixels (Color350x230WidthSamples / 2)
#define Color350x230PixelRowBytes (Color350x230WidthPixels / 2)
#define Color350x230Top 27
#define Color350x230Height 230

#if VRAM_PIXMAP_AVAIL < Color350x230PixelRowBytes * Color350x230Height
#error Available VRAM has become too small for Color350x230 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Color350x230Info = {
    Color350x230WidthSamples / 2, Color350x230Height,
    VideoPixmapFormat::PALETTE_4BIT,
    16,
    1,
    1,
    450, 520,
};

void Color350x230Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 1);
    MakeDefaultPalette(0, 16);
    for(int y = 0; y < Color350x230Height; y++) {
        NTSCGetPaletteForRowPointer()[y] = 0;
    }
}

void Color350x230GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width / 2;
}

int Color350x230SetPaletteEntry(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    if(which >= 16) {
        return 1;
    }
    SetPaletteEntry(palette, which, r, g, b);
    return 0;
}

void Color350x230FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Color350x230Top;

    if((rowWithin >= 0) && (rowWithin < Color350x230Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Color350x230WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Color350x230Left + Color350x230WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *srcPixels = VRAM + rowWithin * (Color350x230WidthSamples / 4);
        unsigned char *dstBytes = rowBuffer + Color350x230Left;

        ntsc_wave_t *palette = NTSCGetPaletteForRow(rowWithin);

        for(int i = 0; i < Color350x230WidthSamples / 2; i++) { /* loop over pixels */

            int colWithin = i;
            int whichByte = colWithin / 2;
            int whichNybble = colWithin % 2;
            int p = (srcPixels[whichByte] >> (whichNybble * 4)) & 0xF;

            ntsc_wave_t word = palette[p];
            uint32_t waveWordShift = whichNybble * 16;

            *dstBytes++ = (word >> (waveWordShift + 0)) & 0xFF;
            *dstBytes++ = (word >> (waveWordShift + 8)) & 0xFF;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// 320 x 192 4-bit bitmap display
// 196, 832, 44, 236

#define Color320x192Left 196
#define Color320x192WidthSamples 640
#define Color320x192WidthPixels (Color320x192WidthSamples / 2)
#define Color320x192PixelRowBytes (Color320x192WidthPixels / 2)
#define Color320x192Top 44
#define Color320x192Height 192

#if VRAM_PIXMAP_AVAIL < Color320x192PixelRowBytes * Color320x192Height
#error Available VRAM has become too small for Color320x192 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Color320x192Info = {
    Color320x192WidthSamples / 2, Color320x192Height,
    VideoPixmapFormat::PALETTE_4BIT,
    16,
    1,
    0,
    450, 520,
};

void Color320x192Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 1);
    MakeDefaultPalette(0, 16);
    for(int y = 0; y < Color320x192Height; y++) {
        NTSCGetPaletteForRowPointer()[y] = 0;
    }
}

void Color320x192GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width / 2;
}

int Color320x192SetPaletteEntry(const VideoModeEntry* modeEntry, int palette, int which, float r, float g, float b)
{
    if(which >= 16) {
        return 1;
    }
    SetPaletteEntry(palette, which, r, g, b);
    return 0;
}

void Color320x192FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Color320x192Top;

    if((rowWithin >= 0) && (rowWithin < Color320x192Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Color320x192WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Color320x192Left + Color320x192WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *srcPixels = VRAM + rowWithin * (Color320x192WidthSamples / 4);
        unsigned char *dstBytes = rowBuffer + Color320x192Left;

        ntsc_wave_t *palette = NTSCGetPaletteForRow(rowWithin);

        for(int i = 0; i < Color320x192WidthSamples / 2; i++) { /* loop over pixels */

            int colWithin = i;
            int whichByte = colWithin / 2;
            int whichNybble = colWithin % 2;
            int p = (srcPixels[whichByte] >> (whichNybble * 4)) & 0xF;

            ntsc_wave_t word = palette[p];
            uint32_t waveWordShift = whichNybble * 16;

            *dstBytes++ = (word >> (waveWordShift + 0)) & 0xFF;
            *dstBytes++ = (word >> (waveWordShift + 8)) & 0xFF;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// 704 x 230 4-gray pixmap display
// to be /8, 164, 868, 27, 257

#define Grayscale704x230Left 164
#define Grayscale704x230WidthSamples 704
#define Grayscale704x230WidthPixels Grayscale704x230WidthSamples
#define Grayscale704x230PixelRowBytes (Grayscale704x230WidthPixels / 4)
#define Grayscale704x230Top 27
#define Grayscale704x230Height 230

#if VRAM_SIZE < Grayscale704x230PixelRowBytes * Grayscale704x230Height
#error Available VRAM has become too small for Grayscale704x230 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Grayscale704x230Info = {
    Grayscale704x230WidthSamples, Grayscale704x230Height,
    VideoPixmapFormat::GRAY_2BIT,
    -1,
    0,
    1,
    225, 520,
};

void Grayscale704x230Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 0);        /* grayscale mode */
}

void Grayscale704x230GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width / 4;
}

void Grayscale704x230FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Grayscale704x230Top;

    if((rowWithin >= 0) && (rowWithin < Grayscale704x230Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Grayscale704x230WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Grayscale704x230Left + Grayscale704x230WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *bitmapRow = VRAM + rowWithin * (Grayscale704x230WidthSamples / 4);
        ntsc_wave_t *dstWords = (ntsc_wave_t*)(rowBuffer + Grayscale704x230Left);

        /* 1 byte is 4 pixels and each pixel is one samples wide so 1 byte yields 4 samples */
        for(int i = 0; i < Grayscale704x230WidthSamples / 4; i++) {
            unsigned char byte = *bitmapRow++;

            unsigned char sample0 = NTSCBlack + (NTSCWhite - NTSCBlack) * ((byte >> 0) & 0x03) / 0x03;
            unsigned char sample1 = NTSCBlack + (NTSCWhite - NTSCBlack) * ((byte >> 2) & 0x03) / 0x03;
            unsigned char sample2 = NTSCBlack + (NTSCWhite - NTSCBlack) * ((byte >> 4) & 0x03) / 0x03;
            unsigned char sample3 = NTSCBlack + (NTSCWhite - NTSCBlack) * ((byte >> 6) & 0x03) / 0x03;

            *dstWords++ = (sample3 << 24) | (sample2 << 16) | (sample1 << 8) | sample0;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// 704 x 230 monochrome bitmap display
// to be /8, 164, 868, 27, 257

#define Bitmap704x230Left 164
#define Bitmap704x230WidthSamples 704
#define Bitmap704x230WidthPixels Bitmap704x230WidthSamples
#define Bitmap704x230PixelRowBytes (Bitmap704x230WidthPixels / 8)
#define Bitmap704x230Top 27
#define Bitmap704x230Height 230

#if VRAM_SIZE < Bitmap704x230PixelRowBytes * Bitmap704x230Height
#error Available VRAM has become too small for Bitmap704x230 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Bitmap704x230Info = {
    Bitmap704x230WidthSamples, Bitmap704x230Height,
    VideoPixmapFormat::BITMAP,
    -1,
    0,
    1,
    225, 520,
};

void Bitmap704x230Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 0);        /* monochrome text */
}

void Bitmap704x230GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width / 8;
}

uint32_t SECTION_CCMRAM NybblesToMasks[16] = {
    0x00000000,
    0x000000FF,
    0x0000FF00,
    0x0000FFFF,
    0x00FF0000,
    0x00FF00FF,
    0x00FFFF00,
    0x00FFFFFF,
    0xFF000000,
    0xFF0000FF,
    0xFF00FF00,
    0xFF00FFFF,
    0xFFFF0000,
    0xFFFF00FF,
    0xFFFFFF00,
    0xFFFFFFFF,
};

void Bitmap704x230FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Bitmap704x230Top;

    if((rowWithin >= 0) && (rowWithin < Bitmap704x230Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Bitmap704x230WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Bitmap704x230Left + Bitmap704x230WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *bitmapRow = VRAM + rowWithin * (Bitmap704x230WidthSamples / 8);
        ntsc_wave_t *dstWords = (ntsc_wave_t*)(rowBuffer + Bitmap704x230Left);

        ntsc_wave_t whiteLong = (NTSCWhite << 24) | (NTSCWhite << 16) | (NTSCWhite << 8) | (NTSCWhite << 0);
        ntsc_wave_t blackLong = (NTSCBlack << 24) | (NTSCBlack << 16) | (NTSCBlack << 8) | (NTSCBlack << 0);

        for(int i = 0; i < Bitmap704x230WidthSamples; i += 8) {
            int whiteMask = 0;
            unsigned char byte = bitmapRow[i / 8];
            whiteMask = NybblesToMasks[byte & 0xF];
            *dstWords++ = (whiteLong & whiteMask) | (blackLong & ~whiteMask);
            whiteMask = NybblesToMasks[(byte >> 4) & 0xF];
            *dstWords++ = (whiteLong & whiteMask) | (blackLong & ~whiteMask);
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

// ----------------------------------------
// 640 x 192 monochrome bitmap display
// 196, 832, 44, 236

#define Bitmap640x192Left 196
#define Bitmap640x192WidthSamples 640
#define Bitmap640x192WidthPixels Bitmap640x192WidthSamples
#define Bitmap640x192PixelRowBytes (Bitmap640x192WidthPixels / 8)
#define Bitmap640x192Top 44
#define Bitmap640x192Height 192

#if VRAM_SIZE < Bitmap640x192PixelRowBytes * Bitmap640x192Height
#error Available VRAM has become too small for Bitmap640x192 mode.
#endif

// Pixmap video mode structs
const static VideoPixmapInfo Bitmap640x192Info = {
    Bitmap640x192WidthSamples, Bitmap640x192Height,
    VideoPixmapFormat::BITMAP,
    -1,
    0,
    0,
    225, 520,
};

void Bitmap640x192Setup(const VideoModeEntry *modeEntry)
{
    /* const VideoTextportInfo *info = (VideoTextportInfo*)info_; */
    NTSCFillBlankLine(NTSCBlankLine, 0);        /* monochrome text */
}

void Bitmap640x192GetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    const VideoPixmapInfo *info = (VideoPixmapInfo*)modeEntry->info;
    VideoPixmapParameters *params = (VideoPixmapParameters*)params_;
    params->base = VRAM;
    params->rowSize = info->width / 8;
}

void Bitmap640x192FillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) - Bitmap640x192Top;

    if((rowWithin >= 0) && (rowWithin < Bitmap640x192Height)) {

        // Clear pixels outside of text area 
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, Bitmap640x192WidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = Bitmap640x192Left + Bitmap640x192WidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        unsigned char *bitmapRow = VRAM + rowWithin * (Bitmap640x192WidthSamples / 8);
        ntsc_wave_t *dstWords = (ntsc_wave_t*)(rowBuffer + Bitmap640x192Left);

        ntsc_wave_t whiteLong = (NTSCWhite << 24) | (NTSCWhite << 16) | (NTSCWhite << 8) | (NTSCWhite << 0);
        ntsc_wave_t blackLong = (NTSCBlack << 24) | (NTSCBlack << 16) | (NTSCBlack << 8) | (NTSCBlack << 0);

        for(int i = 0; i < Bitmap640x192WidthSamples; i += 8) {
            int whiteMask = 0;
            unsigned char byte = bitmapRow[i / 8];
            whiteMask = NybblesToMasks[byte & 0xF];
            *dstWords++ = (whiteLong & whiteMask) | (blackLong & ~whiteMask);
            whiteMask = NybblesToMasks[(byte >> 4) & 0xF];
            *dstWords++ = (whiteLong & whiteMask) | (blackLong & ~whiteMask);
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}

//--------------------------------------------------------------------------
// Mode where scanlines are composed of sequential segments

// WidthSamples *and* Left are Expected to be multiple of 4 by FillRow
#define SegmentedVideoWidthSamples 704
#define SegmentedVideoLeft (512 - SegmentedVideoWidthSamples / 2)
#define SegmentedVideoTop (27 * 2)
#define SegmentedVideoHeight (230 * 2)

#define SegmentedVideoEvenTop (SegmentedVideoTop / 2)
#define SegmentedVideoEvenBottom (SegmentedVideoTop / 2 + SegmentedVideoHeight / 2)
#define SegmentedVideoOddTop (NTSC_FRAME_LINES / 2 + SegmentedVideoTop / 2 + 1)
#define SegmentedVideoOddBottom (NTSC_FRAME_LINES / 2 + SegmentedVideoTop / 2 + SegmentedVideoHeight / 2 + 1)

// Pixmap video mode structs
const static VideoSegmentedInfo SegmentedInfo = {
    SegmentedVideoWidthSamples,
    SegmentedVideoHeight,
    SegmentedVideoWidthSamples,
    SegmentedVideoHeight,
};

int SegmentedSetScanlines(int scanlineCount, VideoSegmentedScanline *scanlines);

void SegmentedGetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    VideoSegmentedParameters *params = (VideoSegmentedParameters*)params_;
    params->setScanlines = SegmentedSetScanlines;
}

#pragma pack(push, 2)
typedef struct SegmentedSegmentPrivate {
    uint16_t pixelCount;
    uint16_t y0, i0, q0;        // Fixed 12
    uint16_t dy, di, dq;        // Fixed 12
} SegmentedSegmentPrivate;
#pragma pack(pop)

int16_t FloatToFixed12(float f)
{
    return f * 4096;
}

float Fixed12ToFloat(int16_t u)
{
    return u / 4096.0f;
}

typedef struct SegmentedVideoPrivate {
    SegmentedSegmentPrivate *scanlineSegments[SegmentedVideoHeight];
} SegmentedVideoPrivate;

#define MAX_SEGMENT_COUNT ((VRAM_SIZE - sizeof(SegmentedVideoPrivate)) / sizeof(SegmentedSegmentPrivate))

void SegmentedSetup(const VideoModeEntry *modeEntry)
{
    NTSCFillBlankLine(NTSCBlankLine, 1);

    SegmentedVideoPrivate *pvt = (SegmentedVideoPrivate *)VRAM;
    SegmentedSegmentPrivate *dstseg = (SegmentedSegmentPrivate *)(VRAM + sizeof(SegmentedVideoPrivate));

    // If we're in vertical retrace, wait to enter visible area
    while(
        !((lineNumber > SegmentedVideoEvenTop && lineNumber < SegmentedVideoEvenBottom) ||
        (lineNumber > SegmentedVideoOddTop && lineNumber < SegmentedVideoOddBottom)));
    // Once we're in visible area, wait to come around again to beginning of vertical retrace
    while(
        (lineNumber > SegmentedVideoEvenTop && lineNumber < SegmentedVideoEvenBottom) ||
        (lineNumber > SegmentedVideoOddTop && lineNumber < SegmentedVideoOddBottom));

    for(int i = 0; i < SegmentedVideoHeight; i++) {
        pvt->scanlineSegments[i] = dstseg;
        dstseg->pixelCount = SegmentedVideoWidthSamples;
        // RGBToYIQ(0, .5, 0, &dstseg->y0, &dstseg->i0, &dstseg->q0); // XXX why did this hang?
        dstseg->y0 = FloatToFixed12(.295);
        dstseg->i0 = FloatToFixed12(-.13865);
        dstseg->q0 = FloatToFixed12(-.26255);
        dstseg->dy = FloatToFixed12(0);
        dstseg->di = FloatToFixed12(0);
        dstseg->dq = FloatToFixed12(0);
        dstseg++;
    }
}

// All scanlines must have 1 or more segments and cover only every pixel on scanline
int SegmentedSetScanlines(int scanlineCount, VideoSegmentedScanline *scanlines)
{
    if(scanlineCount != SegmentedVideoHeight) {
        return 1; // INVALID - incorrect number of scanlines
    }
    size_t totalSegments = 0;
    for(int i = 0; i < scanlineCount; i++) {
        totalSegments += scanlines[i].segmentCount;
        int totalPixels = 0;
        for(int j = 0; j < scanlines[i].segmentCount; j++) {
            totalPixels += scanlines[i].segments[j].pixelCount;
        }
        if(totalPixels < SegmentedVideoWidthSamples) {
            return 3; // INVALID - didn't cover line
        }
        if(totalPixels > SegmentedVideoWidthSamples) {
            return 4; // INVALID - exceeded length of line
        }
    }
    if(totalSegments > MAX_SEGMENT_COUNT - 1) {
        return 2; // INVALID - too many segments
    }

    SegmentedVideoPrivate *pvt = (SegmentedVideoPrivate *)VRAM;
    SegmentedSegmentPrivate *dstseg = (SegmentedSegmentPrivate *)(VRAM + sizeof(SegmentedVideoPrivate));
    SegmentedSegmentPrivate *stopper = dstseg + MAX_SEGMENT_COUNT - 1;

    stopper->pixelCount = SegmentedVideoWidthSamples;
    float Y, I, Q;
    RGBToYIQ(1.0f, 0.0f, 0.0f, &Y, &I, &Q);
    stopper->y0 = FloatToFixed12(Y);
    stopper->i0 = FloatToFixed12(I);
    stopper->q0 = FloatToFixed12(Q);
    stopper->dy = FloatToFixed12(0);
    stopper->di = FloatToFixed12(0);
    stopper->dq = FloatToFixed12(0);

    // Hope that we can update this entire buffer in the non-visible region!
    // Roughly 390000 cycles or around 850 cycles per row, probably
    // enough to convert frame as long we don't do per-line divides
    // or trig.
    // If we're in vertical retrace, wait to enter visible area
    if(1) {
        while(
            !((lineNumber > SegmentedVideoEvenTop && lineNumber < SegmentedVideoEvenBottom) ||
            (lineNumber > SegmentedVideoOddTop && lineNumber < SegmentedVideoOddBottom)));
        // Once we're in visible area, wait to come around again to beginning of vertical retrace
        while(
            (lineNumber > SegmentedVideoEvenTop && lineNumber < SegmentedVideoEvenBottom) ||
            (lineNumber > SegmentedVideoOddTop && lineNumber < SegmentedVideoOddBottom));
    }

    for(int i = 0; i < scanlineCount; i++) {
        SegmentedSegmentPrivate *actual = dstseg;
        pvt->scanlineSegments[i] = stopper; // Hopefully we'll see a glitch instead of a crash
        // printf("scanline %d, %d count\n", i, scanlines[i].segmentCount);  SERIAL_flush();
        if(1) {
            for(int j = 0; j < scanlines[i].segmentCount; j++) {
                VideoSegmentedScanlineSegment *srcseg = scanlines[i].segments + j;
                dstseg->pixelCount = srcseg->pixelCount;
                // printf("    segment %d: count %d\n", j, dstseg->pixelCount);  SERIAL_flush();
                float Y0, I0, Q0;
                float Y1, I1, Q1;
                // XXX type of segment
                RGBToYIQ(srcseg->g.r0, srcseg->g.g0, srcseg->g.b0, &Y0, &I0, &Q0);
                RGBToYIQ(srcseg->g.r1, srcseg->g.g1, srcseg->g.b1, &Y1, &I1, &Q1);
                dstseg->y0 = FloatToFixed12(Y0);
                dstseg->i0 = FloatToFixed12(I0);
                dstseg->q0 = FloatToFixed12(Q0);
                dstseg->dy = FloatToFixed12((Y1 - Y0) / srcseg->pixelCount);
                dstseg->di = FloatToFixed12((I1 - I0) / srcseg->pixelCount);
                dstseg->dq = FloatToFixed12((Q1 - Q0) / srcseg->pixelCount);
                dstseg ++;
            }
            pvt->scanlineSegments[i] = actual;
        }
    }
    // printf("line number at end: %d\n", lineNumber);

    return 0;
}

__attribute__((hot,flatten)) void SegmentedFillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) * 2 + lineNumber / 263 - SegmentedVideoTop;

    if((rowWithin >= 0) && (rowWithin < SegmentedVideoHeight)) {

        // Clear pixels outside of area 
        // XXX These two memsets may be as much as 10% of scanline render time
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, SegmentedVideoWidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = SegmentedVideoLeft + SegmentedVideoWidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        SegmentedVideoPrivate *pvt = (SegmentedVideoPrivate *)VRAM;

        SegmentedSegmentPrivate *segmentp = pvt->scanlineSegments[rowWithin];

        unsigned char *dstSamples = rowBuffer + SegmentedVideoLeft;
        int pixelsRemaining = SegmentedVideoWidthSamples;
        while(pixelsRemaining > 0) {
            int pixelCount = segmentp->pixelCount;
            float Y = Fixed12ToFloat(segmentp->y0);
            float I = Fixed12ToFloat(segmentp->i0);
            float Q = Fixed12ToFloat(segmentp->q0);
            float dy = Fixed12ToFloat(segmentp->dy);
            float di = Fixed12ToFloat(segmentp->di);
            float dq = Fixed12ToFloat(segmentp->dq);

            int pixel = SegmentedVideoWidthSamples - pixelsRemaining;
            while((pixelCount > 0) && ((pixel % 4) != 0)) {
                uint32_t value = NTSCYIQDegreesToDAC(Y, I, Q, (pixel % 4) * 90);
                *dstSamples++ = value;
                Y += dy; I += di; Q += dq;
                pixelCount --;
                pixel++;
            }

            for(int i = 0; i < pixelCount / 4; i++) {

                ntsc_wave_t waveform = NTSCYIQDegreesToDAC(Y, I, Q, 0) << 0;
                Y += dy; I += di; Q += dq;

                waveform |= NTSCYIQDegreesToDAC(Y, I, Q, 90) << 8;
                Y += dy; I += di; Q += dq;

                waveform |= NTSCYIQDegreesToDAC(Y, I, Q, 180) << 16;
                Y += dy; I += di; Q += dq;

                waveform |= NTSCYIQDegreesToDAC(Y, I, Q, 270) << 24;
                Y += dy; I += di; Q += dq;

                *(ntsc_wave_t*)dstSamples = waveform;
                dstSamples += 4;
            }

            pixelCount = pixelCount % 4;

            pixel = 0;
            while(pixelCount > 0) {
                uint32_t value = NTSCYIQDegreesToDAC(Y, I, Q, (pixel % 4) * 90);
                *dstSamples++ = value;
                Y += dy; I += di; Q += dq;
                pixelCount --;
                pixel++;
            }

            pixelsRemaining -= segmentp->pixelCount;
            segmentp++;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}


// ----------------------------------------
// Wolfenstein-style renderer

#define WolfensteinWidthSamples 704
#define WolfensteinLeft (512 - WolfensteinWidthSamples / 2)
#define WolfensteinTop (27 * 2)
#define WolfensteinHeight (230 * 2)

// Pixmap video mode structs
const static VideoWolfensteinInfo WolfensteinInfo = {
    WolfensteinWidthSamples,
    WolfensteinHeight,
};

#define WolfensteinTextureWidth 64
#define WolfensteinTextureHeight 64

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern const uint32_t *Textures[8];

#ifdef __cplusplus
};
#endif /* __cplusplus */


typedef struct WolfensteinPrivateElement {
    const ntsc_wave_t *textureColumnBase;
    int wholePixelsHalfHeight;
    uint32_t textureRowOffsetFixed16;
    uint32_t textureRowPerScanRowFixed16;
    uint32_t bright;
} WolfensteinPrivateElement;


float clamp(float x, float a, float b)
{
    return (x < a) ? a : ((x > b) ? b : x);
}

void WolfensteinSetElements(VideoWolfensteinElement* row);

void WolfensteinGetParameters(const VideoModeEntry *modeEntry, void *params_)
{
    //const VideoWolfensteinInfo *info = (VideoWolfensteinInfo*)modeEntry->info;
    VideoWolfensteinParameters *params = (VideoWolfensteinParameters*)params_;
    params->setElements = WolfensteinSetElements;
}

void WolfensteinSetup(const VideoModeEntry *modeEntry)
{
    NTSCFillBlankLine(NTSCBlankLine, 1);
    WolfensteinPrivateElement *pvt = (WolfensteinPrivateElement *)VRAM;
    for(int i = 0; i < WolfensteinWidthSamples; i++) {
        pvt[i].wholePixelsHalfHeight = 20;
        pvt[i].bright = 0;
        pvt[i].textureRowPerScanRowFixed16 = 0;
        pvt[i].textureRowOffsetFixed16 = 0;
        pvt[i].textureColumnBase = Textures[0];
    }
}

// Must be "width" elements
void WolfensteinSetElements(VideoWolfensteinElement* row)
{
    WolfensteinPrivateElement *pvt = (WolfensteinPrivateElement *)VRAM;
    // XXX race condition - need to swap between buffers atomically
    for(int i = 0; i < WolfensteinWidthSamples; i++) {
        pvt[i].bright = row[i].bright * 0x10101010;
        float pixelsHalfHeight = row[i].height / 2.0f * WolfensteinHeight;
        pvt[i].wholePixelsHalfHeight = roundf(pixelsHalfHeight);
        if(pvt[i].wholePixelsHalfHeight == 0) {
            pvt[i].textureRowPerScanRowFixed16 = 0;
        } else {
            pvt[i].textureRowPerScanRowFixed16 = 65536 * WolfensteinTextureHeight / (pixelsHalfHeight * 2);
        }
        pvt[i].textureRowOffsetFixed16 = pvt[i].textureRowPerScanRowFixed16 * (row[i].height / 2.0f * WolfensteinHeight - (pvt[i].wholePixelsHalfHeight - .5));
        float s = clamp(row[i].textureS, 0, .99999);

        int lod = 5;
        int offset = 0;
        int mipStorageHeight = 64;
        while((lod > 0) && (pvt[i].textureRowPerScanRowFixed16 > 65536) /* && (pvt[i].dsdx > 1.0f) */ ) {
            offset += WolfensteinTextureWidth * mipStorageHeight;
            mipStorageHeight /= 2;
            pvt[i].textureRowPerScanRowFixed16 /= 2;
            pvt[i].textureRowOffsetFixed16 /= 2;
            // pvt[i].dsdx /= 2;
            lod -= 1;
            s /= 2;
        }

        pvt[i].textureColumnBase = Textures[row[i].id] + offset + (int)floorf(s * WolfensteinTextureWidth);

        if(0) printf("%d : %p, %d, %lu\n",
            i,
            pvt[i].textureColumnBase,
            pvt[i].wholePixelsHalfHeight,
            pvt[i].textureRowPerScanRowFixed16);
    }
}

void WolfensteinFillRow(int frameNumber, int lineNumber, unsigned char *rowBuffer)
{
    int rowWithin = (lineNumber % 263) * 2 + lineNumber / 263 - WolfensteinTop;

    if((rowWithin >= 0) && (rowWithin < WolfensteinHeight)) {

        // Clear pixels outside of area 
        // XXX These two memsets may be as much as 10% of scanline render time.
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, WolfensteinWidthSamples - NTSCHSyncClocks - NTSCBackPorchClocks);
        int clocksToRightEdge = WolfensteinLeft + WolfensteinWidthSamples;
        memset(rowBuffer + clocksToRightEdge, NTSCBlack, ROW_SAMPLES - clocksToRightEdge - NTSCFrontPorchClocks);

        int rowFromMiddle = rowWithin - WolfensteinHeight / 2;

        ntsc_wave_t *dstWords = (ntsc_wave_t*)(rowBuffer + WolfensteinLeft);
        WolfensteinPrivateElement *el = (WolfensteinPrivateElement *)VRAM;

        for(int i = 0; i < WolfensteinWidthSamples; i += 4, el += 4) {
            ntsc_wave_t waveform;
            int withinWall = rowFromMiddle + el[0].wholePixelsHalfHeight;
            if(withinWall >= 0 && rowFromMiddle < el[0].wholePixelsHalfHeight) {
                int textureRow = (withinWall * el[0].textureRowPerScanRowFixed16 + el[0].textureRowOffsetFixed16) / 65536;
                waveform = (el[0].textureColumnBase[textureRow * WolfensteinTextureWidth] + el[0].bright) & 0x000000FF;
            } else {
                waveform = 0x00000070;
            }
            withinWall = rowFromMiddle + el[1].wholePixelsHalfHeight;
            if(withinWall >= 0 && rowFromMiddle < el[1].wholePixelsHalfHeight) {
                int textureRow = (withinWall * el[1].textureRowPerScanRowFixed16 + el[1].textureRowOffsetFixed16) / 65536;
                waveform |= (el[1].textureColumnBase[textureRow * WolfensteinTextureWidth] + el[1].bright) & 0x0000FF00;
            } else {
                waveform |= 0x00007000;
            }
            withinWall = rowFromMiddle + el[2].wholePixelsHalfHeight;
            if(withinWall >= 0 && rowFromMiddle < el[2].wholePixelsHalfHeight) {
                int textureRow = (withinWall * el[2].textureRowPerScanRowFixed16 + el[2].textureRowOffsetFixed16) / 65536;
                waveform |= (el[2].textureColumnBase[textureRow * WolfensteinTextureWidth] + el[2].bright) & 0x00FF0000;
            } else {
                waveform |= 0x00700000;
            }
            withinWall = rowFromMiddle + el[3].wholePixelsHalfHeight;
            if(withinWall >= 0 && rowFromMiddle < el[3].wholePixelsHalfHeight) {
                int textureRow = (withinWall * el[3].textureRowPerScanRowFixed16 + el[3].textureRowOffsetFixed16) / 65536;
                waveform |= (el[3].textureColumnBase[textureRow * WolfensteinTextureWidth] + el[3].bright) & 0xFF000000;
            } else {
                waveform |= 0x70000000;
            }
            *dstWords++ = waveform;
        }
    } else {
        memset(rowBuffer + NTSCHSyncClocks + NTSCBackPorchClocks, NTSCBlack, ROW_SAMPLES - NTSCHSyncClocks - NTSCBackPorchClocks - NTSCFrontPorchClocks);
    }
}


// ----------------------------------------
// Video mode table

const static VideoModeEntry NTSCModes[] =
{
    {
        VIDEO_MODE_TEXTPORT,
        &TextportPlain40x24Info,
        TextportPlain40x24Setup,
        TextportPlain40x24GetParameters,
        TextportPlain40x24FillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_TEXTPORT,
        &TextportPlain80x24Info,
        TextportPlain80x24Setup,
        TextportPlain80x24GetParameters,
        TextportPlain80x24FillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Bitmap640x192Info,
        Bitmap640x192Setup,
        Bitmap640x192GetParameters,
        Bitmap640x192FillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Color320x192Info,
        Color320x192Setup,
        Color320x192GetParameters,
        Color320x192FillRow,
        Color320x192SetPaletteEntry,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Color160x192Info,
        Color160x192Setup,
        Color160x192GetParameters,
        Color160x192FillRow,
        Color160x192SetPaletteEntry,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Bitmap704x230Info,
        Bitmap704x230Setup,
        Bitmap704x230GetParameters,
        Bitmap704x230FillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Color350x230Info,
        Color350x230Setup,
        Color350x230GetParameters,
        Color350x230FillRow,
        Color350x230SetPaletteEntry,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Color175x230Info,
        Color175x230Setup,
        Color175x230GetParameters,
        Color175x230FillRow,
        Color175x230SetPaletteEntry,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &ColorMaxResInfo,
        ColorMaxResSetup,
        ColorMaxResGetParameters,
        ColorMaxResFillRow,
        ColorMaxResSetPaletteEntry,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_PIXMAP,
        &Grayscale704x230Info,
        Grayscale704x230Setup,
        Grayscale704x230GetParameters,
        Grayscale704x230FillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_WOLFENSTEIN,
        &WolfensteinInfo,
        WolfensteinSetup,
        WolfensteinGetParameters,
        WolfensteinFillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
    {
        VIDEO_MODE_SEGMENTS,
        &SegmentedInfo,
        SegmentedSetup,
        SegmentedGetParameters,
        SegmentedFillRow,
        SetPaletteEntryAlwaysFails,
        SetRowPalette,
        NULL,
    },
};

// Generic videomode functions

int SECTION_CCMRAM CurrentVideoMode = -1;

int VideoGetModeCount()
{
    return sizeof(NTSCModes) / sizeof(NTSCModes[0]);
}

enum VideoModeType VideoModeGetType(int n)
{
    return NTSCModes[n].type;
}

void VideoModeGetInfo(int n, void *info)
{
    const VideoModeEntry* entry = NTSCModes + n;
    switch(entry->type) {
        case VIDEO_MODE_PIXMAP: {
            *(VideoPixmapInfo*)info = *(VideoPixmapInfo*)entry->info;
            break;
        }
        case VIDEO_MODE_TEXTPORT: {
            *(VideoTextportInfo*)info = *(VideoTextportInfo*)entry->info;
            break;
        }
        case VIDEO_MODE_SEGMENTS: {
            *(VideoSegmentedInfo*)info = *(VideoSegmentedInfo*)entry->info;
            break;
            break;
        }
        case VIDEO_MODE_DCT: {
            break;
        }
        case VIDEO_MODE_WOZ: {
            break;
        }
        case VIDEO_MODE_TMS9918A: {
            break;
        }
        case VIDEO_MODE_VES: {
            break;
        }
        case VIDEO_MODE_WOLFENSTEIN: {
            *(VideoWolfensteinInfo*)info = *(VideoWolfensteinInfo*)entry->info;
            break;
        }
    }
}

void VideoSetMode(int n)
{
    NTSCMode = NTSC_DISPLAY_BLACK;
    const VideoModeEntry* entry = NTSCModes + n;
    entry->setup(entry);
    CurrentVideoMode = n;
    VideoCurrentFillRow = entry->fillRow;
    NTSCMode = NTSC_USE_VIDEO_MODE;
}

int VideoGetCurrentMode()
{
    return CurrentVideoMode;
}

void VideoModeGetParameters(void *params)
{
    const VideoModeEntry* entry = NTSCModes + CurrentVideoMode;
    entry->getParameters(entry, params);
}

int VideoModeSetPaletteEntry(int palette, int which, float r, float g, float b)
{
    const VideoModeEntry* entry = NTSCModes + CurrentVideoMode;
    return entry->setPaletteEntry(entry, palette, which, r, g, b);
}

int VideoModeSetRowPalette(int row, int palette)
{
    const VideoModeEntry* entry = NTSCModes + CurrentVideoMode;
    return entry->setRowPalette(entry, row, palette);
}

void VideoModeWaitFrame()
{
    // NTSC won't actually go lineNumber >= 525...
    while(!(lineNumber > 257 && lineNumber < 262) || (lineNumber > 520 && lineNumber < NTSC_FRAME_LINES)); // Wait for VBLANK; should do something smarter
}

//----------------------------------------------------------------------------
// Text command interface

int showScanoutStats = 0;

void NTSCGenerateLineBuffers();

enum { CIRCLE_COUNT = 10 };

int doTestSegmentMode(int wordCount, char **words)
{
    int which = -1;
    for(int i = 0; i < VideoGetModeCount(); i++) {
        if(VideoModeGetType(i) == VIDEO_MODE_SEGMENTS) {
            which = i;
        }
    }

    if(which == -1) {
        printf("Couldn't find segments video mode\n");
        return COMMAND_FAILED;
    }

    VideoSetMode(which);

    VideoSegmentedInfo info;
    VideoSegmentedParameters params;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    printf("segmented mode is %d by %d pixels\n", info.width, info.height); SERIAL_flush();

    static VideoSegmentBuffer buffer;

    int result = VideoBufferAllocateMembers(&buffer, info.width, 4000, info.height, .2f, .15f, .15f);
    if(result != 0) {
        printf("failed to allocate video buffer with 4000 segments, result = %d\n", result);
        result = VideoBufferAllocateMembers(&buffer, info.width, 3500, info.height, .2f, .15f, .15f);
        if(result != 0) {
            printf("failed to allocate video buffer with 3500 segments, result = %d\n", result);
            result = VideoBufferAllocateMembers(&buffer, info.width, 3000, info.height, .2f, .15f, .15f);
            if(result != 0) {
                printf("failed to allocate video buffer with 3000 segments, result = %d\n", result);
                result = VideoBufferAllocateMembers(&buffer, info.width, 2000, info.height, .2f, .15f, .15f);
                if(result != 0) {
                    printf("failed to allocate video buffer with 2000 segments, result = %d\n", result);
                    return COMMAND_FAILED;
                }
            }
        }
    }

    static int dx[CIRCLE_COUNT];
    static int dy[CIRCLE_COUNT]; 
    static int cx[CIRCLE_COUNT];
    static int cy[CIRCLE_COUNT]; 
    static int cr[CIRCLE_COUNT];
    static int degrees[CIRCLE_COUNT];
    static int whatgradient[CIRCLE_COUNT];
    static GradientDescriptor gradients[CIRCLE_COUNT];
    int quit = 0;

    for(int i = 0; i < CIRCLE_COUNT; i++) {
        dx[i] = (rand() % 2 == 0) ? -(2 + rand() % 5) : (2 + rand() % 5);
        dy[i] = (rand() % 2 == 0) ? -(2 + rand() % 5) : (2 + rand() % 5);
        cx[i] = 10 + (rand() % (info.width - 20));
        cy[i] = 10 + (rand() % (info.height - 20));
        cr[i] = 50 + rand() % 50;
        degrees[i] = rand() % 360;
        whatgradient[i] = rand() % 3;
    }

    while(!quit) {
        VideoBufferReset(&buffer, .2f, .15f, .15f);

        for(int i = 0; i < CIRCLE_COUNT; i++) {
            float x0 = - cosf(degrees[i] / 180.0f * M_PI) * cr[i];
            float y0 = - sinf(degrees[i] / 180.0f * M_PI) * cr[i];
            float x1 = + cosf(degrees[i] / 180.0f * M_PI) * cr[i];
            float y1 = + sinf(degrees[i] / 180.0f * M_PI) * cr[i];
            degrees[i] += 5;
            if(whatgradient[i] == 0) {
                GradientSet(&gradients[i], x0, y0, 1, 0, 0, x1, y1, 0, 1, 0);
            } else if(which == 1) {
                GradientSet(&gradients[i], x0, y0, 0, 1, 0, x1, y1, 0, 0, 1);
            } else {
                GradientSet(&gradients[i], x0, y0, 0, 0, 1, x1, y1, 1, 0, 0);
            }

            int result = CircleToSegmentsGradient(&buffer, cx[i], cy[i], cr[i], &gradients[i]);
            if(result != 0) {
                printf("Return value %d from CircleToSegments\n", result);
                VideoBufferFreeMembers(&buffer);
                return COMMAND_FAILED;
            }
        }

        if(1) {
            result = params.setScanlines(info.height, buffer.scanlines);
            if(result != 0) {
                printf("Return value %d from SegmentedSetScanlines\n", result);
                VideoBufferFreeMembers(&buffer);
                return COMMAND_FAILED;
            }
        }

        if(1)for(int i = 0; i < CIRCLE_COUNT; i++) {
            cx[i] += dx[i];
            cy[i] += dy[i];
            if((cx[i] <= abs(dx[i])) || (cx[i] >= info.width - abs(dx[i]))) {
                dx[i] = -dx[i];
            }
            if((cy[i] <= abs(dy[i])) || (cy[i] >= info.height - abs(dy[i]))) {
                dy[i] = -dy[i];
            }
        }
        quit = (InputGetChar() != -1);
    }

    VideoBufferFreeMembers(&buffer);

    return COMMAND_CONTINUE;
}

static void RegisterCommandTestSegmentMode(void) __attribute__((constructor));
static void RegisterCommandTestSegmentMode(void)
{
    RegisterApp("segtest", 1, doTestSegmentMode, "",
        "test segmented mode"
        );
}

int doCommandTestColor(int wordCount, char **words)
{
    enum VideoModeType type = VideoModeGetType(VideoGetCurrentMode());
    
    if(type != VIDEO_MODE_PIXMAP) {

        printf("current mode is not a pixmap; use \"modes\"\n");
        printf("to list modes and \"pixmap\" to choose a pixmap mode.\n");

    } else {

        VideoPixmapInfo info;
        VideoPixmapParameters params;
        VideoModeGetInfo(VideoGetCurrentMode(), &info);
        VideoModeGetParameters(&params);
        if(info.paletteSize > 0) {
            MakePalette(0, info.paletteSize, NULL);
            for(int y = 0; y < info.height; y++) {
                VideoModeSetRowPalette(y, 0);
            }
        }

        ClearPixmap(0);

        // Yes, my use of rand() here is bad.

        for(int i = 0; i < 100; i++) {
            int cx = 10 + rand() % (info.width - 20);
            int cy = 10 + rand() % (info.height - 20);
            int cr = 5 + rand() % 30;
            int c;
            if(info.paletteSize == -1) {
                if(info.pixelFormat == VideoPixmapFormat::GRAY_2BIT) {
                    c = rand() % 4; 
                } else {
                    c = rand() % 2; 
                }
            } else {
                c = rand() % info.paletteSize;
            }
            DrawFilledCircle(cx, cy, cr, c, info.aspectX, info.aspectY);

            int x0 = 10 + rand() % (info.width - 20);
            int y0 = 10 + rand() % (info.height - 20);
            int x1 = 10 + rand() % (info.width - 20);
            int y1 = 10 + rand() % (info.height - 20);
            DrawLine(x0, y0, x1, y1, c);
        }
    }

    return COMMAND_CONTINUE;
}

static void RegisterCommandTestColor(void) __attribute__((constructor));
static void RegisterCommandTestColor(void)
{
    RegisterApp("testcolor", 1, doCommandTestColor, "",
        "test color modes"
        );
}


int doCommandVideoModes(int wordCount, char **words)
{
    for(int i = 0; i < VideoGetModeCount(); i++) {
        printf("Mode %d\n", i);
        enum VideoModeType type = VideoModeGetType(i);
        switch(type) {
            case VIDEO_MODE_PIXMAP: {
                VideoPixmapInfo info;
                VideoModeGetInfo(i, &info);
                printf("    pixmap, %d by %d\n", info.width, info.height);
                printf("    is %s", info.color ? "color" : "monochrome or grayscale");
                if(info.paletteSize > 0) {
                    printf(", %d palette entries", info.paletteSize);
                }
                printf("\n");
                printf("    %s\n", info.overscan ? "overscan" : "underscan");
                break;
            }
            case VIDEO_MODE_TEXTPORT: {
                VideoTextportInfo info;
                VideoModeGetInfo(i, &info);
                printf("    text, %d by %d\n", info.width, info.height);
                switch(info.attributes) {
                    case TEXT_8BIT_WHITE_ON_BLACK: printf("    8-bit, white on black\n"); break;
                    case TEXT_8BIT_BLACK_ON_WHITE: printf("    8-bit, black on white\n"); break;
                    case TEXT_16BIT_8_3_3_2_RICH: printf("    16-bit, rich formatting\n"); break;
                }
                break;
            }
            case VIDEO_MODE_WOZ: {
                printf("    Woz-type\n");
                break;
            }
            case VIDEO_MODE_TMS9918A: {
                printf("    TMS9918A-type\n");
                break;
            }
            case VIDEO_MODE_SEGMENTS: {
                printf("    real-time segment-renderer-type\n");
                break;
            }
            case VIDEO_MODE_DCT: {
                printf("    DCT-type\n");
                break;
            }
            case VIDEO_MODE_WOLFENSTEIN: {
                printf("    Wolfenstein-type\n");
                break;
            }
            default: {
                printf("    unknown mode type %08X\n", type);
            }
        }
    }
    return COMMAND_CONTINUE;
}

int doCommandTextMode(int wordCount, char **words)
{
    int which = strtol(words[1], NULL, 0);

    if((which < 0) || (which >= VideoGetModeCount())) {
        printf("mode %d is out of range; only %d modes (list modes with \"modes\")\n", which, VideoGetModeCount());
        return COMMAND_FAILED;
    }
    if(VideoModeGetType(which) != VIDEO_MODE_TEXTPORT) {
        printf("mode %d is not a text mode (list modes with \"modes\")\n", which);
        return COMMAND_FAILED;
    }

    TextportSetMode(which);

    return COMMAND_CONTINUE;
}

int doCommandPixmapMode(int wordCount, char **words)
{
    int which = strtol(words[1], NULL, 0);
    if((which < 0) || (which >= VideoGetModeCount())) {
        printf("mode %d is out of range; only %d modes (list modes with \"modes\")\n", which, VideoGetModeCount());
        return COMMAND_FAILED;
    }
    if(VideoModeGetType(which) != VIDEO_MODE_PIXMAP) {
        printf("mode %d is not a pixmap mode (list modes with \"modes\")\n", which);
        return COMMAND_FAILED;
    }
    VideoSetMode(which);
    /* draw a pretty into picture? */
    return COMMAND_CONTINUE;
}

int doCommandStream(int wordCount, char **words)
{
#if 0
    NTSCMode = VIDEO_PALETTIZED;

    const char *formatName = words[1];
    int start = strtol(words[2], NULL, 0);
    int end = strtol(words[3], NULL, 0);

    char formattedName[512];
    float displayTime = 0;
    float endTime = (end - start) / 30.0;
    do {
        int frameNumber = start + (int)(displayTime * 30.0);
        sprintf(formattedName, formatName, frameNumber);
        // Must have been quantized to 256 colors!
        // while(lineNumber < 224); // Wait for VBLANK-ish
        float duration;
        if(readPalettized(formattedName, &duration)) {
            printf("failed to show \"%s\"\n", formattedName); 
            break;
        }
        displayTime += duration;
    } while(displayTime < endTime);
#endif
    return COMMAND_CONTINUE;
}


int doCommandVideoTest(int wordCount, char **words)
{
    NTSCMode = NTSC_COLOR_TEST;
    return COMMAND_CONTINUE;
}

int doCommandScanoutStats(int wordCount, char **words)
{
    showScanoutStats = !showScanoutStats;
    if(!showScanoutStats) {
        debugOverlayEnabled = 0;
    }
    return COMMAND_CONTINUE;
}

int doCommandVideoText(int wordCount, char **words)
{
    gOutputDevices = gOutputDevices ^ OUTPUT_TO_TEXTPORT;
    return COMMAND_CONTINUE;
}

int doCommandShowLineEnd(int wordCount, char **words)
{
    markHandlerInSamples = !markHandlerInSamples;
    return COMMAND_CONTINUE;
}

int doCommandScanoutCycles(int wordCount, char **words)
{
    int cyclesPerLine = SystemCoreClock / 59.94 / 262.5;
    // XXX Use a histogram instead
    int totalRemained = 0;
    for(int i = 0; i < NTSC_FRAME_LINES; i++) {
        printf("row %3d: ", i);
        if(rowDMARemained[i] < 0) {
            printf("overrun by %u%%\n", -rowDMARemained[i] * 100 / ROW_SAMPLES);
        } else { 
            printf("%u%% remained\n", rowDMARemained[i] * 100 / ROW_SAMPLES);
        }
        totalRemained += rowDMARemained[i];
    }
    int totalAvailable = ROW_SAMPLES * NTSC_FRAME_LINES;
    printf("approximately %d%% CPU available per frame\n", totalRemained * 100 / totalAvailable);
    printf("%d cycles were available per line\n", cyclesPerLine);

    return COMMAND_CONTINUE;
}

int doCommandSolidFill(int wordCount, char **words)
{
    solidFillWave = strtoul(words[1], NULL, 0);
    NTSCMode = NTSC_SOLID_FILL;
    printf("solid image with %08lX word\n", solidFillWave);
    return COMMAND_CONTINUE;
}

int doCommandScopeFill(int wordCount, char **words)
{
#if 0
    NTSCMode = VIDEO_PALETTIZED;
    int degrees = strtol(words[1], NULL, 0);
    printf("vector scope image at %d degrees phase\n", degrees);
    showVectorScopeImage(degrees);
#endif
    return COMMAND_CONTINUE;
}

int doCommandTestRect(int wordCount, char **words)
{
    NTSCMode = NTSC_SCAN_TEST;
    do {
        SERIAL_poll_continue();
        unsigned char isEmpty = queue_isempty(&mon_queue);
        if(!isEmpty) {
            unsigned char c = queue_deq(&mon_queue);
            switch(c) {
                case 'l': videoScanTestLeft--; break;
                case 'L': videoScanTestLeft++; break;
                case 'r': videoScanTestRight--; break;
                case 'R': videoScanTestRight++; break;
                case 't': videoScanTestTop--; break;
                case 'T': videoScanTestTop++; break;
                case 'b': videoScanTestBottom--; break;
                case 'B': videoScanTestBottom++; break;
                case 'q': break; break;
            }
            printf("rect is %d, %d, %d, %d\n", videoScanTestLeft, videoScanTestRight, videoScanTestTop, videoScanTestBottom);
            SERIAL_flush();
        }
        delay_ms(10);
    } while(1);
    return COMMAND_CONTINUE;
}

int doCommandDumpKbdData(int wordCount, char **words)
{
    gDumpKeyboardData = !gDumpKeyboardData;
    if(gDumpKeyboardData)
        printf("Dumping keyboard data...\n");
    else
        printf("Not dumping keyboard data...\n");
    return COMMAND_CONTINUE;
}

int doCommandSDReset(int wordCount, char **words)
{
    printf("Resetting SD card...\n");

    if(!SDCARD_init()) {
        printf("Failed to start access to SD card as SPI\n");
    }
    return COMMAND_CONTINUE;
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
    return COMMAND_CONTINUE;
}

int doCommandTestSDSpeed(int wordCount, char **words)
{
    const int megabytes = 2;
    printf("will read %d megabyte of SD\n", megabytes);

    SERIAL_flush();

    unsigned char *sd_buffer = new unsigned char[SD_BLOCK_SIZE];
    if(sd_buffer == NULL) {
        printf("couldn't allocate block buffer\n");
        return COMMAND_FAILED;
    }

    int then = HAL_GetTick();
    for(int i = 0; i < megabytes * 1024 * 1024 / SD_BLOCK_SIZE; i++)
        SDCARD_readblock(i, sd_buffer);
    int now = HAL_GetTick();

    int kilobytes = megabytes * 1024;
    int millis = now - then;
    int kb_per_second = kilobytes * 1000 / millis;

    printf("done, %d KB per second\n", kb_per_second);

    delete[] sd_buffer;

    return COMMAND_CONTINUE;
}

int doCommandReadBlock(int wordCount, char **words)
{
    int n = strtol(words[1], NULL, 0);
    unsigned char *sd_buffer = new unsigned char [SD_BLOCK_SIZE];
    if(sd_buffer == NULL) {
        printf("couldn't allocate block buffer\n");
        return COMMAND_FAILED;
    }
    if(SDCARD_readblock(n, sd_buffer)) {
        dump_buffer_hex(4, sd_buffer, sizeof(sd_buffer));
    }
    delete[] sd_buffer;
    return COMMAND_CONTINUE;
}

int doCommandFlashInfoLED(int wordCount, char **words)
{
    for(int i = 0; i < 8; i++) {
        LED_set_info(1);
        delay_ms(125);
        LED_set_info(0);
        delay_ms(125);
    }
    return COMMAND_CONTINUE;
}

int doCommandPanic(int wordCount, char **words)
{
    printf("panicking now\n");
    panic();
    return COMMAND_CONTINUE; /* notreached */
}

int doCommandShowVersion(int wordCount, char **words)
{
    printf("%s\n", IOBOARD_FIRMWARE_VERSION_STRING);
    return COMMAND_CONTINUE;
}

int doCommandSetDebugLevel(int wordCount, char **words)
{
    gDebugLevel = strtol(words[1], NULL, 0);
    printf("Debug level set to %d\n", gDebugLevel);
    return COMMAND_CONTINUE;
}

int doHalt(int wordCount, char **words)
{
    while(1) {};
}

int doTestFloat(int argc, char **argv)
{
    float foo;

    if(sscanf(argv[1], "%f", &foo) != 1) {
        printf("couldn't sscanf a float from the argument\n");
        return COMMAND_FAILED;
    }

    printf("integer round of float: %d\n", (int)foo);

    printf("float: %f\n", foo);

    return COMMAND_CONTINUE;
}

static void RegisterAllApplets() __attribute__((constructor));
static void RegisterAllApplets()
{
    RegisterApp( "float", 2, doTestFloat, "float",
        "scanf and test and print a float"
    );
    RegisterApp( "modes", 1, doCommandVideoModes, "",
        "list video modes"
    );
    RegisterApp( "pixmap", 2, doCommandPixmapMode, "mode",
        "set pixmap mode"
    );
    RegisterApp( "text", 2, doCommandTextMode, "mode",
        "set text mode"
    );
    RegisterApp( "rect", 1, doCommandTestRect, "",
        "run interactive over/underscan test"
    );
#if 0
    RegisterApp( "stream", 4, doCommandStream, "name M N",
        "stream images from templated name (e.g. 'frame%05d') from number M to N"
    );
#endif
    RegisterApp( "videotest", 1, doCommandVideoTest, "",
        "switch to video test mode"
    );
    RegisterApp( "stats", 1, doCommandScanoutStats, "",
        "toggle viewing of scanout error statistics"
    );
    RegisterApp( "showend", 1, doCommandShowLineEnd, "",
        "put out a marker in the DAC at line processing end"
    );
    RegisterApp( "text", 1, doCommandVideoText, "",
        "toggle sending text to video"
    );
    RegisterApp( "rowcycles", 1, doCommandScanoutCycles, "",
        "print time spent in each row operation"
    );
    RegisterApp( "solid", 2, doCommandSolidFill, "fillword",
        "fill video with specified long int"
    );
    RegisterApp( "dumpkbd", 1, doCommandDumpKbdData, "",
        "dump keyboard data"
    );
    RegisterApp( "sdreset", 1, doCommandSDReset, "",
        "reset SD card"
    );
    RegisterApp( "sdspeed", 1, doCommandTestSDSpeed, "",
        "test SD read speed"
    );
    RegisterApp( "ls", 1, doCommandLS, "",
        "list files on SD"
    );
    RegisterApp( "read", 2, doCommandReadBlock, "N",
        "read and dump out SD block N"
    );
    RegisterApp( "flashinfo", 1, doCommandFlashInfoLED, "",
        "flash the info LED"
    );
    RegisterApp( "panic", 1, doCommandPanic, "",
        "panic"
    );
    RegisterApp( "version", 1, doCommandShowVersion, "",
        "display build version string"
    );
    RegisterApp( "debug", 2, doCommandSetDebugLevel, "N",
        "set debugging level to N"
    );
    RegisterApp( "halt", 1, doHalt, "", "while(1){}");
}

extern int KBDInterrupts;
extern int UARTInterrupts;

FATFS gFATVolume;

int main()
{
    HAL_Init();

    SCB_EnableICache();
    SCB->CACR |= SCB_CACR_FORCEWT_Msk;
    SCB_EnableDCache();

    SystemClock_Config(); 

    // Output PLL / 5 on MCO2
    if(1) {
        HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_5);
    }

    LED_init();
    LED_beat_heart();

    MON_init();
    console_queue_init();
    LED_beat_heart();

    setbuf(stdout, NULL);

    SERIAL_init(); // transmit and receive but global interrupts disabled
    LED_beat_heart();

    printf("\n\nRocinante firmware, %s\n", IOBOARD_FIRMWARE_VERSION_STRING);
    printf("HSERDY: %s %08lX\n", (RCC->CR & (1 << 17)) ? "true" : "false", RCC->CR);
    printf("System core clock: %lu Hz, %lu MHz\n", SystemCoreClock, SystemCoreClock / 1000000);

    float clock = 14.318180;
    NTSCCalculateParameters(clock);

#if 1 // DEBUG
    printf("calculated NTSCLineClocks = %d\n", NTSCLineClocks);
    printf("calculated NTSCHSyncClocks = %d\n", NTSCHSyncClocks);
    printf("calculated NTSCFrontPorchClocks = %d\n", NTSCFrontPorchClocks);
    printf("calculated NTSCBackPorchClocks = %d\n", NTSCBackPorchClocks);
    printf("calculated NTSCEqPulseClocks = %d\n", NTSCEqPulseClocks);
    printf("calculated NTSCVSyncClocks = %d\n", NTSCVSyncClocks);
    printf("RCC->DCKCFGR1 = %08lX\n", RCC->DCKCFGR1);
    printf("FLASH->ACR = %08lX\n", FLASH->ACR);
#endif

    LED_beat_heart();
    SERIAL_flush();

    if(0){ 
        GPIO_InitTypeDef  GPIO_InitStruct;

        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 
        int a;
        while(1) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (a & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (a & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (a & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, (a & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            delay_ms(500);
            a ++;
        }
    }

    SPI_config_for_sd();
    LED_beat_heart();

    if(!SDCARD_init()) {
        printf("Failed to start access to SD card SPI!!\n");
        LED_beat_heart();
        SERIAL_flush();
    } else  {
        printf("Opened SD Card\n");
        LED_beat_heart();
        SERIAL_flush();

        FRESULT result = f_mount(&gFATVolume, "0:", 1);
        if(result != FR_OK) {
            logprintf(DEBUG_ERRORS, "ERROR: FATFS mount result is %d\n", result);
            panic();
        } else {
            printf("Mounted FATFS from SD card successfully.\n");
        }
        LED_beat_heart();
        SERIAL_flush();
    }

#ifdef USE_PS2KBD
    KBD_init();
    LED_beat_heart();
#endif

    // AUDIO
    for(unsigned int i = 0; i < sizeof(audioBuffer); i++) {
        audioBuffer[i] = 128;
    }

    {
        // Set PA4 to ANALOG
        GPIO_InitTypeDef  GPIO_InitStruct;

        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
    }
    DAC->CR |= DAC_CR_EN1;
    DAC->CR |= DAC_CR_BOFF1;

    NTSCGenerateLineBuffers();
    NTSCFillRowBuffer(0, 0, row0);
    
    float colorBurstInCoreClocks = SystemCoreClock / 14318180.0;
    uint32_t DMACountAt14MHz = (colorBurstInCoreClocks + .5);
    // printf("DMACountAt14MHz = %lu, expected %d\n", DMACountAt14MHz, clockConfigs[whichConfig].DMA_TIM_CLOCKS);
    DMACountAt14MHz = clockConfigs[whichConfig].DMA_TIM_CLOCKS; // (colorBurstInCoreClocks + .5);

    DMAStartScanout(DMACountAt14MHz);

    // Wait for a click to progress to the next clock config
    { 
        GPIO_InitTypeDef  GPIO_InitStruct;

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
    }
    delay_ms(50);

    // If button is pushed down, enter clock config inspector
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {

        debugOverlayEnabled = 1;
        memset(debugDisplay, 0, debugDisplayWidth * debugDisplayHeight);

        int debugLine = 0;
        sprintf(debugDisplay[debugLine++], "Release button");
        sprintf(debugDisplay[debugLine++], "For ClockConfig");
        sprintf(debugDisplay[debugLine++], "Inspector");

        delay_ms(50);
        while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
        delay_ms(50);

        int counter = 0; 
        for(;;) {

            whichConfig = counter;

            DMAStopScanout();

            // Probably will break UART and SD and GPIOs:
            DeInitRCCAndPLL();
            SystemClock_Config();

            // Set up LED that toggles for every mode we look at
            GPIO_InitTypeDef  GPIO_InitStruct = {0};
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Pin = 0x100;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
            GPIOC->ODR = GPIOC->ODR & ~0x100;

            if(counter % 2 != 0) {
                GPIOC->ODR |= 0x100; // PC8
            }

            float colorBurstInCoreClocks = SystemCoreClock / 14318180.0;
            // need main clock as close as possible to @ 171.816
            uint32_t DMACountAt14MHz = (colorBurstInCoreClocks + .5);
            // printf("DMACountAt14MHz = %lu, expected %d\n", DMACountAt14MHz, clockConfigs[whichConfig].DMA_TIM_CLOCKS);
            DMACountAt14MHz = clockConfigs[whichConfig].DMA_TIM_CLOCKS; // (colorBurstInCoreClocks + .5);

            DMAStartScanout(DMACountAt14MHz);

            debugOverlayEnabled = 1;
            memset(debugDisplay, 0, debugDisplayWidth * debugDisplayHeight);

            int debugLine = 0;
            sprintf(debugDisplay[debugLine++], "Number %d", whichConfig);
            sprintf(debugDisplay[debugLine++], "PLL_M %lu", clockConfigs[whichConfig].PLL_M);
            sprintf(debugDisplay[debugLine++], "PLL_N %lu", clockConfigs[whichConfig].PLL_N);
            sprintf(debugDisplay[debugLine++], "PLL_P %lu", clockConfigs[whichConfig].PLL_P);
            sprintf(debugDisplay[debugLine++], "(DMA_TIM_CLOCKS %lu)", clockConfigs[whichConfig].DMA_TIM_CLOCKS);
            uint32_t at14MHz = (colorBurstInCoreClocks + .5);
            sprintf(debugDisplay[debugLine++], "calcd DMA BEATS %lu", at14MHz);
            sprintf(debugDisplay[debugLine++], "CPU clock: %d", (int)(clockConfigs[whichConfig].CPUMHz));
            sprintf(debugDisplay[debugLine++], "SCClock: %lu", SystemCoreClock);
            int clockConfigsWhole = clockConfigs[whichConfig].colorburstClock;
            sprintf(debugDisplay[debugLine++], "color %d.%03d", clockConfigsWhole, (int)((clockConfigs[whichConfig].colorburstClock - clockConfigsWhole) * 100000.0f));

            // Wait for a click to progress to the next clock config
            { 
                GPIO_InitTypeDef  GPIO_InitStruct;

                GPIO_InitStruct.Pin = GPIO_PIN_13;
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
            }
            while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
                delay_ms(50);
            }
            delay_ms(250);

            counter++;
            if(counter >= clockConfigCount) {
                memset(debugDisplay, 0, debugDisplayWidth * debugDisplayHeight);
                int debugLine = 0;
                sprintf(debugDisplay[debugLine++], "END OF CONFIGS");
                sprintf(debugDisplay[debugLine++], "HOORAY!");
                break;
            }
        }
        while(1);
    }

    // Kick into first text mode if possible
    int chosen = -1;
    for(int which = 0; (chosen == -1) && (which < VideoGetModeCount()); which++) {
        if(VideoModeGetType(which) == VIDEO_MODE_TEXTPORT) {
            chosen = which;
        }
    }
    if(chosen == -1) {
        printf("Couldn't find a text mode, didn't set up screen output or VT102\n");
    } else {
        TextportSetMode(chosen);
        NTSCMode = NTSC_USE_VIDEO_MODE;
        gOutputDevices = gOutputDevices ^ OUTPUT_TO_TEXTPORT;
    }

    printf("* ");
    SERIAL_flush();

    for(;;) {

        // Should be in VBlank callback so is continuously updated
        if(showScanoutStats) {
            debugOverlayEnabled = 1;
            int currentFrame = frameNumber;
            while(currentFrame == frameNumber); // Wait for VBLANK
            memset(debugDisplay, 0, debugDisplayWidth * debugDisplayHeight);
            sprintf(debugDisplay[debugDisplayHeight - 4], "UART %d", UARTInterrupts);
            sprintf(debugDisplay[debugDisplayHeight - 3], "KBD %d", KBDInterrupts);
            sprintf(debugDisplay[debugDisplayHeight - 2], "FIFO unders %lu", DMAFIFOUnderruns);
            sprintf(debugDisplay[debugDisplayHeight - 1], "DMA errors %lu", DMATransferErrors);
        }

        SERIAL_try_to_transmit_buffers();
        // LED_beat_heart();

        SERIAL_poll_continue();

#ifdef USE_PS2KBD
        int key = KBD_process_queue(gDumpKeyboardData);
        if(key >= 0) {
            disable_interrupts();
            // console_enqueue_key_unsafe(key);
            monitor_enqueue_key_unsafe(key);
            enable_interrupts();
        }
#endif

        ProcessAnyInput();

        check_exceptional_conditions();
    }

    // should not reach
    panic();
}
