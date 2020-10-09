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
// Audio experiment goop

#if 0
// init time
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
#endif

#if 0
// per-line, 15KHz
    DAC1->DHR8R1 = audioBuffer[audioBufferPosition];
    audioBufferPosition = (audioBufferPosition + 1) % sizeof(audioBuffer);
#endif

#if 0
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
#endif

//----------------------------------------------------------------------------
// Text command interface

int showScanoutStats = 0;


enum { CIRCLE_COUNT = 10 };

int doTestSegmentMode(int wordCount, char **words)
{
#if 0 // XXX video rewrite
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

#endif
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
    open a window
    
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
    extern int markHandlerInSamples; // XXX
    markHandlerInSamples = !markHandlerInSamples;
    return COMMAND_CONTINUE;
}

int doCommandScanoutCycles(int wordCount, char **words)
{
#if 0
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

#else
    printf("Command Commented Out, %d\n", __LINE__);
#endif
    return COMMAND_CONTINUE;
}

#if 0
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
RegisterApp( "rect", 1, doCommandTestRect, "",
    "run interactive over/underscan test"
);
#endif

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
    RegisterApp( "pixmap", 2, doCommandPixmapMode, "mode",
        "set pixmap mode"
    );
    RegisterApp( "text", 2, doCommandTextMode, "mode",
        "set text mode"
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

Status CheckStatusAndHalt(Status status, const char *command)
{
    if(status != SUCCESS) {
        printf("FATAL: %s failed with status %d\n", command, status);
        for(;;);
    }
    return status;
}

#define CHECK_HALT(c) CheckStatusAndHalt(c, #c)

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
#if 1
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

    float colorBurstInCoreClocks = SystemCoreClock / 14318180.0;
    uint32_t DMACountAt14MHz = (colorBurstInCoreClocks + .5);
    if(false) {
        printf("DMACountAt14MHz = %lu, expected %lu\n", DMACountAt14MHz, clockConfigs[whichConfig].DMA_TIM_CLOCKS);
    }

    CHECK_HALT(VideoSetSubsystem(GetNTSCVideoSubsystem()));

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
        static char debugLine[80];
        int debugRow = 0;

        sprintf(debugLine, "Release button");
        VideoModeSetDebugRow(debugRow++, debugLine);
        sprintf(debugDisplay[debugLine++], "For ClockConfig");
        VideoModeSetDebugRow(debugRow++, debugLine);
        sprintf(debugDisplay[debugLine++], "Inspector");
        VideoModeSetDebugRow(debugRow++, debugLine);

        delay_ms(50);
        while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
        delay_ms(50);

        int counter = 0; 
        for(;;) {

            whichConfig = counter;

            VideoSubsystemStop();

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

            CHECK_HALT(VideoSetSubsystem(GetNTSCVideoSubsystem()));

            debugOverlayEnabled = 1;
            memset(debugDisplay, 0, debugDisplayWidth * debugDisplayHeight);

            int debugRow = 0;
            sprintf(debugLine, "Number %d", whichConfig);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "PLL_M %lu", clockConfigs[whichConfig].PLL_M);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "PLL_N %lu", clockConfigs[whichConfig].PLL_N);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "PLL_P %lu", clockConfigs[whichConfig].PLL_P);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "(DMA_TIM_CLOCKS %lu)", clockConfigs[whichConfig].DMA_TIM_CLOCKS);
            VideoModeSetDebugRow(debugRow++, debugLine);
            uint32_t at14MHz = (colorBurstInCoreClocks + .5);
            sprintf(debugLine, "calcd DMA BEATS %lu", at14MHz);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "CPU clock: %d", (int)(clockConfigs[whichConfig].CPUMHz));
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "SCClock: %lu", SystemCoreClock);
            VideoModeSetDebugRow(debugRow++, debugLine);
            int clockConfigsWhole = clockConfigs[whichConfig].colorburstClock;
            sprintf(debugLine, "color %d.%03d", clockConfigsWhole, (int)((clockConfigs[whichConfig].colorburstClock - clockConfigsWhole) * 100000.0f));
            VideoModeSetDebugRow(debugRow++, debugLine);

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
                debugRow = 0;
                sprintf(debugLine, "END OF CONFIGS");
                VideoModeSetDebugRow(debugRow++, debugLine);
                sprintf(debugLine, "HOORAY!");
                VideoModeSetDebugRow(debugRow++, debugLine);
                break;
            }
        }
        while(1);
    }

    printf("* ");
    SERIAL_flush();

    for(;;) {

        // Should be in VBlank callback so is continuously updated
        if(showScanoutStats) {
            static char debugLine[80];
            int debugRow = 0;
            debugOverlayEnabled = 1;
            int currentFrame = frameNumber;
            // while(currentFrame == frameNumber); // Wait for VBLANK
            sprintf(debugLine, "UART %d", UARTInterrupts);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "KBD %d", KBDInterrupts);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "FIFO unders %lu", DMAFIFOUnderruns);
            VideoModeSetDebugRow(debugRow++, debugLine);
            sprintf(debugLine, "DMA errors %lu", DMATransferErrors);
            VideoModeSetDebugRow(debugRow++, debugLine);
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
