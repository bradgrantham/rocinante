#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include <stm32f4xx_hal.h>

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

const int gStandaloneARM = 1;
int gSnoopVideo = 1;

static int gDumpKeyboardData = 0;

void panic_worse()
{
    LED_set_panic(1);
    for(;;);
}

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

//----------------------------------------------------------------------------
// System Initialization Goop

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16; // Divide HSE by this
  RCC_OscInitStruct.PLL.PLLN = 336; // Then multiply by this
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // Then divide by this
  RCC_OscInitStruct.PLL.PLLQ = 7; // Divide by this for SD, USB OTG FS, and some other peripherals
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    panic();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // APB1 will be 42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // APB2 will be 84MHz
  // grantham - 5 cycles for 168MHz is stated in Table 10 in the STM32F4 reference manual
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    panic();
  }

  // 415 comes up by default with D$ and I$ enabled and prefetch enabled
  // FLASH->ACR |= FLASH_ACR_PRFTEN

  /* Enable other CLKs */
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
}

void system_init()
{
    HAL_Init();

    SystemClock_Config();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    delay_init();
}

//----------------------------------------------------------------------------
// stdio

#define OUTPUT_TO_SERIAL        0x01
#define OUTPUT_TO_VIDEO         0x02

int gOutputDevices = OUTPUT_TO_SERIAL;

void __io_putchar( char c )
{
    if(gOutputDevices & OUTPUT_TO_SERIAL)
        SERIAL_enqueue_one_char(c);
    if(!gStandaloneARM && (gOutputDevices & OUTPUT_TO_VIDEO)) {
        // if(c == '\n') // XXX
            // BUS_write_IO(VIDEO_BOARD_OUTPUT_ADDR, '\r');
        // BUS_write_IO(VIDEO_BOARD_OUTPUT_ADDR, c);
    }
}

void errorchar(char c)
{
    SERIAL_enqueue_one_char(c);
    if(!gStandaloneARM) {
        // if(c == '\n') // XXX
            // BUS_write_IO(VIDEO_BOARD_OUTPUT_ADDR, '\r');
        // BUS_write_IO(VIDEO_BOARD_OUTPUT_ADDR, c);
    }
}

void errorchar_flush()
{
    SERIAL_flush();
}

//----------------------------------------------------------------------------
// CP/M 8MB Disk definitions

#if 0

#define SECTORS_PER_BLOCK 4
#define SECTORS_PER_TRACK 64
#define TRACKS_PER_DISK 1024
#define SECTOR_SIZE 128
/* disk is 8MB, so 16384 512-byte blocks per disk */
#define BLOCKS_PER_DISK 16384

FATFS gFATVolume;

#define DISK_IMAGE_MAX 8
char gDiskImageFilenames[DISK_IMAGE_MAX][13];
FIL gDiskImageFiles[DISK_IMAGE_MAX];
int gDiskImageCount = 0;

int read_disk_image_list()
{
    FIL f;
    FRESULT result = f_open (&f, "disks.txt", FA_READ | FA_OPEN_EXISTING);

    if(result != FR_OK) {
        logprintf(DEBUG_ERRORS, "ERROR: couldn't open \"disks.txt\" for reading, FatFS result %d\n", result);
        return 0;
    }

    while(!f_eof(&f) && (gDiskImageCount < DISK_IMAGE_MAX)) {
        static char line[80];
        if(f_gets(line, sizeof(line), &f)) {
            if(line[strlen(line) - 1] != '\n') {
                logprintf(DEBUG_WARNINGS, "\"disks.txt\" unexpectedly contained a line longer than 80 characters:\n");
                logprintf(DEBUG_WARNINGS, "\"%s\"\n", line);
                return 0;
            }

            line[strlen(line) - 1] = '\0';

            if(strlen(line) > 12) {
                logprintf(DEBUG_WARNINGS, "Unexpectedly long disk image name \"%s\" ignored\n", line);
            } else {
                strcpy(gDiskImageFilenames[gDiskImageCount], line);
                gDiskImageCount++;
            }
        }
    }

    if(!f_eof(&f)) {
        logprintf(DEBUG_WARNINGS, "Maximum disk images reached (%d), further contents of \"disks.txt\" ignored\n", DISK_IMAGE_MAX);
    }

    f_close(&f);
    return 1;
}

int open_disk_images()
{
    int success = 1;
    int i;

    for(i = 0; i < gDiskImageCount; i++) {
        FRESULT result = f_open (&gDiskImageFiles[i], gDiskImageFilenames[i], FA_READ | FA_WRITE | FA_OPEN_EXISTING);

        if(result != FR_OK) {
            logprintf(DEBUG_ERRORS, "ERROR: couldn't open disk image \"%s\" for rw, FatFS result %d\n", gDiskImageFilenames[i], result);
            success = 0;
            break;
        }

        if(f_size(&gDiskImageFiles[i]) != BLOCKS_PER_DISK * SD_BLOCK_SIZE) {
            logprintf(DEBUG_ERRORS, "ERROR: expected disk image \"%s\" to be 8MB, is %d bytes\n", gDiskImageFilenames[i], f_size(&gDiskImageFiles[i]));
            success = 0;
            break;
        }
    }

    if(!success)
        for(int j = 0; j < i; j++)
            f_close(&gDiskImageFiles[j]);

    return success;
}

#endif

char gMonitorCommandBuffer[80];
unsigned char gMonitorCommandBufferLength = 0;

void usage()
{
    printf("help       - this help message\n");
    printf("debug N    - set debug level\n");
    printf("sdreset    - reset SD\n");
    printf("dumpkbd    - toggle dumping keyboard\n");
    printf("version    - print firmware build version\n");
    printf("read N     - read and dump block N\n");
    printf("panic      - force panic\n");
    printf("flashinfo  - force flashing the info LED\n");
}

#define IOBOARD_FIRMWARE_VERSION_STRING XSTR(IOBOARD_FIRMWARE_VERSION)

volatile unsigned char gSerialInputToMonitor = 1;

unsigned char sd_buffer[SD_BLOCK_SIZE];

uint32_t rowCyclesSpent[262];
int scanOnlyFirstField = 1;

void process_local_key(unsigned char c)
{
    // XXX make this table driven, break into lots smaller functions
    if(c == '\r' || c == '\n') {
        putchar('\n');
        gMonitorCommandBuffer[gMonitorCommandBufferLength] = 0;

        if((strcmp(gMonitorCommandBuffer, "help") == 0) ||
           (strcmp(gMonitorCommandBuffer, "h") == 0) ||
           (strcmp(gMonitorCommandBuffer, "?") == 0)) {

            usage();

        } else if(strcmp(gMonitorCommandBuffer, "sdreset") == 0) {

            printf("Resetting SD card...\n");

            if(!SDCARD_init())
                printf("Failed to start access to SD card as SPI\n");

        } else if(strcmp(gMonitorCommandBuffer, "go") == 0) {

            scanOnlyFirstField = 0;
        } else if(strcmp(gMonitorCommandBuffer, "rowcycles") == 0) {

            for(int i = 0; i < 262; i++) {
                printf("row %3d: %8lu cycles, %lu microseconds", i, rowCyclesSpent[i],
                    rowCyclesSpent[i] / (SystemCoreClock / 1000000));
                if(rowCyclesSpent[i] > 10677) {
                    printf(", overrun by %lu%%\n", rowCyclesSpent[i] * 100 / 10677 - 100);
                } else { 
                    printf(", %lu%% remained\n", (10677 - rowCyclesSpent[i]) * 100 / 10677);
                }
            }

        } else if(strcmp(gMonitorCommandBuffer, "sdspeed") == 0) {

            const int megabytes = 2;
            printf("will read %d megabyte of SD\n", megabytes);

            SERIAL_flush();

            int then = HAL_GetTick();
            for(int i = 0; i < megabytes * 1024 * 1024 / SD_BLOCK_SIZE; i++)
                SDCARD_readblock(i, sd_buffer);
            int now = HAL_GetTick();

            int kilobytes = megabytes * 1024;
            int millis = now - then;
            int kb_per_second = kilobytes * 1000 / millis;

            printf("done, %d KB per second\n", kb_per_second);

        } else if(strcmp(gMonitorCommandBuffer, "dumpkbd") == 0) {

            gDumpKeyboardData = !gDumpKeyboardData;
            if(gDumpKeyboardData)
                printf("Dumping keyboard data...\n");
            else
                printf("Not dumping keyboard data...\n");

        } else if(strcmp(gMonitorCommandBuffer, "flashinfo") == 0) {

            for(int i = 0; i < 8; i++) {
                LED_set_info(1);
                delay_ms(125);
                LED_set_info(0);
                delay_ms(125);
            }

        } else if(strcmp(gMonitorCommandBuffer, "panic") == 0) {

            printf("panicking now\n");
            panic();

        } else if(strcmp(gMonitorCommandBuffer, "version") == 0) {

            printf("%s\n", IOBOARD_FIRMWARE_VERSION_STRING);

        } else if(strncmp(gMonitorCommandBuffer, "debug ", 6) == 0) {

            char *p = gMonitorCommandBuffer + 5;
            while(*p == ' ')
                p++;
            gDebugLevel = strtol(p, NULL, 0);
            printf("Debug level set to %d\n", gDebugLevel);

        } else if(strncmp(gMonitorCommandBuffer, "read ", 5) == 0) {

            char *p = gMonitorCommandBuffer + 4;
            while(*p == ' ')
                p++;
            int n = strtol(p, NULL, 0);
            if(SDCARD_readblock(n, sd_buffer)) {
                dump_buffer_hex(4, sd_buffer, sizeof(sd_buffer));
            }

        } else {

            printf("Unknown command.\n");
            usage();
        }

        printf("* ");
        gMonitorCommandBufferLength = 0;

    } else {

        if(c == 127 || c == '\b') {
            if(gMonitorCommandBufferLength > 0) {
                putchar('\b');
                putchar(' ');
                putchar('\b');
                gMonitorCommandBufferLength--;
            } else {
                bell();
            }
        } else {
            if(gMonitorCommandBufferLength < sizeof(gMonitorCommandBuffer) - 1) {
                putchar(c);
                gMonitorCommandBuffer[gMonitorCommandBufferLength++] = c;
            } else {
                bell();
            }
        }
    // TODO - upload data, write block
    }
}

void check_exceptional_conditions()
{
    if(gConsoleOverflowed) {
        logprintf(DEBUG_WARNINGS, "WARNING: Console input queue overflow\n");
        gConsoleOverflowed = 0;
    }

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
}

void process_monitor_queue()
{
    unsigned char isEmpty = queue_isempty(&mon_queue.q);
    static unsigned char escapeBackToMonitor = 0;

    if(!isEmpty) {
        unsigned char c = queue_deq(&mon_queue.q);
        if(gSerialInputToMonitor)
            process_local_key(c);
        else {
            if(escapeBackToMonitor == 0 && c == 1)
                escapeBackToMonitor = 1;
            else if(escapeBackToMonitor != 0 && c == 2) {
                escapeBackToMonitor = 0;
                gSerialInputToMonitor = 1;
                printf("Serial input returned to monitor\n");
            } else {
                escapeBackToMonitor = 0;
                disable_interrupts();
                console_enqueue_key_unsafe(c);
                enable_interrupts();
            }
        }
    }
}

void uart_received(char c)
{
    queue_enq(&mon_queue.q, c);
}

void console_queue_init()
{
    queue_init(&con_queue.q, CON_QUEUE_CAPACITY);
}

// XXX put these in CCM somehow to relieve main memory pressure
#if 0
unsigned char rowEQSyncPulseBuffer[910];
unsigned char rowVSyncBuffer[910];
unsigned char rowBlankLineBuffer[910];
#else
unsigned char *rowEQSyncPulseBuffer = (unsigned char *)0x1000F000;
unsigned char *rowVSyncBuffer = (unsigned char *)0x1000E000;
unsigned char *rowBlankLineBuffer = (unsigned char *)0x1000D000;
#endif

#if 0
// XXX put these in SRAM2 somehow to relieve main memory pressure
unsigned char row0[910];
unsigned char row1[910];
#else
unsigned char *row0 = (unsigned char *)0x2001C000;
unsigned char *row1 = (unsigned char *)0x2001D000;
#endif

#define NTSC_EQPULSE_LINES	3
#define NTSC_VSYNC_LINES	3
#define NTSC_VBLANK_LINES	10
#define NTSC_FIELD_LINES	262
#define NTSC_PIXEL_LINES	(NTSC_FIELD_LINES - NTSC_VBLANK_LINES - NTSC_VSYNC_LINES - NTSC_EQPULSE_LINES * 2)
#define NTSC_EQ_PULSE_INTERVAL	.04
#define NTSC_VSYNC_BLANK_INTERVAL	.43
#define NTSC_HOR_SYNC_DUR	.075
#define NTSC_FIELDS		60
#define NTSC_FRONTPORCH		.02
#define NTSC_BACKPORCH		.07 /* not including COLORBURST */

const unsigned char syncTipDACValue = 0;
const unsigned char syncPorchDACValue = 54;
const unsigned char blackDACValue = 65;
const unsigned char whiteDACValue = 186;
const unsigned char maxDACValue = 255;

unsigned char videoDACValue(unsigned char luminance) { return blackDACValue + luminance * (whiteDACValue - blackDACValue) / 255; }

int rowNumber;
int fieldNumber;
int whichRowBuffer;

int eqPulseTicks;
int vsyncPulseTicks;
int horSyncTicks;
int lineTicks;
int fieldTicks;
int frontPorchTicks;
int backPorchTicks;

// XXX later put source buffers in CCM

void fillEQPulseBuffer(size_t rowSize, unsigned char *rowBuffer)
{
    for (int col = 0; col < lineTicks; col++) {
        if (col < eqPulseTicks || (col > lineTicks/2 && col < lineTicks/2 + eqPulseTicks)) {
            rowBuffer[col] = syncTipDACValue;
        } else {
            rowBuffer[col] = syncPorchDACValue;
        }
    }
}

void fillVSyncBuffer(size_t rowSize, unsigned char *rowBuffer)
{
    for (int col = 0; col < lineTicks; col++) {
        // if (col < lineTicks/2-vsyncPulseTicks || (col > lineTicks/2 && col < vsyncPulseTicks)) {
        if (col < vsyncPulseTicks || (col > lineTicks/2 && col < lineTicks/2 + vsyncPulseTicks)) {
            rowBuffer[col] = syncTipDACValue;
        } else {
            rowBuffer[col] = syncPorchDACValue;
        }
    }
}

void fillBlankLineBuffer(size_t rowSize, unsigned char *rowBuffer)
{
    for (int col = 0; col < lineTicks; col++) {
        if (col < horSyncTicks) {
            rowBuffer[col] = syncTipDACValue;
        } else {
            rowBuffer[col] = syncPorchDACValue;
        }
    }
}

void fillRowBuffer(int fieldNumber, int rowNumber, size_t rowSize, unsigned char *rowBuffer)
{
    /*
     * Rows 0 through 8 are equalizing pulse, then vsync, then equalizing pulse
     */
    if(rowNumber < NTSC_EQPULSE_LINES) {

        // XXX should just change DMA source address
        memcpy(rowBuffer, rowEQSyncPulseBuffer, rowSize);

    } else if(rowNumber - NTSC_EQPULSE_LINES < NTSC_VSYNC_LINES) {

        // XXX should just change DMA source address
        memcpy(rowBuffer, rowVSyncBuffer, rowSize);

    } else if(rowNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES) < NTSC_EQPULSE_LINES) {

        // XXX should just change DMA source address
        memcpy(rowBuffer, rowEQSyncPulseBuffer, rowSize);

    } else if(rowNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES) < NTSC_VBLANK_LINES) {

        /*
         * Rows 9 through 23 are other part of vertical blank
         */

        // XXX should just change DMA source address
        memcpy(rowBuffer, rowBlankLineBuffer, rowSize);

    } else {

        memcpy(rowBuffer, rowBlankLineBuffer, rowSize);
        int y = rowNumber - (NTSC_EQPULSE_LINES + NTSC_VSYNC_LINES + NTSC_EQPULSE_LINES + NTSC_VBLANK_LINES);
        int yTile = y >> 4;

	if(0) {
            for (int col = horSyncTicks + backPorchTicks; col < lineTicks - frontPorchTicks; col++) {
                int x = col - (horSyncTicks + backPorchTicks); // 0 to 779
                if(0) {
                    int xTile = x >> 4;
                    if((yTile + xTile) & 0x01) {
                        rowBuffer[col] = blackDACValue;
                    } else {
                        rowBuffer[col] = whiteDACValue;
                    }
                } else {
                    if(x > 200 && x < 300 && y > 50 && y < 100) {
                        rowBuffer[col] = whiteDACValue;
                    } else {
                        rowBuffer[col] = blackDACValue;
                    }
                }
            }
        } else {
            if(y < 50) {
                for(int col = horSyncTicks + backPorchTicks; col < lineTicks - frontPorchTicks; col++) { 
                        rowBuffer[col] = blackDACValue;
                }
            } else if(y < 100) {
                for(int col = horSyncTicks + backPorchTicks; col < horSyncTicks + backPorchTicks + 200; col++){
                        rowBuffer[col] = blackDACValue;
                }
                for(int col = horSyncTicks + backPorchTicks + 200; col < horSyncTicks + backPorchTicks + 300; col++){
                        rowBuffer[col] = whiteDACValue;
                }
                for(int col = horSyncTicks + backPorchTicks + 300; col < lineTicks - frontPorchTicks; col++){
                        rowBuffer[col] = blackDACValue;
                }
            } else {
                for(int col = horSyncTicks + backPorchTicks; col < lineTicks - frontPorchTicks; col++) { 
                        rowBuffer[col] = blackDACValue;
                }
            }
        }
    }
    if(rowNumber == 0) {
        rowBuffer[200] = 255;
        rowBuffer[201] = 255;
        rowBuffer[203] = 255;
        rowBuffer[204] = 255;
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    // Clear interrupt flag
    // DMA2->LISR &= ~DMA_TCIF;
    DMA2->LIFCR |= DMA_LIFCR_CTCIF2;

    // Configure timer TIM2
    TIM2->SR = 0;                       /* reset status */
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;            /* enable the timer */

    rowNumber = rowNumber + 1;
    // row to calculate
    if(rowNumber == NTSC_FIELD_LINES) {
        rowNumber = 0;
        fieldNumber++;
    }

    int whichIsScanning = (DMA2_Stream2->CR & DMA_SxCR_CT) ? 1 : 0;
    // int whichIsScanning = whichRowBuffer ? 0 : 1;

    unsigned char *nextRowBuffer = (whichIsScanning == 1) ? row0 : row1;

    if((!scanOnlyFirstField) || (fieldNumber == 0)) { // XXX
        fillRowBuffer(fieldNumber, rowNumber, 910, nextRowBuffer);
        rowCyclesSpent[rowNumber] = TIM2->CNT;
    }

    whichRowBuffer = whichRowBuffer ^ 0x01;
    TIM2->CR1 = 0;            /* stop the timer */
}

int main()
{
    system_init();

    LED_init();
    LED_beat_heart();

    MON_init();
    console_queue_init();
    LED_beat_heart();

    setbuf(stdout, NULL);

    SERIAL_init(); // transmit and receive but global interrupts disabled
    LED_beat_heart();

    printf("\n\nAlice 3 I/O firmware, %s\n", IOBOARD_FIRMWARE_VERSION_STRING);
    printf("System core clock: %lu MHz\n", SystemCoreClock / 1000000);

    float clock = 14.318180;
    fieldTicks = floorf(clock * 1000000.0 / NTSC_FIELDS + 0.5);
    lineTicks = floorf((double)fieldTicks / NTSC_FIELD_LINES + 0.5);
    printf("calculated lineTicks = %d\n", lineTicks);
    lineTicks = 910; // XXX
    horSyncTicks = floorf(lineTicks * NTSC_HOR_SYNC_DUR + 0.5);
    frontPorchTicks = lineTicks * NTSC_FRONTPORCH;
    backPorchTicks = lineTicks * NTSC_BACKPORCH;
    eqPulseTicks = lineTicks * NTSC_EQ_PULSE_INTERVAL;
    vsyncPulseTicks = lineTicks * NTSC_VSYNC_BLANK_INTERVAL;

    printf("calculated horSyncTicks = %d\n", horSyncTicks);
    printf("calculated frontPorchTicks = %d\n", frontPorchTicks);
    printf("calculated backPorchTicks = %d\n", backPorchTicks);
    printf("calculated eqPulseTicks = %d\n", eqPulseTicks);
    printf("calculated vsyncPulseTicks = %d\n", vsyncPulseTicks);


    LED_beat_heart();
    SERIAL_flush();

#if 0 // configure SD, load FAT32, read files
    SPI_config_for_sd();
    LED_beat_heart();

    if(!SDCARD_init())
        printf("Failed to start access to SD cardSPI!!\n");
    else 
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
    SERIAL_flush();

    int success = read_bootrom();
    if(!success) {
        panic();
    }
    LED_beat_heart();

    success = read_disk_image_list();
    if(!success) {
        panic();
    }
    if(0)
        for(int i = 0; i < gDiskImageCount; i++) {
            printf("disk %c: \"%s\"\n", 'A' + i, gDiskImageFilenames[i]);
        }
    success = open_disk_images();
    if(!success) {
        panic();
    }
    LED_beat_heart();

    if(0) {
        unsigned char block[128];
        UINT wasread;
        for(int i = 0; i < gDiskImageCount; i++) {
            result = f_read(&gDiskImageFiles[i], block, sizeof(block), &wasread);
            printf("\"%s\", read resulted in %d, got %d bytes:\n", gDiskImageFilenames[i], result, wasread);
            dump_buffer_hex(4, block, sizeof(block));
        }
    }
#endif

#if 0 // Do not initialize PS/2 keyboard
    KBD_init();
    LED_beat_heart();
#endif

    printf("* ");
    SERIAL_flush();

    GPIO_InitTypeDef  GPIO_InitStruct = {0};
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = 0xFF;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

    // one line = (1 / 3579545) * (455/2)

    // front porch is (.165) * (1 / 15734) / (1 / 3579545) = 37.53812921062726565701 cycles (37.5)
    //     74 cycles at double clock
    // pixels is (1 - .165) * (1 / 15734) / (1 / 3579545) = 189.96568418711380557696 cycles (190)
    //     280 cycles at double clock

    fillEQPulseBuffer(sizeof(rowEQSyncPulseBuffer), rowEQSyncPulseBuffer);
    fillVSyncBuffer(sizeof(rowVSyncBuffer), rowVSyncBuffer);
    fillBlankLineBuffer(sizeof(rowBlankLineBuffer), rowBlankLineBuffer);
    fillRowBuffer(0, 0, 910, row0);

    // Note that the counter starts counting 1 clock cycle after setting the CEN bit in the TIMx_CR1 register.
    uint32_t count = 12;        // 140 MHz, need 14.318180...
                                // need main clock as close as possible to @ 171.816

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    // Configure TIM1_CH2 to drive DMA
    TIM1->CCR2 = count / 2 - 1;         /* 50% duty cycle */ 
    TIM1->CCER |= TIM_CCER_CC2E;        /* enable capture/compare CH2 */
    TIM1->DIER |= TIM_DIER_CC2DE;       /* enable capture/compare updates */

    // Configure timer TIM1
    TIM1->SR = 0;                       /* reset status */
    TIM1->ARR = count - 1;
    TIM1->CR1 = TIM_CR1_CEN;            /* enable the timer */

    // STEP 2: configure DMA to double buffer write 910 bytes from row0 and then row1 at 14.318180
#if 0
stream configuration register DMA_SxCR 
    CHSEL = 000 // channel 0
    MBURST = 00 // single transfer
    PBURST = 00 // single transfer
    CT = 0 // start transfer from buffer 0
    DBM = 1 // double buffer
    PL = 11 // very high
    PINCOS = dont care
    MSIZE = 00 // byte
    PSIZE = 00 // byte
    MINC = 1 // increment
    PINC = 0 // fixed
    CIRC = dont care // will be forced to 1 in DB mode
    DIR = 01 // memory to peripheral
    PFCTRL = 0 // DMA is the flow controller ?
    TCIE = 1 // enable transfer complete interrupt
    HTIE = 0 // disable half transfer interrupt
    TEIE = 0 // transfer error interrupt
    DMEIE = 0 // DMA error interrupt
    EN = 1 // when ready to enable
transfer count DMA_SxNDTR = 910
base memory address DMA_SxM0AR DMA_SxM1AR
    row0 and row1
peripheral memory address DMA_SxPAR
    GPIOC ODR?
FIFO control register DMA_SxFCR
    FEIE = 0 // FIFO error interrupt
    FS // is read-only
    DMDIS = 1 // direct mode disabled ...?
    FTH = 01 // 1/2 full FIFO
    // TCIE, TCIF
    // must clear flag before re-enabling
#endif
    DMA2_Stream2->NDTR = 910;
    DMA2_Stream2->M0AR = (uint32_t)row0;
    DMA2_Stream2->M1AR = (uint32_t)row1;
    DMA2_Stream2->PAR = (uint32_t)&GPIOC->ODR;
    DMA2_Stream2->FCR = DMA_FIFOMODE_ENABLE; // | DMA_FIFO_THRESHOLD_HALFFULL;
    DMA2_Stream2->CR =
        DMA_CHANNEL_6 | 
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_HALFWORD | 
        DMA_SxCR_DBM |
        DMA_PRIORITY_VERY_HIGH |
        DMA_MINC_ENABLE |
        DMA_MEMORY_TO_PERIPH | 
        DMA_IT_TC | 
        0;
    DMA2_Stream2->CR |= DMA_SxCR_EN;

    // STEP 4: set row buffers from a monochrome image

    // for(;;) {
        // GPIOC->ODR = (GPIOC->ODR & 0xffffff00) | img[addr];
        // addr = (addr + 1) % sizeof(img);
        // __asm__ volatile("nop");
    //}

    for(;;) {
        int key;

        SERIAL_try_to_transmit_buffers();
        LED_beat_heart();

        SERIAL_poll_continue();

        process_monitor_queue();

        key = KBD_process_queue(gDumpKeyboardData);
        if(key >= 0) {
            disable_interrupts();
            console_enqueue_key_unsafe(key);
            enable_interrupts();
        }

        check_exceptional_conditions();
    }

    // should not reach
    panic();
}
