#include <cstdio>
#include <videomodeinternal.h>
#include <defs.h>
#include "ntsc_constants.h"
#include "dac_constants.h"

static constexpr int MAXDRIVERS=16;
static int driverCount = 0;
static NTSCModeDriver* drivers[MAXDRIVERS];

void NTSCVideoRegisterDriver(NTSCModeDriver* driver)
{
    if(driverCount >= MAXDRIVERS) {
        printf("Exceeded maximum NTSC mode driver count, will ignore \"%s\"\n", driver.getName());
        return;
    }
    drivers[driverCount++] = driver;
}

int SECTION_CCMRAM markHandlerInSamples = 0;

volatile int16_t SECTION_CCMRAM rowTimeRemained[NTSC_FRAME_LINES];

// These are in CCM to reduce contention with SRAM1 during DMA 
unsigned char SECTION_CCMRAM NTSCEqSyncPulseLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCVSyncLine[ROW_SAMPLES];
unsigned char SECTION_CCMRAM NTSCBlankLine[ROW_SAMPLES];

unsigned char SECTION_CCMRAM NTSCSyncTip;
unsigned char SECTION_CCMRAM NTSCSyncPorch;
unsigned char SECTION_CCMRAM NTSCBlack;
unsigned char SECTION_CCMRAM NTSCWhite;

ntsc_wave_t backgroundColorWave;

int NTSCEqPulseClocks;
int NTSCVSyncClocks;
int NTSCHSyncClocks;
int NTSCLineClocks;
int NTSCFrontPorchClocks;
int NTSCBackPorchClocks;


// (Following was copied from paper chicken scratches)
// On Orion TV, one 14MHz clock is .0225 inches, one 240p row is .052 inches.
// So to find a close value for width, width = sqrt(4 / 3 * .052 / .0225 * VRAM_SIZE)
// Then, height should be no more than width * 3 * .0225 / (4 / .052)
// So for 53248, a reasonable 4:3 framebuffer is 400x128
// 4:3 aspect would be 1.333
// 400 wide would be 9 inches, and 128 high would be 6.656 inches, and that's 1.352, so it's not too bad

// Essentially the remainder of CCMRAM, will need to be shrunk if more goes into CCM
// Put this into Linker Script...
#define VRAM_SIZE  51725

unsigned char SECTION_CCMRAM VRAM[VRAM_SIZE];

// XXX these are in SRAM2 to reduce contention with SRAM1 during DMA
unsigned char __attribute__((section (".sram2"))) row0[ROW_SAMPLES];
unsigned char __attribute__((section (".sram2"))) row1[ROW_SAMPLES];
unsigned char __attribute__((section (".sram2"))) audio0[512];
unsigned char __attribute__((section (".sram2"))) audio1[512];

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
    static const int startOfColorburstClocks = 72; // 80 - 3 * 4; // XXX magic number for current clock

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

#if 0
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
#endif

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
}

//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
// DMA Goop

volatile uint32_t SECTION_CCMRAM DMAFIFOUnderruns = 0;
volatile uint32_t SECTION_CCMRAM DMATransferErrors = 0;

volatile int SECTION_CCMRAM lineNumber = 0;
volatile int SECTION_CCMRAM frameNumber = 0;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

uint32_t oldLISR = 0;

// XXX There will need to be a DMA facility before another video subsystem is defined
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

    TIM9->CR1 = 0;            /* stop the timer */
    rowDMARemained[thisLineNumber] = 912 - TIM9->CNT / 7;
}

void dumpTIM1andDMA2S1regs()
{
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
}

void NTSCStartDMA()
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

    if(false) {
        dumpTIM1andDMA2S1regs();
    }

    status = HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
    if(status != HAL_OK) {
        printf("HAL_TIM_IC_Start status %d\n", status);
        panic();
    }
}

void NTSCStopDMA()
{
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;       /* disable DMA */
    HAL_TIM_Base_Stop(&htim1);
}

struct NTSCVideoSubsystem : VideoSubsystemDriver
{
    virtual void start();
    virtual void stop();
    virtual void setBackgroundColor(float r, float g, float b);
    virtual void setDebugRow(int row, const char *rowText);
    virtual int getModeCount() const;
    virtual VideoModeDriver& getModeDriver(int n) const;
    virtual void waitFrame();
};

void NTSCVideoSubsystem::getModeCount() const
{
    return driverCount;
}

void NTSCVideoSubsystem::setBackgroundColor(float r, float g, float b)
{
    backgroundColorWave = NTSCRGBToWave(r, g, b);
}

void NTSCVideoSubsystem::getModeDriver(int n) const
{
    if(n >= driverCount) {
        return nullptr;
    }
    return drivers[n];
}

void NTSCVideoSubsystem::waitFrame()
{
    // NTSC won't actually go lineNumber >= 525...
    while(!(lineNumber > 257 && lineNumber < 262) || (lineNumber > 520 && lineNumber < NTSC_FRAME_LINES)); // Wait for VBLANK; should do something smarter
}

void NTSCVideoSubsystem::start()
{
    NTSCCalculateParameters();

#if 1 // DEBUG
    printf("calculated NTSCLineClocks = %d\n", NTSCLineClocks);
    printf("calculated NTSCHSyncClocks = %d\n", NTSCHSyncClocks);
    printf("calculated NTSCFrontPorchClocks = %d\n", NTSCFrontPorchClocks);
    printf("calculated NTSCBackPorchClocks = %d\n", NTSCBackPorchClocks);
    printf("calculated NTSCEqPulseClocks = %d\n", NTSCEqPulseClocks);
    printf("calculated NTSCVSyncClocks = %d\n", NTSCVSyncClocks);
#endif

    NTSCGenerateLineBuffers();

    NTSCFillRowBuffer(0, 0, row0);
    
    NTSCStartDMA();
}

void NTSCVideoSubsystem::stop()
{
    NTSCStopDMA();
}

void NTSCVideoSubsystem::setDebugRow(int row, const char *rowText)
{
    if((row >= 0) && (row <= debugDisplayHeight)) {
        size_t to_copy = std::min(debugDisplayWidth, strlen(rowText));
        memcpy(debugDisplay[row], rowText, to_copy);
        memset(debugDisplay[row] + to_copy, ' ', debugDisplayWidth - to_copy);
    }
}

static NTSCVideoSubsystem NTSCVideo;

VideoSubsystemDriver* GetNTSCVideoSubsystem()
{
    return NTSCVideo;
}
