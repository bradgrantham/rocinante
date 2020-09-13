#!/usr/bin/env python3

import sys

# WEIRD_TIM1_MULT = 2.0
WEIRD_TIM1_MULT = 1.0

TARGET_FREQUENCY = 1000000 * 315 / 22 # 14318181818.0

clocksByConfig = []

part = "STM32F7"
# part = "STM32H7"

if part == "STM32F4":
    HSE = 16000000
    APB1MaxFreq = 42000000
    APB2MaxFreq = 84000000
    minAcceptableFreq = 140000000
    maxAcceptableFreq = 200000000
    VCOMaxFreq = 432000000 # ???
elif part == "STM32F7":
    HSE = 20004300 # Freq counter actually shows 4.00086 MHz on MCO2 for HSE / 5
    # HSE = 20000000
    # HSE = 24000000
    # HSE = 16000000
    # HSE = 8000000
    # HSE = 14318181
    # HSE = 25000000
    # HSE = 28636360 # 8x colorburst
    APB1MaxFreq = 53000000
    APB2MaxFreq = 106000000
    minAcceptableFreq = 180000000
    maxAcceptableFreq = 250000000
    VCOMaxFreq = 432000000
else: # "STM32H7"
    # HSE = 24000000
    # HSE = 16000000
    # HSE = 8000000
    # HSE = 14318181
    HSE = 25000000
    # HSE = 28636360 # 8x colorburst
    APB1MaxFreq = 120000000
    APB2MaxFreq = 120000000
    minAcceptableFreq = 400000000
    maxAcceptableFreq = 500000000
    VCOMaxFreq = 836000000

StayInSpec = True

for PLLM in range(2, 64):
    # VCO input frequency = PLL input clock frequency / PLLM with 2 <= PLLM <= 63
    VCO_INPUT = HSE / PLLM

    # PLLM output must be >= .95MHz and <= 2.1Mhz
    if StayInSpec and (VCO_INPUT < 950000 or VCO_INPUT > 2100000):
        continue

    for PLLN in range(50, 433):
        VCO_OUTPUT = VCO_INPUT * PLLN

        # PLLN output must be >= 100 MHz and <= 432 MHz, see note for RCC_PLLCFGR
        if StayInSpec and ((VCO_OUTPUT < 100000000) or (VCO_OUTPUT > VCOMaxFreq)):
            continue

        for PLLP in (2, 4, 6, 8): # (2, 4, 6, 8):
            PLL_OUTPUT = VCO_OUTPUT / PLLP

            # PLLP output must be >= 24 MHz and <= 216 MHz
            if StayInSpec and (PLL_OUTPUT < 24000000 or VCO_INPUT > 216000000):
                continue

            if (PLL_OUTPUT > minAcceptableFreq) and (PLL_OUTPUT < maxAcceptableFreq):

                # XXX need to add AHB prescaler

                APB1_DIV = 1
                while PLL_OUTPUT / APB1_DIV > APB1MaxFreq:
                    APB1_DIV = APB1_DIV * 2
                if APB1_DIV > 16:
                    continue

                APB2_DIV = 1
                while PLL_OUTPUT / APB2_DIV > APB2MaxFreq:
                    APB2_DIV = APB2_DIV * 2
                if APB1_DIV > 16:
                    continue

                # XXX not taking into account that TIM input clock depends on TIMPRE

                # "Reset and Clock Control" "Clock Tree" - "if(APBx presc = 1) x1 else x2"
                if APB2_DIV == 1:
                    TIM_FREQ = PLL_OUTPUT
                else:
                    TIM_FREQ = PLL_OUTPUT / APB2_DIV * 2
                # print("TIM_FREQ = %d\n" % TIM_FREQ)

                # for TIM_CLOCKS in (TIM_FREQ // (TARGET_FREQUENCY * WEIRD_TIM1_MULT), TIM_FREQ // (TARGET_FREQUENCY * WEIRD_TIM1_MULT) + 1):
                for TIM_CLOCKS in range(1, 30):
                    # print("TIM_CLOCKS = %d\n" % TIM_CLOCKS)

                    if TIM_CLOCKS > 0:
                        error = (TIM_FREQ / (TIM_CLOCKS * WEIRD_TIM1_MULT) - TARGET_FREQUENCY) / TARGET_FREQUENCY
                        # print("error = (%f / (%d * %d) - %d) / %d" % (TIM_FREQ, TIM_CLOCKS, WEIRD_TIM1_MULT, TARGET_FREQUENCY, TARGET_FREQUENCY))
                        actual = TIM_FREQ / (TIM_CLOCKS * WEIRD_TIM1_MULT )
                        # print("actual = %f, error = %f" % (actual, error))
                        if True: # (actual / TARGET_FREQUENCY < 1.001) and (TARGET_FREQUENCY / actual < 1.001):
                            clocksByConfig.append((PLL_OUTPUT, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, APB1_DIV, APB2_DIV, actual, error))

# clocksByConfig.sort(key=lambda elem: abs(elem[8])) # sort by dot clock frequency
clocksByConfig.sort(key=lambda elem: -elem[0]) # sort by CPU clock
# clocksByConfig.sort(key=lambda elem: abs(elem[9])) # sort by absolute error
# clocksByConfig.sort(key=lambda elem: elem[9]) # sort by error

print("// PLL_OUTPUT MHz, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, HCLK_DIV, APB1_DIV, APB2_DIV, actual color clock MHz, error")
for (PLL_OUTPUT, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, APB1_DIV, APB2_DIV, actual, error) in clocksByConfig:
    print("%.2f, %d, %d, %d, %d, %d, RCC_SYSCLK_DIV1, RCC_HCLK_DIV%d, RCC_HCLK_DIV%d, %f, %f" % (PLL_OUTPUT / 1000000.0, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, APB1_DIV, APB2_DIV, actual / 1000000.0, error))
