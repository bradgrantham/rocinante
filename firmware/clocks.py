#!/usr/bin/env python3

WEIRD_TIM1_MULT = 2

clocksByConfig = []

part = "STM32F7"

if part == "STM32F4":
    HSE = 16000000
    APB1MaxFreq = 42000000
    APB2MaxFreq = 84000000
    minAcceptableFreq = 140000000
    maxAcceptableFreq = 200000000
else: # "STM32F7"
    # HSE = 16000000
    HSE = 8000000
    # HSE = 14318180
    # HSE = 25000000
    # HSE = 28636360 # 8x colorburst
    APB1MaxFreq = 53000000
    APB2MaxFreq = 106000000
    minAcceptableFreq = 180000000
    maxAcceptableFreq = 225000000

StayInSpec = True

for PLLM in range(2, 64):
    # VCO input frequency = PLL input clock frequency / PLLM with 2 <= PLLM <= 63
    VCO_INPUT = HSE / PLLM

    # PLLM output must be >= .95MHz and <= 2.1Mhz
    if StayInSpec and (VCO_INPUT < 950000 or VCO_INPUT > 2100000):
        continue

    for PLLN in range(50, 432):
        VCO_OUTPUT = VCO_INPUT * PLLN

        # PLLN output must be >= 100 MHz and <= 432 MHz, see note for RCC_PLLCFGR
        if StayInSpec and ((VCO_OUTPUT < 100000000) or (VCO_OUTPUT > 432000000)):
            continue

        for PLLP in (2, 4, 6, 8): # (2, 4, 6, 8):
            PLL_OUTPUT = VCO_OUTPUT / PLLP

            # PLLP output must be >= 24 MHz and <= 216 MHz
            if StayInSpec and (PLL_OUTPUT < 24000000 or VCO_INPUT > 216000000):
                continue

            if (PLL_OUTPUT > minAcceptableFreq) and (PLL_OUTPUT < maxAcceptableFreq):

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

                # XXX TIM input clock depends on APB multipliers and TIMPRE, not taking that into account here.

                if APB2_DIV == 1:
                    TIM_FREQ = PLL_OUTPUT
                else:
                    TIM_FREQ = PLL_OUTPUT / APB2_DIV * 2

                TIM_CLOCKS = TIM_FREQ // (14318180 * WEIRD_TIM1_MULT)

                for dbeats in (0, 1):
                    TIM_CORRECTED = TIM_CLOCKS + dbeats
                    if TIM_CORRECTED > 0:
                        error = (TIM_FREQ / (TIM_CORRECTED * WEIRD_TIM1_MULT) - 14318180) / 14318180
                        actual = TIM_FREQ / (TIM_CORRECTED * WEIRD_TIM1_MULT ) / 4
                        if (actual / 3579545 < 1.025) and (3579545 / actual < 1.025):
                            clocksByConfig.append((PLL_OUTPUT, HSE, PLLM, PLLN, PLLP, TIM_CORRECTED, APB1_DIV, APB2_DIV, actual, error))

clocksByConfig.sort(key=lambda elem: abs(elem[9]))
# clocksByConfig.sort(key=lambda elem: elem[9])

print("// PLL_OUTPUT MHz, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, HCLK_DIV, APB1_DIV, APB2_DIV, actual color clock MHz, error")
for (PLL_OUTPUT, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, APB1_DIV, APB2_DIV, actual, error) in clocksByConfig:
    print("%.2f, %d, %d, %d, %d, %d, RCC_SYSCLK_DIV1, RCC_HCLK_DIV%d, RCC_HCLK_DIV%d, %f, %f" % (PLL_OUTPUT / 1000000.0, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, APB1_DIV, APB2_DIV, actual / 1000000.0, error))
