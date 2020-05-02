#!/usr/bin/env python3
# HSE = 16002676.76554237158493229676
HSE = 16000000
# HSE = 15963353.949525948274897431152

clocksByConfig = []

minAcceptableMHz = 180
maxAcceptableMHz = 220

for PLLM in range(2, 64):
    # VCO input frequency = PLL input clock frequency / PLLM with 2 <= PLLM <= 63
    VCO_INPUT = HSE / PLLM
    for PLLN in range(50, 432):
        VCO_OUTPUT = VCO_INPUT * PLLN
        for PLLP in (2,): # (2, 4, 6, 8):
            PLL_OUTPUT = VCO_OUTPUT / PLLP
            if PLL_OUTPUT > minAcceptableMHz * 1000000 and PLL_OUTPUT < maxAcceptableMHz * 1000000:
                TIM_CLOCKS = int(PLL_OUTPUT / (14318180 * 2))
                for dbeats in (0, 1):
                    error = (PLL_OUTPUT / ((TIM_CLOCKS + dbeats) * 2) - 14318180) / 14318180
                    actual = PLL_OUTPUT / ((TIM_CLOCKS + dbeats) * 2 ) / 4
                    if True: # actual / 3579545 < 1.05 and 3579545 / actual < 1.05:
                        clocksByConfig.append((PLL_OUTPUT, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS + dbeats, actual, error))

# clocksByConfig.sort(key=lambda elem: abs(elem[7]))
clocksByConfig.sort(key=lambda elem: elem[7])

print("PLL_OUTPUT MHz, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, actual color clock MHz, error")
for (PLL_OUTPUT, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, actual, error) in clocksByConfig:
    print("%.2f, %d, %d, %d, %d, %d, %f, %f" % (PLL_OUTPUT / 1000000.0, HSE, PLLM, PLLN, PLLP, TIM_CLOCKS, actual / 1000000.0, error))
