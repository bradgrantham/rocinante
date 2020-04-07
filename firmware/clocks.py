#!/usr/bin/env python3
HSE = 16000000

clocksByConfig = []

for PLLM in range(2, 64):
    # VCO input frequency = PLL input clock frequency / PLLM with 2 <= PLLM <= 63
    VCO_INPUT = HSE / PLLM
    for PLLN in range(50, 432):
        VCO_OUTPUT = VCO_INPUT * PLLN
        for PLLP in (2, 4, 6, 8):
            PLL_OUTPUT = VCO_OUTPUT / PLLP
            clocksByConfig.append((PLL_OUTPUT, HSE, PLLM, PLLN, PLLP))

print("PLL_OUTPUT MHz, HSE, PLLM, PLLN, PLLP")
for (PLL_OUTPUT, HSE, PLLM, PLLN, PLLP) in clocksByConfig:
    print("%f, %d, %d, %d, %d" % (PLL_OUTPUT / 1000000, HSE, PLLM, PLLN, PLLP))
