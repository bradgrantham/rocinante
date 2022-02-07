# Rev1

1. USB + power

     - Glue 2.1x5.5mm barrel jack to underside left
     - Route +5 from barrel to TP2
     * Route GND from barrel to I2S GND
2. Composite, https://github.com/bradgrantham/rocinante/issues/1

     * Cut trace from composite pin 1 (shield / GND) to C20
     * Route from composite pin 2 (signal) to C20
     * Add GND from composite pin 1 to VGA shield GND
3. USER1 conflicts with SPI4_CK/PE2 (which can't be disabled through MX - can it work if disabled by hand?)

     * Cut USER1 trace to PE2
     * Route to TIM3_CH2 / PB5 (MOUSE CLK)
4. Add 21.47277MHz to TIM1_CH3/PA10 (or TIM1_CH1/PA8 if PA10 not possible) for TMS9918A clock

     * Cut PA10/USART1_RX before R46 and join with USART1_TX

     * Glue part near PA10

     * Route CLK to PA10/pin121

     * Route to PA8 / USART1_CK (on current board)

     * Route GND and +3.3V

     * HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_1); status = HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_3); to sync to 21.47MHz
