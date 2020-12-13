default:        main.dfu        hosted


FIRMWARE_VERSION    := $(shell date)

CXXFLAGS        +=       --std=c++17 -Wall -Werror
CFLAGS          +=      -std=c99 -Wall -Werror
OPT             =       -O2 -g -funroll-loops -ffast-math

# TOOLROOT        ?=      $(HOME)/trees/gcc-arm-none-eabi-5_3-2016q1
TOOLROOT        ?=      $(HOME)/packages/gcc-arm-none-eabi-7-2018-q2-update
# TOOLROOT        ?=      $(HOME)/Downloads/gcc-arm-none-eabi-9-2020-q2-update
# STM32FCUBEROOT  ?=      $(HOME)/trees/STM32Cube_FW_F4_V1.9.0
# STM32FCUBEROOT  ?=      $(HOME)/packages/STM32Cube_FW_F4_V1.21.0/
STM32FCUBEROOT  ?=      $(HOME)/STM32Cube/Repository/STM32Cube_FW_F7_V1.15.0/
# GCC_LIB         ?=      $(TOOLROOT)/lib/gcc/arm-none-eabi/5.3.1
GCC_LIB         ?=      $(TOOLROOT)/lib/gcc/arm-none-eabi/7.3.1

SOURCE_ROOT             ?=       $(shell pwd)

CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -mlittle-endian -mslow-flash-data
TARGET = STM32F746xx
STM32F4XX_CMSIS_INC_PATH        = $(STM32FCUBEROOT)/Drivers/CMSIS/Include/

MICRORAY_APP_OBJECT     = # apps/microray.o
MICRORAY_SOURCE         = # $(HOME)/trees/microray/
MICRORAY_LIB_DIR        = # -L$(MICRORAY_SOURCE)/build/arm
MICRORAY_INC_DIR        = # -I$(MICRORAY_SOURCE)/lib/ray
MICRORAY_LIB            = # -lmray

COBJECTS = \
        STM32Cube4/startup_stm32f746xx.o \
        STM32Cube4/system_stm32f7xx.o \
        STM32Cube4/stm32f7xx_it.o \
        STM32Cube4/stm32f7xx_hal_cortex.o \
        STM32Cube4/stm32f7xx_hal_dma.o \
        STM32Cube4/stm32f7xx_hal_gpio.o \
        STM32Cube4/stm32f7xx_hal_rcc.o \
        STM32Cube4/stm32f7xx_hal_uart.o \
        STM32Cube4/stm32f7xx_hal_spi.o \
        STM32Cube4/stm32f7xx_hal_tim.o \
        STM32Cube4/stm32f7xx_hal_tim_ex.o \
        STM32Cube4/stm32f7xx_hal.o \
        leds.o \
        monitor_queue.o \
        console_queue.o \
        uart.o \
        logprintf.o \
        syscalls.o \
        delay.o \
        sd_spi.o \
        crc7.o \
        ff.o \
        diskio.o \
        ff_unicode.o \
        ps2_keyboard.o \
        keyboard.o \
        textport.o \
        wolftextures.o \
        $(NULL)
        # need ff_unicode if ExFAT and LFN support are compiled in

NOTYET = \
        reset_button.o \
        $(NULL)

CXXOBJECTS = \
        $(MICRORAY_APP_OBJECT) \
        main.o \
        commandline.o \
        graphics.o \
        apps/showimage.o \
        apps/basic.o \
        apps/spin.o \
        apps/pocketgl.o \
        utility.o \
        segment_utility.o \
        apps/vile.o \
        apps/less.o \
        apps/wolf.o \
        $(NULL)

OBJECTS = $(CXXOBJECTS) $(COBJECTS)

SOURCES = $(COBJECTS:.o=.c) $(CXXOBJECTS:.o=.cpp)
DEPS = $(COBJECTS:.o=.d) $(CXXOBJECTS:.o=.d)

main.hex: main.elf
	$(TOOLROOT)/bin/arm-none-eabi-objcopy -O ihex $< $@

main.bin: main.elf
	$(TOOLROOT)/bin/arm-none-eabi-objcopy -O binary -S $< $@

-include $(DEPS)

main.elf:  $(OBJECTS)
	$(TOOLROOT)/bin/arm-none-eabi-g++ $(OPT) $(CORTEX_M4_HWFP_CC_FLAGS) -DSTM32F746xx $(MICRORAY_LIB_DIR) -TSTM32F746ZG_FLASH.ld  -lm -Wl,--gc-sections $^ -o $@ $(MICRORAY_LIB)

STM32Cube4/startup_stm32f746xx.o: STM32Cube4/startup_stm32f746xx.s
	$(TOOLROOT)/bin/arm-none-eabi-gcc -Werror -Wall $(OPT) $(CORTEX_M4_HWFP_CC_FLAGS)  $< -c -o $@

%.o: %.c
	$(TOOLROOT)/bin/arm-none-eabi-gcc $(CFLAGS) $(OPT) $(MICRORAY_INC_DIR) -I$(SOURCE_ROOT) -I$(SOURCE_ROOT)/STM32Cube4 -DIOBOARD_FIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin -I$(STM32F4XX_CMSIS_INC_PATH) -D$(TARGET) $(CORTEX_M4_HWFP_CC_FLAGS)  $< -c -o $@ -MMD

%.o: %.cpp
	$(TOOLROOT)/bin/arm-none-eabi-g++ $(CXXFLAGS) $(OPT) $(MICRORAY_INC_DIR) -I$(SOURCE_ROOT) -I$(SOURCE_ROOT)/STM32Cube4 -DIOBOARD_FIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin -I$(STM32F4XX_CMSIS_INC_PATH) -D$(TARGET) $(CORTEX_M4_HWFP_CC_FLAGS)  $< -c -o $@ -MMD

%.annotated: %.c
	$(TOOLROOT)/bin/arm-none-eabi-gcc -Wa,-adhln $(CFLAGS) $(OPT) -I$(SOURCE_ROOT) -I$(SOURCE_ROOT)/STM32Cube4 -DIOBOARD_FIRMWARE_VERSION="$(FIRMWARE_VERSION)" -fno-builtin -I$(STM32F4XX_CMSIS_INC_PATH) -D$(TARGET) $(CORTEX_M4_HWFP_CC_FLAGS)  $< -c > $@

clean:
	rm -f *.hex *.elf *.o $(OBJECTS) $(DEPS) hosted segment_test

main.dfu: main.hex
	python dfu-convert -i main.hex main.dfu

burn: main.dfu main.bin
	# time st-flash --reset write main.bin 0x8000000 # direct over ST-Link
	time dfu-util -d 0483:df11 -a 0 -D main.dfu # DFU requires reset with BOOT0

segment_test: segment_test.cpp segment_utility.cpp

hosted: apps/showimage.cpp utility.cpp graphics.cpp hosted.cpp apps/spin.cpp apps/pocketgl.cpp segment_utility.cpp gl_utility.cpp commandline.cpp
	c++ -DHOSTED $(CXXFLAGS) $(OPT) -I. -I/opt/local/include/ -L/opt/local/lib -std=c++17 -lglfw -framework OpenGL -framework Cocoa -framework IOkit $^ -o $@ 
	# c++ -DHOSTED $(CXXFLAGS) $(OPT) -I. -I/opt/local/include/ -L/opt/local/lib -std=c++17 $^ -o $@ -lglfw -lGLEW -lpthread -lGL

#apps/showimage.cpp utility.cpp graphics.cpp hosted.cpp apps/spin.cpp apps/pocketgl.cpp segment_utility.cpp -o hosted
