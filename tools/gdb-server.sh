#!/bin/bash

# ST_PATH=/opt/st/stm32cubeide_1.4.0/plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.linux64_1.4.0.202007081208/
ST_PATH=/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.macos64_1.4.0.202007081208/

# export LD_LIBRARY_PATH=$ST_PATH/tools/bin/native/linux_x64
ST_TOOL_PATH=$ST_PATH/tools/bin
CP_TOOL_PATH=/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_1.4.0.202007081208/tools/bin

CPU_FREQ=480 # 384 # MHz
CPU_FREQ_HZ=`expr $CPU_FREQ \* 1000000`

#### Options with SWV, Core clock 168MHz, SWO Clock 1 MHz
#$ST_TOOL_PATH/ST-LINK_gdbserver -c $ST_TOOL_PATH/config.txt 

# Enables SWO port
#ST_SWO="-z 61235"

(cd $ST_TOOL_PATH; ./ST-LINK_gdbserver -f /tmp/session.log -e -d $ST_SWO -a $CPU_FREQ_HZ -b $CPU_FREQ -cp $CP_TOOL_PATH $1)

