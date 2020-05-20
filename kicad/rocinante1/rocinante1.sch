EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Micro_SD_Card J?
U 1 1 5EA9D43D
P 10050 1475
F 0 "J?" H 10000 2192 50  0000 C CNN
F 1 "Micro_SD_Card" H 10000 2101 50  0000 C CNN
F 2 "" H 11200 1775 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 10050 1475 50  0001 C CNN
	1    10050 1475
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F7:STM32F746VGHx U?
U 1 1 5EAA31E9
P 2750 4175
F 0 "U?" H 2700 1286 50  0000 C CNN
F 1 "STM32F746VGHx" H 2700 1195 50  0000 C CNN
F 2 "Package_BGA:TFBGA-100_8x8mm_Layout10x10_P0.8mm" H 1950 1575 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00166116.pdf" H 2750 4175 50  0001 C CNN
	1    2750 4175
	1    0    0    -1  
$EndComp
Text GLabel 7950 1775 0    50   Input ~ 0
SPIO_DO
Text GLabel 7950 1575 0    50   Input ~ 0
SPIO_SCK
Text GLabel 7925 1375 0    50   Input ~ 0
SPIO_DI
Text GLabel 7925 1275 0    50   Input ~ 0
SPIO_CS
Wire Wire Line
	9150 1775 8375 1775
Wire Wire Line
	9150 1575 8575 1575
Wire Wire Line
	9150 1275 8975 1275
Wire Wire Line
	9150 1375 8775 1375
$Comp
L power:+3.3V #PWR?
U 1 1 5EAA80F1
P 8175 775
F 0 "#PWR?" H 8175 625 50  0001 C CNN
F 1 "+3.3V" H 8190 948 50  0000 C CNN
F 2 "" H 8175 775 50  0001 C CNN
F 3 "" H 8175 775 50  0001 C CNN
	1    8175 775 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8175 775  8175 800 
Wire Wire Line
	8175 1475 8750 1475
$Comp
L power:GND #PWR?
U 1 1 5EAA91FB
P 8200 2450
F 0 "#PWR?" H 8200 2200 50  0001 C CNN
F 1 "GND" H 8205 2277 50  0000 C CNN
F 2 "" H 8200 2450 50  0001 C CNN
F 3 "" H 8200 2450 50  0001 C CNN
	1    8200 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 1675 9150 1675
$Comp
L Device:R R?
U 1 1 5EAA9C58
P 8375 1025
F 0 "R?" V 8275 975 50  0000 L CNN
F 1 "68K" V 8375 950 50  0000 L CNN
F 2 "" V 8305 1025 50  0001 C CNN
F 3 "~" H 8375 1025 50  0001 C CNN
	1    8375 1025
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EAAAC11
P 8575 1025
F 0 "R?" V 8475 975 50  0000 L CNN
F 1 "68K" V 8575 950 50  0000 L CNN
F 2 "" V 8505 1025 50  0001 C CNN
F 3 "~" H 8575 1025 50  0001 C CNN
	1    8575 1025
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EAAB046
P 8775 1025
F 0 "R?" V 8675 975 50  0000 L CNN
F 1 "68K" V 8775 950 50  0000 L CNN
F 2 "" V 8705 1025 50  0001 C CNN
F 3 "~" H 8775 1025 50  0001 C CNN
	1    8775 1025
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EAAB5F5
P 8975 1025
F 0 "R?" V 8875 975 50  0000 L CNN
F 1 "68K" V 8975 950 50  0000 L CNN
F 2 "" V 8905 1025 50  0001 C CNN
F 3 "~" H 8975 1025 50  0001 C CNN
	1    8975 1025
	1    0    0    -1  
$EndComp
Wire Wire Line
	8375 875  8375 800 
Wire Wire Line
	8375 800  8175 800 
Connection ~ 8175 800 
Wire Wire Line
	8175 800  8175 1475
Wire Wire Line
	8575 850  8575 800 
Wire Wire Line
	8575 800  8375 800 
Connection ~ 8375 800 
Wire Wire Line
	8775 850  8775 800 
Wire Wire Line
	8775 800  8575 800 
Connection ~ 8575 800 
Wire Wire Line
	8975 875  8975 800 
Wire Wire Line
	8975 800  8775 800 
Connection ~ 8775 800 
Wire Wire Line
	8975 1175 8975 1275
Connection ~ 8975 1275
Wire Wire Line
	8975 1275 7925 1275
Wire Wire Line
	8775 1175 8775 1375
Connection ~ 8775 1375
Wire Wire Line
	8775 1375 7925 1375
Wire Wire Line
	8575 1175 8575 1575
Connection ~ 8575 1575
Wire Wire Line
	8575 1575 7950 1575
Wire Wire Line
	8375 1175 8375 1775
Connection ~ 8375 1775
Wire Wire Line
	8375 1775 7950 1775
$Comp
L Device:CP1 C?
U 1 1 5EAB05D5
P 8600 2325
F 0 "C?" V 8348 2325 50  0000 C CNN
F 1 "10µF" V 8439 2325 50  0000 C CNN
F 2 "" H 8600 2325 50  0001 C CNN
F 3 "~" H 8600 2325 50  0001 C CNN
	1    8600 2325
	0    1    1    0   
$EndComp
Wire Wire Line
	8750 2325 8750 1475
Connection ~ 8750 1475
Wire Wire Line
	8750 1475 9150 1475
Wire Wire Line
	8450 2325 8200 2325
Wire Wire Line
	8200 1675 8200 2325
Connection ~ 8200 2325
Wire Wire Line
	8200 2325 8200 2450
$EndSCHEMATC
