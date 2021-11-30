EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 9 10
Title "SATA sniffer"
Date "2021-11-29"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 "FPGA boot flash and level shifting"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L memory-azonenberg:W25Q80BV U15
U 1 1 61B135A7
P 6150 4000
F 0 "U15" H 6150 4687 60  0000 C CNN
F 1 "W25Q128JV" H 6150 4581 60  0000 C CNN
F 2 "azonenberg_pcb:DFN_8_1.27MM_6x5MM" H 6150 4000 60  0001 C CNN
F 3 "" H 6150 4000 60  0000 C CNN
	1    6150 4000
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:74AVC1T45 U11
U 1 1 61B138D5
P 1800 2100
F 0 "U11" H 2075 2697 60  0000 C CNN
F 1 "74AVCH1T45" H 2075 2591 60  0000 C CNN
F 2 "azonenberg_pcb:DFN_6_0.5MM_1.45x1.00MM" H 1800 2100 60  0001 C CNN
F 3 "" H 1800 2100 60  0001 C CNN
	1    1800 2100
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:74AVC1T45 U12
U 1 1 61B13B75
P 1800 2850
F 0 "U12" H 2075 3447 60  0000 C CNN
F 1 "74AVCH1T45" H 2075 3341 60  0000 C CNN
F 2 "azonenberg_pcb:DFN_6_0.5MM_1.45x1.00MM" H 1800 2850 60  0001 C CNN
F 3 "" H 1800 2850 60  0001 C CNN
	1    1800 2850
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:74AVC1T45 U13
U 1 1 61B14167
P 1800 3600
F 0 "U13" H 2075 4197 60  0000 C CNN
F 1 "74AVCH1T45" H 2075 4091 60  0000 C CNN
F 2 "azonenberg_pcb:DFN_6_0.5MM_1.45x1.00MM" H 1800 3600 60  0001 C CNN
F 3 "" H 1800 3600 60  0001 C CNN
	1    1800 3600
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:74AVC1T45 U14
U 1 1 61B14850
P 1800 4350
F 0 "U14" H 2075 4947 60  0000 C CNN
F 1 "74AVCH1T45" H 2075 4841 60  0000 C CNN
F 2 "azonenberg_pcb:DFN_6_0.5MM_1.45x1.00MM" H 1800 4350 60  0001 C CNN
F 3 "" H 1800 4350 60  0001 C CNN
	1    1800 4350
	1    0    0    -1  
$EndComp
Text HLabel 1600 1750 0    50   Input ~ 0
1V35
Text Label 1600 2500 2    50   ~ 0
1V35
Text Label 1600 3250 2    50   ~ 0
1V35
Text HLabel 1600 4000 0    50   Input ~ 0
1V8
Text HLabel 1600 4300 0    50   Input ~ 0
FLASH_SCK
Text Label 2550 4300 0    50   ~ 0
FLASH_SCK_SHIFT
Text HLabel 2550 1750 2    50   Input ~ 0
3V3
Text Label 2550 2500 0    50   ~ 0
3V3
Text Label 2550 3250 0    50   ~ 0
3V3
Text Label 2550 4000 0    50   ~ 0
3V3
Text Label 2550 4100 0    50   ~ 0
GND
Text Label 2550 3350 0    50   ~ 0
GND
Text Label 2550 2600 0    50   ~ 0
GND
Text Label 2550 3550 0    50   ~ 0
FLASH_SI_SHIFT
Text Label 2550 2800 0    50   ~ 0
FLASH_SO_SHIFT
Text Label 2550 2050 0    50   ~ 0
FLASH_CS_SHIFT
Text Label 6900 3600 0    50   ~ 0
3V3
Text Label 5400 3900 2    50   ~ 0
GND
Text Label 5100 3800 2    50   ~ 0
3V3
Text Label 6900 3800 0    50   ~ 0
FLASH_SCK_SHIFT
Text Label 6900 3900 0    50   ~ 0
FLASH_SI_SHIFT
Text Label 5400 3700 2    50   ~ 0
FLASH_SO_SHIFT
Text Label 5400 3600 2    50   ~ 0
FLASH_CS_SHIFT
$Comp
L device:R R26
U 1 1 61B168B3
P 1450 2800
F 0 "R26" V 1350 2650 50  0000 C CNN
F 1 "33" V 1450 2800 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1380 2800 50  0001 C CNN
F 3 "" H 1450 2800 50  0001 C CNN
	1    1450 2800
	0    1    1    0   
$EndComp
Text HLabel 1300 2800 0    50   Output ~ 0
FLASH_SO
Text HLabel 1600 2050 0    50   Input ~ 0
FLASH_CS
Text HLabel 1600 3550 0    50   Input ~ 0
FLASH_SI
$Comp
L device:C C72
U 1 1 61B1A41D
P 1800 5250
F 0 "C72" H 1915 5296 50  0000 L CNN
F 1 "4.7 uF" H 1915 5205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 1838 5100 50  0001 C CNN
F 3 "" H 1800 5250 50  0001 C CNN
	1    1800 5250
	1    0    0    -1  
$EndComp
Text Label 1800 5400 2    50   ~ 0
GND
Text Label 1800 5100 2    50   ~ 0
3V3
$Comp
L device:C C73
U 1 1 61B1AC49
P 1800 5800
F 0 "C73" H 1915 5846 50  0000 L CNN
F 1 "4.7 uF" H 1915 5755 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 1838 5650 50  0001 C CNN
F 3 "" H 1800 5800 50  0001 C CNN
	1    1800 5800
	1    0    0    -1  
$EndComp
Text Label 1800 5650 2    50   ~ 0
1V8
Text Label 1800 5950 2    50   ~ 0
GND
$Comp
L device:C C74
U 1 1 61B1B69D
P 1800 6400
F 0 "C74" H 1915 6446 50  0000 L CNN
F 1 "4.7 uF" H 1915 6355 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 1838 6250 50  0001 C CNN
F 3 "" H 1800 6400 50  0001 C CNN
	1    1800 6400
	1    0    0    -1  
$EndComp
Text Label 1800 6250 2    50   ~ 0
1V35
Text Label 1800 6550 2    50   ~ 0
GND
Wire Wire Line
	1600 1750 1600 1850
Wire Wire Line
	1600 3250 1600 3350
Wire Wire Line
	1600 4000 1600 4100
Text Label 1600 2600 2    50   ~ 0
GND
$Comp
L device:C C75
U 1 1 61B1D6AB
P 2300 5250
F 0 "C75" H 2415 5296 50  0000 L CNN
F 1 "0.47 uF" H 2415 5205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2338 5100 50  0001 C CNN
F 3 "" H 2300 5250 50  0001 C CNN
	1    2300 5250
	1    0    0    -1  
$EndComp
$Comp
L device:C C78
U 1 1 61B1DBDE
P 2900 5250
F 0 "C78" H 3015 5296 50  0000 L CNN
F 1 "0.47 uF" H 3015 5205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2938 5100 50  0001 C CNN
F 3 "" H 2900 5250 50  0001 C CNN
	1    2900 5250
	1    0    0    -1  
$EndComp
$Comp
L device:C C80
U 1 1 61B1E084
P 3500 5250
F 0 "C80" H 3615 5296 50  0000 L CNN
F 1 "0.47 uF" H 3615 5205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 3538 5100 50  0001 C CNN
F 3 "" H 3500 5250 50  0001 C CNN
	1    3500 5250
	1    0    0    -1  
$EndComp
$Comp
L device:C C82
U 1 1 61B1E37E
P 4100 5250
F 0 "C82" H 4215 5296 50  0000 L CNN
F 1 "0.47 uF" H 4215 5205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 5100 50  0001 C CNN
F 3 "" H 4100 5250 50  0001 C CNN
	1    4100 5250
	1    0    0    -1  
$EndComp
$Comp
L device:C C83
U 1 1 61B1E5B0
P 4650 5250
F 0 "C83" H 4765 5296 50  0000 L CNN
F 1 "0.47 uF" H 4765 5205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4688 5100 50  0001 C CNN
F 3 "" H 4650 5250 50  0001 C CNN
	1    4650 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 5100 4100 5100
Connection ~ 2300 5100
Wire Wire Line
	2300 5100 1800 5100
Connection ~ 2900 5100
Wire Wire Line
	2900 5100 2300 5100
Connection ~ 3500 5100
Wire Wire Line
	3500 5100 2900 5100
Connection ~ 4100 5100
Wire Wire Line
	4100 5100 3500 5100
Wire Wire Line
	1800 5400 2300 5400
Connection ~ 2300 5400
Wire Wire Line
	2300 5400 2900 5400
Connection ~ 2900 5400
Wire Wire Line
	2900 5400 3500 5400
Connection ~ 3500 5400
Wire Wire Line
	3500 5400 4100 5400
Connection ~ 4100 5400
Wire Wire Line
	4100 5400 4650 5400
$Comp
L device:C C76
U 1 1 61B1EC0F
P 2300 5800
F 0 "C76" H 2415 5846 50  0000 L CNN
F 1 "0.47 uF" H 2415 5755 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2338 5650 50  0001 C CNN
F 3 "" H 2300 5800 50  0001 C CNN
	1    2300 5800
	1    0    0    -1  
$EndComp
$Comp
L device:C C77
U 1 1 61B1EEC4
P 2300 6400
F 0 "C77" H 2415 6446 50  0000 L CNN
F 1 "0.47 uF" H 2415 6355 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2338 6250 50  0001 C CNN
F 3 "" H 2300 6400 50  0001 C CNN
	1    2300 6400
	1    0    0    -1  
$EndComp
$Comp
L device:C C79
U 1 1 61B1F253
P 2900 6400
F 0 "C79" H 3015 6446 50  0000 L CNN
F 1 "0.47 uF" H 3015 6355 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2938 6250 50  0001 C CNN
F 3 "" H 2900 6400 50  0001 C CNN
	1    2900 6400
	1    0    0    -1  
$EndComp
$Comp
L device:C C81
U 1 1 61B1F5D2
P 3500 6400
F 0 "C81" H 3615 6446 50  0000 L CNN
F 1 "0.47 uF" H 3615 6355 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 3538 6250 50  0001 C CNN
F 3 "" H 3500 6400 50  0001 C CNN
	1    3500 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6250 2900 6250
Connection ~ 2300 6250
Wire Wire Line
	2300 6250 1800 6250
Connection ~ 2900 6250
Wire Wire Line
	2900 6250 2300 6250
Wire Wire Line
	1800 5950 2300 5950
Wire Wire Line
	2300 5650 1800 5650
Wire Wire Line
	3500 6550 2900 6550
Connection ~ 2300 6550
Wire Wire Line
	2300 6550 1800 6550
Connection ~ 2900 6550
Wire Wire Line
	2900 6550 2300 6550
$Comp
L device:R R27
U 1 1 61B2297E
P 5250 3400
F 0 "R27" V 5150 3400 50  0000 C CNN
F 1 "10K" V 5250 3400 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5180 3400 50  0001 C CNN
F 3 "" H 5250 3400 50  0001 C CNN
	1    5250 3400
	0    1    1    0   
$EndComp
Text Label 5100 3400 2    50   ~ 0
3V3
Wire Wire Line
	5400 3400 5400 3600
Text HLabel 2550 1850 2    50   Input ~ 0
GND
$Comp
L device:R R70
U 1 1 61DEFFC0
P 7250 3700
F 0 "R70" V 7150 3700 50  0000 C CNN
F 1 "10K" V 7250 3700 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 7180 3700 50  0001 C CNN
F 3 "" H 7250 3700 50  0001 C CNN
	1    7250 3700
	0    1    1    0   
$EndComp
Text Label 7400 3700 0    50   ~ 0
3V3
Wire Wire Line
	7100 3700 6900 3700
$Comp
L device:R R69
U 1 1 61DF0BC4
P 5250 3800
F 0 "R69" V 5200 3950 50  0000 C CNN
F 1 "10K" V 5250 3800 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5180 3800 50  0001 C CNN
F 3 "" H 5250 3800 50  0001 C CNN
	1    5250 3800
	0    1    1    0   
$EndComp
$EndSCHEMATC
