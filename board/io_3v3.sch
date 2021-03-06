EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 10
Title "SATA sniffer"
Date "2022-02-15"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U?
U 6 1 6191126B
P 2000 6050
AR Path="/618C589A/6191126B" Ref="U?"  Part="3" 
AR Path="/61909AE6/6191126B" Ref="U5"  Part="6" 
F 0 "U5" H 2000 5950 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 2000 5850 60  0000 L CNN
F 2 "azonenberg_pcb:BGA_484_22x22_FULLARRAY_1MM" H 2000 5750 60  0001 C CNN
F 3 "" H 2000 5750 60  0000 C CNN
	6    2000 6050
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:CONN_SFF8087 J?
U 1 1 6198560F
P 7050 3150
AR Path="/61923389/6198560F" Ref="J?"  Part="1" 
AR Path="/61909AE6/6198560F" Ref="J8"  Part="1" 
F 0 "J8" H 7400 5465 50  0000 C CNN
F 1 "CONN_SFF8087" H 7400 5374 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SFF8087_MOLEX_75783-0140" H 7050 3150 50  0001 C CNN
F 3 "" H 7050 3150 50  0001 C CNN
	1    7050 3150
	1    0    0    -1  
$EndComp
Text HLabel 1800 2050 0    50   Output ~ 0
SFP_RS0
Text HLabel 1800 2350 0    50   Output ~ 0
SFP_RS1
Text HLabel 6150 3100 2    50   Output ~ 0
GND
Text Label 7850 3100 0    50   ~ 0
GND
Text Label 6950 2000 2    50   ~ 0
GND
Wire Wire Line
	6950 2000 6950 2100
Connection ~ 6950 2100
Wire Wire Line
	6950 2100 6950 2200
Connection ~ 6950 2200
Wire Wire Line
	6950 2200 6950 2300
Connection ~ 6950 2300
Wire Wire Line
	6950 2300 6950 2400
Connection ~ 6950 2400
Wire Wire Line
	6950 2400 6950 2500
Connection ~ 6950 2500
Wire Wire Line
	6950 2500 6950 2600
Connection ~ 6950 2600
Wire Wire Line
	6950 2600 6950 2700
Connection ~ 6950 2700
Wire Wire Line
	6950 2700 6950 2800
Connection ~ 6950 2800
Wire Wire Line
	6950 2800 6950 2900
Connection ~ 6950 2900
Wire Wire Line
	6950 2900 6950 3000
Connection ~ 6950 3000
Wire Wire Line
	6950 3000 6950 3100
Text Label 5250 2000 2    50   ~ 0
GND
Text Label 5250 1100 2    50   ~ 0
LA0_12V0
Text Label 6950 1100 2    50   ~ 0
LA1_12V0
Wire Wire Line
	6950 1100 6950 1200
Connection ~ 6950 1200
Wire Wire Line
	6950 1200 6950 1300
Text Label 8000 3650 0    50   ~ 0
LA0_12V0
Text Label 5250 1400 2    50   ~ 0
LA0_UART_TX
Text Label 6950 1400 2    50   ~ 0
LA1_UART_TX
Text Label 5250 1500 2    50   ~ 0
LA0_PRESENCE_DETECT
Text Label 6950 1500 2    50   ~ 0
LA1_PRESENCE_DETECT
Text Label 5250 1600 2    50   ~ 0
LA0_12V0
Text Label 5250 1800 2    50   ~ 0
LA0_12V0
Text Label 6950 1600 2    50   ~ 0
LA1_12V0
Text Label 6950 1800 2    50   ~ 0
LA1_12V0
Text Label 5250 1700 2    50   ~ 0
LA0_UART_RX
Text Label 6950 1700 2    50   ~ 0
LA1_UART_RX
Text Label 1800 4550 2    50   ~ 0
LA0_UART_TX
Text Label 1800 2950 2    50   ~ 0
LA0_UART_RX
Text Label 1800 4250 2    50   ~ 0
LA1_UART_RX
Text Label 1800 1350 2    50   ~ 0
LA1_UART_TX
Text Label 6150 3950 2    50   ~ 0
LA0_12V0_EN
Text Label 1800 3350 2    50   ~ 0
LA0_12V0_EN
Text Label 1800 3650 2    50   ~ 0
LA1_12V0_EN
Text Label 7150 3950 0    50   ~ 0
LA0_12V0_FAULT
Text Label 1800 3550 2    50   ~ 0
LA1_12V0_FAULT
Text Label 1800 3250 2    50   ~ 0
LA0_12V0_FAULT
Text HLabel 1800 2250 0    50   Input ~ 0
SFP_RX_LOS
Text HLabel 1800 1950 0    50   Input ~ 0
SFP_MOD_ABS
$Comp
L device:R R53
U 1 1 61A48776
P 2150 7100
F 0 "R53" V 1943 7100 50  0000 C CNN
F 1 "100K" V 2034 7100 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2080 7100 50  0001 C CNN
F 3 "" H 2150 7100 50  0001 C CNN
	1    2150 7100
	0    1    1    0   
$EndComp
Text Label 2300 6750 0    50   ~ 0
LA0_PRESENCE_DETECT
$Comp
L device:R R54
U 1 1 61A4903B
P 2150 6750
F 0 "R54" V 1943 6750 50  0000 C CNN
F 1 "100K" V 2034 6750 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2080 6750 50  0001 C CNN
F 3 "" H 2150 6750 50  0001 C CNN
	1    2150 6750
	0    1    1    0   
$EndComp
Text Label 2000 7100 2    50   ~ 0
3V3
Text Label 2300 7100 0    50   ~ 0
LA1_PRESENCE_DETECT
Text HLabel 1800 1150 0    50   BiDi ~ 0
RAM_SDA
Text HLabel 1800 1650 0    50   Output ~ 0
RAM_SCL
Text HLabel 1800 1750 0    50   Output ~ 0
SFP_SCL
Text HLabel 1800 1450 0    50   BiDi ~ 0
SFP_SDA
Text HLabel 2000 6750 0    50   Input ~ 0
3V3
Text HLabel 1800 1550 0    50   Input ~ 0
SFP_TX_FAULT
Text HLabel 1800 1850 0    50   Output ~ 0
SFP_TX_DISABLE
Text Label 1800 4050 2    50   ~ 0
LED0
Text Label 1800 3950 2    50   ~ 0
LED1
Text Label 1800 3850 2    50   ~ 0
LED2
Text Label 1800 4450 2    50   ~ 0
LED3
Text Label 4150 6400 2    50   ~ 0
PMOD_DQ0
Text Label 4150 6500 2    50   ~ 0
PMOD_DQ1
Text Label 4150 6600 2    50   ~ 0
PMOD_DQ2
Text Label 4150 6700 2    50   ~ 0
PMOD_DQ3
Text Label 4150 6800 2    50   ~ 0
PMOD_DQ4
Text Label 4150 6900 2    50   ~ 0
PMOD_DQ5
Text Label 4150 7000 2    50   ~ 0
PMOD_DQ6
Text Label 4150 7100 2    50   ~ 0
PMOD_DQ7
Text Label 1800 3450 2    50   ~ 0
PMOD_DQ0
Text Label 1800 2150 2    50   ~ 0
PMOD_DQ1
Text Label 1800 2450 2    50   ~ 0
PMOD_DQ2
Text Label 1800 4350 2    50   ~ 0
PMOD_DQ3
Text Label 1800 2550 2    50   ~ 0
PMOD_DQ4
Text Label 1800 1250 2    50   ~ 0
PMOD_DQ5
Text Label 1800 3750 2    50   ~ 0
PMOD_DQ6
Text Label 1800 2750 2    50   ~ 0
PMOD_DQ7
Text HLabel 1800 3150 0    50   Output ~ 0
ETH_LED2_P
Text HLabel 1800 3050 0    50   Output ~ 0
ETH_LED1_P
$Comp
L special-azonenberg:PMOD_HOST J12
U 1 1 61A6CF3D
P 4350 7150
F 0 "J12" H 4350 7100 60  0000 L CNN
F 1 "PMOD_HOST" H 4350 7000 60  0000 L CNN
F 2 "azonenberg_pcb:CONN_HEADER_2.54MM_2x6_RA_PMOD_HOST" H 4350 7150 60  0001 C CNN
F 3 "" H 4350 7150 60  0001 C CNN
	1    4350 7150
	1    0    0    -1  
$EndComp
Text Label 4150 5900 2    50   ~ 0
3V3
Wire Wire Line
	4150 5900 4150 6000
Text Label 4150 6100 2    50   ~ 0
GND
Wire Wire Line
	4150 6100 4150 6200
$Comp
L device:R R55
U 1 1 61A74947
P 5400 4900
F 0 "R55" H 5470 4946 50  0000 L CNN
F 1 "1M" H 5470 4855 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5330 4900 50  0001 C CNN
F 3 "" H 5400 4900 50  0001 C CNN
	1    5400 4900
	1    0    0    -1  
$EndComp
Text HLabel 5400 3650 0    50   Input ~ 0
12V0
Text Label 6150 3750 2    50   ~ 0
GND
Wire Wire Line
	5400 3650 6150 3650
Text Label 5900 5900 2    50   ~ 0
LA0_12V0_EN
Text Label 6200 5900 0    50   ~ 0
GND
Text Label 5900 6100 2    50   ~ 0
LA1_12V0_EN
$Comp
L device:R R60
U 1 1 61A7AD88
P 6050 5900
F 0 "R60" V 5950 5900 50  0000 C CNN
F 1 "10K" V 6050 5900 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5980 5900 50  0001 C CNN
F 3 "" H 6050 5900 50  0001 C CNN
	1    6050 5900
	0    1    1    0   
$EndComp
Text Label 6200 6100 0    50   ~ 0
GND
$Comp
L device:R R56
U 1 1 61A7EF43
P 5400 5400
F 0 "R56" H 5470 5446 50  0000 L CNN
F 1 "93.1K" H 5470 5355 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5330 5400 50  0001 C CNN
F 3 "" H 5400 5400 50  0001 C CNN
	1    5400 5400
	1    0    0    -1  
$EndComp
Text Label 5250 4450 2    50   ~ 0
GND
Wire Wire Line
	5250 4450 5400 4450
Wire Wire Line
	5400 3950 5400 4050
Wire Wire Line
	6150 4050 5400 4050
Connection ~ 5400 4050
Wire Wire Line
	5400 4050 5400 4150
$Comp
L device:C C104
U 1 1 61A85720
P 8000 4900
F 0 "C104" H 8115 4946 50  0000 L CNN
F 1 "22 uF 25V" H 8115 4855 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1812_CAP_NOSILK" H 8038 4750 50  0001 C CNN
F 3 "" H 8000 4900 50  0001 C CNN
	1    8000 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3650 8000 3650
Text Label 8000 3950 0    50   ~ 0
GND
Text Label 8000 4750 0    50   ~ 0
LA1_12V0
Text Label 6150 5050 2    50   ~ 0
LA1_12V0_EN
Text Label 7150 5050 0    50   ~ 0
LA1_12V0_FAULT
$Comp
L device:R R57
U 1 1 61A89074
P 5400 3800
F 0 "R57" H 5470 3846 50  0000 L CNN
F 1 "1M" H 5470 3755 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5330 3800 50  0001 C CNN
F 3 "" H 5400 3800 50  0001 C CNN
	1    5400 3800
	1    0    0    -1  
$EndComp
Text Label 6150 4850 2    50   ~ 0
GND
Wire Wire Line
	5400 4750 6150 4750
$Comp
L device:R R58
U 1 1 61A89081
P 5400 4300
F 0 "R58" H 5470 4346 50  0000 L CNN
F 1 "93.1K" H 5470 4255 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5330 4300 50  0001 C CNN
F 3 "" H 5400 4300 50  0001 C CNN
	1    5400 4300
	1    0    0    -1  
$EndComp
Text Label 5250 5550 2    50   ~ 0
GND
Wire Wire Line
	5250 5550 5400 5550
Wire Wire Line
	5400 5050 5400 5150
Wire Wire Line
	6150 5150 5400 5150
Connection ~ 5400 5150
Wire Wire Line
	5400 5150 5400 5250
$Comp
L device:C C105
U 1 1 61A89091
P 8000 3800
F 0 "C105" H 8115 3846 50  0000 L CNN
F 1 "22 uF 25V" H 8115 3755 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1812_CAP_NOSILK" H 8038 3650 50  0001 C CNN
F 3 "" H 8000 3800 50  0001 C CNN
	1    8000 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4750 8000 4750
Text Label 8000 5050 0    50   ~ 0
GND
Text Label 5400 4750 2    50   ~ 0
12V0
Text Label 8500 1550 2    50   ~ 0
LED0
Text Label 8500 1850 2    50   ~ 0
LED1
Text Label 8500 2150 2    50   ~ 0
LED2
Text Label 8500 2450 2    50   ~ 0
LED3
$Comp
L device:R R71
U 1 1 61E111CA
P 8650 1550
F 0 "R71" V 8550 1550 50  0000 C CNN
F 1 "470" V 8650 1550 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8580 1550 50  0001 C CNN
F 3 "" H 8650 1550 50  0001 C CNN
	1    8650 1550
	0    1    1    0   
$EndComp
$Comp
L device:LED D2
U 1 1 61E1216A
P 8950 1550
F 0 "D2" H 8800 1600 50  0000 C CNN
F 1 "GREEN" H 8943 1386 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_LED" H 8950 1550 50  0001 C CNN
F 3 "~" H 8950 1550 50  0001 C CNN
	1    8950 1550
	-1   0    0    1   
$EndComp
Text Label 9300 1550 0    50   ~ 0
GND
Wire Wire Line
	9300 1550 9100 1550
$Comp
L device:R R72
U 1 1 61E1BE4D
P 8650 1850
F 0 "R72" V 8550 1850 50  0000 C CNN
F 1 "470" V 8650 1850 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8580 1850 50  0001 C CNN
F 3 "" H 8650 1850 50  0001 C CNN
	1    8650 1850
	0    1    1    0   
$EndComp
$Comp
L device:LED D3
U 1 1 61E1C43B
P 8950 1850
F 0 "D3" H 8800 1900 50  0000 C CNN
F 1 "GREEN" H 8943 1686 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_LED" H 8950 1850 50  0001 C CNN
F 3 "~" H 8950 1850 50  0001 C CNN
	1    8950 1850
	-1   0    0    1   
$EndComp
Wire Wire Line
	9300 1850 9100 1850
$Comp
L device:R R73
U 1 1 61E1D2CF
P 8650 2150
F 0 "R73" V 8550 2150 50  0000 C CNN
F 1 "470" V 8650 2150 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8580 2150 50  0001 C CNN
F 3 "" H 8650 2150 50  0001 C CNN
	1    8650 2150
	0    1    1    0   
$EndComp
$Comp
L device:LED D4
U 1 1 61E1D8E5
P 8950 2150
F 0 "D4" H 8800 2200 50  0000 C CNN
F 1 "GREEN" H 8943 1986 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_LED" H 8950 2150 50  0001 C CNN
F 3 "~" H 8950 2150 50  0001 C CNN
	1    8950 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	9300 2150 9100 2150
$Comp
L device:R R74
U 1 1 61E1ECF2
P 8650 2450
F 0 "R74" V 8550 2450 50  0000 C CNN
F 1 "470" V 8650 2450 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8580 2450 50  0001 C CNN
F 3 "" H 8650 2450 50  0001 C CNN
	1    8650 2450
	0    1    1    0   
$EndComp
$Comp
L device:LED D5
U 1 1 61E1F330
P 8950 2450
F 0 "D5" H 8800 2500 50  0000 C CNN
F 1 "GREEN" H 8943 2286 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_LED" H 8950 2450 50  0001 C CNN
F 3 "~" H 8950 2450 50  0001 C CNN
	1    8950 2450
	-1   0    0    1   
$EndComp
Wire Wire Line
	9300 2450 9100 2450
Wire Wire Line
	9300 2450 9300 2150
Connection ~ 9300 1850
Wire Wire Line
	9300 1850 9300 1550
Connection ~ 9300 2150
Wire Wire Line
	9300 2150 9300 1850
NoConn ~ 1800 2650
Connection ~ 5250 1200
Connection ~ 5250 2100
Connection ~ 5250 2200
Connection ~ 5250 2300
Connection ~ 5250 2400
Connection ~ 5250 2500
Connection ~ 5250 2600
Connection ~ 5250 2700
Connection ~ 5250 2800
Connection ~ 5250 2900
Connection ~ 5250 3000
Wire Wire Line
	5250 1200 5250 1300
Wire Wire Line
	5250 1100 5250 1200
Wire Wire Line
	5250 2400 5250 2500
Wire Wire Line
	5250 2200 5250 2300
Wire Wire Line
	5250 3000 5250 3100
Wire Wire Line
	5250 2800 5250 2900
Wire Wire Line
	5250 2600 5250 2700
Wire Wire Line
	5250 2000 5250 2100
Wire Wire Line
	5250 2500 5250 2600
Wire Wire Line
	5250 2300 5250 2400
Wire Wire Line
	5250 2900 5250 3000
Wire Wire Line
	5250 2700 5250 2800
Wire Wire Line
	5250 2100 5250 2200
$Comp
L special-azonenberg:CONN_SFF8087 J?
U 1 1 6198689B
P 5350 3150
AR Path="/61923389/6198689B" Ref="J?"  Part="1" 
AR Path="/61909AE6/6198689B" Ref="J9"  Part="1" 
F 0 "J9" H 5700 5465 50  0000 C CNN
F 1 "CONN_SFF8087" H 5700 5374 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SFF8087_MOLEX_75783-0140" H 5350 3150 50  0001 C CNN
F 3 "" H 5350 3150 50  0001 C CNN
	1    5350 3150
	1    0    0    -1  
$EndComp
$Comp
L power-azonenberg:FPF2498 U17
U 1 1 61A8906A
P 6350 4100
F 0 "U17" H 6650 4797 60  0000 C CNN
F 1 "FPF2498" H 6650 4691 60  0000 C CNN
F 2 "azonenberg_pcb:BGA_6_1.3x1.0_0.4MM" H 6350 4100 60  0001 C CNN
F 3 "" H 6350 4100 60  0001 C CNN
	1    6350 4100
	1    0    0    -1  
$EndComp
$Comp
L power-azonenberg:FPF2498 U16
U 1 1 61A73E55
P 6350 5200
F 0 "U16" H 6650 5897 60  0000 C CNN
F 1 "FPF2498" H 6650 5791 60  0000 C CNN
F 2 "azonenberg_pcb:BGA_6_1.3x1.0_0.4MM" H 6350 5200 60  0001 C CNN
F 3 "" H 6350 5200 60  0001 C CNN
	1    6350 5200
	1    0    0    -1  
$EndComp
$Comp
L device:R R59
U 1 1 61A79A4B
P 6050 6100
F 0 "R59" V 5950 6100 50  0000 C CNN
F 1 "10K" V 6050 6100 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5980 6100 50  0001 C CNN
F 3 "" H 6050 6100 50  0001 C CNN
	1    6050 6100
	0    1    1    0   
$EndComp
Text Label 1800 4150 2    50   ~ 0
LA1_PRESENCE_DETECT
Text Label 1800 2850 2    50   ~ 0
LA0_PRESENCE_DETECT
$EndSCHEMATC
