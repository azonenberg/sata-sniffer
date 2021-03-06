EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 10
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
L power-azonenberg:LTC3374-QFN U2
U 1 1 6189D657
P 1700 4700
F 0 "U2" H 2042 5397 60  0000 C CNN
F 1 "LTC3374-QFN" H 2042 5291 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 1700 4700 60  0001 C CNN
F 3 "" H 1700 4700 60  0000 C CNN
	1    1700 4700
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 2 1 618A03C0
P 1700 5600
F 0 "U2" H 2042 6297 60  0000 C CNN
F 1 "LTC3374-QFN" H 2042 6191 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 1700 5600 60  0001 C CNN
F 3 "" H 1700 5600 60  0000 C CNN
	2    1700 5600
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 3 1 618A0B9D
P 1700 6450
F 0 "U2" H 2042 7147 60  0000 C CNN
F 1 "LTC3374-QFN" H 2042 7041 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 1700 6450 60  0001 C CNN
F 3 "" H 1700 6450 60  0000 C CNN
	3    1700 6450
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 4 1 618A1CBD
P 1700 7300
F 0 "U2" H 2042 7997 60  0000 C CNN
F 1 "LTC3374-QFN" H 2042 7891 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 1700 7300 60  0001 C CNN
F 3 "" H 1700 7300 60  0000 C CNN
	4    1700 7300
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 5 1 618A2E1A
P 4700 4700
F 0 "U2" H 5042 5397 60  0000 C CNN
F 1 "LTC3374-QFN" H 5042 5291 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 4700 4700 60  0001 C CNN
F 3 "" H 4700 4700 60  0000 C CNN
	5    4700 4700
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 6 1 618A3794
P 4700 5550
F 0 "U2" H 5042 6247 60  0000 C CNN
F 1 "LTC3374-QFN" H 5042 6141 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 4700 5550 60  0001 C CNN
F 3 "" H 4700 5550 60  0000 C CNN
	6    4700 5550
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 7 1 618A4B14
P 7550 4700
F 0 "U2" H 7892 5397 60  0000 C CNN
F 1 "LTC3374-QFN" H 7892 5291 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 7550 4700 60  0001 C CNN
F 3 "" H 7550 4700 60  0000 C CNN
	7    7550 4700
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 8 1 618A56B1
P 7550 3600
F 0 "U2" H 7892 4297 60  0000 C CNN
F 1 "LTC3374-QFN" H 7892 4191 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 7550 3600 60  0001 C CNN
F 3 "" H 7550 3600 60  0000 C CNN
	8    7550 3600
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:LTC3374-QFN U2
U 9 1 618A67C7
P 8550 5950
F 0 "U2" H 9000 6647 60  0000 C CNN
F 1 "LTC3374-QFN" H 9000 6541 60  0000 C CNN
F 2 "azonenberg_pcb:QFN_38_0.5MM_5x7MM" H 8550 5950 60  0001 C CNN
F 3 "" H 8550 5950 60  0000 C CNN
	9    8550 5950
	-1   0    0    -1  
$EndComp
$Comp
L power-azonenberg:CONN_3_PWROUT J1
U 1 1 618AC801
P 900 1000
F 0 "J1" H 767 1300 50  0000 C CNN
F 1 "BARREL" H 767 1216 40  0000 C CNN
F 2 "azonenberg_pcb:CONN_CUI_PJ-058BH_HIPWR_BARREL_NOSLOT" H 900 1000 60  0001 C CNN
F 3 "" H 900 1000 60  0000 C CNN
	1    900  1000
	-1   0    0    -1  
$EndComp
NoConn ~ 1250 1100
$Comp
L power-azonenberg:FUSE_PWROUT_2 F1
U 1 1 6189DF9C
P 1750 900
F 0 "F1" H 1750 1114 50  0000 C CNN
F 1 "2A" H 1750 1023 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 1825 900 60  0001 C CNN
F 3 "" H 1825 900 60  0000 C CNN
	1    1750 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 900  1250 900 
Text Label 1250 900  0    50   ~ 0
12V0_RAW
$Comp
L passive-azonenberg:FERRITE_SMALL2 FB1
U 1 1 6189F326
P 2650 900
F 0 "FB1" H 2650 1125 50  0000 C CNN
F 1 "BLM31KN271SH1L" H 2650 1034 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_1206_CAP_NOSILK" H 2650 900 60  0001 C CNN
F 3 "" H 2650 900 60  0000 C CNN
	1    2650 900 
	1    0    0    -1  
$EndComp
Text Label 1950 900  0    50   ~ 0
12V0_FUSED
Wire Wire Line
	1900 900  2450 900 
Wire Wire Line
	2850 900  3050 900 
$Comp
L power-azonenberg:OKL-T_3-W12P-C U1
U 1 1 618A1938
P 4500 1550
F 0 "U1" H 4875 2447 60  0000 C CNN
F 1 "OKL-T_3-W12P-C" H 4875 2341 60  0000 C CNN
F 2 "azonenberg_pcb:MODULE_MURATA_OKL-T3-W12" H 4500 1450 60  0001 C CNN
F 3 "" H 4500 1450 60  0001 C CNN
	1    4500 1550
	1    0    0    -1  
$EndComp
$Comp
L power-azonenberg:MYTNA1R84RELA2RA U3
U 1 1 618A80FB
P 2500 3150
F 0 "U3" H 2800 4225 50  0000 C CNN
F 1 "MYTNA1R84RELA2RA" H 2800 4134 50  0000 C CNN
F 2 "azonenberg_pcb:MODULE_MURATA_MYTNA1R84RELA2RA" H 2500 3150 50  0001 C CNN
F 3 "" H 2500 3150 50  0001 C CNN
	1    2500 3150
	1    0    0    -1  
$EndComp
$Comp
L power-azonenberg:MYTNA1R84RELA2RA U3
U 2 1 618A8748
P 950 3150
F 0 "U3" H 1250 4725 50  0000 C CNN
F 1 "MYTNA1R84RELA2RA" H 1250 4634 50  0000 C CNN
F 2 "azonenberg_pcb:MODULE_MURATA_MYTNA1R84RELA2RA" H 950 3150 50  0001 C CNN
F 3 "" H 950 3150 50  0001 C CNN
	2    950  3150
	1    0    0    -1  
$EndComp
$Comp
L device:R R4
U 1 1 618A9FC8
P 2150 3150
F 0 "R4" V 2050 3150 50  0000 C CNN
F 1 "1.78K" V 2150 3150 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2080 3150 50  0001 C CNN
F 3 "" H 2150 3150 50  0001 C CNN
	1    2150 3150
	-1   0    0    1   
$EndComp
$Comp
L device:R R1
U 1 1 618AA6FD
P 4300 1650
F 0 "R1" V 4200 1650 50  0000 C CNN
F 1 "1.33K" V 4300 1650 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 4230 1650 50  0001 C CNN
F 3 "" H 4300 1650 50  0001 C CNN
	1    4300 1650
	-1   0    0    1   
$EndComp
Text Notes 1150 7400 0    50   ~ 0
1.35
Text Notes 3800 5650 0    50   ~ 0
1.2
Text Notes 6650 4800 0    50   ~ 0
1.8
Text Notes 6650 3700 0    50   ~ 0
3.3
$Comp
L device:R R9
U 1 1 618BC1A7
P 8500 3600
F 0 "R9" H 8570 3646 50  0000 L CNN
F 1 "1.02M" H 8570 3555 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8430 3600 50  0001 C CNN
F 3 "" H 8500 3600 50  0001 C CNN
	1    8500 3600
	1    0    0    -1  
$EndComp
$Comp
L device:R R10
U 1 1 618BC848
P 8500 3900
F 0 "R10" H 8570 3946 50  0000 L CNN
F 1 "324K" H 8570 3855 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8430 3900 50  0001 C CNN
F 3 "" H 8500 3900 50  0001 C CNN
	1    8500 3900
	1    0    0    -1  
$EndComp
$Comp
L device:R R7
U 1 1 618BE174
P 8500 4700
F 0 "R7" H 8570 4746 50  0000 L CNN
F 1 "806K" H 8570 4655 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8430 4700 50  0001 C CNN
F 3 "" H 8500 4700 50  0001 C CNN
	1    8500 4700
	1    0    0    -1  
$EndComp
$Comp
L device:R R8
U 1 1 618BE9F5
P 8500 5000
F 0 "R8" H 8570 5046 50  0000 L CNN
F 1 "649K" H 8570 4955 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8430 5000 50  0001 C CNN
F 3 "" H 8500 5000 50  0001 C CNN
	1    8500 5000
	1    0    0    -1  
$EndComp
$Comp
L passive-azonenberg:INDUCTOR_PWROUT L2
U 1 1 618C2814
P 5550 4550
F 0 "L2" V 5392 4550 40  0000 C CNN
F 1 "NRS5030T2R2NMGJ" V 5468 4550 40  0000 C CNN
F 2 "azonenberg_pcb:INDUCTOR_YUDEN_NRS5030" H 5550 4550 60  0001 C CNN
F 3 "" H 5550 4550 60  0000 C CNN
	1    5550 4550
	0    1    1    0   
$EndComp
$Comp
L passive-azonenberg:INDUCTOR_PWROUT L3
U 1 1 618C33BC
P 8200 4550
F 0 "L3" V 8042 4550 40  0000 C CNN
F 1 "NRS5030T2R2NMGJ" V 8118 4550 40  0000 C CNN
F 2 "azonenberg_pcb:INDUCTOR_YUDEN_NRS5030" H 8200 4550 60  0001 C CNN
F 3 "" H 8200 4550 60  0000 C CNN
	1    8200 4550
	0    1    1    0   
$EndComp
$Comp
L passive-azonenberg:INDUCTOR_PWROUT L4
U 1 1 618C3740
P 8200 3450
F 0 "L4" V 8042 3450 40  0000 C CNN
F 1 "NRS5030T2R2NMGJ" V 8118 3450 40  0000 C CNN
F 2 "azonenberg_pcb:INDUCTOR_YUDEN_NRS5030" H 8200 3450 60  0001 C CNN
F 3 "" H 8200 3450 60  0000 C CNN
	1    8200 3450
	0    1    1    0   
$EndComp
$Comp
L passive-azonenberg:INDUCTOR_PWROUT L1
U 1 1 618C3C2A
P 2400 4550
F 0 "L1" V 2242 4550 40  0000 C CNN
F 1 "NS10145T2R2NNA" V 2318 4550 40  0000 C CNN
F 2 "azonenberg_pcb:INDUCTOR_YUDEN_LSRN_10145" H 2400 4550 60  0001 C CNN
F 3 "" H 2400 4550 60  0000 C CNN
	1    2400 4550
	0    1    1    0   
$EndComp
Text HLabel 1250 1000 2    50   Output ~ 0
GND
Text HLabel 4050 2300 2    50   Output ~ 0
1V0
Text HLabel 6200 4550 2    50   Output ~ 0
1V2
Text HLabel 8700 4550 2    50   Output ~ 0
1V8
Text HLabel 8750 3450 2    50   Output ~ 0
3V3
Text HLabel 8050 1000 2    50   Output ~ 0
VTT
Text HLabel 8050 1100 2    50   Output ~ 0
VREF
Text HLabel 3050 750  2    50   Output ~ 0
12V0
Wire Wire Line
	4300 900  4300 1000
Text Label 4300 1100 2    50   ~ 0
GND
Wire Wire Line
	4300 1100 4300 1200
Text Label 4300 1400 2    50   ~ 0
12V0
Text Label 4200 1800 2    50   ~ 0
GND
Wire Wire Line
	4200 1800 4300 1800
Wire Wire Line
	5450 1100 5450 1000
Connection ~ 5450 1100
$Comp
L device:C C121
U 1 1 61A9B3C8
P 5450 1250
F 0 "C121" H 5565 1296 50  0000 L CNN
F 1 "22 uF" H 5565 1205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1812_CAP_NOSILK" H 5488 1100 50  0001 C CNN
F 3 "" H 5450 1250 50  0001 C CNN
	1    5450 1250
	1    0    0    -1  
$EndComp
Text Label 5450 1400 0    50   ~ 0
GND
Wire Wire Line
	1900 4550 2050 4550
Wire Wire Line
	2050 4550 2050 5450
Wire Wire Line
	2050 7150 1900 7150
Connection ~ 2050 4550
Wire Wire Line
	2050 4550 2100 4550
Wire Wire Line
	1900 6300 2050 6300
Connection ~ 2050 6300
Wire Wire Line
	2050 6300 2050 7150
Wire Wire Line
	1900 5450 2050 5450
Connection ~ 2050 5450
Wire Wire Line
	2050 5450 2050 6300
Text Label 1900 5550 0    50   ~ 0
5V0
Text Label 1900 6400 0    50   ~ 0
5V0
Text Label 1900 7250 0    50   ~ 0
5V0
Text Label 1900 5250 0    50   ~ 0
5V0
Text Label 1900 5150 0    50   ~ 0
GND
Text Label 1900 6000 0    50   ~ 0
GND
Text Label 1900 6850 0    50   ~ 0
GND
Text Label 1900 6100 0    50   ~ 0
5V0
Text Label 1900 6950 0    50   ~ 0
5V0
Wire Wire Line
	1900 4650 2300 4650
Text Label 4900 4350 0    50   ~ 0
5V0
Text Label 4900 5100 0    50   ~ 0
GND
Text Label 4900 5200 0    50   ~ 0
5V0
Text Label 4900 5500 0    50   ~ 0
5V0
Wire Wire Line
	4900 4550 5100 4550
Wire Wire Line
	5100 4550 5100 5400
Wire Wire Line
	5100 5400 4900 5400
Connection ~ 5100 4550
Wire Wire Line
	5100 4550 5250 4550
Text Label 5100 5400 0    50   ~ 0
1V2_SW
Wire Wire Line
	5300 4650 5300 4850
Connection ~ 6000 4550
Wire Wire Line
	6000 4550 5850 4550
Wire Wire Line
	6200 4550 6000 4550
Connection ~ 6000 4850
Wire Wire Line
	5300 4850 6000 4850
$Comp
L device:R R6
U 1 1 618BF2EA
P 6000 5000
F 0 "R6" H 6070 5046 50  0000 L CNN
F 1 "464K" H 6070 4955 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5930 5000 50  0001 C CNN
F 3 "" H 6000 5000 50  0001 C CNN
	1    6000 5000
	1    0    0    -1  
$EndComp
$Comp
L device:R R5
U 1 1 618BF258
P 6000 4700
F 0 "R5" H 6070 4746 50  0000 L CNN
F 1 "232K" H 6070 4655 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5930 4700 50  0001 C CNN
F 3 "" H 6000 4700 50  0001 C CNN
	1    6000 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4550 2850 4550
Text HLabel 2850 4550 2    50   Output ~ 0
1V35
Wire Wire Line
	2300 4650 2300 4850
Text Label 2050 5000 0    50   ~ 0
1V35_SW
Wire Wire Line
	2750 5150 2850 5150
Text Label 2750 5150 2    50   ~ 0
GND
Text Label 2300 4850 0    50   ~ 0
1V35_FB
Connection ~ 2850 4850
Wire Wire Line
	2300 4850 2850 4850
$Comp
L device:R R3
U 1 1 618C05AA
P 2850 5000
F 0 "R3" H 2920 5046 50  0000 L CNN
F 1 "649K" H 2920 4955 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2780 5000 50  0001 C CNN
F 3 "" H 2850 5000 50  0001 C CNN
	1    2850 5000
	1    0    0    -1  
$EndComp
$Comp
L device:R R2
U 1 1 618C00B3
P 2850 4700
F 0 "R2" H 2920 4746 50  0000 L CNN
F 1 "442K" H 2920 4655 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2780 4700 50  0001 C CNN
F 3 "" H 2850 4700 50  0001 C CNN
	1    2850 4700
	1    0    0    -1  
$EndComp
Text Label 5300 4850 0    50   ~ 0
1V2_FB
Text Label 6000 5150 2    50   ~ 0
GND
$Comp
L device:C C106
U 1 1 61ACCACD
P 2750 6100
F 0 "C106" H 2865 6146 50  0000 L CNN
F 1 "10 uF" H 2865 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 2788 5950 50  0001 C CNN
F 3 "" H 2750 6100 50  0001 C CNN
	1    2750 6100
	1    0    0    -1  
$EndComp
Text Label 2750 5950 2    50   ~ 0
5V0
$Comp
L device:C C110
U 1 1 61ACF182
P 3250 6100
F 0 "C110" H 3365 6146 50  0000 L CNN
F 1 "10 uF" H 3365 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 3288 5950 50  0001 C CNN
F 3 "" H 3250 6100 50  0001 C CNN
	1    3250 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C114
U 1 1 61ACF6BA
P 3700 6100
F 0 "C114" H 3815 6146 50  0000 L CNN
F 1 "10 uF" H 3815 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 3738 5950 50  0001 C CNN
F 3 "" H 3700 6100 50  0001 C CNN
	1    3700 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C116
U 1 1 61ACF91A
P 4200 6100
F 0 "C116" H 4315 6146 50  0000 L CNN
F 1 "10 uF" H 4315 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 4238 5950 50  0001 C CNN
F 3 "" H 4200 6100 50  0001 C CNN
	1    4200 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C118
U 1 1 61ACFC3A
P 4700 6100
F 0 "C118" H 4815 6146 50  0000 L CNN
F 1 "10 uF" H 4815 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 4738 5950 50  0001 C CNN
F 3 "" H 4700 6100 50  0001 C CNN
	1    4700 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C119
U 1 1 61ACFEF5
P 5200 6100
F 0 "C119" H 5315 6146 50  0000 L CNN
F 1 "10 uF" H 5315 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 5238 5950 50  0001 C CNN
F 3 "" H 5200 6100 50  0001 C CNN
	1    5200 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C122
U 1 1 61AD01D3
P 5700 6100
F 0 "C122" H 5815 6146 50  0000 L CNN
F 1 "10 uF" H 5815 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 5738 5950 50  0001 C CNN
F 3 "" H 5700 6100 50  0001 C CNN
	1    5700 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C123
U 1 1 61AD06A9
P 6150 6100
F 0 "C123" H 6265 6146 50  0000 L CNN
F 1 "10 uF" H 6265 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 6188 5950 50  0001 C CNN
F 3 "" H 6150 6100 50  0001 C CNN
	1    6150 6100
	1    0    0    -1  
$EndComp
$Comp
L device:C C124
U 1 1 61AD08DA
P 6650 6100
F 0 "C124" H 6765 6146 50  0000 L CNN
F 1 "10 uF" H 6765 6055 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 6688 5950 50  0001 C CNN
F 3 "" H 6650 6100 50  0001 C CNN
	1    6650 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 5950 6150 5950
Connection ~ 3250 5950
Wire Wire Line
	3250 5950 2750 5950
Connection ~ 3700 5950
Wire Wire Line
	3700 5950 3250 5950
Connection ~ 4200 5950
Wire Wire Line
	4200 5950 3700 5950
Connection ~ 4700 5950
Wire Wire Line
	4700 5950 4200 5950
Connection ~ 5200 5950
Wire Wire Line
	5200 5950 4700 5950
Connection ~ 5700 5950
Wire Wire Line
	5700 5950 5200 5950
Connection ~ 6150 5950
Wire Wire Line
	6150 5950 5700 5950
Wire Wire Line
	2750 6250 3250 6250
Connection ~ 3250 6250
Wire Wire Line
	3250 6250 3700 6250
Connection ~ 3700 6250
Wire Wire Line
	3700 6250 4200 6250
Connection ~ 4200 6250
Wire Wire Line
	4200 6250 4700 6250
Connection ~ 4700 6250
Wire Wire Line
	4700 6250 5200 6250
Connection ~ 5200 6250
Wire Wire Line
	5200 6250 5700 6250
Connection ~ 5700 6250
Wire Wire Line
	5700 6250 6150 6250
Connection ~ 6150 6250
Wire Wire Line
	6150 6250 6650 6250
Text Label 2750 6250 2    50   ~ 0
GND
$Comp
L device:C C107
U 1 1 61AD51BA
P 2750 6550
F 0 "C107" H 2865 6596 50  0000 L CNN
F 1 "47 uF" H 2865 6505 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0805_CAP_NOSILK" H 2788 6400 50  0001 C CNN
F 3 "" H 2750 6550 50  0001 C CNN
	1    2750 6550
	1    0    0    -1  
$EndComp
$Comp
L device:C C111
U 1 1 61AD563F
P 3250 6550
F 0 "C111" H 3365 6596 50  0000 L CNN
F 1 "47 uF" H 3365 6505 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0805_CAP_NOSILK" H 3288 6400 50  0001 C CNN
F 3 "" H 3250 6550 50  0001 C CNN
	1    3250 6550
	1    0    0    -1  
$EndComp
Text Label 2750 6400 2    50   ~ 0
1V35
Wire Wire Line
	2750 6400 3250 6400
Text Label 2750 6700 2    50   ~ 0
GND
Wire Wire Line
	2750 6700 3250 6700
$Comp
L device:C C108
U 1 1 61AD8D49
P 2750 7000
F 0 "C108" H 2865 7046 50  0000 L CNN
F 1 "47 uF" H 2865 6955 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0805_CAP_NOSILK" H 2788 6850 50  0001 C CNN
F 3 "" H 2750 7000 50  0001 C CNN
	1    2750 7000
	1    0    0    -1  
$EndComp
Text Label 2750 6850 2    50   ~ 0
1V2
Text Label 2750 7150 2    50   ~ 0
GND
$Comp
L device:C C117
U 1 1 61AD98C7
P 4200 6550
F 0 "C117" H 4315 6596 50  0000 L CNN
F 1 "22 uF" H 4315 6505 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0805_CAP_NOSILK" H 4238 6400 50  0001 C CNN
F 3 "" H 4200 6550 50  0001 C CNN
	1    4200 6550
	1    0    0    -1  
$EndComp
Text Label 4200 6400 2    50   ~ 0
1V8
Text Label 4200 6700 2    50   ~ 0
GND
$Comp
L device:C C120
U 1 1 61ADA948
P 5200 6550
F 0 "C120" H 5315 6596 50  0000 L CNN
F 1 "22 uF" H 5315 6505 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 5238 6400 50  0001 C CNN
F 3 "" H 5200 6550 50  0001 C CNN
	1    5200 6550
	1    0    0    -1  
$EndComp
Text Label 5200 6400 2    50   ~ 0
3V3
Text Label 5200 6700 2    50   ~ 0
GND
Wire Wire Line
	4900 4650 5300 4650
Text Label 7750 4350 0    50   ~ 0
5V0
Wire Wire Line
	7900 4550 7750 4550
Wire Wire Line
	8700 4550 8500 4550
Connection ~ 8500 4550
Wire Wire Line
	7750 4650 7750 4850
Wire Wire Line
	7750 4850 8500 4850
Connection ~ 8500 4850
Text Label 8500 5150 2    50   ~ 0
GND
Wire Wire Line
	7900 3450 7750 3450
Wire Wire Line
	8750 3450 8500 3450
Connection ~ 8500 3450
Wire Wire Line
	7750 3750 8500 3750
Wire Wire Line
	7750 3550 7750 3750
Connection ~ 8500 3750
Text Label 8500 4050 2    50   ~ 0
GND
$Comp
L Connector:Conn_01x01 TP4
U 1 1 61B00CFD
P 7250 6200
F 0 "TP4" H 7168 5975 50  0000 C CNN
F 1 "TESTPOINT" H 7168 6066 50  0000 C CNN
F 2 "azonenberg_pcb:TESTPOINT_SMT_0.5MM" H 7250 6200 50  0001 C CNN
F 3 "~" H 7250 6200 50  0001 C CNN
	1    7250 6200
	-1   0    0    1   
$EndComp
Wire Wire Line
	7450 5900 7450 6200
Text Label 7450 6200 0    50   ~ 0
PSU_TEMP
Text Label 7450 5600 2    50   ~ 0
LTC_PGOOD
Text Label 9450 6350 2    50   ~ 0
LTC_PGOOD
$Comp
L device:R R63
U 1 1 61B0545A
P 9600 6350
F 0 "R63" V 9500 6350 50  0000 C CNN
F 1 "10K" V 9600 6350 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 9530 6350 50  0001 C CNN
F 3 "" H 9600 6350 50  0001 C CNN
	1    9600 6350
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01 TP5
U 1 1 61B05873
P 9250 6150
F 0 "TP5" H 9168 5925 50  0000 C CNN
F 1 "TESTPOINT" H 9168 6016 50  0000 C CNN
F 2 "azonenberg_pcb:TESTPOINT_SMT_0.5MM" H 9250 6150 50  0001 C CNN
F 3 "~" H 9250 6150 50  0001 C CNN
	1    9250 6150
	-1   0    0    1   
$EndComp
Wire Wire Line
	9450 6150 9450 6350
Text Label 9750 6350 0    50   ~ 0
3V3
Text Label 8750 5900 0    50   ~ 0
GND
Text Label 8750 5500 0    50   ~ 0
5V0
Text Label 7450 5700 2    50   ~ 0
GND
Text Label 7450 5800 2    50   ~ 0
5V0
Text Label 7450 5500 2    50   ~ 0
5V0
Text Label 800  1800 2    50   ~ 0
GND
Wire Wire Line
	800  1800 800  1900
Connection ~ 800  1900
Wire Wire Line
	800  1900 800  2000
Connection ~ 800  2000
Wire Wire Line
	800  2000 800  2100
Connection ~ 800  2100
Wire Wire Line
	800  2100 800  2200
Connection ~ 800  2200
Wire Wire Line
	800  2200 800  2300
Connection ~ 800  2300
Wire Wire Line
	800  2300 800  2400
Connection ~ 800  2400
Wire Wire Line
	800  2400 800  2500
Connection ~ 800  2500
Wire Wire Line
	800  2500 800  2600
Connection ~ 800  2600
Wire Wire Line
	800  2600 800  2700
Connection ~ 800  2700
Wire Wire Line
	800  2700 800  2800
Connection ~ 800  2800
Wire Wire Line
	800  2800 800  2900
Connection ~ 800  2900
Wire Wire Line
	800  2900 800  3000
Connection ~ 800  3000
Wire Wire Line
	800  3000 800  3100
Wire Wire Line
	1700 3100 1700 3000
Connection ~ 1700 1900
Wire Wire Line
	1700 1900 1700 1800
Connection ~ 1700 2000
Wire Wire Line
	1700 2000 1700 1900
Connection ~ 1700 2100
Wire Wire Line
	1700 2100 1700 2000
Connection ~ 1700 2200
Wire Wire Line
	1700 2200 1700 2100
Connection ~ 1700 2300
Wire Wire Line
	1700 2300 1700 2200
Connection ~ 1700 2400
Wire Wire Line
	1700 2400 1700 2300
Connection ~ 1700 2500
Wire Wire Line
	1700 2500 1700 2400
Connection ~ 1700 2600
Wire Wire Line
	1700 2600 1700 2500
Connection ~ 1700 2700
Wire Wire Line
	1700 2700 1700 2600
Connection ~ 1700 2800
Wire Wire Line
	1700 2800 1700 2700
Connection ~ 1700 2900
Wire Wire Line
	1700 2900 1700 2800
Connection ~ 1700 3000
Wire Wire Line
	1700 3000 1700 2900
$Comp
L Connector:Conn_Coaxial TP14
U 1 1 61B255CA
P 10300 6150
F 0 "TP14" H 10253 6388 50  0000 C CNN
F 1 "U.FL" H 10253 6297 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 6150 50  0001 C CNN
F 3 "" H 10300 6150 50  0001 C CNN
	1    10300 6150
	-1   0    0    -1  
$EndComp
Text Label 10450 6150 0    50   ~ 0
1V35
Text Label 10300 6350 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP12
U 1 1 61B27502
P 10300 5050
F 0 "TP12" H 10253 5288 50  0000 C CNN
F 1 "U.FL" H 10253 5197 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 5050 50  0001 C CNN
F 3 "" H 10300 5050 50  0001 C CNN
	1    10300 5050
	-1   0    0    -1  
$EndComp
Text Label 10450 5050 0    50   ~ 0
1V2
Text Label 10300 5250 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP13
U 1 1 61B29A87
P 10300 5600
F 0 "TP13" H 10253 5838 50  0000 C CNN
F 1 "U.FL" H 10253 5747 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 5600 50  0001 C CNN
F 3 "" H 10300 5600 50  0001 C CNN
	1    10300 5600
	-1   0    0    -1  
$EndComp
Text Label 10450 5600 0    50   ~ 0
1V0
Text Label 10300 5800 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP11
U 1 1 61B3511C
P 10300 4500
F 0 "TP11" H 10253 4738 50  0000 C CNN
F 1 "U.FL" H 10253 4647 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 4500 50  0001 C CNN
F 3 "" H 10300 4500 50  0001 C CNN
	1    10300 4500
	-1   0    0    -1  
$EndComp
Text Label 10450 4500 0    50   ~ 0
1V8
Text Label 10300 4700 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP10
U 1 1 61B37AC4
P 10300 3950
F 0 "TP10" H 10253 4188 50  0000 C CNN
F 1 "U.FL" H 10253 4097 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 3950 50  0001 C CNN
F 3 "" H 10300 3950 50  0001 C CNN
	1    10300 3950
	-1   0    0    -1  
$EndComp
Text Label 10450 3950 0    50   ~ 0
3V3
Text Label 10300 4150 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP9
U 1 1 61B39EE7
P 10300 3400
F 0 "TP9" H 10253 3638 50  0000 C CNN
F 1 "U.FL" H 10253 3547 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 3400 50  0001 C CNN
F 3 "" H 10300 3400 50  0001 C CNN
	1    10300 3400
	-1   0    0    -1  
$EndComp
Text Label 10450 3400 0    50   ~ 0
5V0
Text Label 10300 3600 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP8
U 1 1 61B3C3F9
P 10300 2850
F 0 "TP8" H 10253 3088 50  0000 C CNN
F 1 "U.FL" H 10253 2997 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 2850 50  0001 C CNN
F 3 "" H 10300 2850 50  0001 C CNN
	1    10300 2850
	-1   0    0    -1  
$EndComp
Text Label 10450 2850 0    50   ~ 0
12V0
Text Label 10300 3050 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP7
U 1 1 61B40B0C
P 10300 2300
F 0 "TP7" H 10253 2538 50  0000 C CNN
F 1 "U.FL" H 10253 2447 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 2300 50  0001 C CNN
F 3 "" H 10300 2300 50  0001 C CNN
	1    10300 2300
	-1   0    0    -1  
$EndComp
Text Label 10450 2300 0    50   ~ 0
VREF
Text Label 10300 2500 0    50   ~ 0
GND
$Comp
L Connector:Conn_Coaxial TP6
U 1 1 61B431A8
P 10300 1750
F 0 "TP6" H 10253 1988 50  0000 C CNN
F 1 "U.FL" H 10253 1897 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_U.FL_TE_1909763-1" H 10300 1750 50  0001 C CNN
F 3 "" H 10300 1750 50  0001 C CNN
	1    10300 1750
	-1   0    0    -1  
$EndComp
Text Label 10450 1750 0    50   ~ 0
VTT
Text Label 10300 1950 0    50   ~ 0
GND
Text Label 2350 2300 2    50   ~ 0
12V0
Wire Wire Line
	2350 2300 2350 2400
Connection ~ 2350 2400
Wire Wire Line
	2350 2400 2350 2500
Connection ~ 2350 2500
Wire Wire Line
	2350 2500 2350 2600
Connection ~ 2350 2600
Wire Wire Line
	2350 2600 2350 2700
Wire Wire Line
	2350 2700 2350 2900
Connection ~ 2350 2700
Wire Wire Line
	3250 2700 3250 2600
Connection ~ 3250 2400
Wire Wire Line
	3250 2400 3250 2300
Connection ~ 3250 2500
Wire Wire Line
	3250 2500 3250 2400
Connection ~ 3250 2600
Wire Wire Line
	3250 2600 3250 2500
Wire Wire Line
	2150 3000 2350 3000
Wire Wire Line
	2350 3100 2350 3300
Wire Wire Line
	2350 3300 2150 3300
$Comp
L device:R R61
U 1 1 61B5BE20
P 2400 3550
F 0 "R61" V 2300 3550 50  0000 C CNN
F 1 "0" V 2400 3550 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2330 3550 50  0001 C CNN
F 3 "" H 2400 3550 50  0001 C CNN
	1    2400 3550
	0    1    1    0   
$EndComp
Text Label 2250 3550 2    50   ~ 0
1V0
Text Label 2550 3550 0    50   ~ 0
1V0_SENSE
Text Label 3250 2900 0    50   ~ 0
1V0_SENSE
$Comp
L device:R R62
U 1 1 61B60EB3
P 3250 3250
F 0 "R62" H 3320 3296 50  0000 L CNN
F 1 "10K" H 3320 3205 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 3180 3250 50  0001 C CNN
F 3 "" H 3250 3250 50  0001 C CNN
	1    3250 3250
	1    0    0    -1  
$EndComp
Text Label 3250 3400 2    50   ~ 0
3V3
Text Label 3250 3100 0    50   ~ 0
1V0_GOOD
Text Label 2550 3800 2    50   ~ 0
1V0_GOOD
$Comp
L Connector:Conn_01x01 TP3
U 1 1 61B65732
P 2750 3800
F 0 "TP3" H 2830 3842 50  0000 L CNN
F 1 "TESTPOINT" H 2830 3751 50  0000 L CNN
F 2 "azonenberg_pcb:TESTPOINT_SMT_0.5MM" H 2750 3800 50  0001 C CNN
F 3 "~" H 2750 3800 50  0001 C CNN
	1    2750 3800
	1    0    0    -1  
$EndComp
$Comp
L device:CP1 C109
U 1 1 61B68339
P 3050 1050
F 0 "C109" H 3165 1096 50  0000 L CNN
F 1 "470 uF" H 3165 1005 50  0000 L CNN
F 2 "azonenberg_pcb:CAP_AL_10MM_SMT" H 3050 1050 50  0001 C CNN
F 3 "" H 3050 1050 50  0001 C CNN
	1    3050 1050
	1    0    0    -1  
$EndComp
Connection ~ 3050 900 
Text Label 3050 1200 2    50   ~ 0
GND
$Comp
L device:C C113
U 1 1 61B6E41F
P 3550 2450
F 0 "C113" H 3665 2496 50  0000 L CNN
F 1 "100 uF" H 3665 2405 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 3588 2300 50  0001 C CNN
F 3 "" H 3550 2450 50  0001 C CNN
	1    3550 2450
	1    0    0    -1  
$EndComp
$Comp
L device:C C115
U 1 1 61B6E7C9
P 4050 2450
F 0 "C115" H 4165 2496 50  0000 L CNN
F 1 "100 uF" H 4165 2405 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 4088 2300 50  0001 C CNN
F 3 "" H 4050 2450 50  0001 C CNN
	1    4050 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2300 3550 2300
Connection ~ 3250 2300
Connection ~ 3550 2300
Wire Wire Line
	3550 2300 3250 2300
Text Label 3550 2600 2    50   ~ 0
GND
Wire Wire Line
	3550 2600 4050 2600
Connection ~ 4300 900 
Wire Wire Line
	3050 750  3050 900 
Wire Wire Line
	3050 900  3550 900 
$Comp
L device:CP1 C112
U 1 1 61B9EB93
P 3550 1050
F 0 "C112" H 3665 1096 50  0000 L CNN
F 1 "470 uF" H 3665 1005 50  0000 L CNN
F 2 "azonenberg_pcb:CAP_AL_10MM_SMT" H 3550 1050 50  0001 C CNN
F 3 "" H 3550 1050 50  0001 C CNN
	1    3550 1050
	1    0    0    -1  
$EndComp
Connection ~ 3550 900 
Wire Wire Line
	3550 900  4300 900 
Wire Wire Line
	3550 1200 3050 1200
$Comp
L power-azonenberg:LP2996A U18
U 1 1 61C146A2
P 7200 1750
F 0 "U18" H 7550 2725 50  0000 C CNN
F 1 "LP2996A" H 7550 2634 50  0000 C CNN
F 2 "azonenberg_pcb:SOIC_8_3.9MM_EPAD" H 7200 1750 50  0001 C CNN
F 3 "" H 7200 1750 50  0001 C CNN
	1    7200 1750
	1    0    0    -1  
$EndComp
Text Label 7050 1100 2    50   ~ 0
3V3
Text Label 7050 1000 2    50   ~ 0
1V35
Text Label 6200 1200 2    50   ~ 0
1V35
Text Label 7050 1400 2    50   ~ 0
GND
Wire Wire Line
	7050 1400 7050 1500
Text Label 7050 1700 2    50   ~ 0
3V3
$Comp
L device:R R65
U 1 1 61C1D24C
P 6350 1200
F 0 "R65" V 6250 1200 50  0000 C CNN
F 1 "100" V 6350 1200 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 6280 1200 50  0001 C CNN
F 3 "" H 6350 1200 50  0001 C CNN
	1    6350 1200
	0    1    1    0   
$EndComp
Text Label 7050 1200 2    50   ~ 0
VDDQ_SENSE
Wire Wire Line
	6500 1200 7050 1200
$Comp
L device:C C126
U 1 1 61C2CE6C
P 7200 2050
F 0 "C126" H 7315 2096 50  0000 L CNN
F 1 "0.1 uF" H 7315 2005 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 7238 1900 50  0001 C CNN
F 3 "" H 7200 2050 50  0001 C CNN
	1    7200 2050
	1    0    0    -1  
$EndComp
Text Label 7200 1900 2    50   ~ 0
VREF
Text Label 7200 2200 2    50   ~ 0
GND
$Comp
L device:C C130
U 1 1 61C312B0
P 8050 1450
F 0 "C130" H 8165 1496 50  0000 L CNN
F 1 "0.1 uF" H 8165 1405 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 8088 1300 50  0001 C CNN
F 3 "" H 8050 1450 50  0001 C CNN
	1    8050 1450
	1    0    0    -1  
$EndComp
Text Label 8050 1300 0    50   ~ 0
VTT_SENSE
$Comp
L device:R R66
U 1 1 61C32F7C
P 8650 1300
F 0 "R66" V 8550 1300 50  0000 C CNN
F 1 "0" V 8650 1300 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 8580 1300 50  0001 C CNN
F 3 "" H 8650 1300 50  0001 C CNN
	1    8650 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	8050 1300 8500 1300
Connection ~ 8050 1300
Text Label 8050 1600 0    50   ~ 0
GND
Text Label 8800 1300 0    50   ~ 0
VTT
$Comp
L device:C C125
U 1 1 61C3866E
P 6500 1350
F 0 "C125" H 6615 1396 50  0000 L CNN
F 1 "47 uF" H 6615 1305 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0805_CAP_NOSILK" H 6538 1200 50  0001 C CNN
F 3 "" H 6500 1350 50  0001 C CNN
	1    6500 1350
	1    0    0    -1  
$EndComp
Connection ~ 6500 1200
Text Label 6500 1500 2    50   ~ 0
GND
$Comp
L device:C C128
U 1 1 61C3B3B2
P 7750 2050
F 0 "C128" H 7865 2096 50  0000 L CNN
F 1 "47 uF" H 7865 2005 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1206_CAP_NOSILK" H 7788 1900 50  0001 C CNN
F 3 "" H 7750 2050 50  0001 C CNN
	1    7750 2050
	1    0    0    -1  
$EndComp
Text Label 7750 1900 2    50   ~ 0
1V35
Wire Wire Line
	7750 2200 7200 2200
$Comp
L device:C C131
U 1 1 61C3F15B
P 8250 2050
F 0 "C131" H 8365 2096 50  0000 L CNN
F 1 "4.7 uF" H 8365 2005 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 8288 1900 50  0001 C CNN
F 3 "" H 8250 2050 50  0001 C CNN
	1    8250 2050
	1    0    0    -1  
$EndComp
$Comp
L device:C C133
U 1 1 61C3F6D4
P 8750 2050
F 0 "C133" H 8865 2096 50  0000 L CNN
F 1 "0.47 uF" H 8865 2005 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 8788 1900 50  0001 C CNN
F 3 "" H 8750 2050 50  0001 C CNN
	1    8750 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 1900 8250 1900
Connection ~ 8250 1900
Wire Wire Line
	8250 1900 7750 1900
Wire Wire Line
	7750 2200 8250 2200
Connection ~ 7750 2200
Connection ~ 8250 2200
Wire Wire Line
	8250 2200 8750 2200
Text Label 7200 2350 2    50   ~ 0
3V3
$Comp
L device:C C127
U 1 1 61C46B3D
P 7200 2500
F 0 "C127" H 7315 2546 50  0000 L CNN
F 1 "0.47 uF" H 7315 2455 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 7238 2350 50  0001 C CNN
F 3 "" H 7200 2500 50  0001 C CNN
	1    7200 2500
	1    0    0    -1  
$EndComp
Text Label 7200 2650 2    50   ~ 0
GND
$Comp
L device:C C129
U 1 1 61C492FD
P 7750 2500
F 0 "C129" H 7865 2546 50  0000 L CNN
F 1 "100 uF" H 7865 2455 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 7788 2350 50  0001 C CNN
F 3 "" H 7750 2500 50  0001 C CNN
	1    7750 2500
	1    0    0    -1  
$EndComp
$Comp
L device:C C132
U 1 1 61C496F1
P 8250 2500
F 0 "C132" H 8365 2546 50  0000 L CNN
F 1 "100 uF" H 8365 2455 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1210_CAP_NOSILK" H 8288 2350 50  0001 C CNN
F 3 "" H 8250 2500 50  0001 C CNN
	1    8250 2500
	1    0    0    -1  
$EndComp
Text Label 7750 2350 2    50   ~ 0
VTT
Text Label 7750 2650 2    50   ~ 0
GND
Wire Wire Line
	7750 2650 8250 2650
Wire Wire Line
	8250 2350 7750 2350
Text HLabel 5450 1000 2    50   Output ~ 0
5V0
Text Label 1700 1800 0    50   ~ 0
GND
Text Notes 4700 4050 0    50   ~ 0
12, 5, 3.3, and 1.0V rails come up first\nthen everything else
Text Label 4900 4250 0    50   ~ 0
1V0_GOOD
Text Label 7750 3150 0    50   ~ 0
5V0
Wire Wire Line
	7750 3150 7750 3250
Text Label 7750 4250 0    50   ~ 0
1V0_GOOD
Text Label 1900 4350 0    50   ~ 0
5V0
Text Label 1900 4250 0    50   ~ 0
1V0_GOOD
NoConn ~ 5450 900 
Text Label 7800 4850 0    50   ~ 0
1V8_FB
Text Label 7800 3750 0    50   ~ 0
3V3_FB
$EndSCHEMATC
