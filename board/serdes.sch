EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 4 10
Title "SATA sniffer"
Date "2021-11-29"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U?
U 9 1 61930307
P 5300 5600
AR Path="/618C589A/61930307" Ref="U?"  Part="3" 
AR Path="/61909AE6/61930307" Ref="U?"  Part="6" 
AR Path="/618C593E/61930307" Ref="U5"  Part="9" 
F 0 "U5" H 5300 5500 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 5300 5400 60  0000 L CNN
F 2 "azonenberg_pcb:BGA_484_22x22_FULLARRAY_1MM" H 5300 5300 60  0001 C CNN
F 3 "" H 5300 5300 60  0000 C CNN
	9    5300 5600
	1    0    0    -1  
$EndComp
$Comp
L osc-azonenberg:OSC_LVDS_NOPAD U6
U 1 1 61D42587
P 12850 1400
F 0 "U6" H 13150 1997 60  0000 C CNN
F 1 "ECX2-LMV-7CN-156.250-TR" H 13150 1891 60  0000 C CNN
F 2 "azonenberg_pcb:OSCILLATOR_LVDS_7.0x5.0" H 12850 1400 60  0001 C CNN
F 3 "" H 12850 1400 60  0000 C CNN
	1    12850 1400
	1    0    0    -1  
$EndComp
Text HLabel 12550 1050 0    50   Input ~ 0
3V3
Text Label 12550 1150 2    50   ~ 0
3V3
Text Label 12550 1250 2    50   ~ 0
GND
$Comp
L device:C C38
U 1 1 61D44088
P 12550 1750
F 0 "C38" H 12665 1796 50  0000 L CNN
F 1 "4.7 uF" H 12665 1705 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 12588 1600 50  0001 C CNN
F 3 "" H 12550 1750 50  0001 C CNN
	1    12550 1750
	1    0    0    -1  
$EndComp
$Comp
L device:C C40
U 1 1 61D4453B
P 13050 1750
F 0 "C40" H 13165 1796 50  0000 L CNN
F 1 "0.47 uF" H 13165 1705 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 13088 1600 50  0001 C CNN
F 3 "" H 13050 1750 50  0001 C CNN
	1    13050 1750
	1    0    0    -1  
$EndComp
Text Label 12550 1600 2    50   ~ 0
3V3
Text Label 12550 1900 2    50   ~ 0
GND
Wire Wire Line
	12550 1900 13050 1900
Wire Wire Line
	13050 1600 12550 1600
Text Label 13750 1150 0    50   ~ 0
REFCLK_156_P
Text Label 13750 1250 0    50   ~ 0
REFCLK_156_N
$Comp
L device:C C43
U 1 1 61D469FD
P 14550 1350
F 0 "C43" V 14450 1500 50  0000 C CNN
F 1 "0.1 uF" V 14650 1550 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 14588 1200 50  0001 C CNN
F 3 "" H 14550 1350 50  0001 C CNN
	1    14550 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	13750 1250 14400 1250
Wire Wire Line
	14400 1250 14400 1350
$Comp
L device:C C42
U 1 1 61D47564
P 14550 1050
F 0 "C42" V 14450 1200 50  0000 C CNN
F 1 "0.1 uF" V 14650 1250 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 14588 900 50  0001 C CNN
F 3 "" H 14550 1050 50  0001 C CNN
	1    14550 1050
	0    1    1    0   
$EndComp
Wire Wire Line
	13750 1150 14400 1150
Wire Wire Line
	14400 1050 14400 1150
Text Label 14900 1050 0    50   ~ 0
REFCLK_156_AC_P
Text Label 14900 1350 0    50   ~ 0
REFCLK_156_AC_N
Wire Wire Line
	14900 1350 14700 1350
Wire Wire Line
	14700 1050 14900 1050
Text Label 5100 3200 2    50   ~ 0
REFCLK_156_AC_P
Text Label 5100 3100 2    50   ~ 0
REFCLK_156_AC_N
$Comp
L osc-azonenberg:OSC_LVDS_NOPAD U7
U 1 1 61D4BD39
P 12850 2800
F 0 "U7" H 13150 3397 60  0000 C CNN
F 1 "ECX2-LMV-3CN-125.000-TR" H 13150 3291 60  0000 C CNN
F 2 "azonenberg_pcb:OSCILLATOR_LVDS_3.2x2.5" H 12850 2800 60  0001 C CNN
F 3 "" H 12850 2800 60  0000 C CNN
	1    12850 2800
	1    0    0    -1  
$EndComp
Text HLabel 12550 2450 0    50   Input ~ 0
3V3
Text Label 12550 2550 2    50   ~ 0
3V3
Text Label 12550 2650 2    50   ~ 0
GND
$Comp
L device:C C39
U 1 1 61D4BD46
P 12550 3150
F 0 "C39" H 12665 3196 50  0000 L CNN
F 1 "4.7 uF" H 12665 3105 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0603_CAP_NOSILK" H 12588 3000 50  0001 C CNN
F 3 "" H 12550 3150 50  0001 C CNN
	1    12550 3150
	1    0    0    -1  
$EndComp
$Comp
L device:C C41
U 1 1 61D4BD50
P 13050 3150
F 0 "C41" H 13165 3196 50  0000 L CNN
F 1 "0.47 uF" H 13165 3105 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 13088 3000 50  0001 C CNN
F 3 "" H 13050 3150 50  0001 C CNN
	1    13050 3150
	1    0    0    -1  
$EndComp
Text Label 12550 3000 2    50   ~ 0
3V3
Text Label 12550 3300 2    50   ~ 0
GND
Wire Wire Line
	12550 3300 13050 3300
Wire Wire Line
	13050 3000 12550 3000
Text Label 13750 2550 0    50   ~ 0
REFCLK_125_P
Text Label 13750 2650 0    50   ~ 0
REFCLK_125_N
$Comp
L device:C C45
U 1 1 61D4BD60
P 14550 2750
F 0 "C45" V 14450 2900 50  0000 C CNN
F 1 "0.1 uF" V 14650 2950 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 14588 2600 50  0001 C CNN
F 3 "" H 14550 2750 50  0001 C CNN
	1    14550 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	13750 2650 14400 2650
Wire Wire Line
	14400 2650 14400 2750
$Comp
L device:C C44
U 1 1 61D4BD6C
P 14550 2450
F 0 "C44" V 14450 2600 50  0000 C CNN
F 1 "0.1 uF" V 14650 2650 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 14588 2300 50  0001 C CNN
F 3 "" H 14550 2450 50  0001 C CNN
	1    14550 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	13750 2550 14400 2550
Wire Wire Line
	14400 2450 14400 2550
Text Label 14900 2450 0    50   ~ 0
REFCLK_125_AC_P
Text Label 14900 2750 0    50   ~ 0
REFCLK_125_AC_N
Wire Wire Line
	14900 2750 14700 2750
Wire Wire Line
	14700 2450 14900 2450
Text Label 5100 3500 2    50   ~ 0
REFCLK_125_AC_P
Text Label 5100 3400 2    50   ~ 0
REFCLK_125_AC_N
$Comp
L device:R R16
U 1 1 61D52124
P 4600 3850
F 0 "R16" H 4670 3896 50  0000 L CNN
F 1 "100" H 4670 3805 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 4530 3850 50  0001 C CNN
F 3 "" H 4600 3850 50  0001 C CNN
	1    4600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3700 4600 3700
Wire Wire Line
	4600 4000 5100 4000
Wire Wire Line
	5100 4000 5100 3900
$Comp
L Connector:Conn_Coaxial J2
U 1 1 61D5F6D6
P 8100 1250
F 0 "J2" H 8053 1488 50  0000 C CNN
F 1 "32K243-40ML5" H 8053 1397 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SMA_EDGE_ROSENBERGER_32K243_40ML5" H 8100 1250 50  0001 C CNN
F 3 "" H 8100 1250 50  0001 C CNN
	1    8100 1250
	-1   0    0    -1  
$EndComp
Text Label 8400 2450 0    50   ~ 0
GTX_TX3_AC_P
$Comp
L device:C C36
U 1 1 61D61CE3
P 2750 950
F 0 "C36" V 2650 700 50  0000 C CNN
F 1 "0.1 uF" V 2650 1150 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2788 800 50  0001 C CNN
F 3 "" H 2750 950 50  0001 C CNN
	1    2750 950 
	0    1    1    0   
$EndComp
$Comp
L device:C C37
U 1 1 61D635C7
P 2750 1150
F 0 "C37" V 2650 900 50  0000 C CNN
F 1 "0.1 uF" V 2650 1350 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2788 1000 50  0001 C CNN
F 3 "" H 2750 1150 50  0001 C CNN
	1    2750 1150
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 650  5100 650 
Wire Wire Line
	5100 650  5100 700 
Wire Wire Line
	4250 850  5100 850 
Wire Wire Line
	5100 850  5100 800 
Text Label 4600 650  0    50   ~ 0
GTX_TX3_P
Text Label 4600 850  0    50   ~ 0
GTX_TX3_N
Text Label 3700 650  2    50   ~ 0
GTX_TX3_AC_P
Text Label 3700 850  2    50   ~ 0
GTX_TX3_AC_N
Wire Wire Line
	3700 850  3950 850 
Wire Wire Line
	3950 650  3700 650 
$Comp
L device:C C34
U 1 1 61D678BF
P 4100 650
F 0 "C34" V 4000 400 50  0000 C CNN
F 1 "0.1 uF" V 4000 850 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 500 50  0001 C CNN
F 3 "" H 4100 650 50  0001 C CNN
	1    4100 650 
	0    1    1    0   
$EndComp
$Comp
L device:C C35
U 1 1 61D678C9
P 4100 850
F 0 "C35" V 4000 600 50  0000 C CNN
F 1 "0.1 uF" V 4000 1050 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 700 50  0001 C CNN
F 3 "" H 4100 850 50  0001 C CNN
	1    4100 850 
	0    1    1    0   
$EndComp
Text Label 5000 1000 2    50   ~ 0
GTX_RX3_P
Text Label 5000 1100 2    50   ~ 0
GTX_RX3_N
Text Label 2350 950  2    50   ~ 0
GTX_RX3_AC_P
Text Label 2350 1150 2    50   ~ 0
GTX_RX3_AC_N
Wire Wire Line
	2350 1150 2600 1150
Wire Wire Line
	2600 950  2350 950 
Wire Wire Line
	5100 1000 3150 1000
Wire Wire Line
	5100 1100 3150 1100
Wire Wire Line
	3150 1100 3150 1150
Wire Wire Line
	3150 1150 2900 1150
Wire Wire Line
	2900 950  3150 950 
Wire Wire Line
	3150 950  3150 1000
Wire Wire Line
	8400 1250 8250 1250
Text Label 8400 1450 0    50   ~ 0
GND
Wire Wire Line
	8400 1450 8100 1450
$Comp
L Connector:Conn_Coaxial J5
U 1 1 619C73C6
P 8100 1850
F 0 "J5" H 8053 2088 50  0000 C CNN
F 1 "32K243-40ML5" H 8053 1997 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SMA_EDGE_ROSENBERGER_32K243_40ML5" H 8100 1850 50  0001 C CNN
F 3 "" H 8100 1850 50  0001 C CNN
	1    8100 1850
	-1   0    0    -1  
$EndComp
Text Label 8400 3050 0    50   ~ 0
GTX_TX3_AC_N
Wire Wire Line
	8400 1850 8250 1850
Text Label 8400 2050 0    50   ~ 0
GND
Wire Wire Line
	8400 2050 8100 2050
$Comp
L Connector:Conn_Coaxial J6
U 1 1 619C88BB
P 8100 2450
F 0 "J6" H 8053 2688 50  0000 C CNN
F 1 "32K243-40ML5" H 8053 2597 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SMA_EDGE_ROSENBERGER_32K243_40ML5" H 8100 2450 50  0001 C CNN
F 3 "" H 8100 2450 50  0001 C CNN
	1    8100 2450
	-1   0    0    -1  
$EndComp
Text Label 8400 1250 0    50   ~ 0
GTX_RX3_AC_P
Wire Wire Line
	8400 2450 8250 2450
Text Label 8400 2650 0    50   ~ 0
GND
Wire Wire Line
	8400 2650 8100 2650
$Comp
L Connector:Conn_Coaxial J7
U 1 1 619C8E35
P 8100 3050
F 0 "J7" H 8053 3288 50  0000 C CNN
F 1 "32K243-40ML5" H 8053 3197 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SMA_EDGE_ROSENBERGER_32K243_40ML5" H 8100 3050 50  0001 C CNN
F 3 "" H 8100 3050 50  0001 C CNN
	1    8100 3050
	-1   0    0    -1  
$EndComp
Text Label 8400 1850 0    50   ~ 0
GTX_RX3_AC_N
Wire Wire Line
	8400 3050 8250 3050
Text Label 8400 3250 0    50   ~ 0
GND
Wire Wire Line
	8400 3250 8100 3250
Text Label 3950 2050 2    50   ~ 0
SATA_HOST_TX_P
Text Label 3950 2450 2    50   ~ 0
SATA_DEV_TX_P
Text Label 3950 2650 2    50   ~ 0
SATA_DEV_TX_N
Text Label 2400 2750 2    50   ~ 0
SATA_DEV_RX_P
Text Label 2400 2950 2    50   ~ 0
SATA_DEV_RX_N
Text Label 6600 7500 2    50   ~ 0
SATA_HOST_TX_P
Text Label 6600 7600 2    50   ~ 0
SATA_HOST_TX_N
Text Label 6600 7800 2    50   ~ 0
SATA_HOST_RX_P
Text Label 6600 7900 2    50   ~ 0
SATA_HOST_RX_N
Text Label 6600 6450 2    50   ~ 0
SATA_DEV_TX_P
Text Label 6600 6550 2    50   ~ 0
SATA_DEV_TX_N
Text Label 6600 6150 2    50   ~ 0
SATA_DEV_RX_P
Text Label 6600 6250 2    50   ~ 0
SATA_DEV_RX_N
Text Label 5100 1300 2    50   ~ 0
SFP_TX_P
Text Label 5100 1400 2    50   ~ 0
SFP_TX_N
Text Label 5100 1600 2    50   ~ 0
SFP_RX_P
Text Label 5100 1700 2    50   ~ 0
SFP_RX_N
Text Label 1300 7350 2    50   ~ 0
SFP_TX_P
Text Label 1300 7450 2    50   ~ 0
SFP_TX_N
Text Label 1300 7050 2    50   ~ 0
SFP_RX_P
Text Label 1300 7150 2    50   ~ 0
SFP_RX_N
Text Notes 8100 3350 0    50   ~ 0
SMA SERDES
$Comp
L special-azonenberg:CONN_SFP_HOST J10
U 1 1 619E03BB
P 1500 8700
F 0 "J10" H 1500 8650 60  0000 L CNN
F 1 "CONN_SFP_HOST" H 1500 8550 60  0000 L CNN
F 2 "azonenberg_pcb:CONN_SFP+_AMPHENOL_UE76_A10_2000T" H 1500 8700 60  0001 C CNN
F 3 "" H 1500 8700 60  0000 C CNN
	1    1500 8700
	1    0    0    -1  
$EndComp
Text HLabel 1300 7650 0    50   Input ~ 0
SFP_RS0
Text HLabel 1300 7750 0    50   Input ~ 0
SFP_RS1
Text Label 1300 6350 2    50   ~ 0
GND
Text Label 1300 6050 2    50   ~ 0
SFP_VDD_RX
Text Label 1300 6150 2    50   ~ 0
SFP_VDD_TX
Wire Wire Line
	1300 6350 1300 6450
Connection ~ 1300 6450
Wire Wire Line
	1300 6450 1300 6550
Connection ~ 1300 6550
Wire Wire Line
	1300 6550 1300 6650
Connection ~ 1300 6650
Wire Wire Line
	1300 6650 1300 6750
Connection ~ 1300 6750
Wire Wire Line
	1300 6750 1300 6850
Text HLabel 1300 7950 0    50   Output ~ 0
SFP_RX_LOS
Text HLabel 1300 8050 0    50   Output ~ 0
SFP_MOD_ABS
$Comp
L device:R R49
U 1 1 619EC48F
P 1300 10500
F 0 "R49" V 1200 10500 50  0000 C CNN
F 1 "10K" V 1300 10500 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1230 10500 50  0001 C CNN
F 3 "" H 1300 10500 50  0001 C CNN
	1    1300 10500
	0    1    1    0   
$EndComp
Text Notes 1200 10650 0    50   ~ 0
SFP pullup/downs
Text Label 1150 10500 2    50   ~ 0
SFP_MOD_ABS
Text Label 1450 10500 0    50   ~ 0
3V3
$Comp
L device:R R48
U 1 1 619F118A
P 1300 10250
F 0 "R48" V 1200 10250 50  0000 C CNN
F 1 "4.7K" V 1300 10250 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1230 10250 50  0001 C CNN
F 3 "" H 1300 10250 50  0001 C CNN
	1    1300 10250
	0    1    1    0   
$EndComp
Text Label 1150 10250 2    50   ~ 0
SFP_RX_LOS
Text Label 1450 10250 0    50   ~ 0
3V3
Text HLabel 1300 8250 0    50   Input ~ 0
SFP_TX_DISABLE
Text HLabel 1300 8350 0    50   Output ~ 0
SFP_TX_FAULT
Text Label 1150 10000 2    50   ~ 0
SFP_TX_FAULT
$Comp
L device:R R47
U 1 1 619F3F7E
P 1300 10000
F 0 "R47" V 1200 10000 50  0000 C CNN
F 1 "4.7K" V 1300 10000 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1230 10000 50  0001 C CNN
F 3 "" H 1300 10000 50  0001 C CNN
	1    1300 10000
	0    1    1    0   
$EndComp
Text Label 1450 10000 0    50   ~ 0
3V3
Text HLabel 1300 8550 0    50   Input ~ 0
SFP_I2C_SCL
Text HLabel 1300 8650 0    50   BiDi ~ 0
SFP_I2C_SDA
$Comp
L device:R R46
U 1 1 619F8CCA
P 1300 9750
F 0 "R46" V 1200 9750 50  0000 C CNN
F 1 "4.7K" V 1300 9750 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1230 9750 50  0001 C CNN
F 3 "" H 1300 9750 50  0001 C CNN
	1    1300 9750
	0    1    1    0   
$EndComp
Text Label 1150 9750 2    50   ~ 0
SFP_I2C_SDA
Text Label 1450 9750 0    50   ~ 0
3V3
$Comp
L device:R R45
U 1 1 619FABCE
P 1300 9500
F 0 "R45" V 1200 9500 50  0000 C CNN
F 1 "4.7K" V 1300 9500 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1230 9500 50  0001 C CNN
F 3 "" H 1300 9500 50  0001 C CNN
	1    1300 9500
	0    1    1    0   
$EndComp
Text Label 1150 9500 2    50   ~ 0
SFP_I2C_SCL
Text Label 1450 9500 0    50   ~ 0
3V3
$Comp
L Connector:Conn_01x01 J11
U 1 1 619FEB4C
P 1550 9100
F 0 "J11" H 1630 9142 50  0000 L CNN
F 1 "SFP_CAGE" H 1630 9051 50  0000 L CNN
F 2 "azonenberg_pcb:CONN_SFP_CAGE_TE_2007194-1" H 1550 9100 50  0001 C CNN
F 3 "~" H 1550 9100 50  0001 C CNN
	1    1550 9100
	1    0    0    -1  
$EndComp
Text Label 1350 9100 2    50   ~ 0
GND
$Comp
L device:R R50
U 1 1 61A04802
P 2700 9000
F 0 "R50" V 2493 9000 50  0000 C CNN
F 1 "0.1" V 2584 9000 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 2630 9000 50  0001 C CNN
F 3 "" H 2700 9000 50  0001 C CNN
	1    2700 9000
	0    1    1    0   
$EndComp
Text Label 2550 9000 2    50   ~ 0
3V3
$Comp
L device:C C98
U 1 1 61A07634
P 3150 9150
F 0 "C98" H 3265 9196 50  0000 L CNN
F 1 "0.1 uF" H 3265 9105 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 3188 9000 50  0001 C CNN
F 3 "" H 3150 9150 50  0001 C CNN
	1    3150 9150
	1    0    0    -1  
$EndComp
$Comp
L passive-azonenberg:INDUCTOR_PWROUT L5
U 1 1 61A0922A
P 3700 9000
F 0 "L5" V 3542 9000 40  0000 C CNN
F 1 "4.7 uH" V 3618 9000 40  0000 C CNN
F 2 "azonenberg_pcb:EIA_0805_INDUCTOR_NOSILK" H 3700 9000 60  0001 C CNN
F 3 "" H 3700 9000 60  0000 C CNN
	1    3700 9000
	0    1    1    0   
$EndComp
Connection ~ 3150 9000
Wire Wire Line
	3150 9000 3400 9000
$Comp
L device:C C100
U 1 1 61A0B67C
P 4000 9150
F 0 "C100" H 4115 9196 50  0000 L CNN
F 1 "22 uF" H 4115 9105 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1206_CAP_NOSILK" H 4038 9000 50  0001 C CNN
F 3 "" H 4000 9150 50  0001 C CNN
	1    4000 9150
	1    0    0    -1  
$EndComp
$Comp
L device:R R51
U 1 1 61A0B92D
P 4000 9450
F 0 "R51" H 3930 9404 50  0000 R CNN
F 1 "0.5" H 3930 9495 50  0000 R CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 3930 9450 50  0001 C CNN
F 3 "" H 4000 9450 50  0001 C CNN
	1    4000 9450
	-1   0    0    1   
$EndComp
$Comp
L device:C C102
U 1 1 61A0C140
P 4450 9150
F 0 "C102" H 4565 9196 50  0000 L CNN
F 1 "0.1 uF" H 4565 9105 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4488 9000 50  0001 C CNN
F 3 "" H 4450 9150 50  0001 C CNN
	1    4450 9150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 9000 4450 9000
Connection ~ 4000 9000
Wire Wire Line
	4450 9300 4450 9600
Wire Wire Line
	4450 9600 4000 9600
Wire Wire Line
	3150 9600 3150 9300
Connection ~ 4000 9600
Wire Wire Line
	4000 9600 3150 9600
$Comp
L device:C C99
U 1 1 61A0F43A
P 3150 9950
F 0 "C99" H 3265 9996 50  0000 L CNN
F 1 "0.1 uF" H 3265 9905 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 3188 9800 50  0001 C CNN
F 3 "" H 3150 9950 50  0001 C CNN
	1    3150 9950
	1    0    0    -1  
$EndComp
$Comp
L passive-azonenberg:INDUCTOR_PWROUT L6
U 1 1 61A0FADE
P 3700 9800
F 0 "L6" V 3542 9800 40  0000 C CNN
F 1 "4.7 uH" V 3618 9800 40  0000 C CNN
F 2 "azonenberg_pcb:EIA_0805_INDUCTOR_NOSILK" H 3700 9800 60  0001 C CNN
F 3 "" H 3700 9800 60  0000 C CNN
	1    3700 9800
	0    1    1    0   
$EndComp
Connection ~ 3150 9800
Wire Wire Line
	3150 9800 3400 9800
$Comp
L device:C C101
U 1 1 61A0FAEB
P 4000 9950
F 0 "C101" H 4115 9996 50  0000 L CNN
F 1 "22 uF" H 4115 9905 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_1206_CAP_NOSILK" H 4038 9800 50  0001 C CNN
F 3 "" H 4000 9950 50  0001 C CNN
	1    4000 9950
	1    0    0    -1  
$EndComp
$Comp
L device:R R52
U 1 1 61A0FAF5
P 4000 10250
F 0 "R52" H 3930 10204 50  0000 R CNN
F 1 "0.5" H 3930 10295 50  0000 R CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 3930 10250 50  0001 C CNN
F 3 "" H 4000 10250 50  0001 C CNN
	1    4000 10250
	-1   0    0    1   
$EndComp
$Comp
L device:C C103
U 1 1 61A0FAFF
P 4450 9950
F 0 "C103" H 4565 9996 50  0000 L CNN
F 1 "0.1 uF" H 4565 9905 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4488 9800 50  0001 C CNN
F 3 "" H 4450 9950 50  0001 C CNN
	1    4450 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 9800 4450 9800
Connection ~ 4000 9800
Wire Wire Line
	4450 10100 4450 10400
Wire Wire Line
	4450 10400 4000 10400
Wire Wire Line
	3150 10400 3150 10100
Connection ~ 4000 10400
Wire Wire Line
	4000 10400 3150 10400
Wire Wire Line
	2850 9000 2950 9000
Wire Wire Line
	2950 9000 2950 9800
Wire Wire Line
	2950 9800 3150 9800
Connection ~ 2950 9000
Wire Wire Line
	2950 9000 3150 9000
Text Label 3150 9600 2    50   ~ 0
GND
Text Label 4450 9000 0    50   ~ 0
SFP_VDD_RX
Text Label 4450 9800 0    50   ~ 0
SFP_VDD_TX
Text HLabel 3150 10400 0    50   Input ~ 0
GND
$Comp
L special-azonenberg:CONN_SATA_DEVICE J13
U 1 1 61D6F9FA
P 6750 7000
F 0 "J13" H 6750 6950 50  0000 L CNN
F 1 "CONN_SATA_DEVICE" H 6750 6850 50  0000 L CNN
F 2 "azonenberg_pcb:CONN_SATA_MOLEX_0674901220" H 6750 7000 50  0001 C CNN
F 3 "" H 6750 7000 50  0001 C CNN
	1    6750 7000
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:CONN_SATA_HOST J14
U 1 1 61D72A82
P 6750 8350
F 0 "J14" H 6750 8300 50  0000 L CNN
F 1 "CONN_SATA_HOST" H 6750 8200 50  0000 L CNN
F 2 "azonenberg_pcb:CONN_SATA_MOLEX_0674901220" H 6750 8350 50  0001 C CNN
F 3 "" H 6750 8350 50  0001 C CNN
	1    6750 8350
	1    0    0    -1  
$EndComp
$Comp
L device:C C149
U 1 1 61D775E4
P 4100 2050
F 0 "C149" V 4000 1800 50  0000 C CNN
F 1 "0.1 uF" V 4000 2250 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 1900 50  0001 C CNN
F 3 "" H 4100 2050 50  0001 C CNN
	1    4100 2050
	0    1    1    0   
$EndComp
$Comp
L device:C C150
U 1 1 61D77D41
P 4100 1850
F 0 "C150" V 4000 1600 50  0000 C CNN
F 1 "0.1 uF" V 4000 2050 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 1700 50  0001 C CNN
F 3 "" H 4100 1850 50  0001 C CNN
	1    4100 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 1850 5100 1850
Wire Wire Line
	4250 2050 5100 2050
Text Label 5000 2050 2    50   ~ 0
GTX_TX2_P
Text Label 5000 1850 2    50   ~ 0
GTX_TX2_N
Wire Wire Line
	2400 2350 2650 2350
Wire Wire Line
	3200 2350 2950 2350
$Comp
L device:C C151
U 1 1 61D8C0F7
P 4100 2450
F 0 "C151" V 4000 2200 50  0000 C CNN
F 1 "0.1 uF" V 4000 2650 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 2300 50  0001 C CNN
F 3 "" H 4100 2450 50  0001 C CNN
	1    4100 2450
	0    1    1    0   
$EndComp
$Comp
L device:C C152
U 1 1 61D8C873
P 4100 2650
F 0 "C152" V 4000 2400 50  0000 C CNN
F 1 "0.1 uF" V 4000 2850 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 4138 2500 50  0001 C CNN
F 3 "" H 4100 2650 50  0001 C CNN
	1    4100 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 2650 5100 2650
Wire Wire Line
	4250 2450 5100 2450
$Comp
L device:C C147
U 1 1 61D9641A
P 2800 2750
F 0 "C147" V 2700 2500 50  0000 C CNN
F 1 "0.1 uF" V 2700 2950 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2838 2600 50  0001 C CNN
F 3 "" H 2800 2750 50  0001 C CNN
	1    2800 2750
	0    1    1    0   
$EndComp
$Comp
L device:C C148
U 1 1 61D96BBA
P 2800 2950
F 0 "C148" V 2700 2700 50  0000 C CNN
F 1 "0.1 uF" V 2700 3150 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2838 2800 50  0001 C CNN
F 3 "" H 2800 2950 50  0001 C CNN
	1    2800 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 2950 2650 2950
Wire Wire Line
	2650 2750 2400 2750
Wire Wire Line
	3200 2950 2950 2950
Wire Wire Line
	2950 2750 3200 2750
Wire Wire Line
	3200 2950 3200 2900
Wire Wire Line
	3200 2900 5100 2900
Wire Wire Line
	3200 2750 3200 2800
Wire Wire Line
	3200 2800 5100 2800
Text Label 5000 2450 2    50   ~ 0
GTX_TX1_P
Text Label 5000 2650 2    50   ~ 0
GTX_TX1_N
Text Label 5000 2800 2    50   ~ 0
GTX_RX1_P
Text Label 5000 2900 2    50   ~ 0
GTX_RX1_N
Text Label 6600 6750 2    50   ~ 0
GND
Wire Wire Line
	6600 6750 6600 6850
Connection ~ 6600 6850
Wire Wire Line
	6600 6850 6600 6950
Wire Wire Line
	6600 8100 6600 8200
Connection ~ 6600 8200
Wire Wire Line
	6600 8200 6600 8300
Text Label 6600 8100 2    50   ~ 0
GND
Text HLabel 4600 3700 0    50   Input ~ 0
GTX_1V2
Text Notes 3400 1400 0    50   ~ 0
SFP+ has internal coupling caps\nNo need to include them on PCB
Wire Wire Line
	5100 2650 5100 2600
Wire Wire Line
	5100 2450 5100 2500
Text Label 5050 2300 2    50   ~ 0
GTX_RX2_N
Text Label 5050 2200 2    50   ~ 0
GTX_RX2_P
Wire Wire Line
	3200 2300 3200 2350
Wire Wire Line
	5100 2300 3200 2300
Wire Wire Line
	3200 2200 5100 2200
Wire Wire Line
	3200 2150 3200 2200
Wire Wire Line
	2950 2150 3200 2150
Wire Wire Line
	2650 2150 2400 2150
$Comp
L device:C C146
U 1 1 61D7F6C1
P 2800 2350
F 0 "C146" V 2700 2100 50  0000 C CNN
F 1 "0.1 uF" V 2700 2550 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2838 2200 50  0001 C CNN
F 3 "" H 2800 2350 50  0001 C CNN
	1    2800 2350
	0    1    1    0   
$EndComp
$Comp
L device:C C145
U 1 1 61D7EF69
P 2800 2150
F 0 "C145" V 2700 1900 50  0000 C CNN
F 1 "0.1 uF" V 2700 2350 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_CAP_NOSILK" H 2838 2000 50  0001 C CNN
F 3 "" H 2800 2150 50  0001 C CNN
	1    2800 2150
	0    1    1    0   
$EndComp
Text Label 2400 2350 2    50   ~ 0
SATA_HOST_RX_N
Text Label 2400 2150 2    50   ~ 0
SATA_HOST_RX_P
Text Label 3950 1850 2    50   ~ 0
SATA_HOST_TX_N
Wire Wire Line
	5100 2050 5100 2000
Wire Wire Line
	5100 1900 5100 1850
Text Notes 2500 3700 0    50   ~ 0
INVERT ON:\nGTX_TX2
$EndSCHEMATC
