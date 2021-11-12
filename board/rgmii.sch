EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 7 10
Title "SATA sniffer"
Date "2021-11-10"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U?
U 7 1 619294FB
P 12100 7750
AR Path="/618C589A/619294FB" Ref="U?"  Part="3" 
AR Path="/61909AE6/619294FB" Ref="U?"  Part="6" 
AR Path="/61923389/619294FB" Ref="U5"  Part="7" 
F 0 "U5" H 12100 7650 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 12100 7550 60  0000 L CNN
F 2 "" H 12100 7450 60  0000 C CNN
F 3 "" H 12100 7450 60  0000 C CNN
	7    12100 7750
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:KSZ9031RN U10
U 1 1 61B10DC4
P 3550 4200
F 0 "U10" H 4375 7537 60  0000 C CNN
F 1 "KSZ9031RN" H 4375 7431 60  0000 C CNN
F 2 "" H 3550 4200 60  0000 C CNN
F 3 "" H 3550 4200 60  0000 C CNN
	1    3550 4200
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:BEL_FUSE_0826-1G1T-23-F J3
U 1 1 61D7E542
P 1200 4000
F 0 "J3" H 1267 6287 60  0000 C CNN
F 1 "BEL_FUSE_0826-1G1T-23-F" H 1267 6181 60  0000 C CNN
F 2 "" H 1200 4000 60  0000 C CNN
F 3 "" H 1200 4000 60  0000 C CNN
	1    1200 4000
	-1   0    0    -1  
$EndComp
$Comp
L osc-azonenberg:OSC_LVDS_NOPAD U?
U 1 1 61D87E30
P 1900 9250
AR Path="/618C593E/61D87E30" Ref="U?"  Part="1" 
AR Path="/61923389/61D87E30" Ref="U8"  Part="1" 
F 0 "U8" H 2200 9847 60  0000 C CNN
F 1 "ECX2-LMV-3CN-125.000-TR" H 2200 9741 60  0000 C CNN
F 2 "" H 1900 9250 60  0000 C CNN
F 3 "" H 1900 9250 60  0000 C CNN
	1    1900 9250
	1    0    0    -1  
$EndComp
Text HLabel 1600 8900 0    50   Input ~ 0
3V3
Text Label 1600 9000 2    50   ~ 0
3V3
Text Label 1600 9100 2    50   ~ 0
GND
$Comp
L device:C C?
U 1 1 61D87E39
P 1600 9600
AR Path="/618C593E/61D87E39" Ref="C?"  Part="1" 
AR Path="/61923389/61D87E39" Ref="C68"  Part="1" 
F 0 "C68" H 1715 9646 50  0000 L CNN
F 1 "4.7 uF" H 1715 9555 50  0000 L CNN
F 2 "" H 1638 9450 50  0001 C CNN
F 3 "" H 1600 9600 50  0001 C CNN
	1    1600 9600
	1    0    0    -1  
$EndComp
$Comp
L device:C C?
U 1 1 61D87E3F
P 2100 9600
AR Path="/618C593E/61D87E3F" Ref="C?"  Part="1" 
AR Path="/61923389/61D87E3F" Ref="C69"  Part="1" 
F 0 "C69" H 2215 9646 50  0000 L CNN
F 1 "0.47 uF" H 2215 9555 50  0000 L CNN
F 2 "" H 2138 9450 50  0001 C CNN
F 3 "" H 2100 9600 50  0001 C CNN
	1    2100 9600
	1    0    0    -1  
$EndComp
Text Label 1600 9450 2    50   ~ 0
3V3
Text Label 1600 9750 2    50   ~ 0
GND
Wire Wire Line
	1600 9750 2100 9750
Wire Wire Line
	2100 9450 1600 9450
Text Label 2800 9000 0    50   ~ 0
CLK_125_P
$Comp
L device:C C?
U 1 1 61D87E4B
P 3600 9200
AR Path="/618C593E/61D87E4B" Ref="C?"  Part="1" 
AR Path="/61923389/61D87E4B" Ref="C71"  Part="1" 
F 0 "C71" V 3500 9350 50  0000 C CNN
F 1 "0.1 uF" V 3700 9400 50  0000 C CNN
F 2 "" H 3638 9050 50  0001 C CNN
F 3 "" H 3600 9200 50  0001 C CNN
	1    3600 9200
	0    1    1    0   
$EndComp
Wire Wire Line
	2800 9100 3450 9100
Wire Wire Line
	3450 9100 3450 9200
$Comp
L device:C C?
U 1 1 61D87E53
P 3600 8900
AR Path="/618C593E/61D87E53" Ref="C?"  Part="1" 
AR Path="/61923389/61D87E53" Ref="C70"  Part="1" 
F 0 "C70" V 3500 9050 50  0000 C CNN
F 1 "0.1 uF" V 3700 9100 50  0000 C CNN
F 2 "" H 3638 8750 50  0001 C CNN
F 3 "" H 3600 8900 50  0001 C CNN
	1    3600 8900
	0    1    1    0   
$EndComp
Wire Wire Line
	2800 9000 3450 9000
Wire Wire Line
	3450 8900 3450 9000
Text Label 3950 8900 0    50   ~ 0
CLK_125_AC_P
Text Label 3950 9200 0    50   ~ 0
CLK_125_AC_N
Wire Wire Line
	3950 9200 3750 9200
Wire Wire Line
	3750 8900 3950 8900
Text Label 2800 9100 0    50   ~ 0
CLK_125_N
Text Label 11900 5550 2    50   ~ 0
CLK_125_AC_P
Text Label 11900 5650 2    50   ~ 0
CLK_125_AC_N
$Comp
L osc-azonenberg:OSC U9
U 1 1 61D8F215
P 2300 10450
F 0 "U9" H 2275 10737 60  0000 C CNN
F 1 "SIT1602BC-72-18S-25.000000" H 2275 10631 60  0000 C CNN
F 2 "" H 2300 10450 60  0000 C CNN
F 3 "" H 2300 10450 60  0000 C CNN
	1    2300 10450
	1    0    0    -1  
$EndComp
$Comp
L device:R R18
U 1 1 61D900AD
P 3200 10450
F 0 "R18" V 3100 10450 50  0000 C CNN
F 1 "33" V 3200 10450 50  0000 C CNN
F 2 "" V 3130 10450 50  0001 C CNN
F 3 "" H 3200 10450 50  0001 C CNN
	1    3200 10450
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 10450 2950 10450
$Comp
L device:C C?
U 1 1 61D90C50
P 1100 10600
AR Path="/618C593E/61D90C50" Ref="C?"  Part="1" 
AR Path="/61923389/61D90C50" Ref="C62"  Part="1" 
F 0 "C62" H 1215 10646 50  0000 L CNN
F 1 "0.47 uF" H 1215 10555 50  0000 L CNN
F 2 "" H 1138 10450 50  0001 C CNN
F 3 "" H 1100 10600 50  0001 C CNN
	1    1100 10600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 10450 1600 10450
Wire Wire Line
	1600 10450 1600 10550
Connection ~ 1600 10450
Wire Wire Line
	1100 10750 1600 10750
Wire Wire Line
	1600 10750 1600 10650
Text Label 1100 10450 2    50   ~ 0
3V3
Text Label 1100 10750 2    50   ~ 0
GND
Text Label 3350 10450 0    50   ~ 0
ETH_CLK_25MHZ
Text Label 5500 3650 0    50   ~ 0
ETH_CLK_25MHZ
Text HLabel 1450 5500 0    50   Input ~ 0
3V3
Wire Wire Line
	2750 1150 3250 1150
Wire Wire Line
	3250 1150 3250 1250
Connection ~ 3250 1150
Connection ~ 3250 1250
Wire Wire Line
	3250 1250 3250 1350
Text HLabel 2750 1950 0    50   Input ~ 0
1V2
Wire Wire Line
	2750 1950 3250 1950
Wire Wire Line
	3250 1950 3250 2050
Connection ~ 3250 1950
Connection ~ 3250 2050
Wire Wire Line
	3250 2050 3250 2150
Connection ~ 3250 2150
Wire Wire Line
	3250 2150 3250 2250
Connection ~ 3250 2250
Wire Wire Line
	3250 2250 3250 2350
Connection ~ 3250 2350
Wire Wire Line
	3250 2350 3250 2450
NoConn ~ 5500 3550
$Comp
L device:R R25
U 1 1 61D99BE4
P 5800 3850
F 0 "R25" V 5700 3900 50  0000 C CNN
F 1 "12.1K 1%" V 5900 3850 50  0000 C CNN
F 2 "" V 5730 3850 50  0001 C CNN
F 3 "" H 5800 3850 50  0001 C CNN
	1    5800 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 3850 5500 3850
Text Label 1500 2500 0    50   ~ 0
ETH_A_P
Text Label 1500 2700 0    50   ~ 0
ETH_A_N
Text Label 1500 2900 0    50   ~ 0
ETH_B_P
Text Label 1500 3100 0    50   ~ 0
ETH_B_N
Text Label 1500 3300 0    50   ~ 0
ETH_C_P
Text Label 1500 3500 0    50   ~ 0
ETH_C_N
Text Label 1500 3700 0    50   ~ 0
ETH_D_P
Text Label 1500 3900 0    50   ~ 0
ETH_D_N
Text Label 3250 3400 2    50   ~ 0
ETH_A_P
Text Label 3250 3500 2    50   ~ 0
ETH_A_N
Text Label 3250 3600 2    50   ~ 0
ETH_B_P
Text Label 3250 3700 2    50   ~ 0
ETH_B_N
Text Label 3250 3800 2    50   ~ 0
ETH_C_P
Text Label 3250 3900 2    50   ~ 0
ETH_C_N
Text Label 3250 4000 2    50   ~ 0
ETH_D_P
Text Label 3250 4100 2    50   ~ 0
ETH_D_N
Text Label 5500 3300 0    50   ~ 0
ETH_RST_N
Text Label 5500 3000 0    50   ~ 0
ETH_MDIO
Text Label 5500 2900 0    50   ~ 0
ETH_MDC
Text Label 5500 1650 0    50   ~ 0
RGMII_TXD0
Text Label 5500 1750 0    50   ~ 0
RGMII_TXD1
Text Label 5500 1850 0    50   ~ 0
RGMII_TXD2
Text Label 5500 1950 0    50   ~ 0
RGMII_TXD3
Text Label 5500 2050 0    50   ~ 0
RGMII_TX_CLK
Text Label 5500 2150 0    50   ~ 0
RGMII_TX_EN
$Comp
L device:R R19
U 1 1 61DB7237
P 5800 2300
F 0 "R19" V 5750 2500 50  0000 C CNN
F 1 "33" V 5800 2300 50  0000 C CNN
F 2 "" V 5730 2300 50  0001 C CNN
F 3 "" H 5800 2300 50  0001 C CNN
	1    5800 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2300 5500 2300
$Comp
L device:R R20
U 1 1 61DB871A
P 5800 2400
F 0 "R20" V 5750 2600 50  0000 C CNN
F 1 "33" V 5800 2400 50  0000 C CNN
F 2 "" V 5730 2400 50  0001 C CNN
F 3 "" H 5800 2400 50  0001 C CNN
	1    5800 2400
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2400 5500 2400
$Comp
L device:R R21
U 1 1 61DB9121
P 5800 2500
F 0 "R21" V 5750 2700 50  0000 C CNN
F 1 "33" V 5800 2500 50  0000 C CNN
F 2 "" V 5730 2500 50  0001 C CNN
F 3 "" H 5800 2500 50  0001 C CNN
	1    5800 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2500 5500 2500
$Comp
L device:R R22
U 1 1 61DB9B1E
P 5800 2600
F 0 "R22" V 5750 2800 50  0000 C CNN
F 1 "33" V 5800 2600 50  0000 C CNN
F 2 "" V 5730 2600 50  0001 C CNN
F 3 "" H 5800 2600 50  0001 C CNN
	1    5800 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2600 5500 2600
$Comp
L device:R R23
U 1 1 61DBA5C6
P 5800 2700
F 0 "R23" V 5750 2900 50  0000 C CNN
F 1 "33" V 5800 2700 50  0000 C CNN
F 2 "" V 5730 2700 50  0001 C CNN
F 3 "" H 5800 2700 50  0001 C CNN
	1    5800 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2700 5500 2700
$Comp
L device:R R24
U 1 1 61DBB3C0
P 5800 2800
F 0 "R24" V 5750 3000 50  0000 C CNN
F 1 "33" V 5800 2800 50  0000 C CNN
F 2 "" V 5730 2800 50  0001 C CNN
F 3 "" H 5800 2800 50  0001 C CNN
	1    5800 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2800 5500 2800
Text Label 6200 2300 0    50   ~ 0
ETH_RXD3
Text Label 6200 2400 0    50   ~ 0
ETH_RXD2
Text Label 6200 2500 0    50   ~ 0
ETH_RXD1
Text Label 6200 2600 0    50   ~ 0
ETH_RXD0
Text Label 6200 2700 0    50   ~ 0
ETH_RX_DV
Text Label 6200 2800 0    50   ~ 0
ETH_RX_CLK
Wire Wire Line
	6200 2800 5950 2800
Wire Wire Line
	5950 2700 6200 2700
Wire Wire Line
	6200 2600 5950 2600
Wire Wire Line
	5950 2500 6200 2500
Wire Wire Line
	6200 2400 5950 2400
Wire Wire Line
	5950 2300 6200 2300
Text HLabel 2750 1450 0    50   Input ~ 0
1V8
Wire Wire Line
	2750 1450 3250 1450
Wire Wire Line
	3250 1450 3250 1550
Connection ~ 3250 1450
Connection ~ 3250 1550
Wire Wire Line
	3250 1550 3250 1650
Text Label 3250 2550 2    50   ~ 0
A1V2_PLL
Text Label 3250 1750 2    50   ~ 0
A1V2
Wire Wire Line
	3250 1750 3250 1850
Wire Wire Line
	3250 3000 3250 3100
Connection ~ 3250 3100
Wire Wire Line
	3250 3100 3250 3200
NoConn ~ 3250 2800
NoConn ~ 5500 3100
$Comp
L device:C C64
U 1 1 618DD53B
P 1950 2600
F 0 "C64" V 1800 2800 50  0000 C CNN
F 1 "0.1 uF" V 1789 2600 50  0000 C CNN
F 2 "" H 1988 2450 50  0001 C CNN
F 3 "" H 1950 2600 50  0001 C CNN
	1    1950 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 2600 1500 2600
Text Label 2100 2600 0    50   ~ 0
GND
$Comp
L device:C C65
U 1 1 618DF25D
P 1950 3000
F 0 "C65" V 1800 3200 50  0000 C CNN
F 1 "0.1 uF" V 1789 3000 50  0000 C CNN
F 2 "" H 1988 2850 50  0001 C CNN
F 3 "" H 1950 3000 50  0001 C CNN
	1    1950 3000
	0    1    1    0   
$EndComp
Text Label 2100 3000 0    50   ~ 0
GND
$Comp
L device:C C66
U 1 1 618E0D84
P 1950 3400
F 0 "C66" V 1800 3600 50  0000 C CNN
F 1 "0.1 uF" V 1789 3400 50  0000 C CNN
F 2 "" H 1988 3250 50  0001 C CNN
F 3 "" H 1950 3400 50  0001 C CNN
	1    1950 3400
	0    1    1    0   
$EndComp
Text Label 2100 3400 0    50   ~ 0
GND
$Comp
L device:C C67
U 1 1 618E1F4D
P 1950 3800
F 0 "C67" V 1800 4000 50  0000 C CNN
F 1 "0.1 uF" V 1789 3800 50  0000 C CNN
F 2 "" H 1988 3650 50  0001 C CNN
F 3 "" H 1950 3800 50  0001 C CNN
	1    1950 3800
	0    1    1    0   
$EndComp
Text Label 2100 3800 0    50   ~ 0
GND
Wire Wire Line
	1800 3000 1500 3000
Wire Wire Line
	1500 3400 1800 3400
Wire Wire Line
	1800 3800 1500 3800
$Comp
L device:R R17
U 1 1 618E6BB5
P 1650 4650
F 0 "R17" V 1550 4650 50  0000 C CNN
F 1 "1M" V 1650 4650 50  0000 C CNN
F 2 "" V 1580 4650 50  0001 C CNN
F 3 "" H 1650 4650 50  0001 C CNN
	1    1650 4650
	0    1    1    0   
$EndComp
$Comp
L device:C C63
U 1 1 618E7122
P 1650 4350
F 0 "C63" V 1500 4550 50  0000 C CNN
F 1 "0.1 uF" V 1489 4350 50  0000 C CNN
F 2 "" H 1688 4200 50  0001 C CNN
F 3 "" H 1650 4350 50  0001 C CNN
	1    1650 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 4100 1500 4350
Connection ~ 1500 4350
Wire Wire Line
	1500 4350 1500 4650
Wire Wire Line
	1800 4350 1800 4650
Text Label 1800 4350 0    50   ~ 0
GND
Text Label 11900 3950 2    50   ~ 0
ETH_RXD3
Text Label 11900 4050 2    50   ~ 0
ETH_RXD2
Text Label 11900 4150 2    50   ~ 0
ETH_RXD1
Text Label 11900 4250 2    50   ~ 0
ETH_RXD0
Text Label 11900 4350 2    50   ~ 0
ETH_RX_DV
Text Label 11900 5150 2    50   ~ 0
ETH_RX_CLK
Text Label 11900 2850 2    50   ~ 0
RGMII_TXD0
Text Label 11900 2950 2    50   ~ 0
RGMII_TXD1
Text Label 11900 3050 2    50   ~ 0
RGMII_TXD2
Text Label 11900 3150 2    50   ~ 0
RGMII_TXD3
Text Label 11900 3250 2    50   ~ 0
RGMII_TX_CLK
Text Label 11900 3350 2    50   ~ 0
RGMII_TX_EN
Text Label 11900 3750 2    50   ~ 0
ETH_MDC
Text Label 11900 3650 2    50   ~ 0
ETH_MDIO
Text Label 11900 3850 2    50   ~ 0
ETH_RST_N
$Comp
L device:R R37
U 1 1 618F6143
P 6200 3200
F 0 "R37" V 6100 3350 50  0000 C CNN
F 1 "10K" V 6200 3200 50  0000 C CNN
F 2 "" V 6130 3200 50  0001 C CNN
F 3 "" H 6200 3200 50  0001 C CNN
	1    6200 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 3200 5500 3200
Text Label 6350 3200 0    50   ~ 0
1V8
Text Notes 6100 3350 0    50   ~ 0
Strap: dual LEDs
$Comp
L device:R R35
U 1 1 618FA6FC
P 5700 1100
F 0 "R35" H 5770 1146 50  0000 L CNN
F 1 "1K" H 5770 1055 50  0000 L CNN
F 2 "" V 5630 1100 50  0001 C CNN
F 3 "" H 5700 1100 50  0001 C CNN
	1    5700 1100
	1    0    0    -1  
$EndComp
$Comp
L device:R R36
U 1 1 618FAC34
P 6150 1000
F 0 "R36" H 6220 1046 50  0000 L CNN
F 1 "1K" H 6220 955 50  0000 L CNN
F 2 "" V 6080 1000 50  0001 C CNN
F 3 "" H 6150 1000 50  0001 C CNN
	1    6150 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 1350 5500 1350
Wire Wire Line
	5500 1250 5700 1250
Text Label 5700 950  2    50   ~ 0
GND
Wire Wire Line
	6450 1250 5700 1250
Connection ~ 5700 1250
Wire Wire Line
	6150 1150 6150 1350
Wire Wire Line
	6150 1350 6450 1350
Connection ~ 6150 1350
Wire Wire Line
	5700 950  5700 850 
Wire Wire Line
	5700 850  6150 850 
Text Notes 5700 850  0    50   ~ 0
Strap: PHYADR[1:0] = 2'b00
Text Label 6450 1350 0    50   ~ 0
ETH_LED1_N_1V8
Text Label 6450 1250 0    50   ~ 0
ETH_LED2_N_1V8
Text Label 11900 3550 2    50   ~ 0
ETH_LED2_N_1V8
Text Label 11900 3450 2    50   ~ 0
ETH_LED1_N_1V8
Text Label 1500 2100 0    50   ~ 0
GND
Text Label 1500 2300 0    50   ~ 0
GND
$Comp
L device:R R33
U 1 1 6190D351
P 1800 2000
F 0 "R33" V 1700 2000 50  0000 C CNN
F 1 "470" V 1800 2000 50  0000 C CNN
F 2 "" V 1730 2000 50  0001 C CNN
F 3 "" H 1800 2000 50  0001 C CNN
	1    1800 2000
	0    1    1    0   
$EndComp
$Comp
L device:R R34
U 1 1 6190DC0A
P 1800 2200
F 0 "R34" V 1700 2200 50  0000 C CNN
F 1 "470" V 1800 2200 50  0000 C CNN
F 2 "" V 1730 2200 50  0001 C CNN
F 3 "" H 1800 2200 50  0001 C CNN
	1    1800 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 2200 1500 2200
Wire Wire Line
	1500 2000 1650 2000
Text HLabel 1950 2200 2    50   Input ~ 0
ETH_LED2_P
Text HLabel 1950 2000 2    50   Input ~ 0
ETH_LED1_P
$Comp
L device:R R38
U 1 1 61917B72
P 7800 1600
F 0 "R38" V 7750 1750 50  0000 C CNN
F 1 "10K" V 7800 1600 50  0000 C CNN
F 2 "" V 7730 1600 50  0001 C CNN
F 3 "" H 7800 1600 50  0001 C CNN
	1    7800 1600
	0    1    1    0   
$EndComp
Text Label 7650 1600 2    50   ~ 0
ETH_RXD3
$Comp
L device:R R39
U 1 1 619197D6
P 7800 1700
F 0 "R39" V 7750 1850 50  0000 C CNN
F 1 "10K" V 7800 1700 50  0000 C CNN
F 2 "" V 7730 1700 50  0001 C CNN
F 3 "" H 7800 1700 50  0001 C CNN
	1    7800 1700
	0    1    1    0   
$EndComp
Text Label 7650 1700 2    50   ~ 0
ETH_RXD2
$Comp
L device:R R40
U 1 1 61919E0D
P 7800 1800
F 0 "R40" V 7750 1950 50  0000 C CNN
F 1 "10K" V 7800 1800 50  0000 C CNN
F 2 "" V 7730 1800 50  0001 C CNN
F 3 "" H 7800 1800 50  0001 C CNN
	1    7800 1800
	0    1    1    0   
$EndComp
Text Label 7650 1800 2    50   ~ 0
ETH_RXD1
$Comp
L device:R R41
U 1 1 6191A22F
P 7800 1900
F 0 "R41" V 7750 2050 50  0000 C CNN
F 1 "10K" V 7800 1900 50  0000 C CNN
F 2 "" V 7730 1900 50  0001 C CNN
F 3 "" H 7800 1900 50  0001 C CNN
	1    7800 1900
	0    1    1    0   
$EndComp
Text Label 7650 1900 2    50   ~ 0
ETH_RXD0
Text Label 8150 1900 0    50   ~ 0
GND
Wire Wire Line
	8150 1900 7950 1900
Text Label 8150 1600 0    50   ~ 0
1V8
Wire Wire Line
	7950 1600 8150 1600
Wire Wire Line
	7950 1700 8150 1700
Wire Wire Line
	8150 1700 8150 1600
Wire Wire Line
	7950 1800 8150 1800
Wire Wire Line
	8150 1800 8150 1700
Connection ~ 8150 1700
Text Notes 7650 2050 0    50   ~ 0
Strap: advertise everything but gig/half
Text Label 7650 2350 2    50   ~ 0
ETH_RX_DV
$Comp
L device:R R42
U 1 1 619242C8
P 7800 2350
F 0 "R42" V 7750 2500 50  0000 C CNN
F 1 "10K" V 7800 2350 50  0000 C CNN
F 2 "" V 7730 2350 50  0001 C CNN
F 3 "" H 7800 2350 50  0001 C CNN
	1    7800 2350
	0    1    1    0   
$EndComp
Text Label 8150 2350 0    50   ~ 0
GND
Wire Wire Line
	8150 2350 7950 2350
Text Notes 7650 2500 0    50   ~ 0
Strap: Disable CLK125_NDO we don't use
Text Label 7650 2800 2    50   ~ 0
ETH_RX_CLK
$Comp
L device:R R43
U 1 1 61926D93
P 7800 2800
F 0 "R43" V 7750 2950 50  0000 C CNN
F 1 "10K" V 7800 2800 50  0000 C CNN
F 2 "" V 7730 2800 50  0001 C CNN
F 3 "" H 7800 2800 50  0001 C CNN
	1    7800 2800
	0    1    1    0   
$EndComp
Text Label 8150 2800 0    50   ~ 0
GND
Wire Wire Line
	8150 2800 7950 2800
Text Notes 7650 2950 0    50   ~ 0
Strap: PHYADDR[2] = 0
Text Label 5950 3850 0    50   ~ 0
GND
Text Label 7650 3300 2    50   ~ 0
ETH_MDIO
$Comp
L device:R R44
U 1 1 6192CD17
P 7800 3300
F 0 "R44" V 7750 3450 50  0000 C CNN
F 1 "1K" V 7800 3300 50  0000 C CNN
F 2 "" V 7730 3300 50  0001 C CNN
F 3 "" H 7800 3300 50  0001 C CNN
	1    7800 3300
	0    1    1    0   
$EndComp
Text Label 8100 3300 0    50   ~ 0
1V8
Wire Wire Line
	8100 3300 7950 3300
Text Label 2700 7300 0    50   ~ 0
A1V2_PLL
Text Label 3300 6650 0    50   ~ 0
A1V2
$Comp
L device:C C84
U 1 1 6193487F
P 1450 5650
F 0 "C84" H 1565 5696 50  0000 L CNN
F 1 "4.7 uF" H 1565 5605 50  0000 L CNN
F 2 "" H 1488 5500 50  0001 C CNN
F 3 "" H 1450 5650 50  0001 C CNN
	1    1450 5650
	1    0    0    -1  
$EndComp
Text Label 3850 5500 0    50   ~ 0
A3V3
$Comp
L passive-azonenberg:FERRITE_SMALL2 FB2
U 1 1 61937038
P 1850 5500
F 0 "FB2" H 1850 5725 50  0000 C CNN
F 1 "600R@100M" H 1850 5634 50  0000 C CNN
F 2 "" H 1850 5500 60  0000 C CNN
F 3 "" H 1850 5500 60  0000 C CNN
	1    1850 5500
	1    0    0    -1  
$EndComp
$Comp
L device:C C88
U 1 1 61937512
P 2200 5650
F 0 "C88" H 2315 5696 50  0000 L CNN
F 1 "4.7 uF" H 2315 5605 50  0000 L CNN
F 2 "" H 2238 5500 50  0001 C CNN
F 3 "" H 2200 5650 50  0001 C CNN
	1    2200 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 5500 2200 5500
Wire Wire Line
	1650 5500 1450 5500
Wire Wire Line
	1450 5800 2200 5800
Text Label 1450 5800 2    50   ~ 0
GND
$Comp
L device:C C94
U 1 1 61940A4A
P 2750 5650
F 0 "C94" H 2865 5696 50  0000 L CNN
F 1 "0.47 uF" H 2865 5605 50  0000 L CNN
F 2 "" H 2788 5500 50  0001 C CNN
F 3 "" H 2750 5650 50  0001 C CNN
	1    2750 5650
	1    0    0    -1  
$EndComp
$Comp
L device:C C95
U 1 1 61941116
P 3300 5650
F 0 "C95" H 3415 5696 50  0000 L CNN
F 1 "0.47 uF" H 3415 5605 50  0000 L CNN
F 2 "" H 3338 5500 50  0001 C CNN
F 3 "" H 3300 5650 50  0001 C CNN
	1    3300 5650
	1    0    0    -1  
$EndComp
$Comp
L device:C C97
U 1 1 619416A3
P 3850 5650
F 0 "C97" H 3965 5696 50  0000 L CNN
F 1 "0.47 uF" H 3965 5605 50  0000 L CNN
F 2 "" H 3888 5500 50  0001 C CNN
F 3 "" H 3850 5650 50  0001 C CNN
	1    3850 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5500 3300 5500
Connection ~ 2200 5500
Connection ~ 2750 5500
Wire Wire Line
	2750 5500 2200 5500
Connection ~ 3300 5500
Wire Wire Line
	3300 5500 2750 5500
Wire Wire Line
	2200 5800 2750 5800
Connection ~ 2200 5800
Connection ~ 2750 5800
Wire Wire Line
	2750 5800 3300 5800
Connection ~ 3300 5800
Wire Wire Line
	3300 5800 3850 5800
Text Label 2750 1150 2    50   ~ 0
A3V3
$Comp
L device:C C85
U 1 1 6194810E
P 1450 6150
F 0 "C85" H 1565 6196 50  0000 L CNN
F 1 "4.7 uF" H 1565 6105 50  0000 L CNN
F 2 "" H 1488 6000 50  0001 C CNN
F 3 "" H 1450 6150 50  0001 C CNN
	1    1450 6150
	1    0    0    -1  
$EndComp
$Comp
L device:C C87
U 1 1 6194A2CE
P 1950 6150
F 0 "C87" H 2065 6196 50  0000 L CNN
F 1 "0.47 uF" H 2065 6105 50  0000 L CNN
F 2 "" H 1988 6000 50  0001 C CNN
F 3 "" H 1950 6150 50  0001 C CNN
	1    1950 6150
	1    0    0    -1  
$EndComp
$Comp
L device:C C91
U 1 1 6194A73C
P 2500 6150
F 0 "C91" H 2615 6196 50  0000 L CNN
F 1 "0.47 uF" H 2615 6105 50  0000 L CNN
F 2 "" H 2538 6000 50  0001 C CNN
F 3 "" H 2500 6150 50  0001 C CNN
	1    2500 6150
	1    0    0    -1  
$EndComp
Text Label 1450 6000 2    50   ~ 0
1V8
Wire Wire Line
	1450 6000 1950 6000
Connection ~ 1950 6000
Wire Wire Line
	1950 6000 2500 6000
Wire Wire Line
	1450 6300 1950 6300
Connection ~ 1950 6300
Wire Wire Line
	1950 6300 2500 6300
Text Label 1450 6300 2    50   ~ 0
GND
Text Label 1450 6650 2    50   ~ 0
1V2
$Comp
L device:C C86
U 1 1 61952F5B
P 1450 6800
F 0 "C86" H 1565 6846 50  0000 L CNN
F 1 "4.7 uF" H 1565 6755 50  0000 L CNN
F 2 "" H 1488 6650 50  0001 C CNN
F 3 "" H 1450 6800 50  0001 C CNN
	1    1450 6800
	1    0    0    -1  
$EndComp
$Comp
L passive-azonenberg:FERRITE_SMALL2 FB3
U 1 1 61955AF1
P 1850 6650
F 0 "FB3" H 1850 6875 50  0000 C CNN
F 1 "600R@100M" H 1850 6784 50  0000 C CNN
F 2 "" H 1850 6650 60  0000 C CNN
F 3 "" H 1850 6650 60  0000 C CNN
	1    1850 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 6650 1450 6650
$Comp
L device:C C89
U 1 1 61958780
P 2200 6800
F 0 "C89" H 2315 6846 50  0000 L CNN
F 1 "4.7 uF" H 2315 6755 50  0000 L CNN
F 2 "" H 2238 6650 50  0001 C CNN
F 3 "" H 2200 6800 50  0001 C CNN
	1    2200 6800
	1    0    0    -1  
$EndComp
$Comp
L device:C C92
U 1 1 61958E8F
P 2700 6800
F 0 "C92" H 2815 6846 50  0000 L CNN
F 1 "0.47 uF" H 2815 6755 50  0000 L CNN
F 2 "" H 2738 6650 50  0001 C CNN
F 3 "" H 2700 6800 50  0001 C CNN
	1    2700 6800
	1    0    0    -1  
$EndComp
$Comp
L device:C C96
U 1 1 619590A5
P 3300 6800
F 0 "C96" H 3415 6846 50  0000 L CNN
F 1 "0.47 uF" H 3415 6755 50  0000 L CNN
F 2 "" H 3338 6650 50  0001 C CNN
F 3 "" H 3300 6800 50  0001 C CNN
	1    3300 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 6650 2700 6650
Connection ~ 2200 6650
Wire Wire Line
	2200 6650 2050 6650
Connection ~ 2700 6650
Wire Wire Line
	2700 6650 2200 6650
Wire Wire Line
	1450 6950 2200 6950
Connection ~ 2200 6950
Wire Wire Line
	2200 6950 2700 6950
Connection ~ 2700 6950
Wire Wire Line
	2700 6950 3300 6950
Text Label 1450 6950 2    50   ~ 0
GND
Text Label 1450 7300 2    50   ~ 0
1V2
$Comp
L passive-azonenberg:FERRITE_SMALL2 FB4
U 1 1 61964499
P 1850 7300
F 0 "FB4" H 1850 7525 50  0000 C CNN
F 1 "600R@100M" H 1850 7434 50  0000 C CNN
F 2 "" H 1850 7300 60  0000 C CNN
F 3 "" H 1850 7300 60  0000 C CNN
	1    1850 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 7300 1450 7300
$Comp
L device:C C90
U 1 1 619644A4
P 2200 7450
F 0 "C90" H 2315 7496 50  0000 L CNN
F 1 "4.7 uF" H 2315 7405 50  0000 L CNN
F 2 "" H 2238 7300 50  0001 C CNN
F 3 "" H 2200 7450 50  0001 C CNN
	1    2200 7450
	1    0    0    -1  
$EndComp
$Comp
L device:C C93
U 1 1 619644AE
P 2700 7450
F 0 "C93" H 2815 7496 50  0000 L CNN
F 1 "0.47 uF" H 2815 7405 50  0000 L CNN
F 2 "" H 2738 7300 50  0001 C CNN
F 3 "" H 2700 7450 50  0001 C CNN
	1    2700 7450
	1    0    0    -1  
$EndComp
Connection ~ 2200 7300
Wire Wire Line
	2200 7300 2050 7300
Wire Wire Line
	2700 7300 2200 7300
Wire Wire Line
	1450 7600 2200 7600
Connection ~ 2200 7600
Wire Wire Line
	2200 7600 2700 7600
Text Label 1450 7600 2    50   ~ 0
GND
$Comp
L special-azonenberg:CONN_SFF8087 J8
U 2 1 6197F0EA
P 8350 7850
F 0 "J8" H 8492 10325 50  0000 C CNN
F 1 "CONN_SFF8087" H 8492 10234 50  0000 C CNN
F 2 "" H 8350 7850 50  0001 C CNN
F 3 "" H 8350 7850 50  0001 C CNN
	2    8350 7850
	-1   0    0    -1  
$EndComp
Text Label 8450 5600 0    50   ~ 0
LA0_DQ0_P
Text Label 8450 5700 0    50   ~ 0
LA0_DQ0_N
Text Label 8450 5900 0    50   ~ 0
LA0_DQ1_P
Text Label 8450 6000 0    50   ~ 0
LA0_DQ1_N
Text Label 8450 6200 0    50   ~ 0
LA0_DQ2_P
Text Label 8450 6300 0    50   ~ 0
LA0_DQ2_N
Text Label 8450 6500 0    50   ~ 0
LA0_DQ3_P
Text Label 8450 6600 0    50   ~ 0
LA0_DQ3_N
Text Label 8450 6800 0    50   ~ 0
LA0_DQ4_P
Text Label 8450 6900 0    50   ~ 0
LA0_DQ4_N
Text Label 8450 7100 0    50   ~ 0
LA0_DQ5_P
Text Label 8450 7200 0    50   ~ 0
LA0_DQ5_N
Text Label 8450 7400 0    50   ~ 0
LA0_DQ6_P
Text Label 8450 7500 0    50   ~ 0
LA0_DQ6_N
Text Label 8450 7700 0    50   ~ 0
LA0_DQ7_P
Text Label 8450 7800 0    50   ~ 0
LA0_DQ7_N
Text Label 11900 6150 2    50   ~ 0
LA0_DQ0_P
Text Label 11900 6250 2    50   ~ 0
LA0_DQ0_N
Text Label 11900 6350 2    50   ~ 0
LA0_DQ1_P
Text Label 11900 6450 2    50   ~ 0
LA0_DQ1_N
Text Label 11900 6550 2    50   ~ 0
LA0_DQ2_P
Text Label 11900 6650 2    50   ~ 0
LA0_DQ2_N
Text Label 11900 6750 2    50   ~ 0
LA0_DQ3_P
Text Label 11900 6850 2    50   ~ 0
LA0_DQ3_N
Text Label 11900 6950 2    50   ~ 0
LA0_DQ4_P
Text Label 11900 7050 2    50   ~ 0
LA0_DQ4_N
Text Label 11900 7150 2    50   ~ 0
LA0_DQ5_P
Text Label 11900 7250 2    50   ~ 0
LA0_DQ5_N
Text Label 11900 7350 2    50   ~ 0
LA0_DQ6_P
Text Label 11900 7450 2    50   ~ 0
LA0_DQ6_N
Text Label 11900 7550 2    50   ~ 0
LA0_DQ7_P
Text Label 11900 7650 2    50   ~ 0
LA0_DQ7_N
Text HLabel 3250 3000 0    50   Input ~ 0
GND
$EndSCHEMATC
