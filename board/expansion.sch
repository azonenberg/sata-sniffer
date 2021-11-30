EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 8 10
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
U 8 1 6192DE5F
P 2450 6450
AR Path="/618C589A/6192DE5F" Ref="U?"  Part="3" 
AR Path="/61909AE6/6192DE5F" Ref="U?"  Part="6" 
AR Path="/6192A55E/6192DE5F" Ref="U5"  Part="8" 
F 0 "U5" H 2450 6350 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 2450 6250 60  0000 L CNN
F 2 "azonenberg_pcb:BGA_484_22x22_FULLARRAY_1MM" H 2450 6150 60  0001 C CNN
F 3 "" H 2450 6150 60  0000 C CNN
	8    2450 6450
	1    0    0    -1  
$EndComp
$Comp
L special-azonenberg:CONN_SFF8087 J?
U 2 1 6198CEDF
P 9600 3300
AR Path="/61923389/6198CEDF" Ref="J?"  Part="2" 
AR Path="/6192A55E/6198CEDF" Ref="J9"  Part="2" 
F 0 "J9" H 9742 5775 50  0000 C CNN
F 1 "CONN_SFF8087" H 9742 5684 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SFF8087_MOLEX_75783-0140" H 9600 3300 50  0001 C CNN
F 3 "" H 9600 3300 50  0001 C CNN
	2    9600 3300
	-1   0    0    -1  
$EndComp
Text Label 9700 1050 0    50   ~ 0
LA1_DQ0_P
Text Label 9700 1150 0    50   ~ 0
LA1_DQ0_N
Text Label 9700 1350 0    50   ~ 0
LA1_DQ1_P
Text Label 9700 1450 0    50   ~ 0
LA1_DQ1_N
Text Label 9700 1650 0    50   ~ 0
LA1_DQ2_P
Text Label 9700 1750 0    50   ~ 0
LA1_DQ2_N
Text Label 9700 1950 0    50   ~ 0
LA1_DQ3_P
Text Label 9700 2050 0    50   ~ 0
LA1_DQ3_N
Text Label 9700 2250 0    50   ~ 0
LA1_DQ4_P
Text Label 9700 2350 0    50   ~ 0
LA1_DQ4_N
Text Label 9700 2550 0    50   ~ 0
LA1_DQ5_P
Text Label 9700 2650 0    50   ~ 0
LA1_DQ5_N
Text Label 9700 2850 0    50   ~ 0
LA1_DQ6_P
Text Label 9700 2950 0    50   ~ 0
LA1_DQ6_N
Text Label 9700 3150 0    50   ~ 0
LA1_DQ7_P
Text Label 9700 3250 0    50   ~ 0
LA1_DQ7_N
Text Label 2250 2250 2    50   ~ 0
LA1_DQ0_P
Text Label 2250 2350 2    50   ~ 0
LA1_DQ0_N
Text Label 2250 4650 2    50   ~ 0
LA1_DQ1_P
Text Label 2250 4750 2    50   ~ 0
LA1_DQ1_N
Text Label 2250 5250 2    50   ~ 0
LA1_DQ2_P
Text Label 2250 5350 2    50   ~ 0
LA1_DQ2_N
Text Label 2250 3250 2    50   ~ 0
LA1_DQ3_P
Text Label 2250 3350 2    50   ~ 0
LA1_DQ3_N
Text Label 2250 2850 2    50   ~ 0
LA1_DQ4_P
Text Label 2250 2950 2    50   ~ 0
LA1_DQ4_N
Text Label 2250 5850 2    50   ~ 0
LA1_DQ5_P
Text Label 2250 5950 2    50   ~ 0
LA1_DQ5_N
Text Label 2250 6050 2    50   ~ 0
LA1_DQ6_P
Text Label 2250 6150 2    50   ~ 0
LA1_DQ6_N
Text Label 2250 6250 2    50   ~ 0
LA1_DQ7_P
Text Label 2250 6350 2    50   ~ 0
LA1_DQ7_N
NoConn ~ 2250 1550
NoConn ~ 2250 6450
$Comp
L special-azonenberg:CONN_LVDS_EXPANSION_HOST J15
U 1 1 61DD3515
P 5050 5600
F 0 "J15" H 5500 9875 50  0000 C CNN
F 1 "CONN_LVDS_EXPANSION_HOST" H 5500 9784 50  0000 C CNN
F 2 "azonenberg_pcb:CONN_SAMTEC_QTH-030-01-L-D-A" H 5050 5600 50  0001 C CNN
F 3 "" H 5050 5600 50  0001 C CNN
	1    5050 5600
	1    0    0    -1  
$EndComp
Text HLabel 6100 1550 2    50   Input ~ 0
5V0
Wire Wire Line
	6100 1550 6100 1650
Text HLabel 6100 1850 2    50   Input ~ 0
GND
Wire Wire Line
	6100 1850 6100 1950
Connection ~ 6100 1950
Wire Wire Line
	6100 1950 6100 2050
Connection ~ 6100 2050
Wire Wire Line
	6100 2050 6100 2150
Connection ~ 6100 2150
Wire Wire Line
	6100 2150 6100 2250
Connection ~ 6100 2250
Wire Wire Line
	6100 2250 6100 2350
Connection ~ 6100 2350
Wire Wire Line
	6100 2350 6100 2450
Connection ~ 6100 2450
Wire Wire Line
	6100 2450 6100 2550
Connection ~ 6100 2550
Wire Wire Line
	6100 2550 6100 2650
Connection ~ 6100 2650
Wire Wire Line
	6100 2650 6100 2750
Connection ~ 6100 2750
Wire Wire Line
	6100 2750 6100 2850
Connection ~ 6100 2850
Wire Wire Line
	6100 2850 6100 2950
Connection ~ 6100 2950
Wire Wire Line
	6100 2950 6100 3050
Connection ~ 6100 3050
Wire Wire Line
	6100 3050 6100 3150
Connection ~ 6100 3150
Wire Wire Line
	6100 3150 6100 3250
Connection ~ 6100 3250
Wire Wire Line
	6100 3250 6100 3350
Connection ~ 6100 3350
Wire Wire Line
	6100 3350 6100 3450
Connection ~ 6100 3450
Wire Wire Line
	6100 3450 6100 3550
Connection ~ 6100 3550
Wire Wire Line
	6100 3550 6100 3650
Text Label 2250 1850 2    50   ~ 0
LVDS0_P
Text Label 2250 1950 2    50   ~ 0
LVDS0_N
Text Label 2250 1650 2    50   ~ 0
LVDS1_P
Text Label 2250 1750 2    50   ~ 0
LVDS1_N
Text Label 2250 2150 2    50   ~ 0
LVDS2_P
Text Label 2250 2050 2    50   ~ 0
LVDS2_N
Text Label 2250 3050 2    50   ~ 0
LVDS3_P
Text Label 2250 3150 2    50   ~ 0
LVDS3_N
Text Label 2250 3850 2    50   ~ 0
LVDS4_P
Text Label 2250 3950 2    50   ~ 0
LVDS4_N
Text Label 2250 3450 2    50   ~ 0
LVDS5_P
Text Label 2250 3550 2    50   ~ 0
LVDS5_N
Text Label 2250 4450 2    50   ~ 0
LVDS6_P
Text Label 2250 4550 2    50   ~ 0
LVDS6_N
Text Label 2250 4850 2    50   ~ 0
LVDS7_P
Text Label 2250 4950 2    50   ~ 0
LVDS7_N
Text Label 2250 5450 2    50   ~ 0
LVDS8_P
Text Label 2250 5550 2    50   ~ 0
LVDS8_N
Text Label 2250 5750 2    50   ~ 0
LVDS9_P
Text Label 2250 5650 2    50   ~ 0
LVDS9_N
Text Label 2250 3650 2    50   ~ 0
LVDS10_P
Text Label 2250 3750 2    50   ~ 0
LVDS10_N
Text Label 2250 2450 2    50   ~ 0
LVDS11_P
Text Label 2250 2550 2    50   ~ 0
LVDS11_N
Text Label 2250 4050 2    50   ~ 0
LVDS12_P
Text Label 2250 4150 2    50   ~ 0
LVDS12_N
Text Label 2250 4250 2    50   ~ 0
LVDS13_P
Text Label 2250 4350 2    50   ~ 0
LVDS13_N
Text Label 2250 2650 2    50   ~ 0
LVDS14_P
Text Label 2250 2750 2    50   ~ 0
LVDS14_N
Text Label 2250 5050 2    50   ~ 0
LVDS15_P
Text Label 2250 5150 2    50   ~ 0
LVDS15_N
Text Label 4900 1550 2    50   ~ 0
LVDS0_P
Text Label 4900 1650 2    50   ~ 0
LVDS0_N
Text Label 4900 1750 2    50   ~ 0
LVDS1_P
Text Label 4900 1850 2    50   ~ 0
LVDS1_N
Text Label 4900 1950 2    50   ~ 0
LVDS2_P
Text Label 4900 2050 2    50   ~ 0
LVDS2_N
Text Label 4900 2150 2    50   ~ 0
LVDS3_P
Text Label 4900 2250 2    50   ~ 0
LVDS3_N
Text Label 4900 2350 2    50   ~ 0
LVDS4_P
Text Label 4900 2450 2    50   ~ 0
LVDS4_N
Text Label 4900 2550 2    50   ~ 0
LVDS5_P
Text Label 4900 2650 2    50   ~ 0
LVDS5_N
Text Label 4900 2750 2    50   ~ 0
LVDS6_P
Text Label 4900 2850 2    50   ~ 0
LVDS6_N
Text Label 4900 2950 2    50   ~ 0
LVDS7_P
Text Label 4900 3050 2    50   ~ 0
LVDS7_N
Text Label 4900 3150 2    50   ~ 0
LVDS8_P
Text Label 4900 3250 2    50   ~ 0
LVDS8_N
Text Label 4900 3350 2    50   ~ 0
LVDS9_P
Text Label 4900 3450 2    50   ~ 0
LVDS9_N
Text Label 4900 3650 2    50   ~ 0
LVDS10_P
Text Label 4900 3750 2    50   ~ 0
LVDS10_N
Text Label 4900 3850 2    50   ~ 0
LVDS11_P
Text Label 4900 3950 2    50   ~ 0
LVDS11_N
Text Label 4900 4050 2    50   ~ 0
LVDS12_P
Text Label 4900 4150 2    50   ~ 0
LVDS12_N
Text Label 4900 4250 2    50   ~ 0
LVDS13_P
Text Label 4900 4350 2    50   ~ 0
LVDS13_N
Text Label 4900 4450 2    50   ~ 0
LVDS14_P
Text Label 4900 4550 2    50   ~ 0
LVDS14_N
Text Label 4900 4650 2    50   ~ 0
LVDS15_P
Text Label 4900 4750 2    50   ~ 0
LVDS15_N
NoConn ~ 4900 4850
NoConn ~ 4900 4950
NoConn ~ 4900 5050
NoConn ~ 4900 5150
NoConn ~ 4900 5250
NoConn ~ 4900 5350
NoConn ~ 4900 5450
NoConn ~ 4900 5550
Text Notes 850  1150 0    50   ~ 0
Pairs swapped for routability:\nLVDS_2\nLVDS_9
$EndSCHEMATC
