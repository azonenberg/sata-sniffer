EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 3 10
Title "SATA sniffer"
Date "2021-11-12"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L memory-azonenberg:DDR3_SODIMM U4
U 1 1 618C5D61
P 1850 5200
F 0 "U4" H 2350 9375 50  0000 C CNN
F 1 "DDR3_SODIMM" H 2350 9284 50  0000 C CNN
F 2 "" H 2100 5150 50  0001 C CNN
F 3 "" H 2100 5150 50  0001 C CNN
	1    1850 5200
	1    0    0    -1  
$EndComp
$Comp
L memory-azonenberg:DDR3_SODIMM U4
U 2 1 618CAA1A
P 4150 3700
F 0 "U4" H 4650 6375 50  0000 C CNN
F 1 "DDR3_SODIMM" H 4650 6284 50  0000 C CNN
F 2 "" H 4400 3650 50  0001 C CNN
F 3 "" H 4400 3650 50  0001 C CNN
	2    4150 3700
	1    0    0    -1  
$EndComp
$Comp
L memory-azonenberg:DDR3_SODIMM U4
U 3 1 618CE33B
P 9850 5400
F 0 "U4" H 10350 10275 50  0000 C CNN
F 1 "DDR3_SODIMM" H 10350 10184 50  0000 C CNN
F 2 "" H 10100 5350 50  0001 C CNN
F 3 "" H 10100 5350 50  0001 C CNN
	3    9850 5400
	1    0    0    -1  
$EndComp
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U5
U 3 1 618D358F
P 6350 10750
F 0 "U5" H 6350 10650 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 6350 10550 60  0000 L CNN
F 2 "" H 6350 10450 60  0000 C CNN
F 3 "" H 6350 10450 60  0000 C CNN
	3    6350 10750
	1    0    0    -1  
$EndComp
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U5
U 4 1 618EA0DE
P 9850 10750
F 0 "U5" H 9850 10650 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 9850 10550 60  0000 L CNN
F 2 "" H 9850 10450 60  0000 C CNN
F 3 "" H 9850 10450 60  0000 C CNN
	4    9850 10750
	1    0    0    -1  
$EndComp
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U5
U 5 1 618F6288
P 3600 10750
F 0 "U5" H 3600 10650 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 3600 10550 60  0000 L CNN
F 2 "" H 3600 10450 60  0000 C CNN
F 3 "" H 3600 10450 60  0000 C CNN
	5    3600 10750
	1    0    0    -1  
$EndComp
Text HLabel 1650 1250 0    50   Input ~ 0
3V3
Text HLabel 3050 1250 2    50   Input ~ 0
GND
Wire Wire Line
	3050 1250 3050 1350
Connection ~ 3050 1350
Wire Wire Line
	3050 1350 3050 1450
Connection ~ 3050 1450
Wire Wire Line
	3050 1450 3050 1550
Connection ~ 3050 1550
Wire Wire Line
	3050 1550 3050 1650
Connection ~ 3050 1650
Wire Wire Line
	3050 1650 3050 1750
Connection ~ 3050 1750
Wire Wire Line
	3050 1750 3050 1850
Connection ~ 3050 1850
Wire Wire Line
	3050 1850 3050 1950
Connection ~ 3050 1950
Wire Wire Line
	3050 1950 3050 2050
Connection ~ 3050 2050
Wire Wire Line
	3050 2050 3050 2150
Connection ~ 3050 2150
Wire Wire Line
	3050 2150 3050 2250
Connection ~ 3050 2250
Wire Wire Line
	3050 2250 3050 2350
Connection ~ 3050 2350
Wire Wire Line
	3050 2350 3050 2450
Connection ~ 3050 2450
Wire Wire Line
	3050 2450 3050 2550
Connection ~ 3050 2550
Wire Wire Line
	3050 2550 3050 2650
Connection ~ 3050 2650
Wire Wire Line
	3050 2650 3050 2750
Connection ~ 3050 2750
Wire Wire Line
	3050 2750 3050 2850
Connection ~ 3050 2850
Wire Wire Line
	3050 2850 3050 2950
Connection ~ 3050 2950
Wire Wire Line
	3050 2950 3050 3050
Connection ~ 3050 3050
Wire Wire Line
	3050 3050 3050 3150
Connection ~ 3050 3150
Wire Wire Line
	3050 3150 3050 3250
Connection ~ 3050 3250
Wire Wire Line
	3050 3250 3050 3350
Connection ~ 3050 3350
Wire Wire Line
	3050 3350 3050 3450
Connection ~ 3050 3450
Wire Wire Line
	3050 3450 3050 3550
Connection ~ 3050 3550
Wire Wire Line
	3050 3550 3050 3650
Connection ~ 3050 3650
Wire Wire Line
	3050 3650 3050 3750
Connection ~ 3050 3750
Wire Wire Line
	3050 3750 3050 3850
Connection ~ 3050 3850
Wire Wire Line
	3050 3850 3050 3950
Connection ~ 3050 3950
Wire Wire Line
	3050 3950 3050 4050
Connection ~ 3050 4050
Wire Wire Line
	3050 4050 3050 4150
Connection ~ 3050 4150
Wire Wire Line
	3050 4150 3050 4250
Connection ~ 3050 4250
Wire Wire Line
	3050 4250 3050 4350
Connection ~ 3050 4350
Wire Wire Line
	3050 4350 3050 4450
Connection ~ 3050 4450
Wire Wire Line
	3050 4450 3050 4550
Connection ~ 3050 4550
Wire Wire Line
	3050 4550 3050 4650
Connection ~ 3050 4650
Wire Wire Line
	3050 4650 3050 4750
Connection ~ 3050 4750
Wire Wire Line
	3050 4750 3050 4850
Connection ~ 3050 4850
Wire Wire Line
	3050 4850 3050 4950
Connection ~ 3050 4950
Wire Wire Line
	3050 4950 3050 5050
Wire Wire Line
	1650 3950 1650 4050
Connection ~ 1650 4050
Wire Wire Line
	1650 4050 1650 4150
Connection ~ 1650 4150
Wire Wire Line
	1650 4150 1650 4250
Connection ~ 1650 4250
Wire Wire Line
	1650 4250 1650 4350
Connection ~ 1650 4350
Wire Wire Line
	1650 4350 1650 4450
Connection ~ 1650 4450
Wire Wire Line
	1650 4450 1650 4550
Connection ~ 1650 4550
Wire Wire Line
	1650 4550 1650 4650
Connection ~ 1650 4650
Wire Wire Line
	1650 4650 1650 4750
Connection ~ 1650 4750
Wire Wire Line
	1650 4750 1650 4850
Connection ~ 1650 4850
Wire Wire Line
	1650 4850 1650 4950
Connection ~ 1650 4950
Wire Wire Line
	1650 4950 1650 5050
Connection ~ 1650 5050
Wire Wire Line
	1650 5050 1650 5150
Wire Wire Line
	1650 2050 1650 2150
Connection ~ 1650 2150
Wire Wire Line
	1650 2150 1650 2250
Connection ~ 1650 2250
Wire Wire Line
	1650 2250 1650 2350
Connection ~ 1650 2350
Wire Wire Line
	1650 2350 1650 2450
Connection ~ 1650 2450
Wire Wire Line
	1650 2450 1650 2550
Connection ~ 1650 2550
Wire Wire Line
	1650 2550 1650 2650
Connection ~ 1650 2650
Wire Wire Line
	1650 2650 1650 2750
Connection ~ 1650 2750
Wire Wire Line
	1650 2750 1650 2850
Connection ~ 1650 2850
Wire Wire Line
	1650 2850 1650 2950
Connection ~ 1650 2950
Wire Wire Line
	1650 2950 1650 3050
Connection ~ 1650 3050
Wire Wire Line
	1650 3050 1650 3150
Connection ~ 1650 3150
Wire Wire Line
	1650 3150 1650 3250
Connection ~ 1650 3250
Wire Wire Line
	1650 3250 1650 3350
Connection ~ 1650 3350
Wire Wire Line
	1650 3350 1650 3450
Connection ~ 1650 3450
Wire Wire Line
	1650 3450 1650 3550
Connection ~ 1650 3550
Wire Wire Line
	1650 3550 1650 3650
Connection ~ 1650 3650
Wire Wire Line
	1650 3650 1650 3750
Text Label 1650 3950 2    50   ~ 0
GND
Text HLabel 1650 2050 0    50   Input ~ 0
1V35
Text HLabel 1650 1750 0    50   Input ~ 0
VTT
Wire Wire Line
	1650 1750 1650 1850
Text HLabel 1650 1450 0    50   Input ~ 0
VREF
Wire Wire Line
	1650 1450 1650 1550
Text Notes 9850 5800 0    50   ~ 0
C/A
Text Notes 6350 5800 0    50   ~ 0
DQ high
Text Notes 3600 5800 0    50   ~ 0
DQ low
Text Label 12600 1150 2    50   ~ 0
VTT
$Comp
L device:C C2
U 1 1 61ABEC04
P 12600 1300
F 0 "C2" H 12715 1346 50  0000 L CNN
F 1 "22 uF" H 12715 1255 50  0000 L CNN
F 2 "" H 12638 1150 50  0001 C CNN
F 3 "" H 12600 1300 50  0001 C CNN
	1    12600 1300
	1    0    0    -1  
$EndComp
$Comp
L device:C C9
U 1 1 61ABF112
P 13150 1300
F 0 "C9" H 13265 1346 50  0000 L CNN
F 1 "0.47 uF" H 13265 1255 50  0000 L CNN
F 2 "" H 13188 1150 50  0001 C CNN
F 3 "" H 13150 1300 50  0001 C CNN
	1    13150 1300
	1    0    0    -1  
$EndComp
$Comp
L device:C C15
U 1 1 61ABF4C0
P 13650 1300
F 0 "C15" H 13765 1346 50  0000 L CNN
F 1 "0.47 uF" H 13765 1255 50  0000 L CNN
F 2 "" H 13688 1150 50  0001 C CNN
F 3 "" H 13650 1300 50  0001 C CNN
	1    13650 1300
	1    0    0    -1  
$EndComp
$Comp
L device:C C21
U 1 1 61ABF772
P 14150 1300
F 0 "C21" H 14265 1346 50  0000 L CNN
F 1 "DNP" H 14265 1255 50  0000 L CNN
F 2 "" H 14188 1150 50  0001 C CNN
F 3 "" H 14150 1300 50  0001 C CNN
	1    14150 1300
	1    0    0    -1  
$EndComp
$Comp
L device:C C26
U 1 1 61ABFCD0
P 14600 1300
F 0 "C26" H 14715 1346 50  0000 L CNN
F 1 "DNP" H 14715 1255 50  0000 L CNN
F 2 "" H 14638 1150 50  0001 C CNN
F 3 "" H 14600 1300 50  0001 C CNN
	1    14600 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	14600 1150 14150 1150
Connection ~ 13150 1150
Wire Wire Line
	13150 1150 12600 1150
Connection ~ 13650 1150
Wire Wire Line
	13650 1150 13150 1150
Connection ~ 14150 1150
Wire Wire Line
	14150 1150 13650 1150
Wire Wire Line
	12600 1450 13150 1450
Connection ~ 13150 1450
Wire Wire Line
	13150 1450 13650 1450
Connection ~ 13650 1450
Wire Wire Line
	13650 1450 14150 1450
Connection ~ 14150 1450
Wire Wire Line
	14150 1450 14600 1450
Text Label 12600 1450 2    50   ~ 0
GND
Text Label 12600 1650 2    50   ~ 0
1V35
Text Label 12600 1950 2    50   ~ 0
GND
$Comp
L device:C C3
U 1 1 61AC8DC3
P 12600 1800
F 0 "C3" H 12715 1846 50  0000 L CNN
F 1 "100 uF" H 12715 1755 50  0000 L CNN
F 2 "" H 12638 1650 50  0001 C CNN
F 3 "" H 12600 1800 50  0001 C CNN
	1    12600 1800
	1    0    0    -1  
$EndComp
$Comp
L device:C C10
U 1 1 61ACC594
P 13150 1800
F 0 "C10" H 13265 1846 50  0000 L CNN
F 1 "100 uF" H 13265 1755 50  0000 L CNN
F 2 "" H 13188 1650 50  0001 C CNN
F 3 "" H 13150 1800 50  0001 C CNN
	1    13150 1800
	1    0    0    -1  
$EndComp
$Comp
L device:C C16
U 1 1 61ACCA4D
P 13650 1800
F 0 "C16" H 13765 1846 50  0000 L CNN
F 1 "4.7 uF" H 13765 1755 50  0000 L CNN
F 2 "" H 13688 1650 50  0001 C CNN
F 3 "" H 13650 1800 50  0001 C CNN
	1    13650 1800
	1    0    0    -1  
$EndComp
$Comp
L device:C C22
U 1 1 61ACCE1D
P 14150 1800
F 0 "C22" H 14265 1846 50  0000 L CNN
F 1 "4.7 uF" H 14265 1755 50  0000 L CNN
F 2 "" H 14188 1650 50  0001 C CNN
F 3 "" H 14150 1800 50  0001 C CNN
	1    14150 1800
	1    0    0    -1  
$EndComp
$Comp
L device:C C27
U 1 1 61ACD366
P 14600 1800
F 0 "C27" H 14715 1846 50  0000 L CNN
F 1 "4.7 uF" H 14715 1755 50  0000 L CNN
F 2 "" H 14638 1650 50  0001 C CNN
F 3 "" H 14600 1800 50  0001 C CNN
	1    14600 1800
	1    0    0    -1  
$EndComp
$Comp
L device:C C4
U 1 1 61ACD544
P 12600 2300
F 0 "C4" H 12715 2346 50  0000 L CNN
F 1 "4.7 uF" H 12715 2255 50  0000 L CNN
F 2 "" H 12638 2150 50  0001 C CNN
F 3 "" H 12600 2300 50  0001 C CNN
	1    12600 2300
	1    0    0    -1  
$EndComp
$Comp
L device:C C11
U 1 1 61ACD8A6
P 13150 2300
F 0 "C11" H 13265 2346 50  0000 L CNN
F 1 "4.7 uF" H 13265 2255 50  0000 L CNN
F 2 "" H 13188 2150 50  0001 C CNN
F 3 "" H 13150 2300 50  0001 C CNN
	1    13150 2300
	1    0    0    -1  
$EndComp
$Comp
L device:C C17
U 1 1 61ACDABF
P 13650 2300
F 0 "C17" H 13765 2346 50  0000 L CNN
F 1 "4.7 uF" H 13765 2255 50  0000 L CNN
F 2 "" H 13688 2150 50  0001 C CNN
F 3 "" H 13650 2300 50  0001 C CNN
	1    13650 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	12600 1650 13150 1650
Connection ~ 13150 1650
Wire Wire Line
	13150 1650 13650 1650
Connection ~ 13650 1650
Wire Wire Line
	13650 1650 14150 1650
Wire Wire Line
	14150 1650 14600 1650
Connection ~ 14150 1650
Wire Wire Line
	12600 1950 13150 1950
Connection ~ 13150 1950
Wire Wire Line
	13150 1950 13650 1950
Connection ~ 13650 1950
Wire Wire Line
	13650 1950 14150 1950
Connection ~ 14150 1950
Wire Wire Line
	14150 1950 14600 1950
Text Label 12600 2150 2    50   ~ 0
1V35
Connection ~ 13150 2150
Wire Wire Line
	13150 2150 13650 2150
Wire Wire Line
	12600 2150 13150 2150
Text Label 12600 2450 2    50   ~ 0
GND
Wire Wire Line
	12600 2450 13150 2450
Connection ~ 13150 2450
Wire Wire Line
	13150 2450 13650 2450
$Comp
L device:C C5
U 1 1 61AE692B
P 12600 2800
F 0 "C5" H 12715 2846 50  0000 L CNN
F 1 "0.47 uF" H 12715 2755 50  0000 L CNN
F 2 "" H 12638 2650 50  0001 C CNN
F 3 "" H 12600 2800 50  0001 C CNN
	1    12600 2800
	1    0    0    -1  
$EndComp
$Comp
L device:C C12
U 1 1 61AE6DA8
P 13150 2800
F 0 "C12" H 13265 2846 50  0000 L CNN
F 1 "0.47 uF" H 13265 2755 50  0000 L CNN
F 2 "" H 13188 2650 50  0001 C CNN
F 3 "" H 13150 2800 50  0001 C CNN
	1    13150 2800
	1    0    0    -1  
$EndComp
$Comp
L device:C C18
U 1 1 61AE7095
P 13700 2800
F 0 "C18" H 13815 2846 50  0000 L CNN
F 1 "0.47 uF" H 13815 2755 50  0000 L CNN
F 2 "" H 13738 2650 50  0001 C CNN
F 3 "" H 13700 2800 50  0001 C CNN
	1    13700 2800
	1    0    0    -1  
$EndComp
$Comp
L device:C C23
U 1 1 61AEA349
P 14250 2800
F 0 "C23" H 14365 2846 50  0000 L CNN
F 1 "0.47 uF" H 14365 2755 50  0000 L CNN
F 2 "" H 14288 2650 50  0001 C CNN
F 3 "" H 14250 2800 50  0001 C CNN
	1    14250 2800
	1    0    0    -1  
$EndComp
$Comp
L device:C C28
U 1 1 61AEA710
P 14800 2800
F 0 "C28" H 14915 2846 50  0000 L CNN
F 1 "0.47 uF" H 14915 2755 50  0000 L CNN
F 2 "" H 14838 2650 50  0001 C CNN
F 3 "" H 14800 2800 50  0001 C CNN
	1    14800 2800
	1    0    0    -1  
$EndComp
$Comp
L device:C C31
U 1 1 61AEA9B8
P 15350 2800
F 0 "C31" H 15465 2846 50  0000 L CNN
F 1 "0.47 uF" H 15465 2755 50  0000 L CNN
F 2 "" H 15388 2650 50  0001 C CNN
F 3 "" H 15350 2800 50  0001 C CNN
	1    15350 2800
	1    0    0    -1  
$EndComp
Text Label 12600 2650 2    50   ~ 0
1V35
Text Label 12600 2950 2    50   ~ 0
GND
Wire Wire Line
	12600 2950 13150 2950
Connection ~ 13150 2950
Wire Wire Line
	13150 2950 13700 2950
Connection ~ 13700 2950
Wire Wire Line
	13700 2950 14250 2950
Connection ~ 14250 2950
Wire Wire Line
	14250 2950 14800 2950
Connection ~ 14800 2950
Wire Wire Line
	14800 2950 15350 2950
Wire Wire Line
	15350 2650 14800 2650
Connection ~ 13150 2650
Wire Wire Line
	13150 2650 12600 2650
Connection ~ 13700 2650
Wire Wire Line
	13700 2650 13150 2650
Connection ~ 14250 2650
Wire Wire Line
	14250 2650 13700 2650
Connection ~ 14800 2650
Wire Wire Line
	14800 2650 14250 2650
$Comp
L device:C C6
U 1 1 61AF37A1
P 12600 3300
F 0 "C6" H 12715 3346 50  0000 L CNN
F 1 "0.47 uF" H 12715 3255 50  0000 L CNN
F 2 "" H 12638 3150 50  0001 C CNN
F 3 "" H 12600 3300 50  0001 C CNN
	1    12600 3300
	1    0    0    -1  
$EndComp
$Comp
L device:C C13
U 1 1 61AF37AB
P 13150 3300
F 0 "C13" H 13265 3346 50  0000 L CNN
F 1 "0.47 uF" H 13265 3255 50  0000 L CNN
F 2 "" H 13188 3150 50  0001 C CNN
F 3 "" H 13150 3300 50  0001 C CNN
	1    13150 3300
	1    0    0    -1  
$EndComp
$Comp
L device:C C19
U 1 1 61AF37B5
P 13700 3300
F 0 "C19" H 13815 3346 50  0000 L CNN
F 1 "0.47 uF" H 13815 3255 50  0000 L CNN
F 2 "" H 13738 3150 50  0001 C CNN
F 3 "" H 13700 3300 50  0001 C CNN
	1    13700 3300
	1    0    0    -1  
$EndComp
$Comp
L device:C C24
U 1 1 61AF37BF
P 14250 3300
F 0 "C24" H 14365 3346 50  0000 L CNN
F 1 "0.47 uF" H 14365 3255 50  0000 L CNN
F 2 "" H 14288 3150 50  0001 C CNN
F 3 "" H 14250 3300 50  0001 C CNN
	1    14250 3300
	1    0    0    -1  
$EndComp
$Comp
L device:C C29
U 1 1 61AF37C9
P 14800 3300
F 0 "C29" H 14915 3346 50  0000 L CNN
F 1 "0.47 uF" H 14915 3255 50  0000 L CNN
F 2 "" H 14838 3150 50  0001 C CNN
F 3 "" H 14800 3300 50  0001 C CNN
	1    14800 3300
	1    0    0    -1  
$EndComp
$Comp
L device:C C32
U 1 1 61AF37D3
P 15350 3300
F 0 "C32" H 15465 3346 50  0000 L CNN
F 1 "0.47 uF" H 15465 3255 50  0000 L CNN
F 2 "" H 15388 3150 50  0001 C CNN
F 3 "" H 15350 3300 50  0001 C CNN
	1    15350 3300
	1    0    0    -1  
$EndComp
Text Label 12600 3150 2    50   ~ 0
1V35
Text Label 12600 3450 2    50   ~ 0
GND
Wire Wire Line
	12600 3450 13150 3450
Connection ~ 13150 3450
Wire Wire Line
	13150 3450 13700 3450
Connection ~ 13700 3450
Wire Wire Line
	13700 3450 14250 3450
Connection ~ 14250 3450
Wire Wire Line
	14250 3450 14800 3450
Connection ~ 14800 3450
Wire Wire Line
	14800 3450 15350 3450
Wire Wire Line
	15350 3150 14800 3150
Connection ~ 13150 3150
Wire Wire Line
	13150 3150 12600 3150
Connection ~ 13700 3150
Wire Wire Line
	13700 3150 13150 3150
Connection ~ 14250 3150
Wire Wire Line
	14250 3150 13700 3150
Connection ~ 14800 3150
Wire Wire Line
	14800 3150 14250 3150
$Comp
L device:C C7
U 1 1 61AF9723
P 12600 3800
F 0 "C7" H 12715 3846 50  0000 L CNN
F 1 "0.47 uF" H 12715 3755 50  0000 L CNN
F 2 "" H 12638 3650 50  0001 C CNN
F 3 "" H 12600 3800 50  0001 C CNN
	1    12600 3800
	1    0    0    -1  
$EndComp
$Comp
L device:C C14
U 1 1 61AF972D
P 13150 3800
F 0 "C14" H 13265 3846 50  0000 L CNN
F 1 "0.47 uF" H 13265 3755 50  0000 L CNN
F 2 "" H 13188 3650 50  0001 C CNN
F 3 "" H 13150 3800 50  0001 C CNN
	1    13150 3800
	1    0    0    -1  
$EndComp
$Comp
L device:C C20
U 1 1 61AF9737
P 13700 3800
F 0 "C20" H 13815 3846 50  0000 L CNN
F 1 "0.47 uF" H 13815 3755 50  0000 L CNN
F 2 "" H 13738 3650 50  0001 C CNN
F 3 "" H 13700 3800 50  0001 C CNN
	1    13700 3800
	1    0    0    -1  
$EndComp
$Comp
L device:C C25
U 1 1 61AF9741
P 14250 3800
F 0 "C25" H 14365 3846 50  0000 L CNN
F 1 "0.47 uF" H 14365 3755 50  0000 L CNN
F 2 "" H 14288 3650 50  0001 C CNN
F 3 "" H 14250 3800 50  0001 C CNN
	1    14250 3800
	1    0    0    -1  
$EndComp
$Comp
L device:C C30
U 1 1 61AF974B
P 14800 3800
F 0 "C30" H 14915 3846 50  0000 L CNN
F 1 "0.47 uF" H 14915 3755 50  0000 L CNN
F 2 "" H 14838 3650 50  0001 C CNN
F 3 "" H 14800 3800 50  0001 C CNN
	1    14800 3800
	1    0    0    -1  
$EndComp
$Comp
L device:C C33
U 1 1 61AF9755
P 15350 3800
F 0 "C33" H 15465 3846 50  0000 L CNN
F 1 "0.47 uF" H 15465 3755 50  0000 L CNN
F 2 "" H 15388 3650 50  0001 C CNN
F 3 "" H 15350 3800 50  0001 C CNN
	1    15350 3800
	1    0    0    -1  
$EndComp
Text Label 12600 3650 2    50   ~ 0
1V35
Text Label 12600 3950 2    50   ~ 0
GND
Wire Wire Line
	12600 3950 13150 3950
Connection ~ 13150 3950
Wire Wire Line
	13150 3950 13700 3950
Connection ~ 13700 3950
Wire Wire Line
	13700 3950 14250 3950
Connection ~ 14250 3950
Wire Wire Line
	14250 3950 14800 3950
Connection ~ 14800 3950
Wire Wire Line
	14800 3950 15350 3950
Wire Wire Line
	15350 3650 14800 3650
Connection ~ 13150 3650
Wire Wire Line
	13150 3650 12600 3650
Connection ~ 13700 3650
Wire Wire Line
	13700 3650 13150 3650
Connection ~ 14250 3650
Wire Wire Line
	14250 3650 13700 3650
Connection ~ 14800 3650
Wire Wire Line
	14800 3650 14250 3650
$Comp
L device:C C1
U 1 1 61B00958
P 12600 750
F 0 "C1" H 12715 796 50  0000 L CNN
F 1 "0.47 uF" H 12715 705 50  0000 L CNN
F 2 "" H 12638 600 50  0001 C CNN
F 3 "" H 12600 750 50  0001 C CNN
	1    12600 750 
	1    0    0    -1  
$EndComp
Text Label 12600 600  2    50   ~ 0
VREF
Text Label 12600 900  2    50   ~ 0
GND
$Comp
L device:C C8
U 1 1 61B0231B
P 13150 750
F 0 "C8" H 13265 796 50  0000 L CNN
F 1 "0.47 uF" H 13265 705 50  0000 L CNN
F 2 "" H 13188 600 50  0001 C CNN
F 3 "" H 13150 750 50  0001 C CNN
	1    13150 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	13150 600  12600 600 
Wire Wire Line
	12600 900  13150 900 
Text HLabel 9150 6050 0    50   Input ~ 0
FLASH_SO
Text HLabel 9150 5950 0    50   Output ~ 0
FLASH_SI
Text HLabel 9150 6950 0    50   Output ~ 0
FLASH_CS
$Comp
L device:R R13
U 1 1 61B2F824
P 9400 5950
F 0 "R13" V 9450 6100 50  0000 C CNN
F 1 "33" V 9400 5950 50  0000 C CNN
F 2 "" V 9330 5950 50  0001 C CNN
F 3 "" H 9400 5950 50  0001 C CNN
	1    9400 5950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9150 5950 9250 5950
Wire Wire Line
	9150 6050 9650 6050
Wire Wire Line
	9650 5950 9550 5950
NoConn ~ 9650 5850
NoConn ~ 9650 6150
NoConn ~ 9650 6250
NoConn ~ 9650 6450
NoConn ~ 9650 6550
NoConn ~ 9650 6650
NoConn ~ 9650 6750
NoConn ~ 9650 6850
$Comp
L device:R R15
U 1 1 61B7104D
P 9400 6950
F 0 "R15" V 9450 7100 50  0000 C CNN
F 1 "33" V 9400 6950 50  0000 C CNN
F 2 "" V 9330 6950 50  0001 C CNN
F 3 "" H 9400 6950 50  0001 C CNN
	1    9400 6950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9550 6950 9650 6950
Wire Wire Line
	9250 6950 9150 6950
$Comp
L device:R R14
U 1 1 61B7E37E
P 9400 6350
F 0 "R14" V 9450 6500 50  0000 C CNN
F 1 "1K" V 9400 6350 50  0000 C CNN
F 2 "" V 9330 6350 50  0001 C CNN
F 3 "" H 9400 6350 50  0001 C CNN
	1    9400 6350
	0    -1   -1   0   
$EndComp
Text Label 9150 6350 2    50   ~ 0
1V35
Wire Wire Line
	9150 6350 9250 6350
Wire Wire Line
	9550 6350 9650 6350
Text Label 3400 7050 2    50   ~ 0
VREF
Text Label 3400 9650 2    50   ~ 0
VREF
Text Label 6150 9650 2    50   ~ 0
VREF
Text Label 6150 7050 2    50   ~ 0
VREF
Text Label 9650 7050 2    50   ~ 0
VREF
Text Label 9650 9650 2    50   ~ 0
VREF
Text Label 9650 750  2    50   ~ 0
RAM_DQS0_P
Text Label 9650 850  2    50   ~ 0
RAM_DQS0_N
Text Label 9650 950  2    50   ~ 0
RAM_DM0
Text Label 9650 1050 2    50   ~ 0
RAM_DQ0_0
Text Label 9650 1150 2    50   ~ 0
RAM_DQ0_1
Text Label 9650 1250 2    50   ~ 0
RAM_DQ0_2
Text Label 9650 1350 2    50   ~ 0
RAM_DQ0_3
Text Label 9650 1450 2    50   ~ 0
RAM_DQ0_4
Text Label 9650 1550 2    50   ~ 0
RAM_DQ0_5
Text Label 9650 1650 2    50   ~ 0
RAM_DQ0_6
Text Label 9650 1750 2    50   ~ 0
RAM_DQ0_7
Text Label 9650 1950 2    50   ~ 0
RAM_DQS1_P
Text Label 9650 2050 2    50   ~ 0
RAM_DQS1_N
Text Label 9650 2150 2    50   ~ 0
RAM_DM1
Text Label 9650 2250 2    50   ~ 0
RAM_DQ1_0
Text Label 9650 2350 2    50   ~ 0
RAM_DQ1_1
Text Label 9650 2450 2    50   ~ 0
RAM_DQ1_2
Text Label 9650 2550 2    50   ~ 0
RAM_DQ1_3
Text Label 9650 2650 2    50   ~ 0
RAM_DQ1_4
Text Label 9650 2750 2    50   ~ 0
RAM_DQ1_5
Text Label 9650 2850 2    50   ~ 0
RAM_DQ1_6
Text Label 9650 2950 2    50   ~ 0
RAM_DQ1_7
Text Label 9650 3150 2    50   ~ 0
RAM_DQS2_P
Text Label 9650 3250 2    50   ~ 0
RAM_DQS2_N
Text Label 9650 3350 2    50   ~ 0
RAM_DM2
Text Label 9650 3450 2    50   ~ 0
RAM_DQ2_0
Text Label 9650 3550 2    50   ~ 0
RAM_DQ2_1
Text Label 9650 3650 2    50   ~ 0
RAM_DQ2_2
Text Label 9650 3750 2    50   ~ 0
RAM_DQ2_3
Text Label 9650 3850 2    50   ~ 0
RAM_DQ2_4
Text Label 9650 3950 2    50   ~ 0
RAM_DQ2_5
Text Label 9650 4050 2    50   ~ 0
RAM_DQ2_6
Text Label 9650 4150 2    50   ~ 0
RAM_DQ2_7
Text Label 9650 4450 2    50   ~ 0
RAM_DQS3_N
Text Label 9650 4550 2    50   ~ 0
RAM_DM3
Text Label 9650 4650 2    50   ~ 0
RAM_DQ3_0
Text Label 9650 4750 2    50   ~ 0
RAM_DQ3_1
Text Label 9650 4850 2    50   ~ 0
RAM_DQ3_2
Text Label 9650 4950 2    50   ~ 0
RAM_DQ3_3
Text Label 9650 5050 2    50   ~ 0
RAM_DQ3_4
Text Label 9650 5150 2    50   ~ 0
RAM_DQ3_5
Text Label 9650 5250 2    50   ~ 0
RAM_DQ3_6
Text Label 9650 5350 2    50   ~ 0
RAM_DQ3_7
Text Label 9650 4350 2    50   ~ 0
RAM_DQS3_P
Text Label 11050 750  0    50   ~ 0
RAM_DQS4_P
Text Label 11050 850  0    50   ~ 0
RAM_DQS4_N
Text Label 11050 950  0    50   ~ 0
RAM_DM4
Text Label 11050 1050 0    50   ~ 0
RAM_DQ4_0
Text Label 11050 1150 0    50   ~ 0
RAM_DQ4_1
Text Label 11050 1250 0    50   ~ 0
RAM_DQ4_2
Text Label 11050 1350 0    50   ~ 0
RAM_DQ4_3
Text Label 11050 1450 0    50   ~ 0
RAM_DQ4_4
Text Label 11050 1550 0    50   ~ 0
RAM_DQ4_5
Text Label 11050 1650 0    50   ~ 0
RAM_DQ4_6
Text Label 11050 1750 0    50   ~ 0
RAM_DQ4_7
Text Label 11050 1950 0    50   ~ 0
RAM_DQS5_P
Text Label 11050 2050 0    50   ~ 0
RAM_DQS5_N
Text Label 11050 2150 0    50   ~ 0
RAM_DM5
Text Label 11050 2250 0    50   ~ 0
RAM_DQ5_0
Text Label 11050 2350 0    50   ~ 0
RAM_DQ5_1
Text Label 11050 2450 0    50   ~ 0
RAM_DQ5_2
Text Label 11050 2550 0    50   ~ 0
RAM_DQ5_3
Text Label 11050 2650 0    50   ~ 0
RAM_DQ5_4
Text Label 11050 2750 0    50   ~ 0
RAM_DQ5_5
Text Label 11050 2850 0    50   ~ 0
RAM_DQ5_6
Text Label 11050 2950 0    50   ~ 0
RAM_DQ5_7
Text Label 11050 3150 0    50   ~ 0
RAM_DQS6_P
Text Label 11050 3250 0    50   ~ 0
RAM_DQS6_N
Text Label 11050 3350 0    50   ~ 0
RAM_DM6
Text Label 11050 3450 0    50   ~ 0
RAM_DQ6_0
Text Label 11050 3550 0    50   ~ 0
RAM_DQ6_1
Text Label 11050 3650 0    50   ~ 0
RAM_DQ6_2
Text Label 11050 3750 0    50   ~ 0
RAM_DQ6_3
Text Label 11050 3850 0    50   ~ 0
RAM_DQ6_4
Text Label 11050 3950 0    50   ~ 0
RAM_DQ6_5
Text Label 11050 4050 0    50   ~ 0
RAM_DQ6_6
Text Label 11050 4150 0    50   ~ 0
RAM_DQ6_7
Text Label 11050 4350 0    50   ~ 0
RAM_DQS7_P
Text Label 11050 4450 0    50   ~ 0
RAM_DQS7_N
Text Label 11050 4550 0    50   ~ 0
RAM_DM7
Text Label 11050 4650 0    50   ~ 0
RAM_DQ7_0
Text Label 11050 4750 0    50   ~ 0
RAM_DQ7_1
Text Label 11050 4850 0    50   ~ 0
RAM_DQ7_2
Text Label 11050 4950 0    50   ~ 0
RAM_DQ7_3
Text Label 11050 5050 0    50   ~ 0
RAM_DQ7_4
Text Label 11050 5150 0    50   ~ 0
RAM_DQ7_5
Text Label 11050 5250 0    50   ~ 0
RAM_DQ7_6
Text Label 11050 5350 0    50   ~ 0
RAM_DQ7_7
Text Label 3400 6350 2    50   ~ 0
RAM_DQS0_P
Text Label 3400 6450 2    50   ~ 0
RAM_DQS0_N
Text Label 3400 5950 2    50   ~ 0
RAM_DM0
Text Label 3400 6050 2    50   ~ 0
RAM_DQ0_0
Text Label 3400 6150 2    50   ~ 0
RAM_DQ0_1
Text Label 3400 6250 2    50   ~ 0
RAM_DQ0_2
Text Label 3400 6550 2    50   ~ 0
RAM_DQ0_3
Text Label 3400 6650 2    50   ~ 0
RAM_DQ0_4
Text Label 3400 6750 2    50   ~ 0
RAM_DQ0_5
Text Label 3400 6850 2    50   ~ 0
RAM_DQ0_6
Text Label 3400 6950 2    50   ~ 0
RAM_DQ0_7
Text Label 3400 7550 2    50   ~ 0
RAM_DQS1_P
Text Label 3400 7650 2    50   ~ 0
RAM_DQS1_N
Text Label 3400 7150 2    50   ~ 0
RAM_DM1
Text Label 3400 7250 2    50   ~ 0
RAM_DQ1_0
Text Label 3400 7350 2    50   ~ 0
RAM_DQ1_1
Text Label 3400 7450 2    50   ~ 0
RAM_DQ1_2
Text Label 3400 7750 2    50   ~ 0
RAM_DQ1_3
Text Label 3400 7850 2    50   ~ 0
RAM_DQ1_4
Text Label 3400 7950 2    50   ~ 0
RAM_DQ1_5
Text Label 3400 8050 2    50   ~ 0
RAM_DQ1_6
Text Label 3400 8150 2    50   ~ 0
RAM_DQ1_7
Text Label 3400 8750 2    50   ~ 0
RAM_DQS2_P
Text Label 3400 8850 2    50   ~ 0
RAM_DQS2_N
Text Label 3400 8350 2    50   ~ 0
RAM_DM2
Text Label 3400 8450 2    50   ~ 0
RAM_DQ2_0
Text Label 3400 8550 2    50   ~ 0
RAM_DQ2_1
Text Label 3400 8650 2    50   ~ 0
RAM_DQ2_2
Text Label 3400 8950 2    50   ~ 0
RAM_DQ2_3
Text Label 3400 9050 2    50   ~ 0
RAM_DQ2_4
Text Label 3400 9150 2    50   ~ 0
RAM_DQ2_5
Text Label 3400 9250 2    50   ~ 0
RAM_DQ2_6
Text Label 3400 9350 2    50   ~ 0
RAM_DQ2_7
Text Label 3400 10050 2    50   ~ 0
RAM_DQS3_N
Text Label 3400 9550 2    50   ~ 0
RAM_DM3
Text Label 3400 9750 2    50   ~ 0
RAM_DQ3_0
Text Label 3400 9850 2    50   ~ 0
RAM_DQ3_1
Text Label 3400 10150 2    50   ~ 0
RAM_DQ3_2
Text Label 3400 10250 2    50   ~ 0
RAM_DQ3_3
Text Label 3400 10350 2    50   ~ 0
RAM_DQ3_4
Text Label 3400 10450 2    50   ~ 0
RAM_DQ3_5
Text Label 3400 10550 2    50   ~ 0
RAM_DQ3_6
Text Label 3400 10650 2    50   ~ 0
RAM_DQ3_7
Text Label 3400 9950 2    50   ~ 0
RAM_DQS3_P
Text Label 6150 6350 2    50   ~ 0
RAM_DQS4_P
Text Label 6150 6450 2    50   ~ 0
RAM_DQS4_N
Text Label 6150 5950 2    50   ~ 0
RAM_DM4
Text Label 6150 6050 2    50   ~ 0
RAM_DQ4_0
Text Label 6150 6150 2    50   ~ 0
RAM_DQ4_1
Text Label 6150 6250 2    50   ~ 0
RAM_DQ4_2
Text Label 6150 6550 2    50   ~ 0
RAM_DQ4_3
Text Label 6150 6650 2    50   ~ 0
RAM_DQ4_4
Text Label 6150 6750 2    50   ~ 0
RAM_DQ4_5
Text Label 6150 6850 2    50   ~ 0
RAM_DQ4_6
Text Label 6150 6950 2    50   ~ 0
RAM_DQ4_7
Text Label 6150 7550 2    50   ~ 0
RAM_DQS5_P
Text Label 6150 7650 2    50   ~ 0
RAM_DQS5_N
Text Label 6150 7150 2    50   ~ 0
RAM_DM5
Text Label 6150 7250 2    50   ~ 0
RAM_DQ5_0
Text Label 6150 7350 2    50   ~ 0
RAM_DQ5_1
Text Label 6150 7450 2    50   ~ 0
RAM_DQ5_2
Text Label 6150 7750 2    50   ~ 0
RAM_DQ5_3
Text Label 6150 7850 2    50   ~ 0
RAM_DQ5_4
Text Label 6150 7950 2    50   ~ 0
RAM_DQ5_5
Text Label 6150 8050 2    50   ~ 0
RAM_DQ5_6
Text Label 6150 8150 2    50   ~ 0
RAM_DQ5_7
Text Label 6150 8750 2    50   ~ 0
RAM_DQS6_P
Text Label 6150 8850 2    50   ~ 0
RAM_DQS6_N
Text Label 6150 8350 2    50   ~ 0
RAM_DM6
Text Label 6150 8450 2    50   ~ 0
RAM_DQ6_0
Text Label 6150 8550 2    50   ~ 0
RAM_DQ6_1
Text Label 6150 8650 2    50   ~ 0
RAM_DQ6_2
Text Label 6150 8950 2    50   ~ 0
RAM_DQ6_3
Text Label 6150 9050 2    50   ~ 0
RAM_DQ6_4
Text Label 6150 9150 2    50   ~ 0
RAM_DQ6_5
Text Label 6150 9250 2    50   ~ 0
RAM_DQ6_6
Text Label 6150 9350 2    50   ~ 0
RAM_DQ6_7
Text Label 6150 9950 2    50   ~ 0
RAM_DQS7_P
Text Label 6150 10050 2    50   ~ 0
RAM_DQS7_N
Text Label 6150 9550 2    50   ~ 0
RAM_DM7
Text Label 6150 9750 2    50   ~ 0
RAM_DQ7_0
Text Label 6150 9850 2    50   ~ 0
RAM_DQ7_1
Text Label 6150 10150 2    50   ~ 0
RAM_DQ7_2
Text Label 6150 10250 2    50   ~ 0
RAM_DQ7_3
Text Label 6150 10350 2    50   ~ 0
RAM_DQ7_4
Text Label 6150 10450 2    50   ~ 0
RAM_DQ7_5
Text Label 6150 10550 2    50   ~ 0
RAM_DQ7_6
Text Label 6150 10650 2    50   ~ 0
RAM_DQ7_7
NoConn ~ 3400 5850
NoConn ~ 3400 8250
NoConn ~ 3400 9450
NoConn ~ 3400 10750
NoConn ~ 6150 5850
NoConn ~ 6150 8250
NoConn ~ 6150 9450
NoConn ~ 6150 10750
Text Label 5350 1750 0    50   ~ 0
RAM_BA0
Text Label 5350 1850 0    50   ~ 0
RAM_BA1
Text Label 5350 1950 0    50   ~ 0
RAM_BA2
Text Label 5350 2150 0    50   ~ 0
RAM_A0
Text Label 5350 2250 0    50   ~ 0
RAM_A1
Text Label 5350 2350 0    50   ~ 0
RAM_A2
Text Label 5350 2450 0    50   ~ 0
RAM_A3
Text Label 5350 2550 0    50   ~ 0
RAM_A4
Text Label 5350 2650 0    50   ~ 0
RAM_A5
Text Label 5350 2750 0    50   ~ 0
RAM_A6
Text Label 5350 2850 0    50   ~ 0
RAM_A7
Text Label 5350 2950 0    50   ~ 0
RAM_A8
Text Label 5350 3050 0    50   ~ 0
RAM_A9
Text Label 5350 3150 0    50   ~ 0
RAM_A10
Text Label 5350 3250 0    50   ~ 0
RAM_A11
Text Label 5350 3350 0    50   ~ 0
RAM_A12
Text Label 5350 3450 0    50   ~ 0
RAM_A13
Text Label 5350 3550 0    50   ~ 0
RAM_A14
Text Label 5350 3650 0    50   ~ 0
RAM_A15
Text Label 9650 7150 2    50   ~ 0
RAM_A0
Text Label 9650 7250 2    50   ~ 0
RAM_A1
Text Label 9650 7350 2    50   ~ 0
RAM_A2
Text Label 9650 7450 2    50   ~ 0
RAM_A3
Text Label 9650 7550 2    50   ~ 0
RAM_A4
Text Label 9650 7650 2    50   ~ 0
RAM_A5
Text Label 9650 7750 2    50   ~ 0
RAM_A6
Text Label 9650 7850 2    50   ~ 0
RAM_A7
Text Label 9650 7950 2    50   ~ 0
RAM_A8
Text Label 9650 8050 2    50   ~ 0
RAM_A9
Text Label 9650 8150 2    50   ~ 0
RAM_A10
Text Label 9650 8250 2    50   ~ 0
RAM_A11
Text Label 9650 8350 2    50   ~ 0
RAM_A12
Text Label 9650 8450 2    50   ~ 0
RAM_A13
Text Label 9650 8550 2    50   ~ 0
RAM_A14
Text Label 9650 8650 2    50   ~ 0
RAM_A15
Text Label 9650 8950 2    50   ~ 0
RAM_BA0
Text Label 9650 8850 2    50   ~ 0
RAM_BA1
Text Label 9650 8750 2    50   ~ 0
RAM_BA2
Text HLabel 3950 1250 0    50   BiDi ~ 0
RAM_SDA
Text HLabel 3950 1350 0    50   Input ~ 0
RAM_SCL
Text Label 3950 1450 2    50   ~ 0
GND
Text Label 3950 1550 2    50   ~ 0
GND
Text Notes 4150 3800 0    50   ~ 0
SPD EEPROM strap to 2'b00
$Comp
L device:R R11
U 1 1 61CAE11F
P 4300 800
F 0 "R11" V 4350 650 50  0000 C CNN
F 1 "DNP" V 4300 800 50  0000 C CNN
F 2 "" V 4230 800 50  0001 C CNN
F 3 "" H 4300 800 50  0001 C CNN
	1    4300 800 
	0    -1   -1   0   
$EndComp
Text Label 4150 800  2    50   ~ 0
1V35
$Comp
L device:R R12
U 1 1 61CAFEFC
P 4300 900
F 0 "R12" V 4350 750 50  0000 C CNN
F 1 "DNP" V 4300 900 50  0000 C CNN
F 2 "" V 4230 900 50  0001 C CNN
F 3 "" H 4300 900 50  0001 C CNN
	1    4300 900 
	0    -1   -1   0   
$EndComp
Text Label 4150 900  2    50   ~ 0
GND
Text Label 4600 800  0    50   ~ 0
RAM_TEST
Wire Wire Line
	4600 800  4550 800 
Wire Wire Line
	4450 900  4550 900 
Wire Wire Line
	4550 900  4550 800 
Connection ~ 4550 800 
Wire Wire Line
	4550 800  4450 800 
Text Label 3950 1750 2    50   ~ 0
RAM_TEST
Text Label 3950 1850 2    50   ~ 0
RAM_RESET_N
Text Label 9650 9050 2    50   ~ 0
RAM_RESET_N
Text Label 3950 2050 2    50   ~ 0
RAM_CKE0
Text Label 3950 2150 2    50   ~ 0
RAM_CK0_P
Text Label 3950 2250 2    50   ~ 0
RAM_CK0_N
Text Label 3950 2450 2    50   ~ 0
RAM_CKE1
Text Label 3950 2550 2    50   ~ 0
RAM_CK1_P
Text Label 3950 2650 2    50   ~ 0
RAM_CK1_N
Text Label 3950 2850 2    50   ~ 0
RAM_CS0_N
Text Label 3950 2950 2    50   ~ 0
RAM_CS1_N
Text Label 3950 3150 2    50   ~ 0
RAM_ODT0
Text Label 3950 3250 2    50   ~ 0
RAM_ODT1
Text Label 3950 3450 2    50   ~ 0
RAM_WE_N
Text Label 3950 3550 2    50   ~ 0
RAM_CAS_N
Text Label 3950 3650 2    50   ~ 0
RAM_RAS_N
Text Label 9650 9150 2    50   ~ 0
RAM_CKE0
Text Label 9650 9250 2    50   ~ 0
RAM_CK0_P
Text Label 9650 9350 2    50   ~ 0
RAM_CK0_N
Text Label 9650 9750 2    50   ~ 0
RAM_CKE1
Text Label 9650 9450 2    50   ~ 0
RAM_CK1_P
Text Label 9650 9550 2    50   ~ 0
RAM_CK1_N
Text Label 9650 9850 2    50   ~ 0
RAM_CS0_N
Text Label 9650 9950 2    50   ~ 0
RAM_CS1_N
Text Label 9650 10050 2    50   ~ 0
RAM_ODT0
Text Label 9650 10150 2    50   ~ 0
RAM_ODT1
Text Label 9650 10250 2    50   ~ 0
RAM_WE_N
Text Label 9650 10350 2    50   ~ 0
RAM_CAS_N
Text Label 9650 10450 2    50   ~ 0
RAM_RAS_N
NoConn ~ 9650 10550
NoConn ~ 9650 10650
NoConn ~ 9650 10750
Text Label 5800 800  2    50   ~ 0
RAM_RESET_N
$Comp
L device:R R64
U 1 1 61BAED6C
P 5950 800
F 0 "R64" V 6000 650 50  0000 C CNN
F 1 "4.7K" V 5950 800 50  0000 C CNN
F 2 "" V 5880 800 50  0001 C CNN
F 3 "" H 5950 800 50  0001 C CNN
	1    5950 800 
	0    -1   -1   0   
$EndComp
Text Label 6300 800  0    50   ~ 0
GND
Wire Wire Line
	6300 800  6100 800 
$Comp
L Connector:Conn_01x01 TP18
U 1 1 61D08689
P 12800 4850
F 0 "TP18" H 12718 5067 50  0000 C CNN
F 1 "TESTTRACE" H 12718 4976 50  0000 C CNN
F 2 "" H 12800 4850 50  0001 C CNN
F 3 "~" H 12800 4850 50  0001 C CNN
	1    12800 4850
	-1   0    0    -1  
$EndComp
$Comp
L device:R R67
U 1 1 61D0A248
P 14150 4700
F 0 "R67" H 14220 4746 50  0000 L CNN
F 1 "1K" H 14220 4655 50  0000 L CNN
F 2 "" V 14080 4700 50  0001 C CNN
F 3 "" H 14150 4700 50  0001 C CNN
	1    14150 4700
	1    0    0    -1  
$EndComp
$Comp
L device:R R68
U 1 1 61D0A5DA
P 14150 5000
F 0 "R68" H 14220 5046 50  0000 L CNN
F 1 "1K" H 14220 4955 50  0000 L CNN
F 2 "" V 14080 5000 50  0001 C CNN
F 3 "" H 14150 5000 50  0001 C CNN
	1    14150 5000
	1    0    0    -1  
$EndComp
Text Label 14350 4850 0    50   ~ 0
TP_VREF
Connection ~ 14150 4850
Text Label 14150 4550 0    50   ~ 0
1V35
Text Label 14350 5150 0    50   ~ 0
GND
$Comp
L device:C C143
U 1 1 61D2942F
P 14750 5000
F 0 "C143" H 14865 5046 50  0000 L CNN
F 1 "4.7 uF" H 14865 4955 50  0000 L CNN
F 2 "" H 14788 4850 50  0001 C CNN
F 3 "" H 14750 5000 50  0001 C CNN
	1    14750 5000
	1    0    0    -1  
$EndComp
$Comp
L device:C C144
U 1 1 61D29D43
P 15250 5000
F 0 "C144" H 15365 5046 50  0000 L CNN
F 1 "0.47 uF" H 15365 4955 50  0000 L CNN
F 2 "" H 15288 4850 50  0001 C CNN
F 3 "" H 15250 5000 50  0001 C CNN
	1    15250 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	14150 4850 14750 4850
Connection ~ 14750 4850
Wire Wire Line
	14750 4850 15250 4850
Wire Wire Line
	14150 5150 14750 5150
Connection ~ 14750 5150
Wire Wire Line
	14750 5150 15250 5150
Text Label 13000 4850 0    50   ~ 0
TP_VREF
$Comp
L Connector:Conn_01x01 TP19
U 1 1 61D3E579
P 12800 5200
F 0 "TP19" H 12718 5417 50  0000 C CNN
F 1 "TESTTRACE" H 12718 5326 50  0000 C CNN
F 2 "" H 12800 5200 50  0001 C CNN
F 3 "~" H 12800 5200 50  0001 C CNN
	1    12800 5200
	-1   0    0    -1  
$EndComp
Text Label 13000 5200 0    50   ~ 0
TP_VREF
$Comp
L Connector:Conn_01x01 TP20
U 1 1 61D450D1
P 12800 5550
F 0 "TP20" H 12718 5767 50  0000 C CNN
F 1 "TESTTRACE" H 12718 5676 50  0000 C CNN
F 2 "" H 12800 5550 50  0001 C CNN
F 3 "~" H 12800 5550 50  0001 C CNN
	1    12800 5550
	-1   0    0    -1  
$EndComp
Text Label 13000 5550 0    50   ~ 0
TP_VREF
$Comp
L Connector:Conn_01x01 TP21
U 1 1 61D4BA99
P 12800 5950
F 0 "TP21" H 12718 6167 50  0000 C CNN
F 1 "TESTTRACE" H 12718 6076 50  0000 C CNN
F 2 "" H 12800 5950 50  0001 C CNN
F 3 "~" H 12800 5950 50  0001 C CNN
	1    12800 5950
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01 TP22
U 1 1 61D4CED3
P 12800 6300
F 0 "TP22" H 12718 6517 50  0000 C CNN
F 1 "TESTTRACE" H 12718 6426 50  0000 C CNN
F 2 "" H 12800 6300 50  0001 C CNN
F 3 "~" H 12800 6300 50  0001 C CNN
	1    12800 6300
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01 TP23
U 1 1 61D4CEDD
P 12800 6650
F 0 "TP23" H 12718 6867 50  0000 C CNN
F 1 "TESTTRACE" H 12718 6776 50  0000 C CNN
F 2 "" H 12800 6650 50  0001 C CNN
F 3 "~" H 12800 6650 50  0001 C CNN
	1    12800 6650
	-1   0    0    -1  
$EndComp
Text Label 13000 5950 0    50   ~ 0
GND
Text Label 13000 6300 0    50   ~ 0
GND
Text Label 13000 6650 0    50   ~ 0
GND
Text Notes 14450 6550 0    50   ~ 0
We want test points on:\n* CLK x2\n* RAS\n* CAS\n* All BA??\n* A10\n* A12\n* WE\n* CS\n* A few random DQ pins\n* DQS for those DQ
Text Notes 13350 7300 0    50   ~ 0
CLK, DQS - diff, no ref needed\nDQ - two ref to each\nCommand bus: ground
$EndSCHEMATC
