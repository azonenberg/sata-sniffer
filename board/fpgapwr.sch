EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 5 10
Title "SATA sniffer"
Date "2021-11-11"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U?
U 10 1 6191887B
P 1850 6100
AR Path="/618C589A/6191887B" Ref="U?"  Part="3" 
AR Path="/61909AE6/6191887B" Ref="U?"  Part="6" 
AR Path="/618F2BCB/6191887B" Ref="U5"  Part="10" 
F 0 "U5" H 1850 6000 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 1850 5900 60  0000 L CNN
F 2 "" H 1850 5800 60  0000 C CNN
F 3 "" H 1850 5800 60  0000 C CNN
	10   1850 6100
	1    0    0    -1  
$EndComp
$Comp
L xilinx-azonenberg:XC7KxT-FBG484 U?
U 2 1 61921EEF
P 5000 6000
AR Path="/618C589A/61921EEF" Ref="U?"  Part="3" 
AR Path="/61909AE6/61921EEF" Ref="U?"  Part="6" 
AR Path="/618F2BCB/61921EEF" Ref="U5"  Part="2" 
F 0 "U5" H 5000 5900 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 5000 5800 60  0000 L CNN
F 2 "" H 5000 5700 60  0000 C CNN
F 3 "" H 5000 5700 60  0000 C CNN
	2    5000 6000
	1    0    0    -1  
$EndComp
Text HLabel 1650 1200 0    50   Input ~ 0
GND
Text HLabel 4800 1100 0    50   Input ~ 0
1V0
Text HLabel 10600 6450 0    50   Input ~ 0
1V2
Text HLabel 4800 4000 0    50   Input ~ 0
1V8
Text HLabel 6650 1450 2    50   Input ~ 0
1V35
Text HLabel 6650 3550 2    50   Input ~ 0
3V3
Wire Wire Line
	1650 1200 1650 1400
Connection ~ 1650 1400
Wire Wire Line
	1650 1400 1650 1500
Connection ~ 1650 1500
Wire Wire Line
	1650 1500 1650 1600
Connection ~ 1650 1600
Wire Wire Line
	1650 1600 1650 1700
Connection ~ 1650 1700
Wire Wire Line
	1650 1700 1650 1800
Connection ~ 1650 1800
Wire Wire Line
	1650 1800 1650 1900
Connection ~ 1650 1900
Wire Wire Line
	1650 1900 1650 2000
Connection ~ 1650 2000
Wire Wire Line
	1650 2000 1650 2100
Connection ~ 1650 2100
Wire Wire Line
	1650 2100 1650 2200
Connection ~ 1650 2200
Wire Wire Line
	1650 2200 1650 2300
Connection ~ 1650 2300
Wire Wire Line
	1650 2300 1650 2400
Connection ~ 1650 2400
Wire Wire Line
	1650 2400 1650 2500
Connection ~ 1650 2500
Wire Wire Line
	1650 2500 1650 2600
Connection ~ 1650 2600
Wire Wire Line
	1650 2600 1650 2700
Connection ~ 1650 2700
Wire Wire Line
	1650 2700 1650 2800
Connection ~ 1650 2800
Wire Wire Line
	1650 2800 1650 2900
Connection ~ 1650 2900
Wire Wire Line
	1650 2900 1650 3000
Connection ~ 1650 3000
Wire Wire Line
	1650 3000 1650 3100
Connection ~ 1650 3100
Wire Wire Line
	1650 3100 1650 3200
Connection ~ 1650 3200
Wire Wire Line
	1650 3200 1650 3300
Connection ~ 1650 3300
Wire Wire Line
	1650 3300 1650 3400
Connection ~ 1650 3400
Wire Wire Line
	1650 3400 1650 3500
Connection ~ 1650 3500
Wire Wire Line
	1650 3500 1650 3600
Connection ~ 1650 3600
Wire Wire Line
	1650 3600 1650 3700
Connection ~ 1650 3700
Wire Wire Line
	1650 3700 1650 3800
Connection ~ 1650 3800
Wire Wire Line
	1650 3800 1650 3900
Connection ~ 1650 3900
Wire Wire Line
	1650 3900 1650 4000
Connection ~ 1650 4000
Wire Wire Line
	1650 4000 1650 4100
Connection ~ 1650 4100
Wire Wire Line
	1650 4100 1650 4200
Connection ~ 1650 4200
Wire Wire Line
	1650 4200 1650 4300
Connection ~ 1650 4300
Wire Wire Line
	1650 4300 1650 4400
Connection ~ 1650 4400
Wire Wire Line
	1650 4400 1650 4500
Connection ~ 1650 4500
Wire Wire Line
	1650 4500 1650 4600
Connection ~ 1650 4600
Wire Wire Line
	1650 4600 1650 4700
Connection ~ 1650 4700
Wire Wire Line
	1650 4700 1650 4800
Connection ~ 1650 4800
Wire Wire Line
	1650 4800 1650 4900
Connection ~ 1650 4900
Wire Wire Line
	1650 4900 1650 5000
Connection ~ 1650 5000
Wire Wire Line
	1650 5000 1650 5100
Connection ~ 1650 5100
Wire Wire Line
	1650 5100 1650 5200
Connection ~ 1650 5200
Wire Wire Line
	1650 5200 1650 5300
Connection ~ 1650 5300
Wire Wire Line
	1650 5300 1650 5400
Connection ~ 1650 5400
Wire Wire Line
	1650 5400 1650 5500
Connection ~ 1650 5500
Wire Wire Line
	1650 5500 1650 5600
Connection ~ 1650 5600
Wire Wire Line
	1650 5600 1650 5700
Connection ~ 1650 5700
Wire Wire Line
	1650 5700 1650 5800
Connection ~ 1650 5800
Wire Wire Line
	1650 5800 1650 5900
Connection ~ 1650 5900
Wire Wire Line
	1650 5900 1650 6000
Connection ~ 1650 6000
Wire Wire Line
	1650 6000 1650 6100
Wire Wire Line
	3500 5000 3500 4900
Connection ~ 3500 1300
Wire Wire Line
	3500 1300 3500 1200
Connection ~ 3500 1400
Wire Wire Line
	3500 1400 3500 1300
Connection ~ 3500 1500
Wire Wire Line
	3500 1500 3500 1400
Connection ~ 3500 1600
Wire Wire Line
	3500 1600 3500 1500
Connection ~ 3500 1700
Wire Wire Line
	3500 1700 3500 1600
Connection ~ 3500 1800
Wire Wire Line
	3500 1800 3500 1700
Connection ~ 3500 1900
Wire Wire Line
	3500 1900 3500 1800
Connection ~ 3500 2000
Wire Wire Line
	3500 2000 3500 1900
Connection ~ 3500 2100
Wire Wire Line
	3500 2100 3500 2000
Connection ~ 3500 2200
Wire Wire Line
	3500 2200 3500 2100
Connection ~ 3500 2300
Wire Wire Line
	3500 2300 3500 2200
Connection ~ 3500 2400
Wire Wire Line
	3500 2400 3500 2300
Connection ~ 3500 2500
Wire Wire Line
	3500 2500 3500 2400
Connection ~ 3500 2600
Wire Wire Line
	3500 2600 3500 2500
Connection ~ 3500 2700
Wire Wire Line
	3500 2700 3500 2600
Connection ~ 3500 2800
Wire Wire Line
	3500 2800 3500 2700
Connection ~ 3500 2900
Wire Wire Line
	3500 2900 3500 2800
Connection ~ 3500 3000
Wire Wire Line
	3500 3000 3500 2900
Connection ~ 3500 3100
Wire Wire Line
	3500 3100 3500 3000
Connection ~ 3500 3200
Wire Wire Line
	3500 3200 3500 3100
Connection ~ 3500 3300
Wire Wire Line
	3500 3300 3500 3200
Connection ~ 3500 3400
Wire Wire Line
	3500 3400 3500 3300
Connection ~ 3500 3500
Wire Wire Line
	3500 3500 3500 3400
Connection ~ 3500 3600
Wire Wire Line
	3500 3600 3500 3500
Connection ~ 3500 3700
Wire Wire Line
	3500 3700 3500 3600
Connection ~ 3500 3800
Wire Wire Line
	3500 3800 3500 3700
Connection ~ 3500 3900
Wire Wire Line
	3500 3900 3500 3800
Connection ~ 3500 4000
Wire Wire Line
	3500 4000 3500 3900
Connection ~ 3500 4100
Wire Wire Line
	3500 4100 3500 4000
Connection ~ 3500 4200
Wire Wire Line
	3500 4200 3500 4100
Connection ~ 3500 4300
Wire Wire Line
	3500 4300 3500 4200
Connection ~ 3500 4400
Wire Wire Line
	3500 4400 3500 4300
Connection ~ 3500 4500
Wire Wire Line
	3500 4500 3500 4400
Connection ~ 3500 4600
Wire Wire Line
	3500 4600 3500 4500
Connection ~ 3500 4700
Wire Wire Line
	3500 4700 3500 4600
Connection ~ 3500 4800
Wire Wire Line
	3500 4800 3500 4700
Connection ~ 3500 4900
Wire Wire Line
	3500 4900 3500 4800
Text Label 3500 1200 0    50   ~ 0
GND
Wire Wire Line
	4800 1100 4800 1200
Connection ~ 4800 1200
Wire Wire Line
	4800 1200 4800 1300
Connection ~ 4800 1300
Wire Wire Line
	4800 1300 4800 1400
Connection ~ 4800 1400
Wire Wire Line
	4800 1400 4800 1500
Connection ~ 4800 1500
Wire Wire Line
	4800 1500 4800 1600
Connection ~ 4800 1600
Wire Wire Line
	4800 1600 4800 1700
Connection ~ 4800 1700
Wire Wire Line
	4800 1700 4800 1800
Connection ~ 4800 1800
Wire Wire Line
	4800 1800 4800 1900
Connection ~ 4800 1900
Wire Wire Line
	4800 1900 4800 2000
Connection ~ 4800 2000
Wire Wire Line
	4800 2000 4800 2100
Connection ~ 4800 2100
Wire Wire Line
	4800 2100 4800 2200
Connection ~ 4800 2200
Wire Wire Line
	4800 2200 4800 2300
Connection ~ 4800 2300
Wire Wire Line
	4800 2300 4800 2400
Connection ~ 4800 2400
Wire Wire Line
	4800 2400 4800 2600
Connection ~ 4800 2600
Wire Wire Line
	4800 2600 4800 2700
Connection ~ 4800 2700
Wire Wire Line
	4800 2700 4800 2800
Text Label 4800 3000 2    50   ~ 0
GTX_1V0
Wire Wire Line
	4800 3000 4800 3100
Connection ~ 4800 3100
Wire Wire Line
	4800 3100 4800 3200
Connection ~ 4800 3200
Wire Wire Line
	4800 3200 4800 3300
Text Label 4800 3500 2    50   ~ 0
GTX_1V2
Wire Wire Line
	4800 3500 4800 3600
Connection ~ 4800 3600
Wire Wire Line
	4800 3600 4800 3700
Connection ~ 4800 3700
Wire Wire Line
	4800 3700 4800 3800
Wire Wire Line
	4800 4000 4800 4100
Connection ~ 4800 4100
Wire Wire Line
	4800 4100 4800 4200
Connection ~ 4800 4200
Wire Wire Line
	4800 4200 4800 4300
Connection ~ 4800 4300
Wire Wire Line
	4800 4300 4800 4400
Connection ~ 4800 4400
Wire Wire Line
	4800 4400 4800 4500
Connection ~ 4800 4500
Wire Wire Line
	4800 4500 4800 4600
Connection ~ 4800 4600
Wire Wire Line
	4800 4600 4800 4800
Text Label 4800 5000 2    50   ~ 0
GTX_1V8
Text Label 4800 5150 2    50   ~ 0
1V8
Text Label 1850 7000 2    50   ~ 0
1V0
Text Label 1850 7300 2    50   ~ 0
GND
$Comp
L device:C C51
U 1 1 6195A054
P 2700 7150
F 0 "C51" H 2815 7196 50  0000 L CNN
F 1 "100 uF" H 2815 7105 50  0000 L CNN
F 2 "" H 2738 7000 50  0001 C CNN
F 3 "" H 2700 7150 50  0001 C CNN
	1    2700 7150
	1    0    0    -1  
$EndComp
Text Notes 1850 7400 0    50   ~ 0
VCCINT
Text Notes 2700 7400 0    50   ~ 0
VCCBRAM
Wire Wire Line
	1850 7300 2700 7300
Wire Wire Line
	1850 7000 2700 7000
$Comp
L device:C C55
U 1 1 61965A4D
P 3250 7150
F 0 "C55" H 3365 7196 50  0000 L CNN
F 1 "4.7 uF" H 3365 7105 50  0000 L CNN
F 2 "" H 3288 7000 50  0001 C CNN
F 3 "" H 3250 7150 50  0001 C CNN
	1    3250 7150
	1    0    0    -1  
$EndComp
$Comp
L device:C C57
U 1 1 61965F5E
P 3750 7150
F 0 "C57" H 3865 7196 50  0000 L CNN
F 1 "4.7 uF" H 3865 7105 50  0000 L CNN
F 2 "" H 3788 7000 50  0001 C CNN
F 3 "" H 3750 7150 50  0001 C CNN
	1    3750 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 7000 3250 7000
Connection ~ 2700 7000
Connection ~ 3250 7000
Wire Wire Line
	3250 7000 2700 7000
Wire Wire Line
	2700 7300 3250 7300
Connection ~ 2700 7300
Connection ~ 3250 7300
Wire Wire Line
	3250 7300 3750 7300
$Comp
L device:C C46
U 1 1 619579CB
P 1850 7150
F 0 "C46" H 1965 7196 50  0000 L CNN
F 1 "330 uF" H 1965 7105 50  0000 L CNN
F 2 "" H 1888 7000 50  0001 C CNN
F 3 "" H 1850 7150 50  0001 C CNN
	1    1850 7150
	1    0    0    -1  
$EndComp
$Comp
L device:C C47
U 1 1 61977FBC
P 1850 7800
F 0 "C47" H 1965 7846 50  0000 L CNN
F 1 "47 uF" H 1965 7755 50  0000 L CNN
F 2 "" H 1888 7650 50  0001 C CNN
F 3 "" H 1850 7800 50  0001 C CNN
	1    1850 7800
	1    0    0    -1  
$EndComp
Text Label 1850 7650 2    50   ~ 0
1V8
Text Label 1850 7950 2    50   ~ 0
GND
$Comp
L device:C C50
U 1 1 619794B8
P 2300 7800
F 0 "C50" H 2415 7846 50  0000 L CNN
F 1 "47 uF" H 2415 7755 50  0000 L CNN
F 2 "" H 2338 7650 50  0001 C CNN
F 3 "" H 2300 7800 50  0001 C CNN
	1    2300 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 7650 1850 7650
Wire Wire Line
	1850 7950 2300 7950
$Comp
L device:C C52
U 1 1 61985B0B
P 2750 7800
F 0 "C52" H 2865 7846 50  0000 L CNN
F 1 "4.7 uF" H 2865 7755 50  0000 L CNN
F 2 "" H 2788 7650 50  0001 C CNN
F 3 "" H 2750 7800 50  0001 C CNN
	1    2750 7800
	1    0    0    -1  
$EndComp
Text Notes 1850 8050 0    50   ~ 0
VCCAUX
$Comp
L device:C C56
U 1 1 61985F05
P 3250 7800
F 0 "C56" H 3365 7846 50  0000 L CNN
F 1 "4.7 uF" H 3365 7755 50  0000 L CNN
F 2 "" H 3288 7650 50  0001 C CNN
F 3 "" H 3250 7800 50  0001 C CNN
	1    3250 7800
	1    0    0    -1  
$EndComp
$Comp
L device:C C58
U 1 1 6198629C
P 3750 7800
F 0 "C58" H 3865 7846 50  0000 L CNN
F 1 "4.7 uF" H 3865 7755 50  0000 L CNN
F 2 "" H 3788 7650 50  0001 C CNN
F 3 "" H 3750 7800 50  0001 C CNN
	1    3750 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 7650 3250 7650
Connection ~ 2300 7650
Connection ~ 2750 7650
Wire Wire Line
	2750 7650 2300 7650
Connection ~ 3250 7650
Wire Wire Line
	3250 7650 2750 7650
Wire Wire Line
	2300 7950 2750 7950
Connection ~ 2300 7950
Connection ~ 2750 7950
Wire Wire Line
	2750 7950 3250 7950
Connection ~ 3250 7950
Wire Wire Line
	3250 7950 3750 7950
Text Label 6650 1150 0    50   ~ 0
1V8
Wire Wire Line
	6650 1150 6650 1250
Wire Wire Line
	6650 1450 6650 1550
Connection ~ 6650 1550
Wire Wire Line
	6650 1550 6650 1650
Connection ~ 6650 1650
Wire Wire Line
	6650 1650 6650 1750
Connection ~ 6650 1750
Wire Wire Line
	6650 1750 6650 1850
Connection ~ 6650 1850
Wire Wire Line
	6650 1850 6650 1950
Connection ~ 6650 1950
Wire Wire Line
	6650 1950 6650 2150
Connection ~ 6650 2150
Wire Wire Line
	6650 2150 6650 2250
Connection ~ 6650 2250
Wire Wire Line
	6650 2250 6650 2350
Connection ~ 6650 2350
Wire Wire Line
	6650 2350 6650 2450
Connection ~ 6650 2450
Wire Wire Line
	6650 2450 6650 2550
Connection ~ 6650 2550
Wire Wire Line
	6650 2550 6650 2650
Connection ~ 6650 2650
Wire Wire Line
	6650 2650 6650 2850
Connection ~ 6650 2850
Wire Wire Line
	6650 2850 6650 2950
Connection ~ 6650 2950
Wire Wire Line
	6650 2950 6650 3050
Connection ~ 6650 3050
Wire Wire Line
	6650 3050 6650 3150
Connection ~ 6650 3150
Wire Wire Line
	6650 3150 6650 3250
Connection ~ 6650 3250
Wire Wire Line
	6650 3250 6650 3350
Wire Wire Line
	6650 3550 6650 3650
Connection ~ 6650 3650
Wire Wire Line
	6650 3650 6650 3750
Connection ~ 6650 3750
Wire Wire Line
	6650 3750 6650 3850
Text Label 6650 4050 0    50   ~ 0
1V8
Wire Wire Line
	6650 4050 6650 4150
Connection ~ 6650 4150
Wire Wire Line
	6650 4150 6650 4250
Connection ~ 6650 4250
Wire Wire Line
	6650 4250 6650 4350
Connection ~ 6650 4350
Wire Wire Line
	6650 4350 6650 4450
Connection ~ 6650 4450
Wire Wire Line
	6650 4450 6650 4550
Connection ~ 6650 4550
Wire Wire Line
	6650 4550 6650 4750
Connection ~ 6650 4750
Wire Wire Line
	6650 4750 6650 4850
Connection ~ 6650 4850
Wire Wire Line
	6650 4850 6650 4950
Connection ~ 6650 4950
Wire Wire Line
	6650 4950 6650 5050
Connection ~ 6650 5050
Wire Wire Line
	6650 5050 6650 5150
Connection ~ 6650 5150
Wire Wire Line
	6650 5150 6650 5250
Text Label 10650 6600 2    50   ~ 0
GTX_1V8
Text Label 10700 6750 2    50   ~ 0
GTX_1V2
Text Label 10700 6900 2    50   ~ 0
GTX_1V0
$Comp
L device:C C48
U 1 1 619B33AF
P 1850 8400
F 0 "C48" H 1965 8446 50  0000 L CNN
F 1 "47 uF" H 1965 8355 50  0000 L CNN
F 2 "" H 1888 8250 50  0001 C CNN
F 3 "" H 1850 8400 50  0001 C CNN
	1    1850 8400
	1    0    0    -1  
$EndComp
Text Label 1850 8250 2    50   ~ 0
1V8
Text Label 1850 8550 2    50   ~ 0
GND
$Comp
L device:C C49
U 1 1 619B41BE
P 1850 8950
F 0 "C49" H 1965 8996 50  0000 L CNN
F 1 "100 uF" H 1965 8905 50  0000 L CNN
F 2 "" H 1888 8800 50  0001 C CNN
F 3 "" H 1850 8950 50  0001 C CNN
	1    1850 8950
	1    0    0    -1  
$EndComp
Text Notes 1850 8650 0    50   ~ 0
VCCO_0
Text Label 1850 8800 2    50   ~ 0
1V35
Text Label 1850 9100 2    50   ~ 0
GND
Text Notes 1850 9250 0    50   ~ 0
VCCO_13_14_15
$Comp
L device:C C53
U 1 1 619BFA95
P 2750 8400
F 0 "C53" H 2865 8446 50  0000 L CNN
F 1 "47 uF" H 2865 8355 50  0000 L CNN
F 2 "" H 2788 8250 50  0001 C CNN
F 3 "" H 2750 8400 50  0001 C CNN
	1    2750 8400
	1    0    0    -1  
$EndComp
Text Label 2750 8250 2    50   ~ 0
3V3
Text Label 2750 8550 2    50   ~ 0
GND
Text Notes 2750 8650 0    50   ~ 0
VCCO_16
$Comp
L device:C C54
U 1 1 619C1346
P 2750 8950
F 0 "C54" H 2865 8996 50  0000 L CNN
F 1 "100 uF" H 2865 8905 50  0000 L CNN
F 2 "" H 2788 8800 50  0001 C CNN
F 3 "" H 2750 8950 50  0001 C CNN
	1    2750 8950
	1    0    0    -1  
$EndComp
Text Label 2750 8800 2    50   ~ 0
1V8
Text Label 2750 9100 2    50   ~ 0
GND
Text Notes 2750 9250 0    50   ~ 0
VCCO_33_34
Text Label 5050 7000 2    50   ~ 0
GTX_1V8
Text Label 5050 7650 2    50   ~ 0
GTX_1V2
Text Label 5900 7000 2    50   ~ 0
GTX_1V0
$Comp
L device:C C59
U 1 1 619CC852
P 5050 7150
F 0 "C59" H 5165 7196 50  0000 L CNN
F 1 "4.7 uF" H 5165 7105 50  0000 L CNN
F 2 "" H 5088 7000 50  0001 C CNN
F 3 "" H 5050 7150 50  0001 C CNN
	1    5050 7150
	1    0    0    -1  
$EndComp
Text Label 5050 7300 2    50   ~ 0
GND
$Comp
L device:C C60
U 1 1 619CCD03
P 5050 7800
F 0 "C60" H 5165 7846 50  0000 L CNN
F 1 "4.7 uF" H 5165 7755 50  0000 L CNN
F 2 "" H 5088 7650 50  0001 C CNN
F 3 "" H 5050 7800 50  0001 C CNN
	1    5050 7800
	1    0    0    -1  
$EndComp
Text Label 5050 7950 2    50   ~ 0
GND
$Comp
L device:C C61
U 1 1 619D591D
P 5900 7150
F 0 "C61" H 6015 7196 50  0000 L CNN
F 1 "4.7 uF" H 6015 7105 50  0000 L CNN
F 2 "" H 5938 7000 50  0001 C CNN
F 3 "" H 5900 7150 50  0001 C CNN
	1    5900 7150
	1    0    0    -1  
$EndComp
Text Label 5900 7300 2    50   ~ 0
GND
Text Notes 9450 7050 0    50   ~ 0
TODO: filtering for GTX rails to <10 mV p-p ripple
$EndSCHEMATC
