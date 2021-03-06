EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 10 10
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
U 1 1 61D1C162
P 2600 6400
AR Path="/618C589A/61D1C162" Ref="U?"  Part="3" 
AR Path="/61909AE6/61D1C162" Ref="U?"  Part="6" 
AR Path="/618F2BCB/61D1C162" Ref="U?"  Part="10" 
AR Path="/61D11C1F/61D1C162" Ref="U5"  Part="1" 
F 0 "U5" H 2600 6300 60  0000 L CNN
F 1 "XC7K70T-2FBG484" H 2600 6200 60  0000 L CNN
F 2 "azonenberg_pcb:BGA_484_22x22_FULLARRAY_1MM" H 2600 6100 60  0001 C CNN
F 3 "" H 2600 6100 60  0000 C CNN
	1    2600 6400
	1    0    0    -1  
$EndComp
$Comp
L xilinx-azonenberg:XILINX_JTAG J4
U 1 1 61D1CEA0
P 5750 3200
F 0 "J4" H 6469 3908 60  0000 L CNN
F 1 "XILINX_JTAG" H 6469 3802 60  0000 L CNN
F 2 "azonenberg_pcb:XILINX_JTAG_PTH_MOLEX_0878311420" H 5750 3200 60  0001 C CNN
F 3 "" H 5750 3200 60  0000 C CNN
	1    5750 3200
	1    0    0    -1  
$EndComp
Text Label 2400 4400 2    50   ~ 0
GND
Text Label 2400 4500 2    50   ~ 0
GND
Text Label 2400 4700 2    50   ~ 0
GND
Text Label 2400 4800 2    50   ~ 0
GND
Text Label 2400 4200 2    50   ~ 0
GND
Text Label 2400 4100 2    50   ~ 0
GND
Text Label 2400 5000 2    50   ~ 0
JTAG_TCK
Text Label 1750 5100 2    50   ~ 0
JTAG_TDO
Text Label 2400 5200 2    50   ~ 0
JTAG_TDI
Text Label 2400 5300 2    50   ~ 0
JTAG_TMS
$Comp
L device:R R30
U 1 1 61D223DE
P 1900 5100
F 0 "R30" V 1800 5100 50  0000 C CNN
F 1 "33" V 1900 5100 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1830 5100 50  0001 C CNN
F 3 "" H 1900 5100 50  0001 C CNN
	1    1900 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 5100 2400 5100
$Comp
L device:R R31
U 1 1 61D2353D
P 1900 5500
F 0 "R31" V 1800 5500 50  0000 C CNN
F 1 "33" V 1900 5500 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1830 5500 50  0001 C CNN
F 3 "" H 1900 5500 50  0001 C CNN
	1    1900 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 5500 2400 5500
Text HLabel 1750 5500 0    50   Output ~ 0
FLASH_SCK
$Comp
L special-azonenberg:SSM6N58NU_DUAL_NMOS Q1
U 1 1 61D25108
P 5100 5200
F 0 "Q1" H 5244 5306 60  0000 L CNN
F 1 "SSM6N58NU" H 5244 5200 60  0000 L CNN
F 2 "azonenberg_pcb:DFN_6_0.65MM_2x2MM_GDS" H 5100 5200 60  0001 C CNN
F 3 "transistors/mos/*.*" H 5244 5094 60  0000 L CNN
	1    5100 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 5550 5200 5400
$Comp
L device:LED D1
U 1 1 61D26408
P 5200 4850
F 0 "D1" V 5239 4732 50  0000 R CNN
F 1 "GREEN" V 5148 4732 50  0000 R CNN
F 2 "azonenberg_pcb:EIA_0402_LED" H 5200 4850 50  0001 C CNN
F 3 "~" H 5200 4850 50  0001 C CNN
	1    5200 4850
	0    -1   -1   0   
$EndComp
$Comp
L device:R R32
U 1 1 61D270AC
P 5200 4550
F 0 "R32" H 5270 4596 50  0000 L CNN
F 1 "470" H 5270 4505 50  0000 L CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 5130 4550 50  0001 C CNN
F 3 "" H 5200 4550 50  0001 C CNN
	1    5200 4550
	1    0    0    -1  
$EndComp
Text HLabel 5200 4400 0    50   Input ~ 0
3V3
Text Label 4900 5200 2    50   ~ 0
FPGA_DONE
Text Label 2400 5700 2    50   ~ 0
FPGA_DONE
Text Label 2400 6100 2    50   ~ 0
GND
$Comp
L device:R R28
U 1 1 61D2B9DD
P 1700 5800
F 0 "R28" V 1650 5950 50  0000 C CNN
F 1 "4.7K" V 1700 5800 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1630 5800 50  0001 C CNN
F 3 "" H 1700 5800 50  0001 C CNN
	1    1700 5800
	0    1    1    0   
$EndComp
Text Label 1550 5800 2    50   ~ 0
1V8
Wire Wire Line
	1850 5800 2400 5800
Text Label 2400 5800 2    50   ~ 0
FPGA_INIT_B
$Comp
L Connector:Conn_01x01 TP1
U 1 1 61D2EB74
P 2700 7000
F 0 "TP1" H 2618 6775 50  0000 C CNN
F 1 "TESTPOINT" H 2618 6866 50  0000 C CNN
F 2 "azonenberg_pcb:TESTPOINT_SMT_0.5MM" H 2700 7000 50  0001 C CNN
F 3 "~" H 2700 7000 50  0001 C CNN
	1    2700 7000
	-1   0    0    1   
$EndComp
Text Label 2900 7000 0    50   ~ 0
FPGA_DONE
$Comp
L Connector:Conn_01x01 TP2
U 1 1 61D30054
P 2700 7350
F 0 "TP2" H 2618 7125 50  0000 C CNN
F 1 "TESTPOINT" H 2618 7216 50  0000 C CNN
F 2 "azonenberg_pcb:TESTPOINT_SMT_0.5MM" H 2700 7350 50  0001 C CNN
F 3 "~" H 2700 7350 50  0001 C CNN
	1    2700 7350
	-1   0    0    1   
$EndComp
Text Label 2900 7350 0    50   ~ 0
FPGA_INIT_B
$Comp
L device:R R29
U 1 1 61D32E74
P 1700 5900
F 0 "R29" V 1650 6050 50  0000 C CNN
F 1 "4.7K" V 1700 5900 50  0000 C CNN
F 2 "azonenberg_pcb:EIA_0402_RES_NOSILK" V 1630 5900 50  0001 C CNN
F 3 "" H 1700 5900 50  0001 C CNN
	1    1700 5900
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 5800 1550 5900
Text Label 1950 5900 0    50   ~ 0
FPGA_RST_N
Wire Wire Line
	2400 5900 1850 5900
Text Label 2400 6200 2    50   ~ 0
1V8
Text Label 2400 6300 2    50   ~ 0
GND
Text Label 2400 6400 2    50   ~ 0
GND
Text Label 5750 2900 2    50   ~ 0
JTAG_TCK
Text Label 5750 3100 2    50   ~ 0
JTAG_TDI
Text Label 5750 3000 2    50   ~ 0
JTAG_TDO
Text Label 5750 2800 2    50   ~ 0
JTAG_TMS
NoConn ~ 5750 2700
NoConn ~ 5750 3200
Text Label 5750 2200 2    50   ~ 0
GND
Wire Wire Line
	5750 2200 5750 2300
Connection ~ 5750 2300
Wire Wire Line
	5750 2300 5750 2400
Connection ~ 5750 2400
Wire Wire Line
	5750 2400 5750 2500
Connection ~ 5750 2500
Wire Wire Line
	5750 2500 5750 2600
Text HLabel 5200 5550 0    50   Input ~ 0
GND
Text HLabel 5750 2100 0    50   Input ~ 0
1V8
$EndSCHEMATC
