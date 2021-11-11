EESchema Schematic File Version 4
LIBS:sniffer-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 10
Title "SATA sniffer"
Date "2021-11-10"
Rev "0.1"
Comp ""
Comment1 "Andrew D. Zonenberg"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1000 1050 1000 1000
U 6189D312
F0 "Power Supply" 50
F1 "psu.sch" 50
F2 "GND" O R 2000 1650 50 
F3 "1V0" O R 2000 1550 50 
F4 "1V2" O R 2000 1450 50 
F5 "1V35" O R 2000 1350 50 
F6 "1V8" O R 2000 1250 50 
F7 "3V3" O R 2000 1150 50 
F8 "VTT" O R 2000 1850 50 
F9 "VREF" O R 2000 1950 50 
$EndSheet
Text Notes 5700 5500 0    50   ~ 0
Available I/O banks\n\n13, 14, 15: HR\n1.35V for DDR3.\nBoot flash lives in extra byte group of 14\nBoot flash: 5 (DQ[3:0], CS#)\nNeed level shifter\n\n16: HR (35)\nGPIO LEDs: 4\nLA pods: 10 (2x UART, present, fault, pwren)\nPMOD: 8\nSFP+: 8 (I2C, TX_DIS, TX_FAULT, RX_LOS, MOD_ABS, RS0, RS1)\nRAM: I2C\n3 unused\n\n33: HP\nLVDS for LA pods: 32 (2 pods * 8 pairs)\nRGMII: 15 (RGMII, MDIO, RST#)\nClock in: 2\n1 available\n\n34: HP\nLVDS for expansion connector
$Sheet
S 1000 3000 1000 1500
U 618C589A
F0 "DDR" 50
F1 "DDR.sch" 50
F2 "3V3" I R 2000 3050 50 
F3 "GND" I R 2000 3350 50 
F4 "1V35" I R 2000 3250 50 
F5 "VTT" I R 2000 4050 50 
F6 "VREF" I R 2000 3950 50 
F7 "FLASH_SO" I R 2000 3550 50 
F8 "FLASH_SI" O R 2000 3750 50 
F9 "FLASH_CS" O R 2000 3650 50 
F10 "RAM_SDA" B R 2000 4250 50 
F11 "RAM_SCL" I R 2000 4350 50 
$EndSheet
$Sheet
S 1000 4900 1050 1500
U 618C593E
F0 "SERDES interfaces" 50
F1 "serdes.sch" 50
$EndSheet
$Sheet
S 2450 1050 750  700 
U 618F2BCB
F0 "FPGA power" 50
F1 "fpgapwr.sch" 50
F2 "GND" I L 2450 1650 50 
F3 "1V0" I L 2450 1550 50 
F4 "1V2" I L 2450 1450 50 
F5 "1V8" I L 2450 1250 50 
F6 "1V35" I L 2450 1350 50 
F7 "3V3" I L 2450 1150 50 
$EndSheet
$Sheet
S 7000 1000 1050 1500
U 61909AE6
F0 "3.3V I/O" 50
F1 "io_3v3.sch" 50
$EndSheet
$Sheet
S 8500 1000 1000 1500
U 61923389
F0 "RGMII PHY" 50
F1 "rgmii.sch" 50
$EndSheet
$Sheet
S 8500 3000 1000 1500
U 6192A55E
F0 "Expansion connector" 50
F1 "expansion.sch" 50
$EndSheet
Wire Wire Line
	2000 1150 2450 1150
Wire Wire Line
	2450 1250 2000 1250
Wire Wire Line
	2000 1350 2450 1350
Wire Wire Line
	2450 1450 2000 1450
Wire Wire Line
	2000 1550 2450 1550
Wire Wire Line
	2450 1650 2000 1650
Text Label 2100 1650 0    50   ~ 0
GND
Text Label 2100 1550 0    50   ~ 0
1V0
Text Label 2100 1450 0    50   ~ 0
1V2
Text Label 2100 1350 0    50   ~ 0
1V35
Text Label 2100 1250 0    50   ~ 0
1V8
Text Label 2100 1150 0    50   ~ 0
3V3
$Sheet
S 2500 3000 1050 850 
U 61B1350A
F0 "Boot flash" 50
F1 "flash.sch" 50
F2 "1V35" I L 2500 3250 50 
F3 "1V8" I L 2500 3150 50 
F4 "FLASH_SCK" I R 3550 3550 50 
F5 "3V3" I L 2500 3050 50 
F6 "FLASH_SO" O L 2500 3550 50 
F7 "FLASH_CS" I L 2500 3650 50 
F8 "FLASH_SI" I L 2500 3750 50 
F9 "GND" I L 2500 3350 50 
$EndSheet
Text Label 2500 3050 2    50   ~ 0
3V3
Text Label 2500 3150 2    50   ~ 0
1V8
Text Label 2500 3250 2    50   ~ 0
1V35
Text Label 2500 3350 2    50   ~ 0
GND
Text Label 2000 3350 0    50   ~ 0
GND
Text Label 2000 3050 0    50   ~ 0
3V3
Text Label 2000 3250 0    50   ~ 0
1V35
Text Label 2000 1850 0    50   ~ 0
RAM_VTT
Text Label 2000 1950 0    50   ~ 0
RAM_VREF
Wire Wire Line
	2000 3550 2500 3550
Wire Wire Line
	2500 3650 2000 3650
Wire Wire Line
	2000 3750 2500 3750
Text Label 2000 3950 0    50   ~ 0
RAM_VREF
Text Label 2000 4050 0    50   ~ 0
RAM_VTT
$Sheet
S 4000 3000 800  850 
U 61D11C1F
F0 "FPGA support" 50
F1 "fpgasupport.sch" 50
F2 "FLASH_SCK" O L 4000 3550 50 
F3 "3V3" I L 4000 3050 50 
F4 "GND" I L 4000 3150 50 
$EndSheet
Text Label 4000 3150 2    50   ~ 0
GND
Text Label 4000 3050 2    50   ~ 0
3V3
Wire Wire Line
	4000 3550 3550 3550
$EndSCHEMATC
