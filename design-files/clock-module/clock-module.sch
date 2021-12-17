EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "SoT8 Clock Module"
Date ""
Rev "0.1"
Comp "Copyright (c) 2021 Jason R. Thorpe.  See LICENSE.txt."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Timer:NE555P U101
U 1 1 61B8134C
P 2150 2300
F 0 "U101" H 1900 2800 50  0000 C CNN
F 1 "NE555P" H 1900 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 2800 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ne555.pdf" H 3000 1900 50  0001 C CNN
	1    2150 2300
	1    0    0    -1  
$EndComp
$Comp
L Timer:NE555P U102
U 1 1 61B84A01
P 5050 2300
F 0 "U102" H 4800 2800 50  0000 C CNN
F 1 "NE555P" H 4800 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 5700 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ne555.pdf" H 5900 1900 50  0001 C CNN
	1    5050 2300
	1    0    0    -1  
$EndComp
$Comp
L Timer:NE555P U103
U 1 1 61B85B0D
P 7900 2300
F 0 "U103" H 7650 2800 50  0000 C CNN
F 1 "NE555P" H 7650 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 8550 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ne555.pdf" H 8750 1900 50  0001 C CNN
	1    7900 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C101
U 1 1 61B87CAB
P 2500 1650
F 0 "C101" H 2592 1696 50  0000 L CNN
F 1 "0.1µF" H 2592 1605 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 2500 1650 50  0001 C CNN
F 3 "~" H 2500 1650 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 2500 1650 50  0001 C CNN "Mouser"
	1    2500 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C107
U 1 1 61B8B698
P 8250 1650
F 0 "C107" H 8342 1696 50  0000 L CNN
F 1 "0.1µF" H 8342 1605 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 8250 1650 50  0001 C CNN
F 3 "~" H 8250 1650 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 8250 1650 50  0001 C CNN "Mouser"
	1    8250 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 61B8C11C
P 2500 1750
F 0 "#PWR0101" H 2500 1500 50  0001 C CNN
F 1 "GND" H 2505 1577 50  0000 C CNN
F 2 "" H 2500 1750 50  0001 C CNN
F 3 "" H 2500 1750 50  0001 C CNN
	1    2500 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61B8CB89
P 5400 1750
F 0 "#PWR0102" H 5400 1500 50  0001 C CNN
F 1 "GND" H 5405 1577 50  0000 C CNN
F 2 "" H 5400 1750 50  0001 C CNN
F 3 "" H 5400 1750 50  0001 C CNN
	1    5400 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 61B8CDCC
P 8250 1750
F 0 "#PWR0103" H 8250 1500 50  0001 C CNN
F 1 "GND" H 8255 1577 50  0000 C CNN
F 2 "" H 8250 1750 50  0001 C CNN
F 3 "" H 8250 1750 50  0001 C CNN
	1    8250 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 61B8D4F5
P 7150 3100
F 0 "#PWR0104" H 7150 2850 50  0001 C CNN
F 1 "GND" H 7155 2927 50  0000 C CNN
F 2 "" H 7150 3100 50  0001 C CNN
F 3 "" H 7150 3100 50  0001 C CNN
	1    7150 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1900 2150 1550
Wire Wire Line
	2150 1550 2500 1550
Connection ~ 2500 1550
Wire Wire Line
	5050 1900 5050 1550
Wire Wire Line
	5050 1550 5400 1550
Connection ~ 5400 1550
Wire Wire Line
	7900 1900 7900 1550
Wire Wire Line
	7900 1550 8250 1550
Wire Wire Line
	2150 1550 1550 1550
Wire Wire Line
	1550 1550 1550 2500
Wire Wire Line
	1550 2500 1650 2500
Connection ~ 2150 1550
$Comp
L Device:CP1_Small C103
U 1 1 61B9014A
P 1750 2900
F 0 "C103" H 1841 2946 50  0000 L CNN
F 1 "1µF" H 1841 2855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 1750 2900 50  0001 C CNN
F 3 "~" H 1750 2900 50  0001 C CNN
F 4 "667-ECA-2VM010B" H 1750 2900 50  0001 C CNN "Mouser"
	1    1750 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C102
U 1 1 61B90475
P 1350 2900
F 0 "C102" H 1050 2950 50  0000 L CNN
F 1 "0.01µF" H 950 2850 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1350 2900 50  0001 C CNN
F 3 "~" H 1350 2900 50  0001 C CNN
F 4 "80-C315C103K5R-TR" H 1350 2900 50  0001 C CNN "Mouser"
	1    1350 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 2100 1450 2100
Wire Wire Line
	1650 2300 1350 2300
Wire Wire Line
	1450 2100 1450 2800
Wire Wire Line
	1450 2800 1750 2800
Wire Wire Line
	1350 2300 1350 2800
$Comp
L power:GND #PWR0105
U 1 1 61B96824
P 1750 3100
F 0 "#PWR0105" H 1750 2850 50  0001 C CNN
F 1 "GND" H 1755 2927 50  0000 C CNN
F 2 "" H 1750 3100 50  0001 C CNN
F 3 "" H 1750 3100 50  0001 C CNN
	1    1750 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3000 1350 3100
Wire Wire Line
	1350 3100 1750 3100
Wire Wire Line
	1750 3000 1750 3100
Connection ~ 1750 3100
Wire Wire Line
	2150 2700 2150 3100
Wire Wire Line
	2150 3100 1750 3100
$Comp
L Device:R_Small R102
U 1 1 61B97F45
P 3100 2300
F 0 "R102" V 3200 2300 50  0000 C CNN
F 1 "1K" V 3000 2300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 3100 2300 50  0001 C CNN
F 3 "~" H 3100 2300 50  0001 C CNN
F 4 "594-SFR16S0001001JR5" V 3100 2300 50  0001 C CNN "Mouser"
	1    3100 2300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R101
U 1 1 61B9A93D
P 2850 1900
F 0 "R101" H 2909 1946 50  0000 L CNN
F 1 "1K" H 2909 1855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 2850 1900 50  0001 C CNN
F 3 "~" H 2850 1900 50  0001 C CNN
F 4 "594-SFR16S0001001JR5" H 2850 1900 50  0001 C CNN "Mouser"
	1    2850 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1550 2850 1550
Wire Wire Line
	2850 1550 2850 1800
Wire Wire Line
	2850 2300 2650 2300
Wire Wire Line
	2850 2000 2850 2300
Wire Wire Line
	2850 2300 3000 2300
Connection ~ 2850 2300
$Comp
L Device:R_POT_Small RV101
U 1 1 61B9D08B
P 3400 2300
F 0 "RV101" H 3340 2254 50  0000 R CNN
F 1 "1M" H 3340 2345 50  0000 R CNN
F 2 "jrt-Potentiometers:Potentiometer_Alpha_RV09AF-40_vert" H 3400 2300 50  0001 C CNN
F 3 "~" H 3400 2300 50  0001 C CNN
F 4 "317-2090F-1M" H 3400 2300 50  0001 C CNN "Mouser"
	1    3400 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 2800 2650 2800
Connection ~ 1750 2800
Wire Wire Line
	2650 2500 2650 2800
Wire Wire Line
	3200 2300 3300 2300
Wire Wire Line
	2650 2800 3400 2800
Wire Wire Line
	3400 2800 3400 2400
Connection ~ 2650 2800
NoConn ~ 3400 2200
$Comp
L Switch:SW_Push SW101
U 1 1 61BA668D
P 4100 1900
F 0 "SW101" H 4100 1800 50  0000 C CNN
F 1 "SW_Push" H 4100 1700 50  0000 C CNN
F 2 "Button_Switch_THT:SW_TH_Tactile_Omron_B3F-10xx" H 4100 2100 50  0001 C CNN
F 3 "~" H 4100 2100 50  0001 C CNN
F 4 "179-TS0266150BK100SC" H 4100 1900 50  0001 C CNN "Mouser"
	1    4100 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 1550 4450 1550
Connection ~ 5050 1550
Wire Wire Line
	4300 1750 4300 1900
Wire Wire Line
	4550 2100 4300 2100
Wire Wire Line
	4300 2100 4300 1900
Connection ~ 4300 1900
Wire Wire Line
	4550 2500 4450 2500
Wire Wire Line
	4450 2500 4450 1550
Connection ~ 4450 1550
Wire Wire Line
	4450 1550 4300 1550
$Comp
L Device:C_Small C105
U 1 1 61BA94EE
P 4300 2900
F 0 "C105" H 4400 2950 50  0000 L CNN
F 1 "0.01µF" H 4400 2850 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 4300 2900 50  0001 C CNN
F 3 "~" H 4300 2900 50  0001 C CNN
F 4 "80-C315C103K5R-TR" H 4300 2900 50  0001 C CNN "Mouser"
	1    4300 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 61BAA633
P 4300 3100
F 0 "#PWR0106" H 4300 2850 50  0001 C CNN
F 1 "GND" H 4305 2927 50  0000 C CNN
F 2 "" H 4300 3100 50  0001 C CNN
F 3 "" H 4300 3100 50  0001 C CNN
	1    4300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3000 4300 3100
Wire Wire Line
	5050 2700 5050 3100
Wire Wire Line
	5050 3100 4300 3100
Connection ~ 4300 3100
Wire Wire Line
	3900 1900 3800 1900
Wire Wire Line
	3800 1900 3800 3100
Wire Wire Line
	3800 3100 4300 3100
Wire Wire Line
	4550 2300 4300 2300
Wire Wire Line
	4300 2300 4300 2800
$Comp
L Device:R_Small R103
U 1 1 61BA3864
P 4300 1650
F 0 "R103" H 4241 1604 50  0000 R CNN
F 1 "1K" H 4241 1695 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 4300 1650 50  0001 C CNN
F 3 "~" H 4300 1650 50  0001 C CNN
F 4 "594-SFR16S0001001JR5" H 4300 1650 50  0001 C CNN "Mouser"
	1    4300 1650
	1    0    0    1   
$EndComp
$Comp
L Device:C_Small C106
U 1 1 61BAED84
P 5800 2900
F 0 "C106" H 5900 2950 50  0000 L CNN
F 1 "0.1µF" H 5900 2850 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 5800 2900 50  0001 C CNN
F 3 "~" H 5800 2900 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 5800 2900 50  0001 C CNN "Mouser"
	1    5800 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C104
U 1 1 61B89EA1
P 5400 1650
F 0 "C104" H 5100 1700 50  0000 L CNN
F 1 "0.1µF" H 5100 1600 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 5400 1650 50  0001 C CNN
F 3 "~" H 5400 1650 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 5400 1650 50  0001 C CNN "Mouser"
	1    5400 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R104
U 1 1 61BB3456
P 5800 1650
F 0 "R104" H 6050 1600 50  0000 R CNN
F 1 "1M" H 5950 1700 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 5800 1650 50  0001 C CNN
F 3 "~" H 5800 1650 50  0001 C CNN
F 4 "594-SFR16S0001004JA5" H 5800 1650 50  0001 C CNN "Mouser"
	1    5800 1650
	1    0    0    1   
$EndComp
Wire Wire Line
	5400 1550 5800 1550
Wire Wire Line
	5800 1750 5800 2300
Wire Wire Line
	5800 3000 5800 3100
Wire Wire Line
	5800 3100 5050 3100
Connection ~ 5050 3100
Wire Wire Line
	5550 2300 5800 2300
Connection ~ 5800 2300
Wire Wire Line
	5800 2300 5800 2500
Wire Wire Line
	5550 2500 5800 2500
Connection ~ 5800 2500
Wire Wire Line
	5800 2500 5800 2800
$Comp
L Device:C_Small C108
U 1 1 61BC4210
P 7150 2900
F 0 "C108" H 7250 2950 50  0000 L CNN
F 1 "0.01µF" H 7250 2850 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 7150 2900 50  0001 C CNN
F 3 "~" H 7150 2900 50  0001 C CNN
F 4 "80-C315C103K5R-TR" H 7150 2900 50  0001 C CNN "Mouser"
	1    7150 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3000 7150 3100
Wire Wire Line
	7900 2700 7900 3100
Wire Wire Line
	7900 3100 7150 3100
Connection ~ 7150 3100
Wire Wire Line
	8400 2500 8500 2500
Wire Wire Line
	8500 2500 8500 3100
Wire Wire Line
	8500 3100 7900 3100
Connection ~ 7900 3100
NoConn ~ 8400 2300
Wire Wire Line
	7400 2300 7150 2300
Wire Wire Line
	7150 2300 7150 2800
$Comp
L Device:R_Small R106
U 1 1 61BCB4C4
P 7050 1650
F 0 "R106" H 7300 1600 50  0000 R CNN
F 1 "1K" H 7200 1700 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 7050 1650 50  0001 C CNN
F 3 "~" H 7050 1650 50  0001 C CNN
F 4 "594-SFR16S0001001JR5" H 7050 1650 50  0001 C CNN "Mouser"
	1    7050 1650
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R105
U 1 1 61BCBA0F
P 6850 1650
F 0 "R105" H 6791 1604 50  0000 R CNN
F 1 "1K" H 6791 1695 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 6850 1650 50  0001 C CNN
F 3 "~" H 6850 1650 50  0001 C CNN
F 4 "594-SFR16S0001001JR5" H 6850 1650 50  0001 C CNN "Mouser"
	1    6850 1650
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_SPDT SW102
U 1 1 61BCC770
P 6650 2400
F 0 "SW102" H 6650 2685 50  0000 C CNN
F 1 "SW_SPDT" H 6650 2594 50  0000 C CNN
F 2 "jrt-Switches:NKK_BB16AP" H 6650 2400 50  0001 C CNN
F 3 "~" H 6650 2400 50  0001 C CNN
F 4 "633-BB16APFA" H 6650 2400 50  0001 C CNN "Mouser"
	1    6650 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1750 6850 2100
Wire Wire Line
	7400 2500 7050 2500
Wire Wire Line
	7050 1750 7050 2500
Connection ~ 7050 2500
Wire Wire Line
	7050 2500 6850 2500
Wire Wire Line
	7400 2100 6850 2100
Connection ~ 6850 2100
Wire Wire Line
	6850 2100 6850 2300
Wire Wire Line
	6450 2400 6450 3100
Wire Wire Line
	6450 3100 7150 3100
Wire Wire Line
	6850 1550 7050 1550
Wire Wire Line
	7050 1550 7900 1550
Connection ~ 7050 1550
Connection ~ 7900 1550
Text GLabel 2950 2100 2    50   Output Italic 0
auto_clk
Wire Wire Line
	2650 2100 2950 2100
Text GLabel 5900 2100 2    50   Output Italic 0
man_clk
Wire Wire Line
	5550 2100 5900 2100
Text GLabel 8700 2100 2    50   Output Italic 0
clk_sel
Wire Wire Line
	8400 2100 8700 2100
$Comp
L 74xx:74HCT00 U104
U 1 1 61BE5527
P 2250 4050
F 0 "U104" H 2250 4375 50  0000 C CNN
F 1 "74HCT00" H 2250 4284 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 2250 4050 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 2250 4050 50  0001 C CNN
	1    2250 4050
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT00 U104
U 2 1 61BE9444
P 3050 3950
F 0 "U104" H 3050 4275 50  0000 C CNN
F 1 "74HCT00" H 3050 4184 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 3050 3950 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 3050 3950 50  0001 C CNN
	2    3050 3950
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT00 U104
U 3 1 61BECEBD
P 3050 4550
F 0 "U104" H 3050 4875 50  0000 C CNN
F 1 "74HCT00" H 3050 4784 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 3050 4550 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 3050 4550 50  0001 C CNN
	3    3050 4550
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT00 U104
U 4 1 61BEEBE7
P 3850 4250
F 0 "U104" H 3850 4575 50  0000 C CNN
F 1 "74HCT00" H 3850 4484 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 3850 4250 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 3850 4250 50  0001 C CNN
	4    3850 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3950 1950 4050
Text GLabel 1650 4050 0    50   Input Italic 0
clk_sel
Wire Wire Line
	1650 4050 1950 4050
Connection ~ 1950 4050
Wire Wire Line
	1950 4050 1950 4150
Wire Wire Line
	2550 4050 2750 4050
Text GLabel 1650 3600 0    50   Input Italic 0
man_clk
Wire Wire Line
	1650 3600 2750 3600
Wire Wire Line
	2750 3600 2750 3850
Text GLabel 1650 4650 0    50   Input Italic 0
auto_clk
Wire Wire Line
	1650 4650 2750 4650
Wire Wire Line
	1950 4150 1950 4450
Wire Wire Line
	1950 4450 2750 4450
Connection ~ 1950 4150
Wire Wire Line
	3350 3950 3550 3950
Wire Wire Line
	3550 3950 3550 4150
Wire Wire Line
	3350 4550 3550 4550
Wire Wire Line
	3550 4550 3550 4350
$Comp
L 74xx:74HCT00 U104
U 5 1 61C1F422
P 7150 4450
F 0 "U104" H 7380 4496 50  0000 L CNN
F 1 "74HCT00" H 7380 4405 50  0000 L CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 7150 4450 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 7150 4450 50  0001 C CNN
	5    7150 4450
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0107
U 1 1 61C29F7B
P 2150 1550
F 0 "#PWR0107" H 2150 1400 50  0001 C CNN
F 1 "VCC" H 2165 1723 50  0000 C CNN
F 2 "" H 2150 1550 50  0001 C CNN
F 3 "" H 2150 1550 50  0001 C CNN
	1    2150 1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0108
U 1 1 61C2A946
P 5050 1550
F 0 "#PWR0108" H 5050 1400 50  0001 C CNN
F 1 "VCC" H 5065 1723 50  0000 C CNN
F 2 "" H 5050 1550 50  0001 C CNN
F 3 "" H 5050 1550 50  0001 C CNN
	1    5050 1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0109
U 1 1 61C2AF89
P 7900 1500
F 0 "#PWR0109" H 7900 1350 50  0001 C CNN
F 1 "VCC" H 7915 1673 50  0000 C CNN
F 2 "" H 7900 1500 50  0001 C CNN
F 3 "" H 7900 1500 50  0001 C CNN
	1    7900 1500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT00 U105
U 1 1 61C2C1D1
P 4650 4350
F 0 "U105" H 4650 4675 50  0000 C CNN
F 1 "74HCT00" H 4650 4584 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 4650 4350 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 4650 4350 50  0001 C CNN
	1    4650 4350
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HCT00 U105
U 2 1 61C30F8B
P 5450 4350
F 0 "U105" H 5450 4675 50  0000 C CNN
F 1 "74HCT00" H 5450 4584 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 5450 4350 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 5450 4350 50  0001 C CNN
	2    5450 4350
	1    0    0    -1  
$EndComp
Text GLabel 1650 4950 0    50   Input ~ 0
~HLT
Wire Wire Line
	1650 4950 4350 4950
Wire Wire Line
	4350 4950 4350 4450
Wire Wire Line
	4150 4250 4350 4250
Wire Wire Line
	5150 4250 5150 4350
Wire Wire Line
	4950 4350 5000 4350
Connection ~ 5150 4350
Wire Wire Line
	5150 4350 5150 4450
Text GLabel 5900 4350 2    50   Output ~ 0
CLK
Text GLabel 5900 4650 2    50   Output ~ 0
~CLK
Wire Wire Line
	5000 4350 5000 4650
Wire Wire Line
	5000 4650 5900 4650
Connection ~ 5000 4350
Wire Wire Line
	5000 4350 5150 4350
Wire Wire Line
	5750 4350 5800 4350
$Comp
L 74xx:74HCT00 U105
U 5 1 61C46955
P 8100 4450
F 0 "U105" H 8330 4496 50  0000 L CNN
F 1 "74HCT00" H 8330 4405 50  0000 L CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket" H 8100 4450 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 8100 4450 50  0001 C CNN
	5    8100 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 61C493C1
P 7150 4950
F 0 "#PWR0110" H 7150 4700 50  0001 C CNN
F 1 "GND" H 7155 4777 50  0000 C CNN
F 2 "" H 7150 4950 50  0001 C CNN
F 3 "" H 7150 4950 50  0001 C CNN
	1    7150 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 61C49C28
P 8100 4950
F 0 "#PWR0111" H 8100 4700 50  0001 C CNN
F 1 "GND" H 8105 4777 50  0000 C CNN
F 2 "" H 8100 4950 50  0001 C CNN
F 3 "" H 8100 4950 50  0001 C CNN
	1    8100 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C109
U 1 1 61C4A3B3
P 7550 3900
F 0 "C109" H 7250 3950 50  0000 L CNN
F 1 "0.1µF" H 7250 3850 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 7550 3900 50  0001 C CNN
F 3 "~" H 7550 3900 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 7550 3900 50  0001 C CNN "Mouser"
	1    7550 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C110
U 1 1 61C4B11F
P 8500 3900
F 0 "C110" H 8200 3950 50  0000 L CNN
F 1 "0.1µF" H 8200 3850 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 8500 3900 50  0001 C CNN
F 3 "~" H 8500 3900 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 8500 3900 50  0001 C CNN "Mouser"
	1    8500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3950 7150 3800
Wire Wire Line
	7150 3800 7550 3800
Wire Wire Line
	8100 3950 8100 3800
Wire Wire Line
	8100 3800 8500 3800
$Comp
L power:GND #PWR0112
U 1 1 61C537B0
P 7550 4000
F 0 "#PWR0112" H 7550 3750 50  0001 C CNN
F 1 "GND" H 7555 3827 50  0000 C CNN
F 2 "" H 7550 4000 50  0001 C CNN
F 3 "" H 7550 4000 50  0001 C CNN
	1    7550 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 61C543B9
P 8500 4000
F 0 "#PWR0113" H 8500 3750 50  0001 C CNN
F 1 "GND" H 8505 3827 50  0000 C CNN
F 2 "" H 8500 4000 50  0001 C CNN
F 3 "" H 8500 4000 50  0001 C CNN
	1    8500 4000
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0114
U 1 1 61C54C1D
P 7150 3800
F 0 "#PWR0114" H 7150 3650 50  0001 C CNN
F 1 "VCC" H 7165 3973 50  0000 C CNN
F 2 "" H 7150 3800 50  0001 C CNN
F 3 "" H 7150 3800 50  0001 C CNN
	1    7150 3800
	1    0    0    -1  
$EndComp
Connection ~ 7150 3800
$Comp
L power:VCC #PWR0115
U 1 1 61C5546C
P 8100 3800
F 0 "#PWR0115" H 8100 3650 50  0001 C CNN
F 1 "VCC" H 8115 3973 50  0000 C CNN
F 2 "" H 8100 3800 50  0001 C CNN
F 3 "" H 8100 3800 50  0001 C CNN
	1    8100 3800
	1    0    0    -1  
$EndComp
Connection ~ 8100 3800
$Comp
L Device:LED D101
U 1 1 61C55EB3
P 5800 3850
F 0 "D101" V 5747 3732 50  0000 R CNN
F 1 "BLUE" V 5838 3732 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5800 3850 50  0001 C CNN
F 3 "~" H 5800 3850 50  0001 C CNN
F 4 "941-C503BBCSCW0X0452" V 5800 3850 50  0001 C CNN "Mouser"
	1    5800 3850
	0    -1   1    0   
$EndComp
$Comp
L Device:R_Small R107
U 1 1 61C5AA02
P 5800 3500
F 0 "R107" H 5741 3454 50  0000 R CNN
F 1 "330" H 5741 3545 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 5800 3500 50  0001 C CNN
F 3 "~" H 5800 3500 50  0001 C CNN
F 4 "594-SFR16S0003300JA5" H 5800 3500 50  0001 C CNN "Mouser"
	1    5800 3500
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 61C5C4F5
P 5800 3400
F 0 "#PWR0116" H 5800 3150 50  0001 C CNN
F 1 "GND" H 5805 3227 50  0000 C CNN
F 2 "" H 5800 3400 50  0001 C CNN
F 3 "" H 5800 3400 50  0001 C CNN
	1    5800 3400
	-1   0    0    1   
$EndComp
Wire Wire Line
	5800 3600 5800 3700
Wire Wire Line
	5800 4000 5800 4350
Connection ~ 5800 4350
Wire Wire Line
	5800 4350 5900 4350
$Comp
L Device:Q_PNP_BCE Q101
U 1 1 61C7A218
P 2050 6050
F 0 "Q101" H 2240 6004 50  0000 L CNN
F 1 "2N6491" H 2240 6095 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 2250 6150 50  0001 C CNN
F 3 "~" H 2050 6050 50  0001 C CNN
F 4 "610-2N6491" H 2050 6050 50  0001 C CNN "Mouser"
	1    2050 6050
	1    0    0    1   
$EndComp
$Comp
L power:+5V #PWR0117
U 1 1 61C7FDAF
P 2150 5650
F 0 "#PWR0117" H 2150 5500 50  0001 C CNN
F 1 "+5V" H 2165 5823 50  0000 C CNN
F 2 "" H 2150 5650 50  0001 C CNN
F 3 "" H 2150 5650 50  0001 C CNN
	1    2150 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5650 2150 5850
$Comp
L Device:CP1_Small C111
U 1 1 61C87F07
P 2400 6500
F 0 "C111" H 2491 6546 50  0000 L CNN
F 1 "1µF" H 2491 6455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 2400 6500 50  0001 C CNN
F 3 "~" H 2400 6500 50  0001 C CNN
F 4 "667-ECA-2VM010B" H 2400 6500 50  0001 C CNN "Mouser"
	1    2400 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 6250 2400 6250
Wire Wire Line
	2400 6250 2400 6400
$Comp
L power:VCC #PWR0118
U 1 1 61C909FA
P 2750 6250
F 0 "#PWR0118" H 2750 6100 50  0001 C CNN
F 1 "VCC" V 2765 6378 50  0000 L CNN
F 2 "" H 2750 6250 50  0001 C CNN
F 3 "" H 2750 6250 50  0001 C CNN
	1    2750 6250
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 6250 2750 6250
Connection ~ 2400 6250
$Comp
L power:GND #PWR0119
U 1 1 61C95171
P 2400 6600
F 0 "#PWR0119" H 2400 6350 50  0001 C CNN
F 1 "GND" H 2405 6427 50  0000 C CNN
F 2 "" H 2400 6600 50  0001 C CNN
F 3 "" H 2400 6600 50  0001 C CNN
	1    2400 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D102
U 1 1 61C96D4E
P 1400 6450
F 0 "D102" V 1439 6530 50  0000 L CNN
F 1 "GREEN" V 1348 6530 50  0000 L CNN
F 2 "LED_THT:LED_D5.0mm" H 1400 6450 50  0001 C CNN
F 3 "~" H 1400 6450 50  0001 C CNN
F 4 "941-C503BGCSCY0C0792" V 1400 6450 50  0001 C CNN "Mouser"
	1    1400 6450
	0    1    -1   0   
$EndComp
$Comp
L Device:R_Small R108
U 1 1 61C99077
P 1400 6800
F 0 "R108" H 1341 6754 50  0000 R CNN
F 1 "330" H 1341 6845 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 1400 6800 50  0001 C CNN
F 3 "~" H 1400 6800 50  0001 C CNN
F 4 "594-SFR16S0003300JA5" H 1400 6800 50  0001 C CNN "Mouser"
	1    1400 6800
	1    0    0    1   
$EndComp
Wire Wire Line
	1400 6150 1400 6300
Wire Wire Line
	1400 6600 1400 6700
$Comp
L power:GND #PWR0120
U 1 1 61CA1414
P 1400 6900
F 0 "#PWR0120" H 1400 6650 50  0001 C CNN
F 1 "GND" H 1405 6727 50  0000 C CNN
F 2 "" H 1400 6900 50  0001 C CNN
F 3 "" H 1400 6900 50  0001 C CNN
	1    1400 6900
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT SW103
U 1 1 61CB251D
P 1500 6150
F 0 "SW103" H 1500 6435 50  0000 C CNN
F 1 "SW_SPDT" H 1500 6344 50  0000 C CNN
F 2 "jrt-Switches:NKK_BB16AP" H 1500 6150 50  0001 C CNN
F 3 "~" H 1500 6150 50  0001 C CNN
F 4 "633-BB16APFA" H 1500 6150 50  0001 C CNN "Mouser"
	1    1500 6150
	1    0    0    -1  
$EndComp
NoConn ~ 1700 6250
Wire Wire Line
	1700 6050 1850 6050
$Comp
L jrt-Connectors:CUI_PJ-097BH-5_5-2_5mm-5A J101
U 1 1 6409DB36
P 4000 6300
F 0 "J101" H 4058 6715 50  0000 C CNN
F 1 "CUI_PJ-097BH-5_5-2_5mm-5A" H 4058 6624 50  0000 C CNN
F 2 "jrt-Connectors:CUI_PJ-097BH-5_5-2_5mm-5A" H 4200 6200 50  0001 C CNN
F 3 "" H 4200 6200 50  0001 C CNN
	1    4000 6300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0121
U 1 1 640A04F3
P 4450 6150
F 0 "#PWR0121" H 4450 6000 50  0001 C CNN
F 1 "+5V" V 4465 6278 50  0000 L CNN
F 2 "" H 4450 6150 50  0001 C CNN
F 3 "" H 4450 6150 50  0001 C CNN
	1    4450 6150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 640A13FE
P 4450 6450
F 0 "#PWR0122" H 4450 6200 50  0001 C CNN
F 1 "GND" V 4455 6322 50  0000 R CNN
F 2 "" H 4450 6450 50  0001 C CNN
F 3 "" H 4450 6450 50  0001 C CNN
	1    4450 6450
	0    -1   -1   0   
$EndComp
NoConn ~ 4450 6300
$EndSCHEMATC
