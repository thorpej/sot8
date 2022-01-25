EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "SoT8 Special Register Module"
Date ""
Rev "0.1"
Comp "Copyright (c) 2021 Jason R. Thorpe.  See LICENSE.txt."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 74xx:74LS377 U402
U 1 1 61BD63C2
P 1850 1850
F 0 "U402" H 2100 2650 50  0000 C CNN
F 1 "74HCT377" H 2100 2550 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 1850 1850 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS377" H 1850 1850 50  0001 C CNN
	1    1850 1850
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS245 U405
U 1 1 61BD884C
P 4000 1700
F 0 "U405" V 4250 1000 50  0000 R CNN
F 1 "74HCT245" V 4150 1050 50  0000 R CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 4000 1700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 4000 1700 50  0001 C CNN
	1    4000 1700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2350 1350 2400 1350
Wire Wire Line
	2400 1350 2400 2200
Wire Wire Line
	2400 2200 3500 2200
Wire Wire Line
	2350 1450 2450 1450
Wire Wire Line
	2450 1450 2450 2250
Wire Wire Line
	2450 2250 3600 2250
Wire Wire Line
	3600 2250 3600 2200
Wire Wire Line
	2350 1550 2500 1550
Wire Wire Line
	2500 1550 2500 2300
Wire Wire Line
	2500 2300 3700 2300
Wire Wire Line
	3700 2300 3700 2200
Wire Wire Line
	2350 1650 2550 1650
Wire Wire Line
	2550 1650 2550 2350
Wire Wire Line
	2550 2350 3800 2350
Wire Wire Line
	3800 2350 3800 2200
Wire Wire Line
	2350 1750 2600 1750
Wire Wire Line
	2600 1750 2600 2400
Wire Wire Line
	2600 2400 3900 2400
Wire Wire Line
	3900 2400 3900 2200
Wire Wire Line
	2350 1850 2650 1850
Wire Wire Line
	2650 1850 2650 2450
Wire Wire Line
	2650 2450 4000 2450
Wire Wire Line
	4000 2450 4000 2200
Wire Wire Line
	2350 1950 2700 1950
Wire Wire Line
	2700 1950 2700 2500
Wire Wire Line
	2700 2500 4100 2500
Wire Wire Line
	4100 2500 4100 2200
Wire Wire Line
	2350 2050 2750 2050
Wire Wire Line
	2750 2050 2750 2550
Wire Wire Line
	2750 2550 4200 2550
Wire Wire Line
	4200 2550 4200 2200
Text GLabel 1350 1350 0    50   Input ~ 0
BUS_D0
Text GLabel 1350 1450 0    50   Input ~ 0
BUS_D1
Text GLabel 1350 1550 0    50   Input ~ 0
BUS_D2
Text GLabel 1350 1650 0    50   Input ~ 0
BUS_D3
Text GLabel 1350 1750 0    50   Input ~ 0
BUS_D4
Text GLabel 1350 1850 0    50   Input ~ 0
BUS_D5
Text GLabel 1350 1950 0    50   Input ~ 0
BUS_D6
Text GLabel 1350 2050 0    50   Input ~ 0
BUS_D7
Text GLabel 1350 2250 0    50   Input ~ 0
CLK
Text GLabel 1350 2350 0    50   Input ~ 0
~SPW
Text GLabel 4600 2200 2    50   Input ~ 0
~SPE
Wire Wire Line
	4500 2200 4600 2200
$Comp
L power:GND #PWR0101
U 1 1 61BDD0F2
P 4800 1700
F 0 "#PWR0101" H 4800 1450 50  0001 C CNN
F 1 "GND" V 4805 1572 50  0000 R CNN
F 2 "" H 4800 1700 50  0001 C CNN
F 3 "" H 4800 1700 50  0001 C CNN
	1    4800 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61BDD330
P 1850 2650
F 0 "#PWR0102" H 1850 2400 50  0001 C CNN
F 1 "GND" H 1855 2477 50  0000 C CNN
F 2 "" H 1850 2650 50  0001 C CNN
F 3 "" H 1850 2650 50  0001 C CNN
	1    1850 2650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0103
U 1 1 61BDD6A6
P 4400 2200
F 0 "#PWR0103" H 4400 2050 50  0001 C CNN
F 1 "VCC" H 4415 2373 50  0000 C CNN
F 2 "" H 4400 2200 50  0001 C CNN
F 3 "" H 4400 2200 50  0001 C CNN
	1    4400 2200
	-1   0    0    1   
$EndComp
Text GLabel 3500 1200 1    50   Output ~ 0
BUS_D0
Text GLabel 3600 1200 1    50   Output ~ 0
BUS_D1
Text GLabel 3700 1200 1    50   Output ~ 0
BUS_D2
Text GLabel 3800 1200 1    50   Output ~ 0
BUS_D3
Text GLabel 3900 1200 1    50   Output ~ 0
BUS_D4
Text GLabel 4000 1200 1    50   Output ~ 0
BUS_D5
Text GLabel 4100 1200 1    50   Output ~ 0
BUS_D6
Text GLabel 4200 1200 1    50   Output ~ 0
BUS_D7
$Comp
L 74xx:74LS245 U408
U 1 1 61BE426D
P 9200 1700
F 0 "U408" V 9450 1000 50  0000 R CNN
F 1 "74HCT245" V 9350 1050 50  0000 R CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 9200 1700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 9200 1700 50  0001 C CNN
	1    9200 1700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7550 1350 7600 1350
Wire Wire Line
	7600 1350 7600 2200
Wire Wire Line
	7600 2200 8700 2200
Wire Wire Line
	7550 1450 7650 1450
Wire Wire Line
	7650 1450 7650 2250
Wire Wire Line
	7650 2250 8800 2250
Wire Wire Line
	8800 2250 8800 2200
Wire Wire Line
	7550 1550 7700 1550
Wire Wire Line
	7700 1550 7700 2300
Wire Wire Line
	7700 2300 8900 2300
Wire Wire Line
	8900 2300 8900 2200
Wire Wire Line
	7550 1650 7750 1650
Wire Wire Line
	7750 1650 7750 2350
Wire Wire Line
	7750 2350 9000 2350
Wire Wire Line
	9000 2350 9000 2200
Wire Wire Line
	7550 1750 7800 1750
Wire Wire Line
	7800 1750 7800 2400
Wire Wire Line
	7800 2400 9100 2400
Wire Wire Line
	9100 2400 9100 2200
Wire Wire Line
	7550 1850 7850 1850
Wire Wire Line
	7850 1850 7850 2450
Wire Wire Line
	7850 2450 9200 2450
Wire Wire Line
	9200 2450 9200 2200
Wire Wire Line
	7550 1950 7900 1950
Wire Wire Line
	7900 1950 7900 2500
Wire Wire Line
	7900 2500 9300 2500
Wire Wire Line
	9300 2500 9300 2200
Wire Wire Line
	7550 2050 7950 2050
Wire Wire Line
	7950 2050 7950 2550
Wire Wire Line
	7950 2550 9400 2550
Wire Wire Line
	9400 2550 9400 2200
Text GLabel 6550 1350 0    50   Input ~ 0
BUS_D0
Text GLabel 6550 1450 0    50   Input ~ 0
BUS_D1
Text GLabel 6550 1550 0    50   Input ~ 0
BUS_D2
Text GLabel 6550 1650 0    50   Input ~ 0
BUS_D3
Text GLabel 6550 1750 0    50   Input ~ 0
BUS_D4
Text GLabel 6550 1850 0    50   Input ~ 0
BUS_D5
Text GLabel 6550 1950 0    50   Input ~ 0
BUS_D6
Text GLabel 6550 2050 0    50   Input ~ 0
BUS_D7
Text GLabel 6550 2250 0    50   Input ~ 0
CLK
Text GLabel 6550 2350 0    50   Input ~ 0
~IVW
Text GLabel 9800 2200 2    50   Input ~ 0
~IVE
Wire Wire Line
	9700 2200 9800 2200
$Comp
L power:GND #PWR0104
U 1 1 61BE429E
P 10000 1700
F 0 "#PWR0104" H 10000 1450 50  0001 C CNN
F 1 "GND" V 10005 1572 50  0000 R CNN
F 2 "" H 10000 1700 50  0001 C CNN
F 3 "" H 10000 1700 50  0001 C CNN
	1    10000 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 61BE42A4
P 7050 2650
F 0 "#PWR0105" H 7050 2400 50  0001 C CNN
F 1 "GND" H 7055 2477 50  0000 C CNN
F 2 "" H 7050 2650 50  0001 C CNN
F 3 "" H 7050 2650 50  0001 C CNN
	1    7050 2650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0106
U 1 1 61BE42AA
P 9600 2200
F 0 "#PWR0106" H 9600 2050 50  0001 C CNN
F 1 "VCC" H 9615 2373 50  0000 C CNN
F 2 "" H 9600 2200 50  0001 C CNN
F 3 "" H 9600 2200 50  0001 C CNN
	1    9600 2200
	-1   0    0    1   
$EndComp
Text GLabel 8700 1200 1    50   Output ~ 0
BUS_D0
Text GLabel 8800 1200 1    50   Output ~ 0
BUS_D1
Text GLabel 8900 1200 1    50   Output ~ 0
BUS_D2
Text GLabel 9000 1200 1    50   Output ~ 0
BUS_D3
Text GLabel 9100 1200 1    50   Output ~ 0
BUS_D4
Text GLabel 9200 1200 1    50   Output ~ 0
BUS_D5
Text GLabel 9300 1200 1    50   Output ~ 0
BUS_D6
Text GLabel 9400 1200 1    50   Output ~ 0
BUS_D7
Text GLabel 1250 2850 0    50   Input ~ 0
~ucSR
Text Notes 850  1250 0    50   Italic 10
Stack Pointer
Text Notes 5900 1250 0    50   Italic 10
Immediate Value
Text GLabel 3500 2600 3    50   Output ~ 0
sp_0
Text GLabel 3600 2600 3    50   Output ~ 0
sp_1
Text GLabel 3700 2600 3    50   Output ~ 0
sp_2
Text GLabel 3800 2600 3    50   Output ~ 0
sp_3
Text GLabel 3900 2600 3    50   Output ~ 0
sp_4
Text GLabel 4000 2600 3    50   Output ~ 0
sp_5
Text GLabel 4100 2600 3    50   Output ~ 0
sp_6
Text GLabel 4200 2600 3    50   Output ~ 0
sp_7
Wire Wire Line
	4200 2600 4200 2550
Connection ~ 4200 2550
Wire Wire Line
	4100 2600 4100 2500
Connection ~ 4100 2500
Wire Wire Line
	4000 2600 4000 2450
Connection ~ 4000 2450
Wire Wire Line
	3900 2600 3900 2400
Connection ~ 3900 2400
Wire Wire Line
	3800 2600 3800 2350
Connection ~ 3800 2350
Wire Wire Line
	3700 2600 3700 2300
Connection ~ 3700 2300
Wire Wire Line
	3600 2600 3600 2250
Connection ~ 3600 2250
Wire Wire Line
	3500 2600 3500 2200
Connection ~ 3500 2200
Text GLabel 9400 2600 3    50   Output ~ 0
iv_7
Text GLabel 8700 2600 3    50   Output ~ 0
iv_0
Text GLabel 8800 2600 3    50   Output ~ 0
iv_1
Text GLabel 8900 2600 3    50   Output ~ 0
iv_2
Text GLabel 9000 2600 3    50   Output ~ 0
iv_3
Text GLabel 9100 2600 3    50   Output ~ 0
iv_4
Text GLabel 9200 2600 3    50   Output ~ 0
iv_5
Text GLabel 9300 2600 3    50   Output ~ 0
iv_6
Wire Wire Line
	9400 2550 9400 2600
Connection ~ 9400 2550
Wire Wire Line
	9300 2500 9300 2600
Connection ~ 9300 2500
Wire Wire Line
	9200 2450 9200 2600
Connection ~ 9200 2450
Wire Wire Line
	9100 2400 9100 2600
Connection ~ 9100 2400
Wire Wire Line
	9000 2350 9000 2600
Connection ~ 9000 2350
Wire Wire Line
	8900 2300 8900 2600
Connection ~ 8900 2300
Wire Wire Line
	8800 2250 8800 2600
Connection ~ 8800 2250
Wire Wire Line
	8700 2200 8700 2600
Connection ~ 8700 2200
$Comp
L 74xx:74LS161 U403
U 1 1 61C30604
P 1850 4400
F 0 "U403" H 2100 5200 50  0000 C CNN
F 1 "74HCT161" H 2100 5100 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 1850 4400 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS161" H 1850 4400 50  0001 C CNN
	1    1850 4400
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS161 U404
U 1 1 61C311A2
P 1850 6800
F 0 "U404" H 2100 7600 50  0000 C CNN
F 1 "74HCT161" H 2100 7500 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 1850 6800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS161" H 1850 6800 50  0001 C CNN
	1    1850 6800
	1    0    0    -1  
$EndComp
Text Notes 700  3750 0    50   Italic 10
Program Counter
Text GLabel 1350 3900 0    50   Input ~ 0
BUS_D0
Text GLabel 1350 4000 0    50   Input ~ 0
BUS_D1
Text GLabel 1350 4100 0    50   Input ~ 0
BUS_D2
Text GLabel 1350 4200 0    50   Input ~ 0
BUS_D3
Text GLabel 1350 6300 0    50   Input ~ 0
BUS_D4
Text GLabel 1350 6400 0    50   Input ~ 0
BUS_D5
Text GLabel 1350 6500 0    50   Input ~ 0
BUS_D6
Text GLabel 1350 6600 0    50   Input ~ 0
BUS_D7
$Comp
L 74xx:74LS245 U406
U 1 1 61C36A43
P 4000 4250
F 0 "U406" V 4250 3550 50  0000 R CNN
F 1 "74HCT245" V 4150 3600 50  0000 R CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 4000 4250 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 4000 4250 50  0001 C CNN
	1    4000 4250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 61C36A49
P 4800 4250
F 0 "#PWR0107" H 4800 4000 50  0001 C CNN
F 1 "GND" V 4805 4122 50  0000 R CNN
F 2 "" H 4800 4250 50  0001 C CNN
F 3 "" H 4800 4250 50  0001 C CNN
	1    4800 4250
	0    -1   -1   0   
$EndComp
Text GLabel 3500 3750 1    50   Output ~ 0
BUS_D0
Text GLabel 3600 3750 1    50   Output ~ 0
BUS_D1
Text GLabel 3700 3750 1    50   Output ~ 0
BUS_D2
Text GLabel 3800 3750 1    50   Output ~ 0
BUS_D3
Text GLabel 3900 3750 1    50   Output ~ 0
BUS_D4
Text GLabel 4000 3750 1    50   Output ~ 0
BUS_D5
Text GLabel 4100 3750 1    50   Output ~ 0
BUS_D6
Text GLabel 4200 3750 1    50   Output ~ 0
BUS_D7
Wire Wire Line
	2350 3900 2400 3900
Wire Wire Line
	2400 3900 2400 4750
Wire Wire Line
	2400 4750 3000 4750
Wire Wire Line
	2350 4000 2450 4000
Wire Wire Line
	2450 4000 2450 4800
Wire Wire Line
	2450 4800 3100 4800
Wire Wire Line
	3600 4800 3600 4750
Wire Wire Line
	2350 4100 2500 4100
Wire Wire Line
	2500 4100 2500 4850
Wire Wire Line
	2500 4850 3200 4850
Wire Wire Line
	3700 4850 3700 4750
Wire Wire Line
	2350 4200 2550 4200
Wire Wire Line
	2550 4200 2550 4900
Wire Wire Line
	2550 4900 3300 4900
Wire Wire Line
	3800 4900 3800 4750
Wire Wire Line
	2350 6300 3900 6300
Wire Wire Line
	3900 6300 3900 5200
Wire Wire Line
	2350 6400 4000 6400
Wire Wire Line
	4000 6400 4000 5250
Wire Wire Line
	2350 6500 4100 6500
Wire Wire Line
	2350 6600 4200 6600
Text GLabel 1350 4900 0    50   Input ~ 0
~RST
Text GLabel 1350 7300 0    50   Input ~ 0
~RST
$Comp
L power:GND #PWR0108
U 1 1 61C4D2AA
P 1850 7600
F 0 "#PWR0108" H 1850 7350 50  0001 C CNN
F 1 "GND" H 1855 7427 50  0000 C CNN
F 2 "" H 1850 7600 50  0001 C CNN
F 3 "" H 1850 7600 50  0001 C CNN
	1    1850 7600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 61C4D67F
P 1850 5200
F 0 "#PWR0109" H 1850 4950 50  0001 C CNN
F 1 "GND" H 1855 5027 50  0000 C CNN
F 2 "" H 1850 5200 50  0001 C CNN
F 3 "" H 1850 5200 50  0001 C CNN
	1    1850 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4400 2350 5500
Wire Wire Line
	2350 5500 800  5500
Wire Wire Line
	800  5500 800  7000
Wire Wire Line
	800  7000 1350 7000
Text GLabel 1350 7100 0    50   Input ~ 0
CLK
Text GLabel 1350 4700 0    50   Input ~ 0
CLK
NoConn ~ 2350 6800
Text GLabel 1350 4400 0    50   Input ~ 0
~PCW
Text GLabel 1350 6800 0    50   Input ~ 0
~PCW
Text GLabel 1350 4500 0    50   Input ~ 0
PCA
Text GLabel 1350 6900 0    50   Input ~ 0
PCA
$Comp
L power:VCC #PWR0110
U 1 1 61C5847B
P 1350 4600
F 0 "#PWR0110" H 1350 4450 50  0001 C CNN
F 1 "VCC" V 1365 4727 50  0000 L CNN
F 2 "" H 1350 4600 50  0001 C CNN
F 3 "" H 1350 4600 50  0001 C CNN
	1    1350 4600
	0    -1   -1   0   
$EndComp
Text GLabel 3700 5400 3    50   Output ~ 0
pc_7
Text GLabel 3600 5400 3    50   Output ~ 0
pc_6
Text GLabel 3500 5400 3    50   Output ~ 0
pc_5
Text GLabel 3400 5400 3    50   Output ~ 0
pc_4
Text GLabel 3300 5400 3    50   Output ~ 0
pc_3
Text GLabel 3200 5400 3    50   Output ~ 0
pc_2
Text GLabel 3100 5400 3    50   Output ~ 0
pc_1
Text GLabel 3000 5400 3    50   Output ~ 0
pc_0
Wire Wire Line
	3700 5400 3700 5350
Wire Wire Line
	3700 5350 4200 5350
Connection ~ 4200 5350
Wire Wire Line
	4200 5350 4200 6600
Wire Wire Line
	4200 4750 4200 5350
Wire Wire Line
	4100 6500 4100 5300
Wire Wire Line
	3600 5400 3600 5300
Wire Wire Line
	3600 5300 4100 5300
Connection ~ 4100 5300
Wire Wire Line
	4100 5300 4100 4750
Wire Wire Line
	3500 5400 3500 5250
Wire Wire Line
	3500 5250 4000 5250
Connection ~ 4000 5250
Wire Wire Line
	4000 5250 4000 4750
Wire Wire Line
	3400 5400 3400 5200
Wire Wire Line
	3400 5200 3900 5200
Connection ~ 3900 5200
Wire Wire Line
	3900 5200 3900 4750
Wire Wire Line
	3300 5400 3300 4900
Connection ~ 3300 4900
Wire Wire Line
	3300 4900 3800 4900
Wire Wire Line
	3200 5400 3200 4850
Connection ~ 3200 4850
Wire Wire Line
	3200 4850 3700 4850
Wire Wire Line
	3100 5400 3100 4800
Connection ~ 3100 4800
Wire Wire Line
	3100 4800 3600 4800
Wire Wire Line
	3000 5400 3000 4750
Connection ~ 3000 4750
Wire Wire Line
	3000 4750 3500 4750
$Comp
L power:VCC #PWR0111
U 1 1 61C91C62
P 4400 4750
F 0 "#PWR0111" H 4400 4600 50  0001 C CNN
F 1 "VCC" H 4415 4923 50  0000 C CNN
F 2 "" H 4400 4750 50  0001 C CNN
F 3 "" H 4400 4750 50  0001 C CNN
	1    4400 4750
	-1   0    0    1   
$EndComp
Text GLabel 4600 4750 2    50   Input ~ 0
~PCE
Wire Wire Line
	4600 4750 4500 4750
$Comp
L Device:LED D401
U 1 1 61C978A0
P 7350 3300
F 0 "D401" V 7389 3182 50  0000 R CNN
F 1 "RED" V 7298 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7350 3300 50  0001 C CNN
F 3 "~" H 7350 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7350 3300 50  0001 C CNN "Mouser"
	1    7350 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D404
U 1 1 61C97DAA
P 7750 3300
F 0 "D404" V 7789 3182 50  0000 R CNN
F 1 "RED" V 7698 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7750 3300 50  0001 C CNN
F 3 "~" H 7750 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7750 3300 50  0001 C CNN "Mouser"
	1    7750 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D407
U 1 1 61C97F13
P 8150 3300
F 0 "D407" V 8189 3182 50  0000 R CNN
F 1 "RED" V 8098 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8150 3300 50  0001 C CNN
F 3 "~" H 8150 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8150 3300 50  0001 C CNN "Mouser"
	1    8150 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D410
U 1 1 61C98058
P 8550 3300
F 0 "D410" V 8589 3182 50  0000 R CNN
F 1 "RED" V 8498 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8550 3300 50  0001 C CNN
F 3 "~" H 8550 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8550 3300 50  0001 C CNN "Mouser"
	1    8550 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D413
U 1 1 61C9819D
P 8950 3300
F 0 "D413" V 8989 3182 50  0000 R CNN
F 1 "RED" V 8898 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8950 3300 50  0001 C CNN
F 3 "~" H 8950 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8950 3300 50  0001 C CNN "Mouser"
	1    8950 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D416
U 1 1 61C982E2
P 9350 3300
F 0 "D416" V 9389 3182 50  0000 R CNN
F 1 "RED" V 9298 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 9350 3300 50  0001 C CNN
F 3 "~" H 9350 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 9350 3300 50  0001 C CNN "Mouser"
	1    9350 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D419
U 1 1 61C98445
P 9750 3300
F 0 "D419" V 9789 3182 50  0000 R CNN
F 1 "RED" V 9698 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 9750 3300 50  0001 C CNN
F 3 "~" H 9750 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 9750 3300 50  0001 C CNN "Mouser"
	1    9750 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D422
U 1 1 61C9858A
P 10150 3300
F 0 "D422" V 10189 3182 50  0000 R CNN
F 1 "RED" V 10098 3182 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 10150 3300 50  0001 C CNN
F 3 "~" H 10150 3300 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 10150 3300 50  0001 C CNN "Mouser"
	1    10150 3300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 61C9C350
P 9100 4050
F 0 "#PWR0119" H 9100 3800 50  0001 C CNN
F 1 "GND" H 9105 3877 50  0000 C CNN
F 2 "" H 9100 4050 50  0001 C CNN
F 3 "" H 9100 4050 50  0001 C CNN
	1    9100 4050
	1    0    0    -1  
$EndComp
Text GLabel 10150 3150 1    50   Input ~ 0
sp_0
Text GLabel 9750 3150 1    50   Input ~ 0
sp_1
Text GLabel 9350 3150 1    50   Input ~ 0
sp_2
Text GLabel 8950 3150 1    50   Input ~ 0
sp_3
Text GLabel 8550 3150 1    50   Input ~ 0
sp_4
Text GLabel 8150 3150 1    50   Input ~ 0
sp_5
Text GLabel 7750 3150 1    50   Input ~ 0
sp_6
Text GLabel 7350 3150 1    50   Input ~ 0
sp_7
$Comp
L Device:LED D402
U 1 1 61CD11BE
P 7350 4650
F 0 "D402" V 7389 4532 50  0000 R CNN
F 1 "RED" V 7298 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7350 4650 50  0001 C CNN
F 3 "~" H 7350 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7350 4650 50  0001 C CNN "Mouser"
	1    7350 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D405
U 1 1 61CD11C5
P 7750 4650
F 0 "D405" V 7789 4532 50  0000 R CNN
F 1 "RED" V 7698 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7750 4650 50  0001 C CNN
F 3 "~" H 7750 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7750 4650 50  0001 C CNN "Mouser"
	1    7750 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D408
U 1 1 61CD11CC
P 8150 4650
F 0 "D408" V 8189 4532 50  0000 R CNN
F 1 "RED" V 8098 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8150 4650 50  0001 C CNN
F 3 "~" H 8150 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8150 4650 50  0001 C CNN "Mouser"
	1    8150 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D411
U 1 1 61CD11D3
P 8550 4650
F 0 "D411" V 8589 4532 50  0000 R CNN
F 1 "RED" V 8498 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8550 4650 50  0001 C CNN
F 3 "~" H 8550 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8550 4650 50  0001 C CNN "Mouser"
	1    8550 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D414
U 1 1 61CD11DA
P 8950 4650
F 0 "D414" V 8989 4532 50  0000 R CNN
F 1 "RED" V 8898 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8950 4650 50  0001 C CNN
F 3 "~" H 8950 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8950 4650 50  0001 C CNN "Mouser"
	1    8950 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D417
U 1 1 61CD11E1
P 9350 4650
F 0 "D417" V 9389 4532 50  0000 R CNN
F 1 "RED" V 9298 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 9350 4650 50  0001 C CNN
F 3 "~" H 9350 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 9350 4650 50  0001 C CNN "Mouser"
	1    9350 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D420
U 1 1 61CD11E8
P 9750 4650
F 0 "D420" V 9789 4532 50  0000 R CNN
F 1 "RED" V 9698 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 9750 4650 50  0001 C CNN
F 3 "~" H 9750 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 9750 4650 50  0001 C CNN "Mouser"
	1    9750 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D423
U 1 1 61CD11EF
P 10150 4650
F 0 "D423" V 10189 4532 50  0000 R CNN
F 1 "RED" V 10098 4532 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 10150 4650 50  0001 C CNN
F 3 "~" H 10150 4650 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 10150 4650 50  0001 C CNN "Mouser"
	1    10150 4650
	0    -1   -1   0   
$EndComp
Text GLabel 10150 4500 1    50   Input ~ 0
iv_0
Text GLabel 9750 4500 1    50   Input ~ 0
iv_1
Text GLabel 9350 4500 1    50   Input ~ 0
iv_2
Text GLabel 8950 4500 1    50   Input ~ 0
iv_3
Text GLabel 8550 4500 1    50   Input ~ 0
iv_4
Text GLabel 8150 4500 1    50   Input ~ 0
iv_5
Text GLabel 7750 4500 1    50   Input ~ 0
iv_6
Text GLabel 7350 4500 1    50   Input ~ 0
iv_7
$Comp
L Device:LED D403
U 1 1 61CE262D
P 4750 5700
F 0 "D403" V 4789 5582 50  0000 R CNN
F 1 "RED" V 4698 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 4750 5700 50  0001 C CNN
F 3 "~" H 4750 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 4750 5700 50  0001 C CNN "Mouser"
	1    4750 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D406
U 1 1 61CE2634
P 5150 5700
F 0 "D406" V 5189 5582 50  0000 R CNN
F 1 "RED" V 5098 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5150 5700 50  0001 C CNN
F 3 "~" H 5150 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 5150 5700 50  0001 C CNN "Mouser"
	1    5150 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D409
U 1 1 61CE263B
P 5550 5700
F 0 "D409" V 5589 5582 50  0000 R CNN
F 1 "RED" V 5498 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5550 5700 50  0001 C CNN
F 3 "~" H 5550 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 5550 5700 50  0001 C CNN "Mouser"
	1    5550 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D412
U 1 1 61CE2642
P 5950 5700
F 0 "D412" V 5989 5582 50  0000 R CNN
F 1 "RED" V 5898 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5950 5700 50  0001 C CNN
F 3 "~" H 5950 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 5950 5700 50  0001 C CNN "Mouser"
	1    5950 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D415
U 1 1 61CE2649
P 6350 5700
F 0 "D415" V 6389 5582 50  0000 R CNN
F 1 "RED" V 6298 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 6350 5700 50  0001 C CNN
F 3 "~" H 6350 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 6350 5700 50  0001 C CNN "Mouser"
	1    6350 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D418
U 1 1 61CE2650
P 6750 5700
F 0 "D418" V 6789 5582 50  0000 R CNN
F 1 "RED" V 6698 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 6750 5700 50  0001 C CNN
F 3 "~" H 6750 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 6750 5700 50  0001 C CNN "Mouser"
	1    6750 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D421
U 1 1 61CE2657
P 7150 5700
F 0 "D421" V 7189 5582 50  0000 R CNN
F 1 "RED" V 7098 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7150 5700 50  0001 C CNN
F 3 "~" H 7150 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7150 5700 50  0001 C CNN "Mouser"
	1    7150 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D424
U 1 1 61CE265E
P 7550 5700
F 0 "D424" V 7589 5582 50  0000 R CNN
F 1 "RED" V 7498 5582 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7550 5700 50  0001 C CNN
F 3 "~" H 7550 5700 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7550 5700 50  0001 C CNN "Mouser"
	1    7550 5700
	0    -1   -1   0   
$EndComp
Text GLabel 7550 5550 1    50   Input ~ 0
pc_0
Text GLabel 7150 5550 1    50   Input ~ 0
pc_1
Text GLabel 6750 5550 1    50   Input ~ 0
pc_2
Text GLabel 6350 5550 1    50   Input ~ 0
pc_3
Text GLabel 5950 5550 1    50   Input ~ 0
pc_4
Text GLabel 5550 5550 1    50   Input ~ 0
pc_5
Text GLabel 5150 5550 1    50   Input ~ 0
pc_6
Text GLabel 4750 5550 1    50   Input ~ 0
pc_7
$Comp
L Device:C_Small C402
U 1 1 61D13B27
P 1450 850
F 0 "C402" H 1542 896 50  0000 L CNN
F 1 "0.1µF" H 1542 805 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1450 850 50  0001 C CNN
F 3 "~" H 1450 850 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1450 850 50  0001 C CNN "Mouser"
	1    1450 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 61D152AF
P 1450 950
F 0 "#PWR0137" H 1450 700 50  0001 C CNN
F 1 "GND" H 1455 777 50  0000 C CNN
F 2 "" H 1450 950 50  0001 C CNN
F 3 "" H 1450 950 50  0001 C CNN
	1    1450 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 1050 1850 750 
Wire Wire Line
	1850 750  1450 750 
$Comp
L power:VCC #PWR0138
U 1 1 61D2334A
P 1850 750
F 0 "#PWR0138" H 1850 600 50  0001 C CNN
F 1 "VCC" H 1865 923 50  0000 C CNN
F 2 "" H 1850 750 50  0001 C CNN
F 3 "" H 1850 750 50  0001 C CNN
	1    1850 750 
	1    0    0    -1  
$EndComp
Connection ~ 1850 750 
$Comp
L Device:C_Small C403
U 1 1 61D23435
P 1450 3350
F 0 "C403" H 1542 3396 50  0000 L CNN
F 1 "0.1µF" H 1542 3305 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1450 3350 50  0001 C CNN
F 3 "~" H 1450 3350 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1450 3350 50  0001 C CNN "Mouser"
	1    1450 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 61D23A6B
P 1450 3450
F 0 "#PWR0139" H 1450 3200 50  0001 C CNN
F 1 "GND" H 1455 3277 50  0000 C CNN
F 2 "" H 1450 3450 50  0001 C CNN
F 3 "" H 1450 3450 50  0001 C CNN
	1    1450 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 3600 1850 3250
Wire Wire Line
	1850 3250 1450 3250
$Comp
L power:VCC #PWR0140
U 1 1 61D324B1
P 1850 3250
F 0 "#PWR0140" H 1850 3100 50  0001 C CNN
F 1 "VCC" H 1865 3423 50  0000 C CNN
F 2 "" H 1850 3250 50  0001 C CNN
F 3 "" H 1850 3250 50  0001 C CNN
	1    1850 3250
	1    0    0    -1  
$EndComp
Connection ~ 1850 3250
$Comp
L Device:C_Small C404
U 1 1 61D33AF8
P 1450 5850
F 0 "C404" H 1542 5896 50  0000 L CNN
F 1 "0.1µF" H 1542 5805 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1450 5850 50  0001 C CNN
F 3 "~" H 1450 5850 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1450 5850 50  0001 C CNN "Mouser"
	1    1450 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0141
U 1 1 61D33AFE
P 1450 5950
F 0 "#PWR0141" H 1450 5700 50  0001 C CNN
F 1 "GND" H 1455 5777 50  0000 C CNN
F 2 "" H 1450 5950 50  0001 C CNN
F 3 "" H 1450 5950 50  0001 C CNN
	1    1450 5950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0142
U 1 1 61D33B06
P 1850 5750
F 0 "#PWR0142" H 1850 5600 50  0001 C CNN
F 1 "VCC" H 1865 5923 50  0000 C CNN
F 2 "" H 1850 5750 50  0001 C CNN
F 3 "" H 1850 5750 50  0001 C CNN
	1    1850 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 6000 1850 5750
Wire Wire Line
	1850 5750 1450 5750
Connection ~ 1850 5750
$Comp
L Device:C_Small C408
U 1 1 61D5F9CC
P 6650 850
F 0 "C408" H 6742 896 50  0000 L CNN
F 1 "0.1µF" H 6742 805 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 6650 850 50  0001 C CNN
F 3 "~" H 6650 850 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 6650 850 50  0001 C CNN "Mouser"
	1    6650 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 61D5F9D2
P 6650 950
F 0 "#PWR0143" H 6650 700 50  0001 C CNN
F 1 "GND" H 6655 777 50  0000 C CNN
F 2 "" H 6650 950 50  0001 C CNN
F 3 "" H 6650 950 50  0001 C CNN
	1    6650 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1050 7050 750 
Wire Wire Line
	7050 750  6650 750 
$Comp
L power:VCC #PWR0144
U 1 1 61D5F9DA
P 7050 750
F 0 "#PWR0144" H 7050 600 50  0001 C CNN
F 1 "VCC" H 7065 923 50  0000 C CNN
F 2 "" H 7050 750 50  0001 C CNN
F 3 "" H 7050 750 50  0001 C CNN
	1    7050 750 
	1    0    0    -1  
$EndComp
Connection ~ 7050 750 
$Comp
L Device:C_Small C405
U 1 1 61D6F6C2
P 3050 1850
F 0 "C405" H 3142 1896 50  0000 L CNN
F 1 "0.1µF" H 3142 1805 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 3050 1850 50  0001 C CNN
F 3 "~" H 3050 1850 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 3050 1850 50  0001 C CNN "Mouser"
	1    3050 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0147
U 1 1 61D70570
P 3050 1950
F 0 "#PWR0147" H 3050 1700 50  0001 C CNN
F 1 "GND" H 3055 1777 50  0000 C CNN
F 2 "" H 3050 1950 50  0001 C CNN
F 3 "" H 3050 1950 50  0001 C CNN
	1    3050 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1700 3050 1700
Wire Wire Line
	3050 1700 3050 1750
$Comp
L power:VCC #PWR0148
U 1 1 61D78A34
P 3050 1700
F 0 "#PWR0148" H 3050 1550 50  0001 C CNN
F 1 "VCC" H 3065 1873 50  0000 C CNN
F 2 "" H 3050 1700 50  0001 C CNN
F 3 "" H 3050 1700 50  0001 C CNN
	1    3050 1700
	1    0    0    -1  
$EndComp
Connection ~ 3050 1700
$Comp
L Device:C_Small C406
U 1 1 61D79C8B
P 3050 4400
F 0 "C406" H 3142 4446 50  0000 L CNN
F 1 "0.1µF" H 3142 4355 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 3050 4400 50  0001 C CNN
F 3 "~" H 3050 4400 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 3050 4400 50  0001 C CNN "Mouser"
	1    3050 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0149
U 1 1 61D79C91
P 3050 4500
F 0 "#PWR0149" H 3050 4250 50  0001 C CNN
F 1 "GND" H 3055 4327 50  0000 C CNN
F 2 "" H 3050 4500 50  0001 C CNN
F 3 "" H 3050 4500 50  0001 C CNN
	1    3050 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4250 3050 4250
Wire Wire Line
	3050 4250 3050 4300
$Comp
L power:VCC #PWR0150
U 1 1 61D79C99
P 3050 4250
F 0 "#PWR0150" H 3050 4100 50  0001 C CNN
F 1 "VCC" H 3065 4423 50  0000 C CNN
F 2 "" H 3050 4250 50  0001 C CNN
F 3 "" H 3050 4250 50  0001 C CNN
	1    3050 4250
	1    0    0    -1  
$EndComp
Connection ~ 3050 4250
$Comp
L Device:C_Small C409
U 1 1 61D82B7C
P 8250 1850
F 0 "C409" H 8342 1896 50  0000 L CNN
F 1 "0.1µF" H 8342 1805 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 8250 1850 50  0001 C CNN
F 3 "~" H 8250 1850 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 8250 1850 50  0001 C CNN "Mouser"
	1    8250 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0151
U 1 1 61D82B82
P 8250 1950
F 0 "#PWR0151" H 8250 1700 50  0001 C CNN
F 1 "GND" H 8255 1777 50  0000 C CNN
F 2 "" H 8250 1950 50  0001 C CNN
F 3 "" H 8250 1950 50  0001 C CNN
	1    8250 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1700 8250 1700
Wire Wire Line
	8250 1700 8250 1750
$Comp
L power:VCC #PWR0152
U 1 1 61D82B8A
P 8250 1700
F 0 "#PWR0152" H 8250 1550 50  0001 C CNN
F 1 "VCC" H 8265 1873 50  0000 C CNN
F 2 "" H 8250 1700 50  0001 C CNN
F 3 "" H 8250 1700 50  0001 C CNN
	1    8250 1700
	1    0    0    -1  
$EndComp
Connection ~ 8250 1700
$Comp
L Device:R_Pack08 RN402
U 1 1 652C6766
P 8800 3850
F 0 "RN402" H 9188 3896 50  0000 L CNN
F 1 "330 x 8" H 9188 3805 50  0000 L CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" V 9275 3850 50  0001 C CNN
F 3 "~" H 8800 3850 50  0001 C CNN
F 4 " 652-4116R-1-330" H 8800 3850 50  0001 C CNN "Mouser"
	1    8800 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 4050 8500 4050
Wire Wire Line
	8500 4050 8600 4050
Connection ~ 8500 4050
Wire Wire Line
	8600 4050 8700 4050
Connection ~ 8600 4050
Wire Wire Line
	8700 4050 8800 4050
Connection ~ 8700 4050
Wire Wire Line
	8800 4050 8900 4050
Connection ~ 8800 4050
Wire Wire Line
	8900 4050 9000 4050
Connection ~ 8900 4050
Wire Wire Line
	9000 4050 9100 4050
Connection ~ 9000 4050
Connection ~ 9100 4050
Wire Wire Line
	10150 3650 9100 3650
Wire Wire Line
	7350 3650 8400 3650
Wire Wire Line
	10150 3450 10150 3650
Wire Wire Line
	9750 3450 9750 3600
Wire Wire Line
	9750 3600 9000 3600
Wire Wire Line
	9000 3600 9000 3650
Wire Wire Line
	9350 3450 9350 3550
Wire Wire Line
	9350 3550 8900 3550
Wire Wire Line
	8900 3550 8900 3650
Wire Wire Line
	8950 3450 8950 3500
Wire Wire Line
	8950 3500 8800 3500
Wire Wire Line
	8800 3500 8800 3650
Wire Wire Line
	8550 3450 8550 3500
Wire Wire Line
	8550 3500 8700 3500
Wire Wire Line
	8700 3500 8700 3650
Wire Wire Line
	8150 3450 8150 3550
Wire Wire Line
	8150 3550 8600 3550
Wire Wire Line
	8600 3550 8600 3650
Wire Wire Line
	7750 3450 7750 3600
Wire Wire Line
	7750 3600 8500 3600
Wire Wire Line
	8500 3600 8500 3650
Wire Wire Line
	7350 3450 7350 3650
$Comp
L power:GND #PWR0112
U 1 1 654213BE
P 9100 5400
F 0 "#PWR0112" H 9100 5150 50  0001 C CNN
F 1 "GND" H 9105 5227 50  0000 C CNN
F 2 "" H 9100 5400 50  0001 C CNN
F 3 "" H 9100 5400 50  0001 C CNN
	1    9100 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Pack08 RN403
U 1 1 654213C4
P 8800 5200
F 0 "RN403" H 9188 5246 50  0000 L CNN
F 1 "330 x 8" H 9188 5155 50  0000 L CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" V 9275 5200 50  0001 C CNN
F 3 "~" H 8800 5200 50  0001 C CNN
F 4 "652-4116R-1-330" H 8800 5200 50  0001 C CNN "Mouser"
	1    8800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 5400 8500 5400
Wire Wire Line
	8500 5400 8600 5400
Connection ~ 8500 5400
Wire Wire Line
	8600 5400 8700 5400
Connection ~ 8600 5400
Wire Wire Line
	8700 5400 8800 5400
Connection ~ 8700 5400
Wire Wire Line
	8800 5400 8900 5400
Connection ~ 8800 5400
Wire Wire Line
	8900 5400 9000 5400
Connection ~ 8900 5400
Wire Wire Line
	9000 5400 9100 5400
Connection ~ 9000 5400
Connection ~ 9100 5400
Wire Wire Line
	10150 5000 9100 5000
Wire Wire Line
	7350 5000 8400 5000
Wire Wire Line
	10150 4800 10150 5000
Wire Wire Line
	9750 4800 9750 4950
Wire Wire Line
	9750 4950 9000 4950
Wire Wire Line
	9000 4950 9000 5000
Wire Wire Line
	9350 4800 9350 4900
Wire Wire Line
	9350 4900 8900 4900
Wire Wire Line
	8900 4900 8900 5000
Wire Wire Line
	8950 4800 8950 4850
Wire Wire Line
	8950 4850 8800 4850
Wire Wire Line
	8800 4850 8800 5000
Wire Wire Line
	8550 4800 8550 4850
Wire Wire Line
	8550 4850 8700 4850
Wire Wire Line
	8700 4850 8700 5000
Wire Wire Line
	8150 4800 8150 4900
Wire Wire Line
	8150 4900 8600 4900
Wire Wire Line
	8600 4900 8600 5000
Wire Wire Line
	7750 4800 7750 4950
Wire Wire Line
	7750 4950 8500 4950
Wire Wire Line
	8500 4950 8500 5000
Wire Wire Line
	7350 4800 7350 5000
$Comp
L power:GND #PWR0113
U 1 1 654563E4
P 6500 6450
F 0 "#PWR0113" H 6500 6200 50  0001 C CNN
F 1 "GND" H 6505 6277 50  0000 C CNN
F 2 "" H 6500 6450 50  0001 C CNN
F 3 "" H 6500 6450 50  0001 C CNN
	1    6500 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Pack08 RN401
U 1 1 654563EA
P 6200 6250
F 0 "RN401" H 6588 6296 50  0000 L CNN
F 1 "330 x 8" H 6588 6205 50  0000 L CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" V 6675 6250 50  0001 C CNN
F 3 "~" H 6200 6250 50  0001 C CNN
F 4 " 652-4116R-1-330" H 6200 6250 50  0001 C CNN "Mouser"
	1    6200 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 6450 5900 6450
Wire Wire Line
	5900 6450 6000 6450
Connection ~ 5900 6450
Wire Wire Line
	6000 6450 6100 6450
Connection ~ 6000 6450
Wire Wire Line
	6100 6450 6200 6450
Connection ~ 6100 6450
Wire Wire Line
	6200 6450 6300 6450
Connection ~ 6200 6450
Wire Wire Line
	6300 6450 6400 6450
Connection ~ 6300 6450
Wire Wire Line
	6400 6450 6500 6450
Connection ~ 6400 6450
Connection ~ 6500 6450
Wire Wire Line
	7550 6050 6500 6050
Wire Wire Line
	4750 6050 5800 6050
Wire Wire Line
	7550 5850 7550 6050
Wire Wire Line
	7150 5850 7150 6000
Wire Wire Line
	7150 6000 6400 6000
Wire Wire Line
	6400 6000 6400 6050
Wire Wire Line
	6750 5850 6750 5950
Wire Wire Line
	6750 5950 6300 5950
Wire Wire Line
	6300 5950 6300 6050
Wire Wire Line
	6350 5850 6350 5900
Wire Wire Line
	6350 5900 6200 5900
Wire Wire Line
	6200 5900 6200 6050
Wire Wire Line
	5950 5850 5950 5900
Wire Wire Line
	5950 5900 6100 5900
Wire Wire Line
	6100 5900 6100 6050
Wire Wire Line
	5550 5850 5550 5950
Wire Wire Line
	5550 5950 6000 5950
Wire Wire Line
	6000 5950 6000 6050
Wire Wire Line
	5150 5850 5150 6000
Wire Wire Line
	5150 6000 5900 6000
Wire Wire Line
	5900 6000 5900 6050
Wire Wire Line
	4750 5850 4750 6050
$Comp
L 74xx:74LS377 U407
U 1 1 66CFD9C5
P 7050 1850
F 0 "U407" H 7300 2650 50  0000 C CNN
F 1 "74HCT377" H 7300 2550 50  0000 C CNN
F 2 "" H 7050 1850 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS377" H 7050 1850 50  0001 C CNN
	1    7050 1850
	1    0    0    -1  
$EndComp
$EndSCHEMATC
