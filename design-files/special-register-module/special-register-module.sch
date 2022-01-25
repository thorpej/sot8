EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr B 17000 11000
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
L 74xx:74LS245 U408
U 1 1 61BE426D
P 14950 2000
F 0 "U408" V 15200 1300 50  0000 R CNN
F 1 "74HCT245" V 15100 1350 50  0000 R CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 14950 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 14950 2000 50  0001 C CNN
	1    14950 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13300 1650 13350 1650
Wire Wire Line
	13350 1650 13350 2500
Wire Wire Line
	13350 2500 14450 2500
Wire Wire Line
	13300 1750 13400 1750
Wire Wire Line
	13400 1750 13400 2550
Wire Wire Line
	13400 2550 14550 2550
Wire Wire Line
	14550 2550 14550 2500
Wire Wire Line
	13300 1850 13450 1850
Wire Wire Line
	13450 1850 13450 2600
Wire Wire Line
	13450 2600 14650 2600
Wire Wire Line
	14650 2600 14650 2500
Wire Wire Line
	13300 1950 13500 1950
Wire Wire Line
	13500 1950 13500 2650
Wire Wire Line
	13500 2650 14750 2650
Wire Wire Line
	14750 2650 14750 2500
Wire Wire Line
	13300 2050 13550 2050
Wire Wire Line
	13550 2050 13550 2700
Wire Wire Line
	13550 2700 14850 2700
Wire Wire Line
	14850 2700 14850 2500
Wire Wire Line
	13300 2150 13600 2150
Wire Wire Line
	13600 2150 13600 2750
Wire Wire Line
	13600 2750 14950 2750
Wire Wire Line
	14950 2750 14950 2500
Wire Wire Line
	13300 2250 13650 2250
Wire Wire Line
	13650 2250 13650 2800
Wire Wire Line
	13650 2800 15050 2800
Wire Wire Line
	15050 2800 15050 2500
Wire Wire Line
	13300 2350 13700 2350
Wire Wire Line
	13700 2350 13700 2850
Wire Wire Line
	13700 2850 15150 2850
Wire Wire Line
	15150 2850 15150 2500
Text GLabel 12300 1650 0    50   Input ~ 0
BUS_D0
Text GLabel 12300 1750 0    50   Input ~ 0
BUS_D1
Text GLabel 12300 1850 0    50   Input ~ 0
BUS_D2
Text GLabel 12300 1950 0    50   Input ~ 0
BUS_D3
Text GLabel 12300 2050 0    50   Input ~ 0
BUS_D4
Text GLabel 12300 2150 0    50   Input ~ 0
BUS_D5
Text GLabel 12300 2250 0    50   Input ~ 0
BUS_D6
Text GLabel 12300 2350 0    50   Input ~ 0
BUS_D7
Text GLabel 12300 2550 0    50   Input ~ 0
CLK
Text GLabel 12300 2650 0    50   Input ~ 0
~IVW
Text GLabel 15550 2500 2    50   Input ~ 0
~IVE
Wire Wire Line
	15450 2500 15550 2500
$Comp
L power:GND #PWR0104
U 1 1 61BE429E
P 15750 2000
F 0 "#PWR0104" H 15750 1750 50  0001 C CNN
F 1 "GND" V 15755 1872 50  0000 R CNN
F 2 "" H 15750 2000 50  0001 C CNN
F 3 "" H 15750 2000 50  0001 C CNN
	1    15750 2000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 61BE42A4
P 12800 2950
F 0 "#PWR0105" H 12800 2700 50  0001 C CNN
F 1 "GND" H 12805 2777 50  0000 C CNN
F 2 "" H 12800 2950 50  0001 C CNN
F 3 "" H 12800 2950 50  0001 C CNN
	1    12800 2950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0106
U 1 1 61BE42AA
P 15350 2500
F 0 "#PWR0106" H 15350 2350 50  0001 C CNN
F 1 "VCC" H 15365 2673 50  0000 C CNN
F 2 "" H 15350 2500 50  0001 C CNN
F 3 "" H 15350 2500 50  0001 C CNN
	1    15350 2500
	-1   0    0    1   
$EndComp
Text GLabel 14450 1500 1    50   Output ~ 0
BUS_D0
Text GLabel 14550 1500 1    50   Output ~ 0
BUS_D1
Text GLabel 14650 1500 1    50   Output ~ 0
BUS_D2
Text GLabel 14750 1500 1    50   Output ~ 0
BUS_D3
Text GLabel 14850 1500 1    50   Output ~ 0
BUS_D4
Text GLabel 14950 1500 1    50   Output ~ 0
BUS_D5
Text GLabel 15050 1500 1    50   Output ~ 0
BUS_D6
Text GLabel 15150 1500 1    50   Output ~ 0
BUS_D7
Text Notes 11650 1550 0    50   Italic 10
Immediate Value
Text GLabel 15150 2900 3    50   Output ~ 0
iv_7
Text GLabel 14450 2900 3    50   Output ~ 0
iv_0
Text GLabel 14550 2900 3    50   Output ~ 0
iv_1
Text GLabel 14650 2900 3    50   Output ~ 0
iv_2
Text GLabel 14750 2900 3    50   Output ~ 0
iv_3
Text GLabel 14850 2900 3    50   Output ~ 0
iv_4
Text GLabel 14950 2900 3    50   Output ~ 0
iv_5
Text GLabel 15050 2900 3    50   Output ~ 0
iv_6
Wire Wire Line
	15150 2850 15150 2900
Connection ~ 15150 2850
Wire Wire Line
	15050 2800 15050 2900
Connection ~ 15050 2800
Wire Wire Line
	14950 2750 14950 2900
Connection ~ 14950 2750
Wire Wire Line
	14850 2700 14850 2900
Connection ~ 14850 2700
Wire Wire Line
	14750 2650 14750 2900
Connection ~ 14750 2650
Wire Wire Line
	14650 2600 14650 2900
Connection ~ 14650 2600
Wire Wire Line
	14550 2550 14550 2900
Connection ~ 14550 2550
Wire Wire Line
	14450 2500 14450 2900
Connection ~ 14450 2500
$Comp
L 74xx:74LS161 U401
U 1 1 61C30604
P 2050 2050
F 0 "U401" H 2300 2850 50  0000 C CNN
F 1 "74HCT161" H 2300 2750 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 2050 2050 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS161" H 2050 2050 50  0001 C CNN
	1    2050 2050
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS161 U402
U 1 1 61C311A2
P 2050 4450
F 0 "U402" H 2300 5250 50  0000 C CNN
F 1 "74HCT161" H 2300 5150 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 2050 4450 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS161" H 2050 4450 50  0001 C CNN
	1    2050 4450
	1    0    0    -1  
$EndComp
Text Notes 900  1400 0    50   Italic 10
Program Counter
Text GLabel 1550 1550 0    50   Input ~ 0
BUS_D0
Text GLabel 1550 1650 0    50   Input ~ 0
BUS_D1
Text GLabel 1550 1750 0    50   Input ~ 0
BUS_D2
Text GLabel 1550 1850 0    50   Input ~ 0
BUS_D3
Text GLabel 1550 3950 0    50   Input ~ 0
BUS_D4
Text GLabel 1550 4050 0    50   Input ~ 0
BUS_D5
Text GLabel 1550 4150 0    50   Input ~ 0
BUS_D6
Text GLabel 1550 4250 0    50   Input ~ 0
BUS_D7
$Comp
L 74xx:74LS245 U403
U 1 1 61C36A43
P 4200 1900
F 0 "U403" V 4450 1200 50  0000 R CNN
F 1 "74HCT245" V 4350 1250 50  0000 R CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 4200 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 4200 1900 50  0001 C CNN
	1    4200 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 61C36A49
P 5000 1900
F 0 "#PWR0107" H 5000 1650 50  0001 C CNN
F 1 "GND" V 5005 1772 50  0000 R CNN
F 2 "" H 5000 1900 50  0001 C CNN
F 3 "" H 5000 1900 50  0001 C CNN
	1    5000 1900
	0    -1   -1   0   
$EndComp
Text GLabel 3700 1400 1    50   Output ~ 0
BUS_D0
Text GLabel 3800 1400 1    50   Output ~ 0
BUS_D1
Text GLabel 3900 1400 1    50   Output ~ 0
BUS_D2
Text GLabel 4000 1400 1    50   Output ~ 0
BUS_D3
Text GLabel 4100 1400 1    50   Output ~ 0
BUS_D4
Text GLabel 4200 1400 1    50   Output ~ 0
BUS_D5
Text GLabel 4300 1400 1    50   Output ~ 0
BUS_D6
Text GLabel 4400 1400 1    50   Output ~ 0
BUS_D7
Wire Wire Line
	2550 1550 2600 1550
Wire Wire Line
	2600 1550 2600 2400
Wire Wire Line
	2600 2400 3200 2400
Wire Wire Line
	2550 1650 2650 1650
Wire Wire Line
	2650 1650 2650 2450
Wire Wire Line
	2650 2450 3300 2450
Wire Wire Line
	3800 2450 3800 2400
Wire Wire Line
	2550 1750 2700 1750
Wire Wire Line
	2700 1750 2700 2500
Wire Wire Line
	2700 2500 3400 2500
Wire Wire Line
	3900 2500 3900 2400
Wire Wire Line
	2550 1850 2750 1850
Wire Wire Line
	2750 1850 2750 2550
Wire Wire Line
	2750 2550 3500 2550
Wire Wire Line
	4000 2550 4000 2400
Wire Wire Line
	2550 3950 4100 3950
Wire Wire Line
	4100 3950 4100 2850
Wire Wire Line
	2550 4050 4200 4050
Wire Wire Line
	4200 4050 4200 2900
Wire Wire Line
	2550 4150 4300 4150
Wire Wire Line
	2550 4250 4400 4250
Text GLabel 1550 2550 0    50   Input ~ 0
~RST
Text GLabel 1550 4950 0    50   Input ~ 0
~RST
$Comp
L power:GND #PWR0108
U 1 1 61C4D2AA
P 2050 5250
F 0 "#PWR0108" H 2050 5000 50  0001 C CNN
F 1 "GND" H 2055 5077 50  0000 C CNN
F 2 "" H 2050 5250 50  0001 C CNN
F 3 "" H 2050 5250 50  0001 C CNN
	1    2050 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 61C4D67F
P 2050 2850
F 0 "#PWR0109" H 2050 2600 50  0001 C CNN
F 1 "GND" H 2055 2677 50  0000 C CNN
F 2 "" H 2050 2850 50  0001 C CNN
F 3 "" H 2050 2850 50  0001 C CNN
	1    2050 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2050 2550 3150
Wire Wire Line
	2550 3150 1000 3150
Wire Wire Line
	1000 3150 1000 4650
Wire Wire Line
	1000 4650 1550 4650
Text GLabel 1550 4750 0    50   Input ~ 0
CLK
Text GLabel 1550 2350 0    50   Input ~ 0
CLK
NoConn ~ 2550 4450
Text GLabel 1550 2050 0    50   Input ~ 0
~PCW
Text GLabel 1550 4450 0    50   Input ~ 0
~PCW
Text GLabel 1550 2150 0    50   Input ~ 0
PCA
Text GLabel 1550 4550 0    50   Input ~ 0
PCA
$Comp
L power:VCC #PWR0110
U 1 1 61C5847B
P 1550 2250
F 0 "#PWR0110" H 1550 2100 50  0001 C CNN
F 1 "VCC" V 1565 2377 50  0000 L CNN
F 2 "" H 1550 2250 50  0001 C CNN
F 3 "" H 1550 2250 50  0001 C CNN
	1    1550 2250
	0    -1   -1   0   
$EndComp
Text GLabel 3900 3050 3    50   Output ~ 0
pc_7
Text GLabel 3800 3050 3    50   Output ~ 0
pc_6
Text GLabel 3700 3050 3    50   Output ~ 0
pc_5
Text GLabel 3600 3050 3    50   Output ~ 0
pc_4
Text GLabel 3500 3050 3    50   Output ~ 0
pc_3
Text GLabel 3400 3050 3    50   Output ~ 0
pc_2
Text GLabel 3300 3050 3    50   Output ~ 0
pc_1
Text GLabel 3200 3050 3    50   Output ~ 0
pc_0
Wire Wire Line
	3900 3050 3900 3000
Wire Wire Line
	3900 3000 4400 3000
Connection ~ 4400 3000
Wire Wire Line
	4400 3000 4400 4250
Wire Wire Line
	4400 2400 4400 3000
Wire Wire Line
	4300 4150 4300 2950
Wire Wire Line
	3800 3050 3800 2950
Wire Wire Line
	3800 2950 4300 2950
Connection ~ 4300 2950
Wire Wire Line
	4300 2950 4300 2400
Wire Wire Line
	3700 3050 3700 2900
Wire Wire Line
	3700 2900 4200 2900
Connection ~ 4200 2900
Wire Wire Line
	4200 2900 4200 2400
Wire Wire Line
	3600 3050 3600 2850
Wire Wire Line
	3600 2850 4100 2850
Connection ~ 4100 2850
Wire Wire Line
	4100 2850 4100 2400
Wire Wire Line
	3500 3050 3500 2550
Connection ~ 3500 2550
Wire Wire Line
	3500 2550 4000 2550
Wire Wire Line
	3400 3050 3400 2500
Connection ~ 3400 2500
Wire Wire Line
	3400 2500 3900 2500
Wire Wire Line
	3300 3050 3300 2450
Connection ~ 3300 2450
Wire Wire Line
	3300 2450 3800 2450
Wire Wire Line
	3200 3050 3200 2400
Connection ~ 3200 2400
Wire Wire Line
	3200 2400 3700 2400
$Comp
L power:VCC #PWR0111
U 1 1 61C91C62
P 4600 2400
F 0 "#PWR0111" H 4600 2250 50  0001 C CNN
F 1 "VCC" H 4615 2573 50  0000 C CNN
F 2 "" H 4600 2400 50  0001 C CNN
F 3 "" H 4600 2400 50  0001 C CNN
	1    4600 2400
	-1   0    0    1   
$EndComp
Text GLabel 4800 2400 2    50   Input ~ 0
~PCE
Wire Wire Line
	4800 2400 4700 2400
$Comp
L Device:LED D409
U 1 1 61C978A0
P 7300 7500
F 0 "D409" V 7339 7382 50  0000 R CNN
F 1 "RED" V 7248 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7300 7500 50  0001 C CNN
F 3 "~" H 7300 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7300 7500 50  0001 C CNN "Mouser"
	1    7300 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D410
U 1 1 61C97DAA
P 7700 7500
F 0 "D410" V 7739 7382 50  0000 R CNN
F 1 "RED" V 7648 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 7700 7500 50  0001 C CNN
F 3 "~" H 7700 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 7700 7500 50  0001 C CNN "Mouser"
	1    7700 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D411
U 1 1 61C97F13
P 8100 7500
F 0 "D411" V 8139 7382 50  0000 R CNN
F 1 "RED" V 8048 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8100 7500 50  0001 C CNN
F 3 "~" H 8100 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8100 7500 50  0001 C CNN "Mouser"
	1    8100 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D412
U 1 1 61C98058
P 8500 7500
F 0 "D412" V 8539 7382 50  0000 R CNN
F 1 "RED" V 8448 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8500 7500 50  0001 C CNN
F 3 "~" H 8500 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8500 7500 50  0001 C CNN "Mouser"
	1    8500 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D413
U 1 1 61C9819D
P 8900 7500
F 0 "D413" V 8939 7382 50  0000 R CNN
F 1 "RED" V 8848 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 8900 7500 50  0001 C CNN
F 3 "~" H 8900 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 8900 7500 50  0001 C CNN "Mouser"
	1    8900 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D414
U 1 1 61C982E2
P 9300 7500
F 0 "D414" V 9339 7382 50  0000 R CNN
F 1 "RED" V 9248 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 9300 7500 50  0001 C CNN
F 3 "~" H 9300 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 9300 7500 50  0001 C CNN "Mouser"
	1    9300 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D415
U 1 1 61C98445
P 9700 7500
F 0 "D415" V 9739 7382 50  0000 R CNN
F 1 "RED" V 9648 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 9700 7500 50  0001 C CNN
F 3 "~" H 9700 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 9700 7500 50  0001 C CNN "Mouser"
	1    9700 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D416
U 1 1 61C9858A
P 10100 7500
F 0 "D416" V 10139 7382 50  0000 R CNN
F 1 "RED" V 10048 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 10100 7500 50  0001 C CNN
F 3 "~" H 10100 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 10100 7500 50  0001 C CNN "Mouser"
	1    10100 7500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 61C9C350
P 9050 8250
F 0 "#PWR0119" H 9050 8000 50  0001 C CNN
F 1 "GND" H 9055 8077 50  0000 C CNN
F 2 "" H 9050 8250 50  0001 C CNN
F 3 "" H 9050 8250 50  0001 C CNN
	1    9050 8250
	1    0    0    -1  
$EndComp
Text GLabel 10100 7350 1    50   Input ~ 0
sp_0
Text GLabel 9700 7350 1    50   Input ~ 0
sp_1
Text GLabel 9300 7350 1    50   Input ~ 0
sp_2
Text GLabel 8900 7350 1    50   Input ~ 0
sp_3
Text GLabel 8500 7350 1    50   Input ~ 0
sp_4
Text GLabel 8100 7350 1    50   Input ~ 0
sp_5
Text GLabel 7700 7350 1    50   Input ~ 0
sp_6
Text GLabel 7300 7350 1    50   Input ~ 0
sp_7
$Comp
L Device:LED D417
U 1 1 61CD11BE
P 11200 7500
F 0 "D417" V 11239 7382 50  0000 R CNN
F 1 "RED" V 11148 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 11200 7500 50  0001 C CNN
F 3 "~" H 11200 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 11200 7500 50  0001 C CNN "Mouser"
	1    11200 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D418
U 1 1 61CD11C5
P 11600 7500
F 0 "D418" V 11639 7382 50  0000 R CNN
F 1 "RED" V 11548 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 11600 7500 50  0001 C CNN
F 3 "~" H 11600 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 11600 7500 50  0001 C CNN "Mouser"
	1    11600 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D419
U 1 1 61CD11CC
P 12000 7500
F 0 "D419" V 12039 7382 50  0000 R CNN
F 1 "RED" V 11948 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 12000 7500 50  0001 C CNN
F 3 "~" H 12000 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 12000 7500 50  0001 C CNN "Mouser"
	1    12000 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D420
U 1 1 61CD11D3
P 12400 7500
F 0 "D420" V 12439 7382 50  0000 R CNN
F 1 "RED" V 12348 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 12400 7500 50  0001 C CNN
F 3 "~" H 12400 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 12400 7500 50  0001 C CNN "Mouser"
	1    12400 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D421
U 1 1 61CD11DA
P 12800 7500
F 0 "D421" V 12839 7382 50  0000 R CNN
F 1 "RED" V 12748 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 12800 7500 50  0001 C CNN
F 3 "~" H 12800 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 12800 7500 50  0001 C CNN "Mouser"
	1    12800 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D422
U 1 1 61CD11E1
P 13200 7500
F 0 "D422" V 13239 7382 50  0000 R CNN
F 1 "RED" V 13148 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 13200 7500 50  0001 C CNN
F 3 "~" H 13200 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 13200 7500 50  0001 C CNN "Mouser"
	1    13200 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D432
U 1 1 61CD11E8
P 13600 7500
F 0 "D432" V 13639 7382 50  0000 R CNN
F 1 "RED" V 13548 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 13600 7500 50  0001 C CNN
F 3 "~" H 13600 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 13600 7500 50  0001 C CNN "Mouser"
	1    13600 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D424
U 1 1 61CD11EF
P 14000 7500
F 0 "D424" V 14039 7382 50  0000 R CNN
F 1 "RED" V 13948 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 14000 7500 50  0001 C CNN
F 3 "~" H 14000 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 14000 7500 50  0001 C CNN "Mouser"
	1    14000 7500
	0    -1   -1   0   
$EndComp
Text GLabel 14000 7350 1    50   Input ~ 0
iv_0
Text GLabel 13600 7350 1    50   Input ~ 0
iv_1
Text GLabel 13200 7350 1    50   Input ~ 0
iv_2
Text GLabel 12800 7350 1    50   Input ~ 0
iv_3
Text GLabel 12400 7350 1    50   Input ~ 0
iv_4
Text GLabel 12000 7350 1    50   Input ~ 0
iv_5
Text GLabel 11600 7350 1    50   Input ~ 0
iv_6
Text GLabel 11200 7350 1    50   Input ~ 0
iv_7
$Comp
L Device:LED D401
U 1 1 61CE262D
P 3300 7500
F 0 "D401" V 3339 7382 50  0000 R CNN
F 1 "RED" V 3248 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 3300 7500 50  0001 C CNN
F 3 "~" H 3300 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 3300 7500 50  0001 C CNN "Mouser"
	1    3300 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D402
U 1 1 61CE2634
P 3700 7500
F 0 "D402" V 3739 7382 50  0000 R CNN
F 1 "RED" V 3648 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 3700 7500 50  0001 C CNN
F 3 "~" H 3700 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 3700 7500 50  0001 C CNN "Mouser"
	1    3700 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D403
U 1 1 61CE263B
P 4100 7500
F 0 "D403" V 4139 7382 50  0000 R CNN
F 1 "RED" V 4048 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 4100 7500 50  0001 C CNN
F 3 "~" H 4100 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 4100 7500 50  0001 C CNN "Mouser"
	1    4100 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D404
U 1 1 61CE2642
P 4500 7500
F 0 "D404" V 4539 7382 50  0000 R CNN
F 1 "RED" V 4448 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 4500 7500 50  0001 C CNN
F 3 "~" H 4500 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 4500 7500 50  0001 C CNN "Mouser"
	1    4500 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D405
U 1 1 61CE2649
P 4900 7500
F 0 "D405" V 4939 7382 50  0000 R CNN
F 1 "RED" V 4848 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 4900 7500 50  0001 C CNN
F 3 "~" H 4900 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 4900 7500 50  0001 C CNN "Mouser"
	1    4900 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D406
U 1 1 61CE2650
P 5300 7500
F 0 "D406" V 5339 7382 50  0000 R CNN
F 1 "RED" V 5248 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5300 7500 50  0001 C CNN
F 3 "~" H 5300 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 5300 7500 50  0001 C CNN "Mouser"
	1    5300 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D407
U 1 1 61CE2657
P 5700 7500
F 0 "D407" V 5739 7382 50  0000 R CNN
F 1 "RED" V 5648 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5700 7500 50  0001 C CNN
F 3 "~" H 5700 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 5700 7500 50  0001 C CNN "Mouser"
	1    5700 7500
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D408
U 1 1 61CE265E
P 6100 7500
F 0 "D408" V 6139 7382 50  0000 R CNN
F 1 "RED" V 6048 7382 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 6100 7500 50  0001 C CNN
F 3 "~" H 6100 7500 50  0001 C CNN
F 4 "941-C503BRCNCW0Z0AA2" V 6100 7500 50  0001 C CNN "Mouser"
	1    6100 7500
	0    -1   -1   0   
$EndComp
Text GLabel 6100 7350 1    50   Input ~ 0
pc_0
Text GLabel 5700 7350 1    50   Input ~ 0
pc_1
Text GLabel 5300 7350 1    50   Input ~ 0
pc_2
Text GLabel 4900 7350 1    50   Input ~ 0
pc_3
Text GLabel 4500 7350 1    50   Input ~ 0
pc_4
Text GLabel 4100 7350 1    50   Input ~ 0
pc_5
Text GLabel 3700 7350 1    50   Input ~ 0
pc_6
Text GLabel 3300 7350 1    50   Input ~ 0
pc_7
$Comp
L Device:C_Small C401
U 1 1 61D23435
P 1650 1000
F 0 "C401" H 1742 1046 50  0000 L CNN
F 1 "0.1µF" H 1742 955 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1650 1000 50  0001 C CNN
F 3 "~" H 1650 1000 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1650 1000 50  0001 C CNN "Mouser"
	1    1650 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 61D23A6B
P 1650 1100
F 0 "#PWR0139" H 1650 850 50  0001 C CNN
F 1 "GND" H 1655 927 50  0000 C CNN
F 2 "" H 1650 1100 50  0001 C CNN
F 3 "" H 1650 1100 50  0001 C CNN
	1    1650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1250 2050 900 
Wire Wire Line
	2050 900  1650 900 
$Comp
L power:VCC #PWR0140
U 1 1 61D324B1
P 2050 900
F 0 "#PWR0140" H 2050 750 50  0001 C CNN
F 1 "VCC" H 2065 1073 50  0000 C CNN
F 2 "" H 2050 900 50  0001 C CNN
F 3 "" H 2050 900 50  0001 C CNN
	1    2050 900 
	1    0    0    -1  
$EndComp
Connection ~ 2050 900 
$Comp
L Device:C_Small C402
U 1 1 61D33AF8
P 1650 3500
F 0 "C402" H 1742 3546 50  0000 L CNN
F 1 "0.1µF" H 1742 3455 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1650 3500 50  0001 C CNN
F 3 "~" H 1650 3500 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1650 3500 50  0001 C CNN "Mouser"
	1    1650 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0141
U 1 1 61D33AFE
P 1650 3600
F 0 "#PWR0141" H 1650 3350 50  0001 C CNN
F 1 "GND" H 1655 3427 50  0000 C CNN
F 2 "" H 1650 3600 50  0001 C CNN
F 3 "" H 1650 3600 50  0001 C CNN
	1    1650 3600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0142
U 1 1 61D33B06
P 2050 3400
F 0 "#PWR0142" H 2050 3250 50  0001 C CNN
F 1 "VCC" H 2065 3573 50  0000 C CNN
F 2 "" H 2050 3400 50  0001 C CNN
F 3 "" H 2050 3400 50  0001 C CNN
	1    2050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3650 2050 3400
Wire Wire Line
	2050 3400 1650 3400
Connection ~ 2050 3400
$Comp
L Device:C_Small C407
U 1 1 61D5F9CC
P 12400 1150
F 0 "C407" H 12492 1196 50  0000 L CNN
F 1 "0.1µF" H 12492 1105 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 12400 1150 50  0001 C CNN
F 3 "~" H 12400 1150 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 12400 1150 50  0001 C CNN "Mouser"
	1    12400 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 61D5F9D2
P 12400 1250
F 0 "#PWR0143" H 12400 1000 50  0001 C CNN
F 1 "GND" H 12405 1077 50  0000 C CNN
F 2 "" H 12400 1250 50  0001 C CNN
F 3 "" H 12400 1250 50  0001 C CNN
	1    12400 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 1350 12800 1050
Wire Wire Line
	12800 1050 12400 1050
$Comp
L power:VCC #PWR0144
U 1 1 61D5F9DA
P 12800 1050
F 0 "#PWR0144" H 12800 900 50  0001 C CNN
F 1 "VCC" H 12815 1223 50  0000 C CNN
F 2 "" H 12800 1050 50  0001 C CNN
F 3 "" H 12800 1050 50  0001 C CNN
	1    12800 1050
	1    0    0    -1  
$EndComp
Connection ~ 12800 1050
$Comp
L Device:C_Small C403
U 1 1 61D79C8B
P 3250 2050
F 0 "C403" H 3342 2096 50  0000 L CNN
F 1 "0.1µF" H 3342 2005 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 3250 2050 50  0001 C CNN
F 3 "~" H 3250 2050 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 3250 2050 50  0001 C CNN "Mouser"
	1    3250 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0149
U 1 1 61D79C91
P 3250 2150
F 0 "#PWR0149" H 3250 1900 50  0001 C CNN
F 1 "GND" H 3255 1977 50  0000 C CNN
F 2 "" H 3250 2150 50  0001 C CNN
F 3 "" H 3250 2150 50  0001 C CNN
	1    3250 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1900 3250 1900
Wire Wire Line
	3250 1900 3250 1950
$Comp
L power:VCC #PWR0150
U 1 1 61D79C99
P 3250 1900
F 0 "#PWR0150" H 3250 1750 50  0001 C CNN
F 1 "VCC" H 3265 2073 50  0000 C CNN
F 2 "" H 3250 1900 50  0001 C CNN
F 3 "" H 3250 1900 50  0001 C CNN
	1    3250 1900
	1    0    0    -1  
$EndComp
Connection ~ 3250 1900
$Comp
L Device:C_Small C408
U 1 1 61D82B7C
P 14000 2150
F 0 "C408" H 14092 2196 50  0000 L CNN
F 1 "0.1µF" H 14092 2105 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 14000 2150 50  0001 C CNN
F 3 "~" H 14000 2150 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 14000 2150 50  0001 C CNN "Mouser"
	1    14000 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0151
U 1 1 61D82B82
P 14000 2250
F 0 "#PWR0151" H 14000 2000 50  0001 C CNN
F 1 "GND" H 14005 2077 50  0000 C CNN
F 2 "" H 14000 2250 50  0001 C CNN
F 3 "" H 14000 2250 50  0001 C CNN
	1    14000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	14150 2000 14000 2000
Wire Wire Line
	14000 2000 14000 2050
$Comp
L power:VCC #PWR0152
U 1 1 61D82B8A
P 14000 2000
F 0 "#PWR0152" H 14000 1850 50  0001 C CNN
F 1 "VCC" H 14015 2173 50  0000 C CNN
F 2 "" H 14000 2000 50  0001 C CNN
F 3 "" H 14000 2000 50  0001 C CNN
	1    14000 2000
	1    0    0    -1  
$EndComp
Connection ~ 14000 2000
$Comp
L Device:R_Pack08 RN402
U 1 1 652C6766
P 8750 8050
F 0 "RN402" H 9138 8096 50  0000 L CNN
F 1 "330 x 8" H 9138 8005 50  0000 L CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" V 9225 8050 50  0001 C CNN
F 3 "~" H 8750 8050 50  0001 C CNN
F 4 " 652-4116R-1-330" H 8750 8050 50  0001 C CNN "Mouser"
	1    8750 8050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 8250 8450 8250
Wire Wire Line
	8450 8250 8550 8250
Connection ~ 8450 8250
Wire Wire Line
	8550 8250 8650 8250
Connection ~ 8550 8250
Wire Wire Line
	8650 8250 8750 8250
Connection ~ 8650 8250
Wire Wire Line
	8750 8250 8850 8250
Connection ~ 8750 8250
Wire Wire Line
	8850 8250 8950 8250
Connection ~ 8850 8250
Wire Wire Line
	8950 8250 9050 8250
Connection ~ 8950 8250
Connection ~ 9050 8250
Wire Wire Line
	10100 7850 9050 7850
Wire Wire Line
	7300 7850 8350 7850
Wire Wire Line
	10100 7650 10100 7850
Wire Wire Line
	9700 7650 9700 7800
Wire Wire Line
	9700 7800 8950 7800
Wire Wire Line
	8950 7800 8950 7850
Wire Wire Line
	9300 7650 9300 7750
Wire Wire Line
	9300 7750 8850 7750
Wire Wire Line
	8850 7750 8850 7850
Wire Wire Line
	8900 7650 8900 7700
Wire Wire Line
	8900 7700 8750 7700
Wire Wire Line
	8750 7700 8750 7850
Wire Wire Line
	8500 7650 8500 7700
Wire Wire Line
	8500 7700 8650 7700
Wire Wire Line
	8650 7700 8650 7850
Wire Wire Line
	8100 7650 8100 7750
Wire Wire Line
	8100 7750 8550 7750
Wire Wire Line
	8550 7750 8550 7850
Wire Wire Line
	7700 7650 7700 7800
Wire Wire Line
	7700 7800 8450 7800
Wire Wire Line
	8450 7800 8450 7850
Wire Wire Line
	7300 7650 7300 7850
$Comp
L power:GND #PWR0112
U 1 1 654213BE
P 12950 8250
F 0 "#PWR0112" H 12950 8000 50  0001 C CNN
F 1 "GND" H 12955 8077 50  0000 C CNN
F 2 "" H 12950 8250 50  0001 C CNN
F 3 "" H 12950 8250 50  0001 C CNN
	1    12950 8250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Pack08 RN403
U 1 1 654213C4
P 12650 8050
F 0 "RN403" H 13038 8096 50  0000 L CNN
F 1 "330 x 8" H 13038 8005 50  0000 L CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" V 13125 8050 50  0001 C CNN
F 3 "~" H 12650 8050 50  0001 C CNN
F 4 "652-4116R-1-330" H 12650 8050 50  0001 C CNN "Mouser"
	1    12650 8050
	1    0    0    -1  
$EndComp
Wire Wire Line
	12250 8250 12350 8250
Wire Wire Line
	12350 8250 12450 8250
Connection ~ 12350 8250
Wire Wire Line
	12450 8250 12550 8250
Connection ~ 12450 8250
Wire Wire Line
	12550 8250 12650 8250
Connection ~ 12550 8250
Wire Wire Line
	12650 8250 12750 8250
Connection ~ 12650 8250
Wire Wire Line
	12750 8250 12850 8250
Connection ~ 12750 8250
Wire Wire Line
	12850 8250 12950 8250
Connection ~ 12850 8250
Connection ~ 12950 8250
Wire Wire Line
	14000 7850 12950 7850
Wire Wire Line
	11200 7850 12250 7850
Wire Wire Line
	14000 7650 14000 7850
Wire Wire Line
	13600 7650 13600 7800
Wire Wire Line
	13600 7800 12850 7800
Wire Wire Line
	12850 7800 12850 7850
Wire Wire Line
	13200 7650 13200 7750
Wire Wire Line
	13200 7750 12750 7750
Wire Wire Line
	12750 7750 12750 7850
Wire Wire Line
	12800 7650 12800 7700
Wire Wire Line
	12800 7700 12650 7700
Wire Wire Line
	12650 7700 12650 7850
Wire Wire Line
	12400 7650 12400 7700
Wire Wire Line
	12400 7700 12550 7700
Wire Wire Line
	12550 7700 12550 7850
Wire Wire Line
	12000 7650 12000 7750
Wire Wire Line
	12000 7750 12450 7750
Wire Wire Line
	12450 7750 12450 7850
Wire Wire Line
	11600 7650 11600 7800
Wire Wire Line
	11600 7800 12350 7800
Wire Wire Line
	12350 7800 12350 7850
Wire Wire Line
	11200 7650 11200 7850
$Comp
L power:GND #PWR0113
U 1 1 654563E4
P 5050 8250
F 0 "#PWR0113" H 5050 8000 50  0001 C CNN
F 1 "GND" H 5055 8077 50  0000 C CNN
F 2 "" H 5050 8250 50  0001 C CNN
F 3 "" H 5050 8250 50  0001 C CNN
	1    5050 8250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Pack08 RN401
U 1 1 654563EA
P 4750 8050
F 0 "RN401" H 5138 8096 50  0000 L CNN
F 1 "330 x 8" H 5138 8005 50  0000 L CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" V 5225 8050 50  0001 C CNN
F 3 "~" H 4750 8050 50  0001 C CNN
F 4 " 652-4116R-1-330" H 4750 8050 50  0001 C CNN "Mouser"
	1    4750 8050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 8250 4450 8250
Wire Wire Line
	4450 8250 4550 8250
Connection ~ 4450 8250
Wire Wire Line
	4550 8250 4650 8250
Connection ~ 4550 8250
Wire Wire Line
	4650 8250 4750 8250
Connection ~ 4650 8250
Wire Wire Line
	4750 8250 4850 8250
Connection ~ 4750 8250
Wire Wire Line
	4850 8250 4950 8250
Connection ~ 4850 8250
Wire Wire Line
	4950 8250 5050 8250
Connection ~ 4950 8250
Connection ~ 5050 8250
Wire Wire Line
	6100 7850 5050 7850
Wire Wire Line
	3300 7850 4350 7850
Wire Wire Line
	6100 7650 6100 7850
Wire Wire Line
	5700 7650 5700 7800
Wire Wire Line
	5700 7800 4950 7800
Wire Wire Line
	4950 7800 4950 7850
Wire Wire Line
	5300 7650 5300 7750
Wire Wire Line
	5300 7750 4850 7750
Wire Wire Line
	4850 7750 4850 7850
Wire Wire Line
	4900 7650 4900 7700
Wire Wire Line
	4900 7700 4750 7700
Wire Wire Line
	4750 7700 4750 7850
Wire Wire Line
	4500 7650 4500 7700
Wire Wire Line
	4500 7700 4650 7700
Wire Wire Line
	4650 7700 4650 7850
Wire Wire Line
	4100 7650 4100 7750
Wire Wire Line
	4100 7750 4550 7750
Wire Wire Line
	4550 7750 4550 7850
Wire Wire Line
	3700 7650 3700 7800
Wire Wire Line
	3700 7800 4450 7800
Wire Wire Line
	4450 7800 4450 7850
Wire Wire Line
	3300 7650 3300 7850
$Comp
L 74xx:74LS377 U407
U 1 1 66CFD9C5
P 12800 2150
F 0 "U407" H 13050 2950 50  0000 C CNN
F 1 "74HCT377" H 13050 2850 50  0000 C CNN
F 2 "" H 12800 2150 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS377" H 12800 2150 50  0001 C CNN
	1    12800 2150
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS161 U404
U 1 1 624E602C
P 7350 2050
F 0 "U404" H 7600 2850 50  0000 C CNN
F 1 "74HCT161" H 7600 2750 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 7350 2050 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS161" H 7350 2050 50  0001 C CNN
	1    7350 2050
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS161 U405
U 1 1 624E6032
P 7350 4450
F 0 "U405" H 7600 5250 50  0000 C CNN
F 1 "74HCT161" H 7600 5150 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 7350 4450 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS161" H 7350 4450 50  0001 C CNN
	1    7350 4450
	1    0    0    -1  
$EndComp
Text Notes 6200 1400 0    50   Italic 10
Stack Pointer
Text GLabel 6850 1550 0    50   Input ~ 0
BUS_D0
Text GLabel 6850 1650 0    50   Input ~ 0
BUS_D1
Text GLabel 6850 1750 0    50   Input ~ 0
BUS_D2
Text GLabel 6850 1850 0    50   Input ~ 0
BUS_D3
Text GLabel 6850 3950 0    50   Input ~ 0
BUS_D4
Text GLabel 6850 4050 0    50   Input ~ 0
BUS_D5
Text GLabel 6850 4150 0    50   Input ~ 0
BUS_D6
Text GLabel 6850 4250 0    50   Input ~ 0
BUS_D7
$Comp
L 74xx:74LS245 U406
U 1 1 624E6041
P 9500 1900
F 0 "U406" V 9750 1200 50  0000 R CNN
F 1 "74HCT245" V 9650 1250 50  0000 R CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 9500 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 9500 1900 50  0001 C CNN
	1    9500 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624E6047
P 10300 1900
F 0 "#PWR?" H 10300 1650 50  0001 C CNN
F 1 "GND" V 10305 1772 50  0000 R CNN
F 2 "" H 10300 1900 50  0001 C CNN
F 3 "" H 10300 1900 50  0001 C CNN
	1    10300 1900
	0    -1   -1   0   
$EndComp
Text GLabel 9000 1400 1    50   Output ~ 0
BUS_D0
Text GLabel 9100 1400 1    50   Output ~ 0
BUS_D1
Text GLabel 9200 1400 1    50   Output ~ 0
BUS_D2
Text GLabel 9300 1400 1    50   Output ~ 0
BUS_D3
Text GLabel 9400 1400 1    50   Output ~ 0
BUS_D4
Text GLabel 9500 1400 1    50   Output ~ 0
BUS_D5
Text GLabel 9600 1400 1    50   Output ~ 0
BUS_D6
Text GLabel 9700 1400 1    50   Output ~ 0
BUS_D7
Wire Wire Line
	7850 1550 7900 1550
Wire Wire Line
	7900 1550 7900 2400
Wire Wire Line
	7900 2400 8500 2400
Wire Wire Line
	7850 1650 7950 1650
Wire Wire Line
	7950 1650 7950 2450
Wire Wire Line
	7950 2450 8600 2450
Wire Wire Line
	9100 2450 9100 2400
Wire Wire Line
	7850 1750 8000 1750
Wire Wire Line
	8000 1750 8000 2500
Wire Wire Line
	8000 2500 8700 2500
Wire Wire Line
	9200 2500 9200 2400
Wire Wire Line
	7850 1850 8050 1850
Wire Wire Line
	8050 1850 8050 2550
Wire Wire Line
	8050 2550 8800 2550
Wire Wire Line
	9300 2550 9300 2400
Wire Wire Line
	7850 3950 9400 3950
Wire Wire Line
	9400 3950 9400 2850
Wire Wire Line
	7850 4050 9500 4050
Wire Wire Line
	9500 4050 9500 2900
Wire Wire Line
	7850 4150 9600 4150
Wire Wire Line
	7850 4250 9700 4250
Text GLabel 6850 2550 0    50   Input ~ 0
~RST
Text GLabel 6850 4950 0    50   Input ~ 0
~RST
$Comp
L power:GND #PWR?
U 1 1 624E606C
P 7350 5250
F 0 "#PWR?" H 7350 5000 50  0001 C CNN
F 1 "GND" H 7355 5077 50  0000 C CNN
F 2 "" H 7350 5250 50  0001 C CNN
F 3 "" H 7350 5250 50  0001 C CNN
	1    7350 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624E6072
P 7350 2850
F 0 "#PWR?" H 7350 2600 50  0001 C CNN
F 1 "GND" H 7355 2677 50  0000 C CNN
F 2 "" H 7350 2850 50  0001 C CNN
F 3 "" H 7350 2850 50  0001 C CNN
	1    7350 2850
	1    0    0    -1  
$EndComp
Text GLabel 6850 4750 0    50   Input ~ 0
CLK
Text GLabel 6850 2350 0    50   Input ~ 0
CLK
NoConn ~ 7850 4450
Text GLabel 6850 2050 0    50   Input ~ 0
~SPW
Text GLabel 6850 4450 0    50   Input ~ 0
~SPW
Text GLabel 9200 3050 3    50   Output ~ 0
sp_7
Text GLabel 9100 3050 3    50   Output ~ 0
sp_6
Text GLabel 9000 3050 3    50   Output ~ 0
sp_5
Text GLabel 8900 3050 3    50   Output ~ 0
sp_4
Text GLabel 8800 3050 3    50   Output ~ 0
sp_3
Text GLabel 8700 3050 3    50   Output ~ 0
sp_2
Text GLabel 8600 3050 3    50   Output ~ 0
sp_1
Text GLabel 8500 3050 3    50   Output ~ 0
sp_0
Wire Wire Line
	9200 3050 9200 3000
Wire Wire Line
	9200 3000 9700 3000
Connection ~ 9700 3000
Wire Wire Line
	9700 3000 9700 4250
Wire Wire Line
	9700 2400 9700 3000
Wire Wire Line
	9600 4150 9600 2950
Wire Wire Line
	9100 3050 9100 2950
Wire Wire Line
	9100 2950 9600 2950
Connection ~ 9600 2950
Wire Wire Line
	9600 2950 9600 2400
Wire Wire Line
	9000 3050 9000 2900
Wire Wire Line
	9000 2900 9500 2900
Connection ~ 9500 2900
Wire Wire Line
	9500 2900 9500 2400
Wire Wire Line
	8900 3050 8900 2850
Wire Wire Line
	8900 2850 9400 2850
Connection ~ 9400 2850
Wire Wire Line
	9400 2850 9400 2400
Wire Wire Line
	8800 3050 8800 2550
Connection ~ 8800 2550
Wire Wire Line
	8800 2550 9300 2550
Wire Wire Line
	8700 3050 8700 2500
Connection ~ 8700 2500
Wire Wire Line
	8700 2500 9200 2500
Wire Wire Line
	8600 3050 8600 2450
Connection ~ 8600 2450
Wire Wire Line
	8600 2450 9100 2450
Wire Wire Line
	8500 3050 8500 2400
Connection ~ 8500 2400
Wire Wire Line
	8500 2400 9000 2400
$Comp
L power:VCC #PWR?
U 1 1 624E60AF
P 9900 2400
F 0 "#PWR?" H 9900 2250 50  0001 C CNN
F 1 "VCC" H 9915 2573 50  0000 C CNN
F 2 "" H 9900 2400 50  0001 C CNN
F 3 "" H 9900 2400 50  0001 C CNN
	1    9900 2400
	-1   0    0    1   
$EndComp
Text GLabel 10100 2400 2    50   Input ~ 0
~SPE
Wire Wire Line
	10100 2400 10000 2400
$Comp
L Device:C_Small C404
U 1 1 624E60B8
P 6950 1000
F 0 "C404" H 7042 1046 50  0000 L CNN
F 1 "0.1µF" H 7042 955 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 6950 1000 50  0001 C CNN
F 3 "~" H 6950 1000 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 6950 1000 50  0001 C CNN "Mouser"
	1    6950 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624E60BE
P 6950 1100
F 0 "#PWR?" H 6950 850 50  0001 C CNN
F 1 "GND" H 6955 927 50  0000 C CNN
F 2 "" H 6950 1100 50  0001 C CNN
F 3 "" H 6950 1100 50  0001 C CNN
	1    6950 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 1250 7350 900 
Wire Wire Line
	7350 900  6950 900 
$Comp
L power:VCC #PWR?
U 1 1 624E60C6
P 7350 900
F 0 "#PWR?" H 7350 750 50  0001 C CNN
F 1 "VCC" H 7365 1073 50  0000 C CNN
F 2 "" H 7350 900 50  0001 C CNN
F 3 "" H 7350 900 50  0001 C CNN
	1    7350 900 
	1    0    0    -1  
$EndComp
Connection ~ 7350 900 
$Comp
L Device:C_Small C405
U 1 1 624E60CE
P 6950 3500
F 0 "C405" H 7042 3546 50  0000 L CNN
F 1 "0.1µF" H 7042 3455 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 6950 3500 50  0001 C CNN
F 3 "~" H 6950 3500 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 6950 3500 50  0001 C CNN "Mouser"
	1    6950 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624E60D4
P 6950 3600
F 0 "#PWR?" H 6950 3350 50  0001 C CNN
F 1 "GND" H 6955 3427 50  0000 C CNN
F 2 "" H 6950 3600 50  0001 C CNN
F 3 "" H 6950 3600 50  0001 C CNN
	1    6950 3600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 624E60DA
P 7350 3400
F 0 "#PWR?" H 7350 3250 50  0001 C CNN
F 1 "VCC" H 7365 3573 50  0000 C CNN
F 2 "" H 7350 3400 50  0001 C CNN
F 3 "" H 7350 3400 50  0001 C CNN
	1    7350 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 3650 7350 3400
Wire Wire Line
	7350 3400 6950 3400
Connection ~ 7350 3400
$Comp
L Device:C_Small C406
U 1 1 624E60E4
P 8550 2050
F 0 "C406" H 8642 2096 50  0000 L CNN
F 1 "0.1µF" H 8642 2005 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 8550 2050 50  0001 C CNN
F 3 "~" H 8550 2050 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 8550 2050 50  0001 C CNN "Mouser"
	1    8550 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624E60EA
P 8550 2150
F 0 "#PWR?" H 8550 1900 50  0001 C CNN
F 1 "GND" H 8555 1977 50  0000 C CNN
F 2 "" H 8550 2150 50  0001 C CNN
F 3 "" H 8550 2150 50  0001 C CNN
	1    8550 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1900 8550 1900
Wire Wire Line
	8550 1900 8550 1950
$Comp
L power:VCC #PWR?
U 1 1 624E60F2
P 8550 1900
F 0 "#PWR?" H 8550 1750 50  0001 C CNN
F 1 "VCC" H 8565 2073 50  0000 C CNN
F 2 "" H 8550 1900 50  0001 C CNN
F 3 "" H 8550 1900 50  0001 C CNN
	1    8550 1900
	1    0    0    -1  
$EndComp
Connection ~ 8550 1900
Wire Wire Line
	6850 2150 6850 2250
Wire Wire Line
	6850 4550 6850 4650
$Comp
L power:GND #PWR?
U 1 1 626281BC
P 6500 4550
F 0 "#PWR?" H 6500 4300 50  0001 C CNN
F 1 "GND" H 6505 4377 50  0000 C CNN
F 2 "" H 6500 4550 50  0001 C CNN
F 3 "" H 6500 4550 50  0001 C CNN
	1    6500 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 626281F5
P 6500 2150
F 0 "#PWR?" H 6500 1900 50  0001 C CNN
F 1 "GND" H 6505 1977 50  0000 C CNN
F 2 "" H 6500 2150 50  0001 C CNN
F 3 "" H 6500 2150 50  0001 C CNN
	1    6500 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2150 6500 2150
Connection ~ 6850 2150
Wire Wire Line
	6850 4550 6500 4550
Connection ~ 6850 4550
NoConn ~ 7850 2050
$EndSCHEMATC
