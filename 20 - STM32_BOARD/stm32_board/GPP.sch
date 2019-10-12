EESchema Schematic File Version 4
LIBS:stm32_board-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5DA17A2F
P 950 7200
F 0 "J1" H 869 7514 50  0000 C CNN
F 1 "Conn_01x04" H 869 7425 50  0000 C CNN
F 2 "Connectors_JST:JST_SH_SM04B-SRSS-TB_04x1.00mm_Angled" H 950 7200 50  0001 C CNN
F 3 "~" H 950 7200 50  0001 C CNN
	1    950  7200
	-1   0    0    -1  
$EndComp
Text Label 1150 7100 0    50   ~ 0
5V_LIDAR
Text Label 1150 7200 0    50   ~ 0
LD1_RXi
Text Label 1150 7300 0    50   ~ 0
LD1_TXo
$Comp
L power:GND #PWR0101
U 1 1 5DA196C0
P 1200 7450
F 0 "#PWR0101" H 1200 7200 50  0001 C CNN
F 1 "GND" H 1205 7279 50  0000 C CNN
F 2 "" H 1200 7450 50  0001 C CNN
F 3 "" H 1200 7450 50  0001 C CNN
	1    1200 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 7400 1200 7400
Wire Wire Line
	1200 7400 1200 7450
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5DA19F56
P 1650 7200
F 0 "J2" H 1569 7514 50  0000 C CNN
F 1 "Conn_01x04" H 1569 7425 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 1650 7200 50  0001 C CNN
F 3 "~" H 1650 7200 50  0001 C CNN
	1    1650 7200
	-1   0    0    -1  
$EndComp
Text Label 1850 7100 0    50   ~ 0
5V_LIDAR
Text Label 1850 7200 0    50   ~ 0
LD1_RXi
Text Label 1850 7300 0    50   ~ 0
LD1_TXo
$Comp
L power:GND #PWR0102
U 1 1 5DA19F63
P 1900 7450
F 0 "#PWR0102" H 1900 7200 50  0001 C CNN
F 1 "GND" H 1905 7279 50  0000 C CNN
F 2 "" H 1900 7450 50  0001 C CNN
F 3 "" H 1900 7450 50  0001 C CNN
	1    1900 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 7400 1900 7400
Wire Wire Line
	1900 7400 1900 7450
$EndSCHEMATC
