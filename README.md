# SoftCtrlDC-M
SoftCtrlDC-M is a control software designed to facilitate the development of a direct current (DC) motor driver by implementing core control code utilizing proportional~(P) and integral~(I) gains. 
The P gain is dynamically updated using fuzzy-logic~(FL) based on the Takagi-Sugeno~(TS) model. This type of controller is called a fuzzy-logic controller (FLC), and the FLC is adept at accommodating non-linearities. 

This software is capable of being embedded in ATMega microcontrollers to assist DC motor driver enabling to provide comfortable and reliable operation. Hence, it stores core control software to assist the operation of a DC motor. 

# Application: AVR MCU ATMega328P
Instructions of operation.
Core control software code is initialized (MCU pins already set). 
It is possible to re-configure to adapt it as desired (read MCU documentation).

Pinout reference:
![image](https://github.com/janice-uaq/SoftCtrlDC-M/assets/150994187/78cbd1bc-4fb1-48da-ace6-e9bc1e5e2c11)
This image is cortesy of ATMega. Please refer to corresponding pinout configuration on case-by-case need.

It contains functions for processing the current, the enconder, the PWM calc for action control, stamping information.
The user inmputs the reference speed in the console as desired.

It works better if it is used with ARduino IDE.
Code is available in source (src) folder.

Notes:
1. This code is able to be copy-paste'd for Arduino interface.
2. This code can be used directly in C code editors for AVRs.


