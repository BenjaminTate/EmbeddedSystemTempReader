# EmbeddedSystemTempReader
This C program is to be used with a Microcontroller Board: Texas Instruments MSP430FR6989 LaunchPad

Pressing the left buttom on the board will display the temperature in Farhenheit and the right button will display the temperature in Celsius

When computer sends a command through UART to the board, the CPU on the board will calculate the average of the 10 temperatures in oC collected in the past 10 seconds. The board will then send back the result to your computer.

# The Board
![MSP-EXP430FR4133_DSL](https://user-images.githubusercontent.com/47281119/147507340-f47df1dd-c2b8-4553-af72-e2fa7bdde0d7.jpg)
