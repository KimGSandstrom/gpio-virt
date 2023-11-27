This test program is made for NVIDIA Jetson AGX Orin
Makefile will compile 'libgpiod.c' to 'gpio' binary which can be used to test gpio pins.

Examples of usage:
./gpio gpiochip1 PBB.00
./gpio gpiochip1 PBB.01
./gpio gpiochip0 PH.00 
./gpio gpiochip0 PN.01 
./gpio gpiochip0 PP.04 

Short list of free unused GPIO pins:
GPIO09 	GPIO3_PBB.00 	40-pin header Pin 32 GPIO/AO DMIC       <- gpiochip1
GPIO08 	GPIO3_PBB.01 	40-pin header Pin 16 GPIO/AO DMIC	<- gpiochip1
GPIO35 	GPIO3_PH.00 	40-pin header Pin 18 GPIO/PWM           <- gpiochip0
GPIO27 	GPIO3_PN.01 	40-pin header Pin 15 GPIO/PWM           <- gpiochip0
GPIO17 	GPIO3_PP.04 	40-pin header Pin 22 GPIO      		<- gpiochip0 

