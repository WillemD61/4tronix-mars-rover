# 4tronix M.A.R.S. Rover installation

This installation guide explains how to install the basic software for your 4tronix M.A.R.S. Rover on a Raspberry Pi Zero so you can then manually control it from a PC/laptop. Future additions are described at the end.

The ultimate aim is to develop an autonomous (i.e. without human interference) search and rescue robot using the 4tronix M.A.R.S. Rover platform.

The base software is based on the test software provided by 4tronix but then integrated into one total library, called from one separate control program. One of the main additions is the Ackermann steering geometry so that the wheels turn to the correct position to ensure minimum slippage (Wikipedia for more info). Also the mast position and LED color and behaviour is linked to the driving behaviour.

This guide assumes basic knowledge about installing and configuring a Raspberry Pi. For more info visit raspberrypi.com and go to the documentation section.

# Installation steps:

1)	Assemble the 4tronix M.A.R.S Rover as per the instructions on the 4tronix website, but do not attach the wheels to the motors yet (so skip step 9 for now). 
Note the following changes have been applied to the 4tronix setup:
*	The mast has been attached to pin 24 instead of pin 23 to allow a bit more turning space.
*	The servo calibration is now based on three reference points per wheel instead of one. Calibration is to be done without the wheels for this reason.
2)	Install the Raspberry Pi operating system for the P Zero W with the Raspberry Pi Imager. For the basic Rover setup you can use Bullseye Lite version (without desktop), with SSH enabled, with your WIFI network configured, with your own choice of hostname, user and password.
3)	Install PuTTY and related programs on your PC/laptop to give you a terminal window to your Raspberry Pi (headless setup).
4)	Once the image installation is complete, insert the SD card in your Raspberry Pi and start it up. Find the IP address, start PuTTY, open the IP address and login into the system en then run “sudo apt update” and “sudo apt upgrade”. 
5)	Run “sudo raspi-config” and enable SPI and I2C in the interface options, followed by a reboot.
6)	Run “sudo apt install python3-pip” to install pip, which in turn is used to install other packages.
7)	Run “sudo pip install rpi_ws281x” to enable control of the LEDs on your Rover.
8)	Run “sudo pip install smbus2” to enable control of the SERVOs on your Rover.
9)	Create a subdirectory (using “mkdir basic-setup”) and change to that directory (using “cd basic-setup”). This is where the basic rover software will be installed.
10)	Copy the following files from  github and then, using PSFTP (which comes with the PuTTY installation), transfer the files to the Raspberry Pi basic-setup directory.
*	roverlib.py
*	manualcontrols.py
*	fullServoCalibrate.py
11)	Run “sudo python3 fullServoCalibrate.py” to perform the calibration of the four corner servos. Follow the on-screen instructions to select each servo in turn and move each one into the 3 positions (right 90 degrees,middle,left 90 degrees). Use the arrow keys to make the positions as accurate as possible and press s when ready.
12)	After the calibration you can attach the wheels to the motors.
13)	Now you can run “sudo python3 manualcontrol.py” to drive and control your M.A.R.S Rover using the keyboard of your PC/laptop. The lights and mast will follow the movements. Note that obstacle detection is active and the Rover will therefore stop (and show red brakelights) if an obstacle is detected ahead. The keys to control your M.A.R.S Rover are shown on screen.

# Have fun !!!   

# Further development

Some further development has been done but has not been published yet. 

The ultimate aim is to develop an autonomous (i.e. without human interference) search and rescue robot. Step 1 to 5 below were already succesfully implemented on the Raspberry Pi Zero, but during further development it was found that the Raspberry Pi Zero is too heavily loaded if all functions need to be performed on the Raspberry Pi, so the plan now it to develop communication between the Raspberry Pi and a laptop so some higher level functions can then be offloaded to the laptop.

To be published in the near future:
1) More high level controls, such as "drive straight" and "drive arc" functions.
2) Color and object recognition using the camera.
3) Navigation using the camera.
4) The addition of wheelcounters and a grabber/claw. (implementation of MAG sensors failed so far)
5) Autonomous behaviour to search and rescue a human (playmobil) and then return home.
6) Map building, route tracking and route planning.
7) mqtt implementation to offload some high level tasks to a PC/laptop.
