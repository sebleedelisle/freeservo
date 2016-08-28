CD V3.10 updates:

- Updated the PIC24F2431 code to compile with MPLAB IDE V8.60
- Rewrote the config directive 
- Removed the PDC3H/L referance from all projects, which are not available on PIC18F2431



---------------------------------------------------------------------------------------------------------------

PICDEM MC LV : BLDC motor control evaluation board:
Installation and documentation files:


This CD is intended to be used with the PICDEM MC LV evaluation kit.
The following software/documents are available on this installation CD:

1) Motor COntrol GUI		- Motor control evaluation software GUI
	To install click on the setup.exe and follow the installation guidelines
	System Requirements :

	Processor		:	Pentium 133MHz or above
	Operating system	:	Windows 98SE, 2000, XP, ME
	Memory 			:	16MB hard drive, 16 MB RAM

2) PICDEM MC LV User's Guide	- Users guide for the PICDEM MC LV board and above software

3) Demo application files in "Code Examples" directory
	- This includes application example for PIC18Fxx31 and dsPIC30F devices.
				
	PIC18Fxx31 Examples: This has four sub-directories
		A) Program Hex File : This has the Hex code that is programmed in to the PIC18F2431 
					mounted on the board. At any time if the user want to revert back to the beginning, 
					this file can be used to re programmed the PIC.
		B) Sensored_With GUI : This directory has assembly files, include files and project files for BLDC motor 					control using Hall sensors, and the motor controlled using the supplied GUI.
		C) Sensorless 	: This directory has assembly files, include files and project files for sensorless BLDC 					motor control. The present version does not communicate with the supplied GUI.
		D) Sensored_Without GUI : This directory has assembly files, include files and project files for BLDC motor 					control using Hall sensors. The present version does not communicate with the supplied GUI.
	
	dsPIC30F Examples: This has three sub-directories
		A) Program Hex File : This has the Hex code that is programmed in to the dsPIC30F3011 supplied with the 					board. At any time if the user want to revert back to the beginning, this file can be 					used to re programmed the PIC.
		B) Sensorless 	: This directory has C files, header files and project files for sensorless BLDC 					motor control. The present version does not communicate with the supplied GUI.
		D) Sensored : This directory has C files, header files and project files for BLDC motor 					control using Hall sensors. The present version does not communicate with the supplied GUI.

4) Application Notes: 	This directory has applications notes released as of Feb 2006. At Microchip we update application notes and publish new ones time to time. Please visit www.microchip.com for latest additions and modifications.