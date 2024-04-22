/*
	PGA460_USSC.cpp
	
	BSD 2-clause "Simplified" License
	Copyright (c) 2017, Texas Instruments
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY TEXAS INSTRUMENTS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL TEXAS INSTRUMENTS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	The views and conclusions contained in the software and documentation are those
	of the authors and should not be interpreted as representing official policies,
	either expressed or implied, of the FreeBSD Project.
	
	Last Updated: Jul 2019
	By: A. Whitehead <make@energia.nu>
*/

#include "PGA460_USSC.h"
int serial_fd;
#define EnUART		//enables UART communication at TXD and RXD
#define EnOWU		//enables OWU communication at IO
#define EnTCI		//enables TCI communication at TCI
//#define EnSPI		//enables SPI communication at TXD, RXD, and SCLK
#define EnTri		//enables triangulation
#define EnAutoThr	//enables autotset thresholding

#ifdef ENUART  // Include for UART communication
#include <fcntl.h>
#include <termios.h>
#endif

/*------------------------------------------------- Global Variables -----
 |  Global Variables
 |
 |  Purpose:  Variables shared throughout the PGA460_USSC.cpp functions
 *-------------------------------------------------------------------*/
#pragma region globals
// Pin mapping of BOOSTXL-PGA460 to LaunchPad by pin name
	#define DECPL_A 2
	#define RXD_LP 3
	#define TXD_LP 4
	#define DECPL_D 5
	#define TEST_A 6
	#define TCI_CLK 7
	#define TEST_D 8
	#define MEM_SOMI 9
	#define MEM_SIMO 10
	#define TCI_RX 14
	#define TCI_TX 15
	#define COM_SEL 17
	#define COM_PD 18
	#define SPI_CS 33
	#define SCLK_CLK 34
	#define MEM_HOLD 36
	#define MEM_CS 37
	#define DS1_LED 38
	#define F_DIAG_LED 39
	#define V_DIAG_LED 40
	
	#define GPIO_DI 0


	
// Serial read timeout in milliseconds
	#define MAX_MILLIS_TO_WAIT 350//250

//Energia to Arduino mapping
	uint8_t  RED_LED = 31;
	uint8_t  GREEN_LED = 32;
	
// Define UART commands by name
	// Single Address
		uint8_t  P1BL = 0x00;
		uint8_t  P2BL = 0x01;
		uint8_t  P1LO = 0x02;
		uint8_t  P2LO = 0x03;
		uint8_t  TNLM = 0x04;
		uint8_t  UMR = 0x05;
		uint8_t  TNLR = 0x06;
		uint8_t  TEDD = 0x07;
		uint8_t  SD = 0x08;
		uint8_t  SRR = 0x09; 
		uint8_t  SRW = 0x0A;
		uint8_t  EEBR = 0x0B;
		uint8_t  EEBW = 0x0C;
		uint8_t  TVGBR = 0x0D;
		uint8_t  TVGBW = 0x0E;
		uint8_t  THRBR = 0x0F;
		uint8_t  THRBW = 0x10; 
	//Broadcast
		uint8_t  BC_P1BL = 0x11;
		uint8_t  BC_P2BL = 0x12;
		uint8_t  BC_P1LO = 0x13;
		uint8_t  BC_P2LO = 0x14;
		uint8_t  BC_TNLM = 0x15;
		uint8_t  BC_RW = 0x16;
		uint8_t  BC_EEBW = 0x17;
		uint8_t  BC_TVGBW = 0x18;
		uint8_t  BC_THRBW = 0x19;
		//CMDs 26-31 are reserved

// List user registers by name with default settings from TI factory
	uint8_t  USER_DATA1 = 0x00;
	uint8_t  USER_DATA2 = 0x00;
	uint8_t  USER_DATA3 = 0x00;
	uint8_t  USER_DATA4 = 0x00;
	uint8_t  USER_DATA5 = 0x00;
	uint8_t  USER_DATA6 = 0x00;
	uint8_t  USER_DATA7 = 0x00;
	uint8_t  USER_DATA8 = 0x00;
	uint8_t  USER_DATA9 = 0x00;
	uint8_t  USER_DATA10 = 0x00;
	uint8_t  USER_DATA11 = 0x00;
	uint8_t  USER_DATA12 = 0x00;
	uint8_t  USER_DATA13 = 0x00;
	uint8_t  USER_DATA14 = 0x00;
	uint8_t  USER_DATA15 = 0x00;
	uint8_t  USER_DATA16 = 0x00;
	uint8_t  USER_DATA17 = 0x00;
	uint8_t  USER_DATA18 = 0x00;
	uint8_t  USER_DATA19 = 0x00;
	uint8_t  USER_DATA20 = 0x00;
	uint8_t  TVGAIN0 = 0xAF;
	uint8_t  TVGAIN1 = 0xFF;
	uint8_t  TVGAIN2 = 0xFF;
	uint8_t  TVGAIN3 = 0x2D;
	uint8_t  TVGAIN4 = 0x68;
	uint8_t  TVGAIN5 = 0x36;
	uint8_t  TVGAIN6 = 0xFC;
	uint8_t  INIT_GAIN = 0xC0;
	uint8_t  FREQUENCY  = 0x8C;
	uint8_t  DEADTIME = 0x00;
	uint8_t  PULSE_P1 = 0x01;
	uint8_t  PULSE_P2 = 0x12;
	uint8_t  CURR_LIM_P1 = 0x47;
	uint8_t  CURR_LIM_P2 = 0xFF;
	uint8_t  REC_LENGTH = 0x1C;
	uint8_t  FREQ_DIAG = 0x00;
	uint8_t  SAT_FDIAG_TH = 0xEE;
	uint8_t  FVOLT_DEC = 0x7C;
	uint8_t  DECPL_TEMP = 0x0A;
	uint8_t  DSP_SCALE = 0x00;
	uint8_t  TEMP_TRIM = 0x00;
	uint8_t  P1_GAIN_CTRL = 0x00;
	uint8_t  P2_GAIN_CTRL = 0x00;
	uint8_t  EE_CRC = 0xFF;
	uint8_t  EE_CNTRL = 0x00;
	uint8_t  P1_THR_0 = 0x88;
	uint8_t  P1_THR_1 = 0x88;
	uint8_t  P1_THR_2 = 0x88;
	uint8_t  P1_THR_3 = 0x88;
	uint8_t  P1_THR_4 = 0x88;
	uint8_t  P1_THR_5 = 0x88;
	uint8_t  P1_THR_6 = 0x84;
	uint8_t  P1_THR_7 = 0x21;
	uint8_t  P1_THR_8 = 0x08;
	uint8_t  P1_THR_9 = 0x42;
	uint8_t  P1_THR_10 = 0x10;
	uint8_t  P1_THR_11 = 0x80;
	uint8_t  P1_THR_12 = 0x80;
	uint8_t  P1_THR_13 = 0x80;
	uint8_t  P1_THR_14 = 0x80;
	uint8_t  P1_THR_15 = 0x80;
	uint8_t  P2_THR_0 = 0x88;
	uint8_t  P2_THR_1 = 0x88;
	uint8_t  P2_THR_2 = 0x88;
	uint8_t  P2_THR_3 = 0x88;
	uint8_t  P2_THR_4 = 0x88;
	uint8_t  P2_THR_5 = 0x88;
	uint8_t  P2_THR_6 = 0x84;
	uint8_t  P2_THR_7 = 0x21;
	uint8_t  P2_THR_8 = 0x08;
	uint8_t  P2_THR_9 = 0x42;
	uint8_t  P2_THR_10 = 0x10;
	uint8_t  P2_THR_11 = 0x80;
	uint8_t  P2_THR_12 = 0x80;
	uint8_t  P2_THR_13 = 0x80;
	uint8_t  P2_THR_14 = 0x80;
	uint8_t  P2_THR_15 = 0x80;

// Miscellaneous variables; (+) indicates OWU transmitted uint8_t  offset
	uint8_t  checksum = 0x00; 			// UART checksum value	
	uint8_t  ChecksumInput[44]; 		// data uint8_t  array for checksum calculator
	uint8_t  ultraMeasResult[34+3]; 	// data uint8_t  array for cmd5 and tciB+L return
	uint8_t  diagMeasResult[5+3]; 		// data uint8_t  array for cmd8 and index1 return
	uint8_t  tempNoiseMeasResult[4+3]; 	// data uint8_t  array for cmd6 and index0&1 return
	uint8_t  echoDataDump[130+3]; 		// data uint8_t  array for cmd7 and index12 return
	uint8_t  tempOrNoise = 0; 			// data uint8_t  to determine if temp or noise measurement is to be performed
	uint8_t  comm = 0; 					// indicates UART (0), TCI (1), OWU (2) communication mode	
	unsigned long starttime; 			// used for function time out
	uint8_t  bulkThr[34+3];				// data uint8_t  array for bulk threhsold commands
	//UART & OWU exclusive variables
		uint8_t  syncByte  = 0x55; 		// data uint8_t  for Sync field set UART baud rate of PGA460
		uint8_t  regAddr = 0x00; 		// data uint8_t  for Register Address
		uint8_t  regData = 0x00; 		// data uint8_t  for Register Data
		uint8_t  uartAddr = 0; 			// PGA460 UART device address (0-7). '0' is factory default address
		uint8_t  numObj = 1; 			// number of objects to detect
		//OWU exclusive variables
			signed int owuShift = 0;	// accoutns for OWU receiver buffer offset for capturing master transmitted data - always 0 for standard two-wire UART
	//TCI exclusive variables
		uint8_t  bufRecv[128]; 			// TCI receive data buffer for all commands	
		unsigned long tciToggle;		// used to log TCI burst+listen time of object
		unsigned int objTime[8];		// array to capture up to eight object TCI burst+listen toggles
	//SPI exclusive variables
		uint8_t  misoBuf[131]; 			// SPI MISO receive data buffer for all commands	
#pragma endregion globals

/*------------------------------------------------- PGA460 Top Level -----
 |  PGA460 Top Level Scope Resolution Operator
 |
 | Use the double colon operator (::) to qualify a C++ member function, a top
 | level function, or a variable with global scope with:
 | • An overloaded name (same name used with different argument types)
 | • An ambiguous name (same name used in different classes)
 *-------------------------------------------------------------------*/
pga460::pga460(){}

/*------------------------------------------------- initBoostXLPGA460 -----
 |  Function initBoostXLPGA460
 |
 |  Purpose:  Configure the master communication mode and BOOSTXL-PGA460 hardware to operate in UART, TCI, or OWU mode.
 |  Configures master serial baud rate for UART/OWU modes. Updates UART address based on sketch input.
 |
 |  Parameters:
 |		mode (IN) -- sets communicaiton mode. 
 |			0=UART 
 |			1=TCI 
 |			2=OWU 
 |			3-SPI (Synchronous Mode)
 |			4 = Not Used
 |			5 = Not Used
 |			6=Bus_Demo_Bulk_TVG_or_Threshold_Broadcast_is_True
 |			7=Bus_Demo_UART_Mode
 |			8=Bus_Demo_OWU_One_Time_Setup
 |			9=Bus_Demo_OWU_Mode
 | 		baud (IN) -- PGA460 accepts a baud rate of 9600 to 115.2k bps
 | 		uartAddrUpdate (IN) -- PGA460 address range from 0 to 7
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initBoostXLPGA460(uint8_t mode, uint32_t baud, uint8_t uartAddrUpdate) 
{
    serial_fd = serialOpen(UART_PORT, baud);
	// Check for errors in opening the serial port
	if (serial_fd < 0) 
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		// Handle the error
	}


    // Check for valid UART address
    if (uartAddrUpdate > 7) 
	{
        uartAddrUpdate = 0; // default to '0'
        printf("ERROR - Invalid UART Address!\n");
    }
    // Globally update target PGA460 UART address and commands	
    if (uartAddr != uartAddrUpdate) 
	{
        // Update commands to account for new UART addr
        // Single Address
        P1BL = 0x00 + (uartAddrUpdate << 5);	   
		   P2BL = 0x01 + (uartAddrUpdate << 5);
		   P1LO = 0x02 + (uartAddrUpdate << 5);
		   P2LO = 0x03 + (uartAddrUpdate << 5);
		   TNLM = 0x04 + (uartAddrUpdate << 5);
		   UMR = 0x05 + (uartAddrUpdate << 5);
		   TNLR = 0x06 + (uartAddrUpdate << 5);
		   TEDD = 0x07 + (uartAddrUpdate << 5);
		   SD = 0x08 + (uartAddrUpdate << 5);
		   SRR = 0x09 + (uartAddrUpdate << 5); 
		   SRW = 0x0A + (uartAddrUpdate << 5);
		   EEBR = 0x0B + (uartAddrUpdate << 5);
		   EEBW = 0x0C + (uartAddrUpdate << 5);
		   TVGBR = 0x0D + (uartAddrUpdate << 5);
		   TVGBW = 0x0E + (uartAddrUpdate << 5);
		   THRBR = 0x0F + (uartAddrUpdate << 5);
		   THRBW = 0x10 + (uartAddrUpdate << 5);
    }
    uartAddr = uartAddrUpdate;

#if GPIO_DI
    // Turn on LP's Red LED to indicate code has started to run
    pinMode(RED_LED, OUTPUT);
    digitalWrite(RED_LED, HIGH);

    // Turn off BOOSTXL-PGA460's diagnostic LEDs
    pinMode(DS1_LED, OUTPUT);
    digitalWrite(DS1_LED, LOW);

    pinMode(F_DIAG_LED, OUTPUT);
    digitalWrite(F_DIAG_LED, LOW);

    pinMode(V_DIAG_LED, OUTPUT);
    digitalWrite(V_DIAG_LED, LOW);
#endif
    // Set communication mode flag
    if (mode < 4) 
	{ // 0=UART, 1=TCI, 2=OWU, 3=SPI
        comm = mode;
        // Disable synchronous mode dump to external memory
#if GPIO_DI
        pinMode(MEM_HOLD, OUTPUT);
        digitalWrite(MEM_HOLD, HIGH);
        pinMode(MEM_CS, OUTPUT);
        digitalWrite(MEM_CS, HIGH);
#endif
    } 
	else if (mode == 6) 
	{
        comm = 6; // bus demo user input mode only, and threshold or TVG bulk write broadcast commands are true
    } 
	else if ((mode == 7) || (mode == 9)) 
	{
        comm = mode - 7; // bus demo only for either UART or OWU mode
    } 
	else 
	{
        comm = 99; // invalid communication type
    }

    switch (mode) 
	{
        case 0: // UART Mode
            // Enable PGA460 UART communication mode
#if GPIO_DI
            pinMode(COM_PD, OUTPUT);
            digitalWrite(COM_PD, LOW);
            pinMode(COM_SEL, OUTPUT);
            digitalWrite(COM_SEL, LOW);
#endif
            serialOpen(UART_PORT, baud); // ToDO: Check this, Initialize COM UART serial channel
            //serialOpen(UART_PORT, baud); // ToDO: Check this, Initialize PGA460 UART serial channel
            // Set timeout for serial communication
            // serialGetcharTimeout(250);
            break;
        case 1: // TCI Mode	
            // Enable PGA460 TCI communication mode
            pinMode(COM_PD, OUTPUT);
            digitalWrite(COM_PD, LOW);
            pinMode(COM_SEL, OUTPUT);
            digitalWrite(COM_SEL, LOW);
            pinMode(TCI_TX, OUTPUT);
            digitalWrite(TCI_TX, HIGH);
            pinMode(TCI_RX, INPUT);
            serialOpen(UART_PORT, baud); // Initialize COM UART serial channel
            serialOpen(UART_PORT, baud); // Initialize PGA460 UART serial channel
            // Set timeout for serial communication
            // serialGetcharTimeout(250);
            break;
        case 2: // OWU setup (part I)
            // Enable PGA460 UART communication mode
            pinMode(COM_PD, OUTPUT);
            digitalWrite(COM_PD, LOW);
            pinMode(COM_SEL, OUTPUT);
            digitalWrite(COM_SEL, LOW);
            serialOpen(UART_PORT, baud); // Initialize COM UART serial channel
            serialOpen(UART_PORT, baud); // Initialize PGA460 UART serial channel
            // Update IO_IF_SEL bit to '1' for OWU mode for bulk EEPROM write
            // PULSE_P1 = 0x80 | PULSE_P1;
            break;
        case 3: // SPI mode

            break;
        default:
            break;
    }

    // OWU setup (part II)
    if ((comm == 2) || (mode == 8)) 
	{ // mode8 is for one-time setup of OWU per slave device for bus demo
        // UART write to register PULSE_P1 (addr 0x1E) to set device into OWU mode
        // Fill in UART write code here
        // enable PGA460 OWU communication mode
        pinMode(COM_SEL, OUTPUT);
        digitalWrite(COM_SEL, HIGH);
    }


    // Visibly show initialization status
    pinMode(GREEN_LED, OUTPUT);
    if ((mode == 7) || (mode == 9)) 
	{
        // Do not delay bus demo loop by blinking Green LED
        digitalWrite(RED_LED, LOW); // turn off LaunchPad's Red LED
    } 
	else 
	{ // blink LP's Green LED twice to indicate initialization is complete
        digitalWrite(GREEN_LED, HIGH);
        for(int loops = 0; loops < 5; loops++) 
		{
            digitalWrite(GREEN_LED, HIGH);
            delay(200);
            digitalWrite(GREEN_LED, LOW);
            delay(200);
        }
    }
}

/*------------------------------------------------- defaultPGA460 -----
 |  Function defaultPGA460
 |
 |  Purpose:  Updates user EEPROM values, and performs bulk EEPROM write.
 |
 |  Parameters:
 |		xdcr (IN) -- updates user EEPROM based on predefined listing for a specific transducer.
 |			Modify existing case statements, or append additional case-statement for custom user EEPROM configurations.
 |			• 0 = Murata MA58MF14-7N
 |			• 1 = Murata MA40H1S-R
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
	void pga460::defaultPGA460(uint8_t  xdcr)
	{
		switch (xdcr)
		{
			case 0: // Murata MA58MF14-7N
			USER_DATA1 = 0x00;
			USER_DATA2 = 0x00;
			USER_DATA3 = 0x00;
			USER_DATA4 = 0x00;
			USER_DATA5 = 0x00;
			USER_DATA6 = 0x00;
			USER_DATA7 = 0x00;
			USER_DATA8 = 0x00;
			USER_DATA9 = 0x00;
			USER_DATA10 = 0x00;
			USER_DATA11 = 0x00;
			USER_DATA12 = 0x00;
			USER_DATA13 = 0x00;
			USER_DATA14 = 0x00;
			USER_DATA15 = 0x00;
			USER_DATA16 = 0x00;
			USER_DATA17 = 0x00;
			USER_DATA18 = 0x00;
			USER_DATA19 = 0x00;
			USER_DATA20 = 0x00;
			TVGAIN0 = 0xAA;
			TVGAIN1 = 0xAA;
			TVGAIN2 = 0xAA;
			TVGAIN3 = 0x82;
			TVGAIN4 = 0x08;
			TVGAIN5 = 0x20;
			TVGAIN6 = 0x80;
			INIT_GAIN = 0x60;
			FREQUENCY  = 0x8F;
			DEADTIME = 0xA0;
			if (comm == 2)
			{
					PULSE_P1 = 0x80 | 0x04;
			}
			else
			{
					PULSE_P1 = 0x04;
			}
			PULSE_P2 = 0x10;
			CURR_LIM_P1 = 0x55;
			CURR_LIM_P2 = 0x55;
			REC_LENGTH = 0x19;
			FREQ_DIAG = 0x33;
			SAT_FDIAG_TH = 0xEE;
			FVOLT_DEC = 0x7C;
			DECPL_TEMP = 0x4F;
			DSP_SCALE = 0x00;
			TEMP_TRIM = 0x00;
			P1_GAIN_CTRL = 0x09;
			P2_GAIN_CTRL = 0x09;
				break;		
			case 1: // Murata MA40H1SR
			USER_DATA1 = 0x00;
			USER_DATA2 = 0x00;
			USER_DATA3 = 0x00;
			USER_DATA4 = 0x00;
			USER_DATA5 = 0x00;
			USER_DATA6 = 0x00;
			USER_DATA7 = 0x00;
			USER_DATA8 = 0x00;
			USER_DATA9 = 0x00;
			USER_DATA10 = 0x00;
			USER_DATA11 = 0x00;
			USER_DATA12 = 0x00;
			USER_DATA13 = 0x00;
			USER_DATA14 = 0x00;
			USER_DATA15 = 0x00;
			USER_DATA16 = 0x00;
			USER_DATA17 = 0x00;
			USER_DATA18 = 0x00;
			USER_DATA19 = 0x00;
			USER_DATA20 = 0x00;
			TVGAIN0 = 0xAA;
			TVGAIN1 = 0xAA;
			TVGAIN2 = 0xAA;
			TVGAIN3 = 0x51;
			TVGAIN4 = 0x45;
			TVGAIN5 = 0x14;
			TVGAIN6 = 0x50;
			INIT_GAIN = 0x54;
			FREQUENCY  = 0x32;
			DEADTIME = 0xA0;
			if (comm == 2)
			{
					PULSE_P1 = 0x80 | 0x08;
			}
			else
			{
					PULSE_P1 = 0x08;
			}
			PULSE_P2 = 0x10;
			CURR_LIM_P1 = 0x40;
			CURR_LIM_P2 = 0x40;
			REC_LENGTH = 0x19;
			FREQ_DIAG = 0x33;
			SAT_FDIAG_TH = 0xEE;
			FVOLT_DEC = 0x7C;
			DECPL_TEMP = 0x4F;
			DSP_SCALE = 0x00;
			TEMP_TRIM = 0x00;
			P1_GAIN_CTRL = 0x09;
			P2_GAIN_CTRL = 0x09;
				break;
			case 2: // user custom //DEBUG
			{
				USER_DATA1 = 0x00;
			USER_DATA2 = 0x00;
			USER_DATA3 = 0x00;
			USER_DATA4 = 0x00;
			USER_DATA5 = 0x00;
			USER_DATA6 = 0x00;
			USER_DATA7 = 0x00;
			USER_DATA8 = 0x00;
			USER_DATA9 = 0x00;
			USER_DATA10 = 0x00;
			USER_DATA11 = 0x00;
			USER_DATA12 = 0x00;
			USER_DATA13 = 0x00;
			USER_DATA14 = 0x00;
			USER_DATA15 = 0x00;
			USER_DATA16 = 0x00;
			USER_DATA17 = 0x00;
			USER_DATA18 = 0x00;
			USER_DATA19 = 0x00;
			USER_DATA20 = 0x00;
			TVGAIN0 = 0xAA;
			TVGAIN1 = 0xAA;
			TVGAIN2 = 0xAA;
			TVGAIN3 = 0x82;
			TVGAIN4 = 0x08;
			TVGAIN5 = 0x20;
			TVGAIN6 = 0x80;
			INIT_GAIN = 0x60;
			FREQUENCY  = 0x8F;
			DEADTIME = 0xA0;
			if (comm == 2)
			{
					PULSE_P1 = 0x80 | 0x04;
			}
			else
			{
					PULSE_P1 = 0x04;
			}
			PULSE_P2 = 0x50; //UART_ADDR=2
			CURR_LIM_P1 = 0x55;
			CURR_LIM_P2 = 0x55;
			REC_LENGTH = 0x19;
			FREQ_DIAG = 0x33;
			SAT_FDIAG_TH = 0xEE;
			FVOLT_DEC = 0x7C;
			DECPL_TEMP = 0x4F;
			DSP_SCALE = 0x00;
			TEMP_TRIM = 0x00;
			P1_GAIN_CTRL = 0x09;
			P2_GAIN_CTRL = 0x09;
			break;
			}	
			case 3: // user custom //DEBUG
			{
				USER_DATA1 = 0x00;
			USER_DATA2 = 0x00;
			USER_DATA3 = 0x00;
			USER_DATA4 = 0x00;
			USER_DATA5 = 0x00;
			USER_DATA6 = 0x00;
			USER_DATA7 = 0x00;
			USER_DATA8 = 0x00;
			USER_DATA9 = 0x00;
			USER_DATA10 = 0x00;
			USER_DATA11 = 0x00;
			USER_DATA12 = 0x00;
			USER_DATA13 = 0x00;
			USER_DATA14 = 0x00;
			USER_DATA15 = 0x00;
			USER_DATA16 = 0x00;
			USER_DATA17 = 0x00;
			USER_DATA18 = 0x00;
			USER_DATA19 = 0x00;
			USER_DATA20 = 0x00;
			TVGAIN0 = 0xAA;
			TVGAIN1 = 0xAA;
			TVGAIN2 = 0xAA;
			TVGAIN3 = 0x82;
			TVGAIN4 = 0x08;
			TVGAIN5 = 0x20;
			TVGAIN6 = 0x80;
			INIT_GAIN = 0x60;
			FREQUENCY  = 0x8F;
			DEADTIME = 0xA0;
			if (comm == 2)
			{
					PULSE_P1 = 0x80 | 0x04;
			}
			else
			{
					PULSE_P1 = 0x04;
			}
			PULSE_P2 = 0x70; //UART_ADDR=3
			CURR_LIM_P1 = 0x55;
			CURR_LIM_P2 = 0x55;
			REC_LENGTH = 0x19;
			FREQ_DIAG = 0x33;
			SAT_FDIAG_TH = 0xEE;
			FVOLT_DEC = 0x7C;
			DECPL_TEMP = 0x4F;
			DSP_SCALE = 0x00;
			TEMP_TRIM = 0x00;
			P1_GAIN_CTRL = 0x09;
			P2_GAIN_CTRL = 0x09;
			break;
			}	
			case 4: // user custom //DEBUG
			{
				USER_DATA1 = 0x00;
			USER_DATA2 = 0x00;
			USER_DATA3 = 0x00;
			USER_DATA4 = 0x00;
			USER_DATA5 = 0x00;
			USER_DATA6 = 0x00;
			USER_DATA7 = 0x00;
			USER_DATA8 = 0x00;
			USER_DATA9 = 0x00;
			USER_DATA10 = 0x00;
			USER_DATA11 = 0x00;
			USER_DATA12 = 0x00;
			USER_DATA13 = 0x00;
			USER_DATA14 = 0x00;
			USER_DATA15 = 0x00;
			USER_DATA16 = 0x00;
			USER_DATA17 = 0x00;
			USER_DATA18 = 0x00;
			USER_DATA19 = 0x00;
			USER_DATA20 = 0x00;
			TVGAIN0 = 0xAA;
			TVGAIN1 = 0xAA;
			TVGAIN2 = 0xAA;
			TVGAIN3 = 0x82;
			TVGAIN4 = 0x08;
			TVGAIN5 = 0x20;
			TVGAIN6 = 0x80;
			INIT_GAIN = 0x60;
			FREQUENCY  = 0x8F;
			DEADTIME = 0xA0;
			if (comm == 2)
			{
					PULSE_P1 = 0x80 | 0x04;
			}
			else
			{
					PULSE_P1 = 0x04;
			}
			PULSE_P2 = 0x90; //UART_ADDR=4
			CURR_LIM_P1 = 0x55;
			CURR_LIM_P2 = 0x55;
			REC_LENGTH = 0x19;
			FREQ_DIAG = 0x33;
			SAT_FDIAG_TH = 0xEE;
			FVOLT_DEC = 0x7C;
			DECPL_TEMP = 0x4F;
			DSP_SCALE = 0x00;
			TEMP_TRIM = 0x00;
			P1_GAIN_CTRL = 0x09;
			P2_GAIN_CTRL = 0x09;
			break;
			}			
			default: break;
		}
			
			if ((comm == 0 || comm == 2 || comm ==3) && (comm !=6)) // USART or OWU mode and not busDemo6
			{
				uint8_t  buf12[46] = {syncByte , EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
					USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, 
					USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
					TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
					PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
					DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(EEBW)};
			
				if (comm == 0 || comm == 2) // UART or OWU mode
				{
//					serialOpen(SERIAL_PORT, 9600); // Open serial port
//					serialPuts(SERIAL_PORT, buf12); // Write data to serial port
//					serialClose(SERIAL_PORT); // Close serial port
//					int serial_fd2 = serialOpen(SERIAL_PORT, 9600); // Open serial port
					int serial_fd2 = serialOpen(UART_PORT, 115200); // Open serial port
					if (serial_fd2 != -1) 
					{
    					//serialPuts(serial_fd2, buf12); // Write data to serial port
						serialPuts(serial_fd2, reinterpret_cast<const char*>(buf12)); // Write data to serial port

    					serialClose(serial_fd2); // Close serial port
					} 
					else 
					{
    					// Handle error opening serial port
						std::cout << "ERROR OPENING SERIAL"<< std::endl;
					}

				}

				delay(50);
				
				// Update targeted UART_ADDR to address defined in EEPROM bulk switch-case
				uint8_t  uartAddrUpdate = (PULSE_P2 >> 5) & 0x07;
				if (uartAddr != uartAddrUpdate)
				{
					// Update commands to account for new UART addr
					// Single Address
					P1BL = 0x00 + (uartAddrUpdate << 5);	   
					P2BL = 0x01 + (uartAddrUpdate << 5);
					P1LO = 0x02 + (uartAddrUpdate << 5);
					P2LO = 0x03 + (uartAddrUpdate << 5);
					TNLM = 0x04 + (uartAddrUpdate << 5);
					UMR = 0x05 + (uartAddrUpdate << 5);
					TNLR = 0x06 + (uartAddrUpdate << 5);
					TEDD = 0x07 + (uartAddrUpdate << 5);
					SD = 0x08 + (uartAddrUpdate << 5);
					SRR = 0x09 + (uartAddrUpdate << 5); 
					SRW = 0x0A + (uartAddrUpdate << 5);
					EEBR = 0x0B + (uartAddrUpdate << 5);
					EEBW = 0x0C + (uartAddrUpdate << 5);
					TVGBR = 0x0D + (uartAddrUpdate << 5);
					TVGBW = 0x0E + (uartAddrUpdate << 5);
					THRBR = 0x0F + (uartAddrUpdate << 5);
					THRBW = 0x10 + (uartAddrUpdate << 5);				
				}
				uartAddr = uartAddrUpdate;
			}
			else if (comm == 6)
			{
				return;
			}
			else if (comm == 1) // TCI mode
			{			
				tciIndexRW(13, true);	// TCI index 13 write		
			}
			else
			{
				//do nothing
			}

		return;
	}

/*------------------------------------------------- initThresholds -----
 |  Function initThresholds
 |
 |  Purpose:  Updates threshold mapping for both presets, and performs bulk threshold write
 |
 |  Parameters:
 |		thr (IN) -- updates all threshold levels to a fixed level based on specific percentage of the maximum level. 
 |			All times are mid-code (1.4ms intervals).
 |			Modify existing case statements, or append additional case-statement for custom user threshold configurations.
 |			• 0 = 25% Levels 64 of 255
 |			• 1 = 50% Levels 128 of 255
 |			• 2 = 75% Levels 192 of 255
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initThresholds(uint8_t  thr)
{
	switch (thr)
	{
		case 0: //25% Levels 64 of 255
		   P1_THR_0 = 0x88;
		   P1_THR_1 = 0x88;
		   P1_THR_2 = 0x88;
		   P1_THR_3 = 0x88;
		   P1_THR_4 = 0x88;
		   P1_THR_5 = 0x88;
		   P1_THR_6 = 0x42;
		   P1_THR_7 = 0x10;
		   P1_THR_8 = 0x84;
		   P1_THR_9 = 0x21;
		   P1_THR_10 = 0x08;
		   P1_THR_11 = 0x40;
		   P1_THR_12 = 0x40;
		   P1_THR_13 = 0x40;
		   P1_THR_14 = 0x40;
		   P1_THR_15 = 0x00;
		   P2_THR_0 = 0x88;
		   P2_THR_1 = 0x88;
		   P2_THR_2 = 0x88;
		   P2_THR_3 = 0x88;
		   P2_THR_4 = 0x88;
		   P2_THR_5 = 0x88;
		   P2_THR_6 = 0x42;
		   P2_THR_7 = 0x10;
		   P2_THR_8 = 0x84;
		   P2_THR_9 = 0x21;
		   P2_THR_10 = 0x08;
		   P2_THR_11 = 0x40;
		   P2_THR_12 = 0x40;
		   P2_THR_13 = 0x40;
		   P2_THR_14 = 0x40;
		   P2_THR_15 = 0x00;
		break;
		
		case 1: //50% Level (midcode) 128 of 255
		   P1_THR_0 = 0x88;
		   P1_THR_1 = 0x88;
		   P1_THR_2 = 0x88;
		   P1_THR_3 = 0x88;
		   P1_THR_4 = 0x88;
		   P1_THR_5 = 0x88;
		   P1_THR_6 = 0x84;
		   P1_THR_7 = 0x21;
		   P1_THR_8 = 0x08;
		   P1_THR_9 = 0x42;
		   P1_THR_10 = 0x10;
		   P1_THR_11 = 0x80;
		   P1_THR_12 = 0x80;
		   P1_THR_13 = 0x80;
		   P1_THR_14 = 0x80;
		   P1_THR_15 = 0x00;
		   P2_THR_0 = 0x88;
		   P2_THR_1 = 0x88;
		   P2_THR_2 = 0x88;
		   P2_THR_3 = 0x88;
		   P2_THR_4 = 0x88;
		   P2_THR_5 = 0x88;
		   P2_THR_6 = 0x84;
		   P2_THR_7 = 0x21;
		   P2_THR_8 = 0x08;
		   P2_THR_9 = 0x42;
		   P2_THR_10 = 0x10;
		   P2_THR_11 = 0x80;
		   P2_THR_12 = 0x80;
		   P2_THR_13 = 0x80;
		   P2_THR_14 = 0x80;
		   P2_THR_15 = 0x00;		
		break;
		
		case 2: //75% Levels 192 of 255
		   P1_THR_0 = 0x88;
		   P1_THR_1 = 0x88;
		   P1_THR_2 = 0x88;
		   P1_THR_3 = 0x88;
		   P1_THR_4 = 0x88;
		   P1_THR_5 = 0x88;
		   P1_THR_6 = 0xC6;
		   P1_THR_7 = 0x31;
		   P1_THR_8 = 0x8C;
		   P1_THR_9 = 0x63;
		   P1_THR_10 = 0x18;
		   P1_THR_11 = 0xC0;
		   P1_THR_12 = 0xC0;
		   P1_THR_13 = 0xC0;
		   P1_THR_14 = 0xC0;
		   P1_THR_15 = 0x00;
		   P2_THR_0 = 0x88;
		   P2_THR_1 = 0x88;
		   P2_THR_2 = 0x88;
		   P2_THR_3 = 0x88;
		   P2_THR_4 = 0x88;
		   P2_THR_5 = 0x88;
		   P2_THR_6 = 0xC6;
		   P2_THR_7 = 0x31;
		   P2_THR_8 = 0x8C;
		   P2_THR_9 = 0x63;
		   P2_THR_10 = 0x18;
		   P2_THR_11 = 0xC0;
		   P2_THR_12 = 0xC0;
		   P2_THR_13 = 0xC0;
		   P2_THR_14 = 0xC0;
		   P2_THR_15 = 0x00;
		break;
		
		case 3: //Custom
		   P1_THR_0 = 0x41;
		   P1_THR_1 = 0x44;
		   P1_THR_2 = 0x10;
		   P1_THR_3 = 0x06;
		   P1_THR_4 = 0x69;
		   P1_THR_5 = 0x99;
		   P1_THR_6 = 0xDD;
		   P1_THR_7 = 0x4C;
		   P1_THR_8 = 0x31;
		   P1_THR_9 = 0x08;
		   P1_THR_10 = 0x42;
		   P1_THR_11 = 0x18;
		   P1_THR_12 = 0x20;
		   P1_THR_13 = 0x24;
		   P1_THR_14 = 0x2A;
		   P1_THR_15 = 0x00;
		   P2_THR_0 = 0x41;
		   P2_THR_1 = 0x44;
		   P2_THR_2 = 0x10;
		   P2_THR_3 = 0x06;
		   P2_THR_4 = 0x09;
		   P2_THR_5 = 0x99;
		   P2_THR_6 = 0xDD;
		   P2_THR_7 = 0x4C;
		   P2_THR_8 = 0x31;
		   P2_THR_9 = 0x08;
		   P2_THR_10 = 0x42;
		   P2_THR_11 = 0x24;
		   P2_THR_12 = 0x30;
		   P2_THR_13 = 0x36;
		   P2_THR_14 = 0x3C;
		   P2_THR_15 = 0x00;
		break;
		
		default: break;
	}
				  
	if ((comm == 0 || comm == 2 || comm==3) && (comm !=6)) 	// USART or OWU mode and not busDemo6
	{
		        uint8_t buf16[35] = {syncByte, THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
                              P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
                              P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6,
                              P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
                              calcChecksum(THRBW)};							  
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			int UARTPort = serialOpen(UART_PORT, 115200); // Use the appropriate serial port and baud rate
			if (UARTPort != -1) 
			{
				serialPuts(UARTPort, reinterpret_cast<char*>(buf16));
				serialClose(UARTPort);
			} 
			else 
			{
				// Handle error opening serial port
			}
		}
		
	}
	else if(comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		tciIndexRW(5, true); //TCI Threshold Preset 1 write
		tciIndexRW(6, true); //TCI Threshold Preset 2 write
	}
	else
	{
		//do nothing
	}
	
	delay(100);
	return;
}

/*------------------------------------------------- initTVG -----
 |  Function initTVG
 |
 |  Purpose:  Updates time varying gain (TVG) range and mapping, and performs bulk TVG write
 |
 |  Parameters:
 |		agr (IN) -- updates the analog gain range for the TVG.
 |			• 0 = 32-64dB
 |			• 1 = 46-78dB
 |			• 2 = 52-84dB
 |			• 3 = 58-90dB
 |		tvg (IN) -- updates all TVG levels to a fixed level based on specific percentage of the maximum level. 
 |			All times are mid-code (2.4ms intervals).
 |			Modify existing case statements, or append additional case-statement for custom user TVG configurations
 |			• 0 = 25% Levels of range
 |			• 1 = 50% Levels of range
 |			• 2 = 75% Levels of range
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initTVG(uint8_t  agr, uint8_t  tvg)
{
	uint8_t  gain_range = 0x4F;
	// set AFE gain range
	switch (agr)
	{
		case 3: //58-90dB
			gain_range =  0x0F;
			break;		
		case 2: //52-84dB
			gain_range = 0x4F;
			break;		
		case 1: //46-78dB
			gain_range = 0x8F;
			break;
		case 0: //32-64dB
			gain_range = 0xCF;
			break;
		default: break;
	}	

	if ((comm == 0 || comm == 2 || comm == 3) && (comm !=6)) 	// USART or OWU mode and not busDemo6
	{
		regAddr = 0x26;
		regData = gain_range;	
		uint8_t  buf10[5] = {syncByte , SRW, regAddr, regData, calcChecksum(SRW)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			int UARTPort = serialOpen(UART_PORT, 115200); // Use the appropriate serial port and baud rate
			if (UARTPort != -1) {
				serialPuts(UARTPort, reinterpret_cast<char*>(buf10));
				serialClose(UARTPort);
			} else {
				// Handle error opening serial port
			}
		}
	}
	else if(comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		//TODO enable index 10 write
		//tciIndexRW(10, true);
	}
	else
	{
		//do nothing
	}
	
	//Set fixed AFE gain value
	switch (tvg)
	{
		case 0: //25% Level
		   TVGAIN0 = 0x88;
		   TVGAIN1 = 0x88;
		   TVGAIN2 = 0x88;
		   TVGAIN3 = 0x41;
		   TVGAIN4 = 0x04;
		   TVGAIN5 = 0x10;
		   TVGAIN6 = 0x40;		
		break;
		
		case 1: //50% Levels
		   TVGAIN0 = 0x88;
		   TVGAIN1 = 0x88;
		   TVGAIN2 = 0x88;
		   TVGAIN3 = 0x82;
		   TVGAIN4 = 0x08;
		   TVGAIN5 = 0x20;
		   TVGAIN6 = 0x80;	
		break;
		
		case 2: //75% Levels
		   TVGAIN0 = 0x88;
		   TVGAIN1 = 0x88;
		   TVGAIN2 = 0x88;
		   TVGAIN3 = 0xC3;
		   TVGAIN4 = 0x0C;
		   TVGAIN5 = 0x30;
		   TVGAIN6 = 0xC0;	
		break;
		
		default: break;
	}	
	
	if ((comm == 0 || comm == 2 || comm == 3) && (comm !=6)) 	// USART or OWU mode and not busDemo6
	{
		uint8_t  buf14[10] = {syncByte , TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(TVGBW)};
		
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			int UARTPort = serialOpen(UART_PORT, 115200); // Use the appropriate serial port and baud rate
			if (UARTPort != -1) {
				serialPuts(UARTPort, reinterpret_cast<char*>(buf14));
				serialClose(UARTPort);
			} else {
				// Handle error opening serial port
			}
		}
	}
	else if(comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		tciIndexRW(8, true); //TCI bulk TVG write
	}
	else
	{
		//do nothing
	}
	
	return;
}

/*------------------------------------------------- ultrasonicCmd -----
 |  Function ultrasonicCmd
 |
 |  Purpose:  Issues a burst-and-listen or listen-only command based on the number of objects to be detected.
 |
 |  Parameters:
 |		cmd (IN) -- determines which preset command is run
 |			• 0 = Preset 1 Burst + Listen command
 |			• 1 = Preset 2 Burst + Listen command
 |			• 2 = Preset 1 Listen Only command
 |			• 3 = Preset 2 Listen Only command
 |			• 17 = Preset 1 Burst + Listen broadcast command
 |			• 18 = Preset 2 Burst + Listen broadcast command
 |			• 19 = Preset 1 Listen Only broadcast command
 |			• 20 = Preset 2 Listen Only broadcast command
 |		numObjUpdate (IN) -- PGA460 can capture time-of-flight, width, and amplitude for 1 to 8 objects. 
 |			TCI is limited to time-of-flight measurement data only.
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::ultrasonicCmd(uint8_t  cmd, uint8_t  numObjUpdate)
{	
	numObj = numObjUpdate; // number of objects to detect
	uint8_t  bufCmd[4] = {syncByte , 0xFF, numObj, 0xFF}; // prepare bufCmd with 0xFF placeholders
	
	if (comm!=1)
	{
		memset(objTime, 0xFF, 8); // reset and idle-high TCI object buffer
	}
	
	switch (cmd)
	{
		// SINGLE ADDRESS		
		case 0: // Send Preset 1 Burst + Listen command
		{			
			bufCmd[1] = P1BL;
			bufCmd[3] = calcChecksum(P1BL);
			break;
		}
		case 1: // Send Preset 2 Burst + Listen command
		{			
			bufCmd[1] = P2BL;
			bufCmd[3] = calcChecksum(P2BL);
			break;
		}	
		case 2: // Send Preset 1 Listen Only command
		{			
			bufCmd[1] = P1LO;
			bufCmd[3] = calcChecksum(P1LO);
			break;
		}
		case 3: // Send Preset 2 Listen Only command
		{			
			bufCmd[1] = P2LO;
			bufCmd[3] = calcChecksum(P2LO);
			break;
		}	
		
		// BROADCAST
		case 17: // Send Preset 1 Burst + Listen Broadcast command
		{			
			bufCmd[1] = BC_P1BL;
			bufCmd[3] = calcChecksum(BC_P1BL);
			break;
		}
		case 18: // Send Preset 2 Burst + Listen Broadcast command
		{			
			bufCmd[1] = BC_P2BL;
			bufCmd[3] = calcChecksum(BC_P2BL);
			break;
		}	
		case 19: // Send Preset 1 Listen Only Broadcast command
		{			
			bufCmd[1] = BC_P1LO;
			bufCmd[3] = calcChecksum(BC_P1LO);
			break;
		}
		case 20: // Send Preset 2 Listen Only Broadcast command
		{			
			bufCmd[1] = BC_P2LO;
			bufCmd[3] = calcChecksum(BC_P2LO);
			break;
		}		
		
		default: return;	
	}		
	
	if(comm !=1 ) // USART or OWU modes only
	{
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			int serialPort = serialOpen(UART_PORT, 115200); // Use the appropriate serial port and baud rate
			if (serialPort != -1) {
				serialPuts(serialPort, reinterpret_cast<char*>(bufCmd)); // Send data
				serialClose(serialPort); // Close serial port
			} else 
			{
				// Handle error opening serial port
			}
		}
	}
	else if (comm == 1) // TCI mode
	{
		tciCommand(cmd); // send preset 1 or 2 burst-and-listen or listen-only command
		pga460::tciRecord(numObj); // log up to eight TCI object toggles
	}	
	else
	{
		//do nothing
	}
	
	delay(70); // maximum record length is 65ms, so delay with margin
	return;
}

/*------------------------------------------------- pullUltrasonicMeasResult -----
 |  Function pullUltrasonicMeasResult
 |
 |  Purpose:  Read the ultrasonic measurement result data based on the last busrt and/or listen command issued.
 |	Only applicable to UART and OWU modes.
 |
 |  Parameters:
 |		busDemo (IN) -- When true, do not print error message for a failed reading when running bus demo
 |
 |  Returns:  If measurement data successfully read, return true.
 *-------------------------------------------------------------------*/
bool pga460::pullUltrasonicMeasResult(bool busDemo)
{
	if (comm != 1) // USART or OWU mode
	{
		pga460SerialFlush();
		
		memset(ultraMeasResult, 0, sizeof(ultraMeasResult));
		
		if (comm == 2)
		{
		   owuShift = 2; // OWU receive buffer offset to ignore transmitted data
		}
		
		uint8_t  buf5[3] = {syncByte , UMR, calcChecksum(UMR)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
				//for (int i = 1; i < sizeof(buf5); ++i) 
				//{
				//	serialPutchar(serial_fd, buf5[i]);
				//}

			serialPutchar(serial_fd, buf5[0]); // Transmit first byte
            serialPutchar(serial_fd, buf5[1]); // Transmit second byte
            serialPutchar(serial_fd, buf5[2]); // Transmit third byte

			// Serial1.write(buf5, sizeof(buf5)); //serial transmit master data to read ultrasonic measurement results
		}

		
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			starttime = millis();
			while ( (serialDataAvail(serial_fd) < (5 + owuShift)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) )
			{      
				// wait in this loop until we either get +5 uint8_t s of data, or 0.25 seconds have gone by
			}
			
			if (serialDataAvail(serial_fd) < (5 + owuShift))
			{
				if (busDemo == false)
				{
					// the data didn't come in - handle the problem here
					std::cout << "ERROR - Did not receive measurement results!" <<std::endl;
				}
				return false;
			}
			else
			{
				for(int n=0; n<((2+(numObj*4))+owuShift); n++)
				{			
				   ultraMeasResult[n] = serialGetchar(serial_fd);
				   delay(1);		   
				}

				if (comm == 2) // OWU mode only
				{
					//rearrange array for OWU UMR results
					for(int n=0; n<(2+(numObj*4)); n++)
					{
						ultraMeasResult[n+1] = ultraMeasResult[n+owuShift]; //element 0 skipped due to no diagnostic field returned
					}
				}				
			}
		}
	}
	else
	{
		//do nothing
	}
	return true;
}

/*------------------------------------------------- printUltrasonicMeasResult -----
 |  Function printUltrasonicMeasResult
 |
 |  Purpose:  Converts time-of-flight readout to distance in meters. 
 |		Width and amplitude data only available in UART or OWU mode.
 |
 |  Parameters:
 |		umr (IN) -- Ultrasonic measurement result look-up selector:
 |				Distance (m)	Width	Amplitude
 |				--------------------------------
 |			Obj1		0		1		2
 |			Obj2		3		4		5
 |			Obj3		6		7		8
 |			Obj4		9		10		11
 |			Obj5		12		13		14
 |			Obj6		15		16		17
 |			Obj7		18		19		20
 |			Obj8		21		22		23
 |
 |  Returns:  double representation of distance (m), width (us), or amplitude (8-bit)
 *-------------------------------------------------------------------*/
double pga460::printUltrasonicMeasResult(uint8_t  umr)
{
	int speedSound = 343; // speed of sound in air at room temperature
	pga460::printUltrasonicMeasResultExt(umr, speedSound);

	return 0.0; // or return some appropriate value
}
uint8_t  pga460::printUltrasonicMeasResultRaw(uint8_t  umr)
{
	return ultraMeasResult[umr];
}
double pga460::printUltrasonicMeasResultExt(uint8_t  umr, int speedSound)
{
	double objReturn = 0;
	double digitalDelay = 0; // TODO: compensates the burst time calculated as number_of_pulses/frequency.
	uint16_t objDist = 0;
	uint16_t objWidth = 0;
	uint16_t objAmp = 0;
	
	switch (umr)
	{
		case 0: //Obj1 Distance (m)
		{
			objDist = (ultraMeasResult[1]<<8) + ultraMeasResult[2];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 1: //Obj1 Width (us)
		{
			objWidth = ultraMeasResult[3];
			objReturn= objWidth * 16;
			break;
		}
		case 2: //Obj1 Peak Amplitude
		{
			objAmp = ultraMeasResult[4];
			objReturn= objAmp;
			break;
		}
		
		case 3: //Obj2 Distance (m)
		{
			objDist = (ultraMeasResult[5]<<8) + ultraMeasResult[6];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 4: //Obj2 Width (us)
		{
			objWidth = ultraMeasResult[7];
			objReturn= objWidth * 16;
			break;
		}
		case 5: //Obj2 Peak Amplitude
		{
			objAmp = ultraMeasResult[8];
			objReturn= objAmp;
			break;
		}
		
		case 6: //Obj3 Distance (m)
		{
			objDist = (ultraMeasResult[9]<<8) + ultraMeasResult[10];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 7: //Obj3 Width (us)
		{
			objWidth = ultraMeasResult[11];
			objReturn= objWidth * 16;
			break;
		}
		case 8: //Obj3 Peak Amplitude
		{
			objAmp = ultraMeasResult[12];
			objReturn= objAmp;
			break;
		}
		case 9: //Obj4 Distance (m)
		{
			objDist = (ultraMeasResult[13]<<8) + ultraMeasResult[14];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 10: //Obj4 Width (us)
		{
			objWidth = ultraMeasResult[15];
			objReturn= objWidth * 16;
			break;
		}
		case 11: //Obj4 Peak Amplitude
		{
			objAmp = ultraMeasResult[16];
			objReturn= objAmp;
			break;
		}
		case 12: //Obj5 Distance (m)
		{
			objDist = (ultraMeasResult[17]<<8) + ultraMeasResult[18];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 13: //Obj5 Width (us)
		{
			objWidth = ultraMeasResult[19];
			objReturn= objWidth * 16;
			break;
		}
		case 14: //Obj5 Peak Amplitude
		{
			objAmp = ultraMeasResult[20];
			objReturn= objAmp;
			break;
		}
		case 15: //Obj6 Distance (m)
		{
			objDist = (ultraMeasResult[21]<<8) + ultraMeasResult[22];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 16: //Obj6 Width (us)
		{
			objWidth = ultraMeasResult[23];
			objReturn= objWidth * 16;
			break;
		}
		case 17: //Obj6 Peak Amplitude
		{
			objAmp = ultraMeasResult[24];
			objReturn= objAmp;
			break;
		}
		case 18: //Obj7 Distance (m)
		{
			objDist = (ultraMeasResult[25]<<8) + ultraMeasResult[26];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 19: //Obj7 Width (us)
		{
			objWidth = ultraMeasResult[27];
			objReturn= objWidth * 16;
			break;
		}
		case 20: //Obj7 Peak Amplitude
		{
			objAmp = ultraMeasResult[28];
			objReturn= objAmp;
			break;
		}
		case 21: //Obj8 Distance (m)
		{
			objDist = (ultraMeasResult[29]<<8) + ultraMeasResult[30];
			objReturn = (objDist/2*0.000001*speedSound) - digitalDelay;
			break;
		}
		case 22: //Obj8 Width (us)
		{
			objWidth = ultraMeasResult[31];
			objReturn= objWidth * 16;
			break;
		}
		case 23: //Obj8 Peak Amplitude
		{
			objAmp = ultraMeasResult[32];
			objReturn= objAmp;
			break;
		}		
		default: std::cout << "ERROR - Invalid object result!" << std::endl; break;
	}	
	return objReturn;
}

/*------------------------------------------------- runEchoDataDump -----
 |  Function runEchoDataDump
 |
 |  Purpose:  Runs a preset 1 or 2 burst and or listen command to capture 128 uint8_t s of echo data dump.
 |		Toggle echo data dump enable bit to enable/disable echo data dump mode.
 |
 |  Parameters:
 |		preset (IN) -- determines which preset command is run:
 |			• 0 = Preset 1 Burst + Listen command
 |			• 1 = Preset 2 Burst + Listen command
 |			• 2 = Preset 1 Listen Only command
 |			• 3 = Preset 2 Listen Only command
 |			• 17 = Preset 1 Burst + Listen broadcast command
 |			• 18 = Preset 2 Burst + Listen broadcast command
 |			• 19 = Preset 1 Listen Only broadcast command
 |			• 20 = Preset 2 Listen Only broadcast command
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::runEchoDataDump(uint8_t  preset)
{
	if (comm != 1) // USART or OWU mode
	{	
		// enable Echo Data Dump bit
		regAddr = 0x40;
		regData = 0x80;
		

		uint8_t  writeType = SRW; // default to single address register write (cmd10)
		if(preset>16) // update to broadcast register write if broadcast TOF preset command given
		{
			writeType = BC_RW; // cmd22
		}
		
		uint8_t  buf10[5] = {syncByte , writeType, regAddr, regData, calcChecksum(writeType)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			pga460SerialFlush();
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
            for (int i = 1; i < sizeof(buf10); ++i) {
                serialPutchar(serial_fd, buf10[i]);
            }
		}

		delay(10);
		
		
		// run preset 1 or 2 burst and or listen command
		pga460::ultrasonicCmd(preset, 1);	

		// disbale Echo Data Dump bit
		regData = 0x00;
		buf10[3] = regData;
		buf10[4] = calcChecksum(writeType);
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
            for (int i = 1; i < sizeof(buf10); ++i) {
                serialPutchar(serial_fd, buf10[i]);
            }
		}		
	}
	else if (comm == 1) // TCI mode
	{
		EE_CNTRL = 0x80; 		// enable echo data dump 
		tciIndexRW(11, true); 	// write to index 11
		
		delay(10);
		tciCommand(preset); 	// run burst+listen command
		delay(100);				// delay for maximum record time length with margin
		
		EE_CNTRL = 0x00; 		// disable echo data dump
		tciIndexRW(11, true); 	// write to index 11		
		delay(10);			
	}
	else
	{
		//do nothing
	}
	return;
}

/*------------------------------------------------- pullEchoDataDump -----
 |  Function pullEchoDataDump
 |
 |  Purpose:  Read out 128 uint8_t s of echo data dump (EDD) from latest burst and or listen command. 
 |		For UART and OWU, readout individual echo data dump register values, instead in bulk.
 |		For TCI, perform index 12 read of all echo data dump values in bulk.
 |
 |  Parameters:
 |		element (IN) -- element from the 128 uint8_t  EDD memory
 |
 |  Returns:  uint8_t  representation of EDD element value
 *-------------------------------------------------------------------*/
uint8_t  pga460::pullEchoDataDump(uint8_t  element)
{
	
	
	if (comm != 1 && comm != 3) // UART or OWU mode
	{
		if (element == 0)
		{
			uint8_t  temp = 0;
			pga460SerialFlush();
			
			if (comm == 2)
			{
				owuShift = 2; // OWU receive buffer offset to ignore transmitted data
			}
			else
			{
				owuShift = 0;
			}	
			
			regAddr = 0x80; // start of EDD memory
			uint8_t  buf9[4] = {syncByte , SRR, regAddr, calcChecksum(SRR)}; 
			serialPutchar(serial_fd, syncByte);
            for (int i = 1; i < sizeof(buf9); ++i) {
                serialPutchar(serial_fd, buf9[i]);
            }

			for(int m=0; m<129; m++) // loop readout by iterating through EDD address range
			{
			   buf9[2] = regAddr;
			   buf9[3] = calcChecksum(SRR);
			   serialPutchar(serial_fd, syncByte);
                for (int i = 1; i < sizeof(buf9); ++i) {
                    serialPutchar(serial_fd, buf9[i]);
                }

			   delay(30);	 
			   
			   for(int n=0; n<(128+owuShift); n++)
			   {
				   if(n==(1 + owuShift))
				   {
						echoDataDump[m] = serialGetchar(serial_fd);						
				   }
				   else
				   {
					   temp = serialGetchar(serial_fd);
				   }
			   }
			   regAddr++;
			}			
		}
		return echoDataDump[element];	
	}
	else if (comm == 1) // TCI
	{
		if (element == 0)
		{
			tciIndexRW(12, false); //only run when first calling this function to read out the entire EDD to the receive buffer
			delay (500); // wait until EDD read out is completed with margin
		}		
		delay(10);
		return bufRecv[element];
	}
	else
	{
		//do nothing
	}
	return 0xFF;
}

/*------------------------------------------------- pullEchoDataDumpBulk -----
 |  Function pullEchoDataDumpBulk
 |
 |  Purpose:  Bulk read out 128 uint8_t s of echo data dump (EDD) from latest burst and or listen command. 
 |		For UART and OWU, readout bulk echo data dump register values.
 |		For TCI, perform index 12 read of all echo data dump values in bulk.
 |
 |  Parameters:
 |		none
 |
 |  Returns:  comma delimited string of all EDD values
 *-------------------------------------------------------------------*/
std::string pga460::pullEchoDataDumpBulk()
{
	std::string bulkString = "";
	if (comm != 1 && comm != 3) // UART or OWU mode
	{
		uint8_t  temp = 0;
		pga460SerialFlush();
		
		if (comm == 2)
		{
			owuShift = 2; // OWU receive buffer offset to ignore transmitted data
		}
		else
		{
			owuShift = 0;
		}	
		
		uint8_t  buf7[3] = {syncByte , TEDD, calcChecksum(TEDD)}; 
		serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
        for (int i = 1; i < sizeof(buf7); ++i) {
            serialPutchar(serial_fd, buf7[i]);
        }
		uint8_t eddBulk[130];
		std::cout << "TEST1" << std::endl;
        for (int i = 0; i < 130; ++i) {
            eddBulk[i] = serialGetchar(serial_fd);
			std::cout << i << std::endl;
        }

		
		if(1)//(eddBulk[0] != 0) // if diagnostic field is non-zero
		{						
			for(int n=1+owuShift; n<(129+owuShift); n++)
			{	
				std::cout << n << std::endl;		
			   bulkString = bulkString + "," + std::to_string(eddBulk[n]);
	   
			}
		}
		else
		{
			// the data didn't come in - handle the problem here
			std::cout << "ERROR - Did not receive echo data dump! " << std::endl;	
			for(int n=1+owuShift; n<(129+owuShift); n++)
			{			
			   //bulkString = bulkString + "," + eddBulk[n];	
			   bulkString = bulkString + "," + std::to_string(eddBulk[n]);	   
			}		
		}
		return bulkString;	
	}
	else if (comm == 1) // TCI
	{		
		tciIndexRW(12, false); //only run when first calling this function to read out the entire EDD to the receive buffer
		delay (500); // wait until EDD read out is completed with margin
		delay(10);
		for(int n=1; n<129; n++)
		{			
		   //bulkString = bulkString + "," + bufRecv[n];
		   bulkString = bulkString + "," + std::to_string(bufRecv[n]);
		   
		}
		return bulkString;	
	}
	else
	{
		//do nothing
	}
	return "ERROR - pullEchoDataDumpBulk";
}

/*------------------------------------------------- runDiagnostics -----
 |  Function runDiagnostics
 |
 |  Purpose:  Runs a burst+listen command to capture frequency, decay, and voltage diagnostic.
 |		Runs a listen-only command to capture noise level.
 |		Captures die temperature of PGA460 device.
 |		Converts raw diagnostics to comprehensive units
 |
 |  Parameters:
 |		run (IN) -- issue a preset 1 burst-and-listen command
 |		diag (IN) -- diagnostic value to return:
 |			• 0 = frequency diagnostic (kHz)
 |			• 1 = decay period diagnostic (us)
 |			• 2 = die temperature (degC)
 |			• 3 = noise level (8bit)
 |
 |  Returns:  double representation of last captured diagnostic
 *-------------------------------------------------------------------*/
double pga460::runDiagnostics(uint8_t  run, uint8_t  diag)
{
	double diagReturn = 0;
	pga460SerialFlush();
	int elementOffset = 0; //Only non-zero for OWU mode.
	int owuShiftSysDiag = 0; // Only non-zero for OWU mode.
	
	if (comm != 1) // USART and OWU
	{
		if (comm == 2)
		{
			owuShift = 2; // OWU receive buffer offset to ignore transmitted data
			owuShiftSysDiag = 1;
		}
			
		if (run == 1) // issue  P1 burst+listen, and run system diagnostics command to get latest results
		{
			// run burst+listen command at least once for proper diagnostic analysis
			pga460::ultrasonicCmd(0, 1);	// always run preset 1 (short distance) burst+listen for 1 object for system diagnostic
						
			delay(100); // record time length maximum of 65ms, so add margin
			pga460SerialFlush();
			
			uint8_t  buf8[3] = {syncByte , SD, calcChecksum(SD)};
			
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
				for (int i = 1; i < sizeof(buf8); ++i) 
				{
					serialPutchar(serial_fd, buf8[i]);
				}

				starttime = millis();
				while ((serialDataAvail(serial_fd) < (4 + owuShift - owuShiftSysDiag)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT))
				{
					// wait in this loop until we either get +4 bytes of data or 0.25 seconds have gone by
				}

				if (serialDataAvail(serial_fd) < (4 + owuShift - owuShiftSysDiag))
				{
					// the data didn't come in - handle the problem here
					fprintf(stderr, "ERROR - Did not receive system diagnostics!\n");
				}
				else
				{
					for (int n = 0; n < (4 + owuShift - owuShiftSysDiag); n++)
					{
						diagMeasResult[n] = serialGetchar(serial_fd);
						std::cout << "Testtt: "<< std::endl; std::cout << buf8[3] << std::endl;
					}
				}

			}
		}
		
		if (diag == 2) //run temperature measurement
		{
			tempOrNoise = 0; // temp meas
			uint8_t  buf4[4] = {syncByte , TNLM, tempOrNoise, calcChecksum(TNLM)}; 
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
				for (int i = 1; i < sizeof(buf4); ++i) 
				{
					serialPutchar(serial_fd, buf4[i]);
				}
				delay(10);
				pga460SerialFlush();
				delay(10);
			}
			
			uint8_t  buf6[3] = {syncByte , TNLR, calcChecksum(TNLR)};
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
            for (int i = 1; i < sizeof(buf6); ++i) {
                serialPutchar(serial_fd, buf6[i]);
            }
			
			delay(100);		
		}
			
		if (diag == 3) // run noise level meas
		{
			tempOrNoise = 1; // noise meas
			uint8_t  buf4[4] = {syncByte , TNLM, tempOrNoise, calcChecksum(TNLM)};
			
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
				for (int i = 1; i < sizeof(buf4); ++i) {
					serialPutchar(serial_fd, buf4[i]);
				}
			}			
			delay(10);
			pga460SerialFlush();
			delay(10);
			
			uint8_t  buf6[3] = {syncByte , TNLR, calcChecksum(TNLR)}; //serial transmit master data to read temperature and noise results
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
				for (int i = 1; i < sizeof(buf6); ++i) {
					serialPutchar(serial_fd, buf6[i]);
				}
			}
			
			delay(100);
		}
			
		if (comm == 0 || comm == 2) // UART or OWU mode
		{	
			if (diag == 2 || diag == 3) // pull temp and noise level results
			{
				starttime = millis();
				while ( (serialDataAvail(serial_fd)<(4+owuShift-owuShiftSysDiag)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) )
				{      
					// wait in this loop until we either get +4 uint8_t s of data or 0.25 seconds have gone by
				}
				
				if(serialDataAvail(serial_fd) < (4+owuShift-owuShiftSysDiag))
				{
					// the data didn't come in - handle the problem here
					std::cout << "ERROR - Did not receive temp/noise!" << std::endl;
				}
				else
				{
					for(int n=0; n<(4+owuShift-owuShiftSysDiag); n++)
					{
					   tempNoiseMeasResult[n] = serialGetchar(serial_fd); //TODO: Serial1.read();
					}
							
				}
			}
			elementOffset = owuShift-owuShiftSysDiag; // OWU only
		}
			
	}
	else if (comm == 1) //TCI
	{
		if (run == true)
		{	
			delay(10);	
			tciCommand(6); // run noise level measurement command			
			delay(15);
			tciCommand(1); //run preset 2 burst+listen command
			delay(100);	// maximum record length is 65ms, so wait with margin


			tciIndexRW(1, false); //read index1	
			delay(10);

			for(int n=1; n<4; n++)
			{
			   diagMeasResult[n] = bufRecv[n-1];
			}			
			tempNoiseMeasResult[2] = diagMeasResult[3]; //clone temperature result to element 2			
			
			delay(10);
			tciCommand(5); // run temperature measurement command
			delay(10);
			
			tciIndexRW(0,false); //read index0
			delay(10);			
			
			tempNoiseMeasResult[1] = bufRecv[0]; //store temp readout to element 1
		}
		elementOffset = 0; // no offset required fot TCI
	}
	else
	{
		//do nothing
	}
	
	delay(100);
		
	switch (diag)
	{
		case 0: // convert to transducer frequency in kHz
			{
				diagReturn = (1 / (diagMeasResult[1+elementOffset] * 0.0000005)) / 1000;
			}
			break;
		case 1: // convert to decay period time in us
			{
				diagReturn = diagMeasResult[2+elementOffset] * 16;
			}
			break;
		case 2: //convert to temperature in degC
			{
				diagReturn = (tempNoiseMeasResult[1+elementOffset] - 64) / 1.5;
			}
			break;
		case 3: //noise floor level
			{
				diagReturn = tempNoiseMeasResult[2+elementOffset];
			}
			break;
		default: break;
	}
	
	return diagReturn;
}

/*------------------------------------------------- burnEEPROM -----
 |  Function burnEEPROM
 |
 |  Purpose:  Burns the EEPROM to preserve the working/shadow register values to EEPROM after power
 |		cycling the PGA460 device. Returns EE_PGRM_OK bit to determine if EEPROM burn was successful.
 |
 |  Parameters:
 |		none
 |
 |  Returns:  bool representation of EEPROM program success
 *-------------------------------------------------------------------*/
bool pga460::burnEEPROM()
{
	uint8_t  burnStat = 0;
	uint8_t  temp = 0;
	bool burnSuccess = false;

	if (comm != 1 || comm != 3)
	{	
			
		// Write "0xD" to EE_UNLCK to unlock EEPROM, and '0' to EEPRGM bit at EE_CNTRL register
		regAddr = 0x40; //EE_CNTRL
		regData = 0x68;
		uint8_t  buf10[5] = {syncByte , SRW, regAddr, regData, calcChecksum(SRW)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
            for (int i = 1; i < sizeof(buf10); ++i) {
                serialPutchar(serial_fd, buf10[i]);
            }
		}		
		delay(1);
		
		// Write "0xD" to EE_UNLCK to unlock EEPROM, and '1' to EEPRGM bit at EE_CNTRL register
		regAddr = 0x40; //EE_CNTRL
		regData = 0x69;
		buf10[2] = regAddr;
		buf10[3] = regData;
		buf10[4] = calcChecksum(SRW);
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
            for (int i = 1; i < sizeof(buf10); ++i) {
                serialPutchar(serial_fd, buf10[i]);
            }
		}
		delay(1000);
		
		
		// Read back EEPROM program status
		if (comm == 2)
		{
			owuShift = 1; // OWU receive buffer offset to ignore transmitted data
		}	
		pga460SerialFlush();
		regAddr = 0x40; //EE_CNTRL
		uint8_t  buf9[4] = {syncByte , SRR, regAddr, calcChecksum(SRR)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
            for (int i = 1; i < sizeof(buf9); ++i) {
                serialPutchar(serial_fd, buf9[i]);
            }
		}
		delay(10);
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			int temp = 0;
			for (int n = 0; n < 3; n++)
			{
				if (n == 1 - owuShift)
				{
					burnStat = serialGetchar(serial_fd); // store EE_CNTRL data
				}
				else
				{
					temp = serialGetchar(serial_fd);
				}
			}
		}
	}
	else if (comm == 1) // TCI mode
	{
		EE_CNTRL = 0x68;
		tciIndexRW(11, true); 	// write to index 11 to EE_UNLCK to unlock EEPROM, and '0' to EEPRGM bit at EE_CNTRL register
		delay(1); 				// immediately send the same UART or TCI command with the EEPRGM bit set to '1'.
		EE_CNTRL = 0x69;
		tciIndexRW(11, true); 	// write to index 11 to EE_UNLCK to unlock EEPROM, and '1' to EEPRGM bit at EE_CNTRL register
		delay(1000);
		tciIndexRW(11, false);	// read back index 11 to review EE_PGRM_OK bit	
		burnStat = bufRecv[0];		
	}
	else
	{
		//do nothing
	}
	
	if((burnStat & 0x04) == 0x04){burnSuccess = true;} // check if EE_PGRM_OK bit is '1'
	
	return burnSuccess;
}

/*------------------------------------------------- broadcast -----
 |  Function broadcast
 |
 |  Purpose:  Send a broadcast command to bulk write the user EEPROM, TVG, and/or Threshold values for all devices, regardless of UART_ADDR.
 |		Placehold for user EEPROM broadcast available. Note, all devices will update to the same UART_ADDR in user EEPROM broadcast command.
 |		This function is not applicable to TCI mode.
 |
 |  Parameters:
 |		eeBulk (IN) -- if true, broadcast user EEPROM
 |		tvgBulk (IN) -- if true, broadcast TVG
 |		thrBulk (IN) -- if true, broadcast Threshold
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::broadcast(bool eeBulk, bool tvgBulk, bool thrBulk)
{
	// TVG broadcast command:
	if (tvgBulk == true)
	{
		uint8_t  buf24[10] = {syncByte , BC_TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(BC_TVGBW)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); 
            for (int i = 1; i < sizeof(buf24); ++i) 
			{
                serialPutchar(serial_fd, buf24[i]);
            }
		}		
		delay(10);
	}
	
	// Threshold broadcast command:
	if (thrBulk == true)
	{
		uint8_t  buf25[35] = {syncByte , BC_THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
		  P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
		  P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6, 
		  P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
		  calcChecksum(BC_THRBW)};

		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); 
            for (int i = 1; i < sizeof(buf25); ++i) 
			{
                serialPutchar(serial_fd, buf25[i]);
            }
		}	
		delay(10);		
	}
	
	// User EEPROM broadcast command (placeholder):
	if (eeBulk == true)
	{
		uint8_t  buf23[46] = {syncByte , BC_EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
			USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, 
			USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
			TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
			PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
			DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(BC_EEBW)};
		
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); 
            for (int i = 1; i < sizeof(buf23); ++i) 
			{
                serialPutchar(serial_fd, buf23[i]);
            }
		}	
		delay(50);
	}
	
	return;
}


/*------------------------------------------------- calcChecksum -----
 |  Function calcChecksum
 |
 |  Purpose:  Calculates the UART checksum value based on the selected command and the user EERPOM values associated with the command
 |		This function is not applicable to TCI mode. 
 |
 |  Parameters:
 |		cmd (IN) -- the UART command for which the checksum should be calculated for
 |
 |  Returns: uint8_t  representation of calculated checksum value
 *-------------------------------------------------------------------*/
uint8_t  pga460::calcChecksum(uint8_t  cmd)
{
	int checksumLoops = 0;
	
	cmd = cmd & 0x001F; // zero-mask command address of cmd to select correct switch-case statement
	
	switch(cmd)
	{
		case 0 : //P1BL
		case 1 : //P2BL
		case 2 : //P1LO
		case 3 : //P2LO
		case 17 : //BC_P1BL
		case 18 : //BC_P2BL
		case 19 : //BC_P1LO
		case 20 : //BC_P2LO
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = numObj;
			checksumLoops = 2;
		break;
		case 4 : //TNLM
		case 21 : //TNLM
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = tempOrNoise;
			checksumLoops = 2;
		break;
		case 5 : //UMR
		case 6 : //TNLR
		case 7 : //TEDD
		case 8 : //SD
		case 11 : //EEBR
		case 13 : //TVGBR
		case 15 : //THRBR
			ChecksumInput[0] = cmd;
			checksumLoops = 1;
		break;
		case 9 : //RR
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = regAddr;
			checksumLoops = 2;
		break;
		case 10 : //RW
		case 22 : //BC_RW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = regAddr;
			ChecksumInput[2] = regData;
			checksumLoops = 3;
		break;
		case 14 : //TVGBW
		case 24 : //BC_TVGBW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = TVGAIN0;
			ChecksumInput[2] = TVGAIN1;
			ChecksumInput[3] = TVGAIN2;
			ChecksumInput[4] = TVGAIN3;
			ChecksumInput[5] = TVGAIN4;
			ChecksumInput[6] = TVGAIN5;
			ChecksumInput[7] = TVGAIN6;
			checksumLoops = 8;
		break;
		case 16 : //THRBW
		case 25 : //BC_THRBW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = P1_THR_0;
			ChecksumInput[2] = P1_THR_1;
			ChecksumInput[3] = P1_THR_2;
			ChecksumInput[4] = P1_THR_3;
			ChecksumInput[5] = P1_THR_4;
			ChecksumInput[6] = P1_THR_5;
			ChecksumInput[7] = P1_THR_6;
			ChecksumInput[8] = P1_THR_7;
			ChecksumInput[9] = P1_THR_8;
			ChecksumInput[10] = P1_THR_9;
			ChecksumInput[11] = P1_THR_10;
			ChecksumInput[12] = P1_THR_11;
			ChecksumInput[13] = P1_THR_12;
			ChecksumInput[14] = P1_THR_13;
			ChecksumInput[15] = P1_THR_14;
			ChecksumInput[16] = P1_THR_15;
			ChecksumInput[17] = P2_THR_0;
			ChecksumInput[18] = P2_THR_1;
			ChecksumInput[19] = P2_THR_2;
			ChecksumInput[20] = P2_THR_3;
			ChecksumInput[21] = P2_THR_4;
			ChecksumInput[22] = P2_THR_5;
			ChecksumInput[23] = P2_THR_6;
			ChecksumInput[24] = P2_THR_7;
			ChecksumInput[25] = P2_THR_8;
			ChecksumInput[26] = P2_THR_9;
			ChecksumInput[27] = P2_THR_10;
			ChecksumInput[28] = P2_THR_11;
			ChecksumInput[29] = P2_THR_12;
			ChecksumInput[30] = P2_THR_13;
			ChecksumInput[31] = P2_THR_14;
			ChecksumInput[32] = P2_THR_15;
			checksumLoops = 33;
		break;
		case 12 : //EEBW
		case 23 : //BC_EEBW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = USER_DATA1;
			ChecksumInput[2] = USER_DATA2;
			ChecksumInput[3] = USER_DATA3;
			ChecksumInput[4] = USER_DATA4;
			ChecksumInput[5] = USER_DATA5;
			ChecksumInput[6] = USER_DATA6;
			ChecksumInput[7] = USER_DATA7;
			ChecksumInput[8] = USER_DATA8;
			ChecksumInput[9] = USER_DATA9;
			ChecksumInput[10] = USER_DATA10;
			ChecksumInput[11] = USER_DATA11;
			ChecksumInput[12] = USER_DATA12;
			ChecksumInput[13] = USER_DATA13;
			ChecksumInput[14] = USER_DATA14;
			ChecksumInput[15] = USER_DATA15;
			ChecksumInput[16] = USER_DATA16;
			ChecksumInput[17] = USER_DATA17;
			ChecksumInput[18] = USER_DATA18;
			ChecksumInput[19] = USER_DATA19;
			ChecksumInput[20] = USER_DATA20;
			ChecksumInput[21] = TVGAIN0;
			ChecksumInput[22] = TVGAIN1;
			ChecksumInput[23] = TVGAIN2;
			ChecksumInput[24] = TVGAIN3;
			ChecksumInput[25] = TVGAIN4;
			ChecksumInput[26] = TVGAIN5;
			ChecksumInput[27] = TVGAIN6;
			ChecksumInput[28] = INIT_GAIN;
			ChecksumInput[29] = FREQUENCY;
			ChecksumInput[30] = DEADTIME;
			ChecksumInput[31] = PULSE_P1;
			ChecksumInput[32] = PULSE_P2;
			ChecksumInput[33] = CURR_LIM_P1;
			ChecksumInput[34] = CURR_LIM_P2;
			ChecksumInput[35] = REC_LENGTH;
			ChecksumInput[36] = FREQ_DIAG;
			ChecksumInput[37] = SAT_FDIAG_TH;
			ChecksumInput[38] = FVOLT_DEC;
			ChecksumInput[39] = DECPL_TEMP;
			ChecksumInput[40] = DSP_SCALE;
			ChecksumInput[41] = TEMP_TRIM;
			ChecksumInput[42] = P1_GAIN_CTRL;
			ChecksumInput[43] = P2_GAIN_CTRL;
			checksumLoops = 44;
		break;
		default: break;
	}

	if (ChecksumInput[0]<17) //only re-append command address for non-broadcast commands.
	{
		ChecksumInput[0] = ChecksumInput[0] + (uartAddr << 5);
	}
	
	uint16_t carry = 0;

	for (int i = 0; i < checksumLoops; i++)
	{
		if ((ChecksumInput[i] + carry) < carry)
		{
			carry = carry + ChecksumInput[i] + 1;
		}
		else
		{
			carry = carry + ChecksumInput[i];
		}

		if (carry > 0xFF)
		{
		  carry = carry - 255;
		}
	}
	
	carry = (~carry & 0x00FF);
	return carry;
}

/*------------------------------------------------- tciIndexRW -----
 |  Function tciIndexRW
 |
 |  Purpose:  Read or write the TCI index command.
 |		TODO: Enable all commands to be written. Update user EEPROM variables based on index read.
 |
 |  Parameters:
 |		index (IN) -- TCI index (0-15) to read or write.
 |		wTrue (IN) -- when true, issue a TCI write command. When false, issue a TCI read command.
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::tciIndexRW(uint8_t  index, bool wTrue)
{
	#ifdef EnTCI
	int dataLength = 0;		// number of bits per TCI index
	std::string zeroString = "";	// string of zeros to append to the end of the binary string for the checksum calculation
	std::string dataString = "";	// entire index data string with appended zeros for checksum calculation
	uint8_t  dataLoops = 0;		// based on the number elements to be passed into the checksum calaculation after appending zeros
	uint8_t  bufTCI[46];		// transmit TCI buffer for all index commands
	uint8_t  data = 0xFF;		// idle-high data transmit data
	uint8_t  zeroPadding = 0;	// uint8_t -number of zeros to append to the end of the binary string for the checksum calculation 
	uint8_t  bitIgnore = 0;		// number of bits to ignore at the end of the concatenated bufTCI index string
	
	if (wTrue == true) // TCI write command
	{
		bufTCI[0] = 0x1F & (0x10 + index); // set first uint8_t  with write bit and index
		switch(index)
		{
			case 0: dataLength = 8; break; //read only
			case 1: dataLength = 24; break; //read only
			case 2: zeroPadding = 3; dataLength = 8; zeroString = "000";  bitIgnore = 0; 
				bufTCI[1] = FREQUENCY;
				break;
			case 3: zeroPadding = 1; dataLength = 18; zeroString = "0"; bitIgnore = 0; 
				//TODO
				break;
			case 4: zeroPadding = 3; dataLength = 8; zeroString = "000"; bitIgnore = 0;
				//TODO
				break;
			case 5: zeroPadding = 3; dataLength = 124; zeroString = "000"; bitIgnore = 4; 
					bufTCI[1] = P1_THR_0;
					bufTCI[2] = P1_THR_1;
					bufTCI[3] = P1_THR_2;
					bufTCI[4] = P1_THR_3;
					bufTCI[5] = P1_THR_4;
					bufTCI[6] = P1_THR_5;
					bufTCI[7] = P1_THR_6;
					bufTCI[8] = P1_THR_7;
					bufTCI[9] = P1_THR_8;
					bufTCI[10] = P1_THR_9;
					bufTCI[11] = P1_THR_10;
					bufTCI[12] = P1_THR_11;
					bufTCI[13] = P1_THR_12;
					bufTCI[14] = P1_THR_13;
					bufTCI[15] = P1_THR_14;
					bufTCI[16] = (P1_THR_15 & 0x0F) << 4; //TH_P1_OFF only
				break;
			case 6: zeroPadding = 3; dataLength = 124; zeroString = "000"; bitIgnore = 4;
					bufTCI[1] = P2_THR_0;
					bufTCI[2] = P2_THR_1;
					bufTCI[3] = P2_THR_2;
					bufTCI[4] = P2_THR_3;
					bufTCI[5] = P2_THR_4;
					bufTCI[6] = P2_THR_5;
					bufTCI[7] = P2_THR_6;
					bufTCI[8] = P2_THR_7;
					bufTCI[9] = P2_THR_8;
					bufTCI[10] = P2_THR_9;
					bufTCI[11] = P2_THR_10;
					bufTCI[12] = P2_THR_11;
					bufTCI[13] = P2_THR_12;
					bufTCI[14] = P2_THR_13;
					bufTCI[15] = P2_THR_14;
					bufTCI[16] = (P2_THR_15 & 0x0F) << 4; //TH_P2_OFF only
				break;
			case 7: zeroPadding = 1; dataLength = 42; zeroString = "0"; bitIgnore = 0; 
				//TODO
				break;
			case 8: zeroPadding = 3; dataLength = 56; zeroString = "000"; bitIgnore = 0; 
					bufTCI[1] = TVGAIN0;
					bufTCI[2] = TVGAIN1;
					bufTCI[3] = TVGAIN2;
					bufTCI[4] = TVGAIN3;
					bufTCI[5] = TVGAIN4;
					bufTCI[6] = TVGAIN5;
					bufTCI[7] = TVGAIN6;
				break;
			case 9: zeroPadding = 3; dataLength = 160; zeroString = "000"; bitIgnore = 0; 
				//TODO
				break;
			case 10: zeroPadding = 5; dataLength = 46; zeroString = "00000"; bitIgnore = 0; 
				//TODO
				break;
			case 11: zeroPadding = 3; dataLength = 8; zeroString = "000"; bitIgnore = 0; 
				bufTCI[1] = EE_CNTRL;
				break;
			case 12: dataLength = 1024; break; //read only
			case 13: zeroPadding = 3; zeroString = "000";  dataLength = 352; bitIgnore = 0;			
					bufTCI[1] = USER_DATA1;
					bufTCI[2] = USER_DATA2;
					bufTCI[3] = USER_DATA3;
					bufTCI[4] = USER_DATA4;
					bufTCI[5] = USER_DATA5;
					bufTCI[6] = USER_DATA6;
					bufTCI[7] = USER_DATA7;
					bufTCI[8] = USER_DATA8;
					bufTCI[9] = USER_DATA9;
					bufTCI[10] = USER_DATA10;
					bufTCI[11] = USER_DATA11;
					bufTCI[12] = USER_DATA12;
					bufTCI[13] = USER_DATA13;
					bufTCI[14] = USER_DATA14;
					bufTCI[15] = USER_DATA15;
					bufTCI[16] = USER_DATA16;
					bufTCI[17] = USER_DATA17;
					bufTCI[18] = USER_DATA18;
					bufTCI[19] = USER_DATA19;
					bufTCI[20] = USER_DATA20;
					bufTCI[21] = TVGAIN0;
					bufTCI[22] = TVGAIN1;
					bufTCI[23] = TVGAIN2;
					bufTCI[24] = TVGAIN3;
					bufTCI[25] = TVGAIN4;
					bufTCI[26] = TVGAIN5;
					bufTCI[27] = TVGAIN6;
					bufTCI[28] = INIT_GAIN;
					bufTCI[29] = FREQUENCY;
					bufTCI[30] = DEADTIME;
					bufTCI[31] = PULSE_P1;
					bufTCI[32] = PULSE_P2;
					bufTCI[33] = CURR_LIM_P1;
					bufTCI[34] = CURR_LIM_P2;
					bufTCI[35] = REC_LENGTH;
					bufTCI[36] = FREQ_DIAG;
					bufTCI[37] = SAT_FDIAG_TH;
					bufTCI[38] = FVOLT_DEC;
					bufTCI[39] = DECPL_TEMP;
					bufTCI[40] = DSP_SCALE;
					bufTCI[41] = TEMP_TRIM;
					bufTCI[42] = P1_GAIN_CTRL;
					bufTCI[43] = P2_GAIN_CTRL;
					bufTCI[44] = EE_CRC;
				break;
			case 14: break; //read only (reserved)
			case 15: dataLength = 16; break; //read only						
			default: return;
		}
		
		// calculate checksum	
			// convert uint8_t  to binary string	
				dataLoops = ((dataLength+((zeroPadding+bitIgnore)-3))/8) + 1;		
				
				std::string tempString = "";
				for (int i=0; i<dataLoops; i++)
				{
					//tempString = String((int)bufTCI[i],BIN);//ToDo: Check Implementation
					for (int j = 0; j < dataLoops; j++)
    				{// Convert integer to binary string representation
        				tempString = std::bitset<8>(bufTCI[j]).to_string();
					}
					while (tempString.length() < 8)
					{
						tempString = "0" + tempString;	// add leading zero to get 8 bit BIN representaiton
					}
					//dataString.concat(tempString);
					//dataString += tempString; // Using the + operator
					dataString.append(tempString); // Using the append member function


				}							
				//dataString = dataString.substring(3); // truncate leading zeros
				dataString = dataString.substr(3); // truncate leading zeros
				//dataString.concat(zeroString); // append zero padding to binary string
				dataString.append(zeroString); // append zero padding to binary string
				
			// convert binary string to uint8_t s for checksum calculation
				std::string parsed = "";
				uint8_t  value = 0;
				for(int k=0; k < dataLoops; k++)
				{
					//parsed = dataString.substring(k*8,(k*8)+8);
					parsed = dataString.substr(k*8, 8);

					char s[9];
					//parsed.toCharArray(s,9);
					parsed.copy(s, 9); // Copies the first 9 characters of parsed into s
					s[9] = '\0';       // Null-terminate the string
					for (int i=0; i< strlen(s); i++)  // for every character in the string  strlen(s) returns the length of a char array
					{
					  value *= 2; // double the result so far
					  if (s[i] == '1') value++;  //add 1 if needed
					}
					bufTCI[k] = value;
				}
			
			// generate TCI checksum
			uint16_t carry = 0;	
				for (int i = 0; i < dataLoops; i++)
				{
					if ((bufTCI[i] + carry) < carry)
					{
						carry = carry + bufTCI[i] + 1;
					}
					else
					{
						carry = carry + bufTCI[i];
					}

					if (carry > 0xFF)
					{
					  carry = carry - 255;
					}
				}
				carry = (~carry & 0x00FF);

		// send CFG_TCI low pulse of 1.27ms
		tciCommand(4);
   
		// transmit r/w , index, and data bits. 
			for (int m = 0; m < dataLoops-1; m++)
			{
				data = bufTCI[m];
				pga460::tciByteToggle(data,0); // send bits 7..0			
			} 
			
	   // send last uint8_t  without zero padding
			data = bufTCI[dataLoops-1];
			{
				data = data >> (zeroPadding+bitIgnore);
				pga460::tciByteToggle(data,(zeroPadding+bitIgnore));			
			}		 
			
	   // send checksum
		   data = (uint8_t )carry;
		   pga460::tciByteToggle(data,0);
	}
	
	else // TCI read command
	{
		int recvLength = 0; 	// number of bits to expect for the index to be read
		bool recvState = 0xFF;	// receive state initiated to idle high
		bool lastState = 0xFF;	// last state read initiated to idle high
		int element = 0;		// bufRecv uint8_t  element to save bit capture to
		int bitCount = 0;		// number of bits read to auto increment bufRecv element after 8 hits
		
		switch (index)
		{
			case 0: recvLength = 8; break;
			case 1: recvLength = 24; break;
			case 2: recvLength = 8; break;
			case 3: recvLength = 18; break;
			case 4: recvLength = 8; break;
			case 5: recvLength = 124; break;
			case 6: recvLength = 124; break;
			case 7: recvLength = 42; break;
			case 8: recvLength = 56; break;
			case 9: recvLength = 160; break;
			case 10: recvLength = 46; break;
			case 11: recvLength = 8; break;
			case 12: recvLength = 1024; break;
			case 13: recvLength = 352; break;
			case 14: recvLength = 0; break;
			case 15: recvLength = 16; break;
			default: return;
		}		
				
		memset(bufRecv, 0xFF, sizeof(bufRecv)); // idle-high receive buffer data
		starttime = millis();
		
		// send CFG_TCI low pulse of 1.27ms
		tciCommand(4); 
		data = 0x1F & (0x00 + index);
		pga460::tciByteToggle(data,3);
		delayMicroseconds(100); //TCI deadtime	 
 		
		// capture first response toggle by sampling center of 300us TCI bit indicate 0 or 1
		delayMicroseconds(150);
		lastState=digitalRead(TCI_RX);
		//bitWrite(bufRecv[element], 7-bitCount, digitalRead(TCI_RX)); //ToDO: Check implementation
		if (digitalRead(TCI_RX)) 
		{
    		// Set the bit at position (7 - bitCount) in bufRecv[element] to 1
    		bufRecv[element] |= (1 << (7 - bitCount));
		} 
		else 
		{
    		// Set the bit at position (7 - bitCount) in bufRecv[element] to 0
    		bufRecv[element] &= ~(1 << (7 - bitCount));
		}
 
		bitCount++;
		
		while((millis() - starttime) < 500) // timeout after 0.5 seconds
		{
			recvState = digitalRead(TCI_RX);
			if (((recvState != lastState) && (recvState == 0))) // check for high-to-low toggle
			{
				// sample center of 300us TCI bit indicate 0 or 1
				delayMicroseconds(150);
				//bitWrite(bufRecv[element], 7-bitCount, digitalRead(TCI_RX));	//ToDo: Check
				if (digitalRead(TCI_RX) == HIGH) 
				{
    				bufRecv[element] |= (1 << (7 - bitCount)); // Set the bit to 1
				} else {
    				bufRecv[element] &= ~(1 << (7 - bitCount)); // Set the bit to 0
				}			
				bitCount++;
				if (bitCount == 8)
				{
					bitCount = 0;
					element++;
				}
			}
			lastState = recvState;
			delayMicroseconds(10); // master defined deglitcher timeout
		}
	}
	return;
	#endif
}

/*------------------------------------------------- tciByteToggle -----
 |  Function tciByteToggle
 |
 |  Purpose:  Toggle the TCI_TX pin based on the bit data of the uint8_t  data passed in. 
 |		A bit value of '1' toggles TCI_TX low for 100us, then holds it high for 200us.
 |		A bit value of '0' toggles TCI_TX low for 200us, then holds it high for 100us.
 |
 |  Parameters:
 |		data (IN) -- uint8_t  value to bit parse.
 |		zeroPadding (IN) -- bit toggle based on the number of zeros padded. Zero padding is for checksum calculation only.
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::tciByteToggle(uint8_t  data, uint8_t  zeroPadding)
{
	#ifdef EnTCI
	uint8_t  mask = 0x80;
	int numBits = 8;
	switch (zeroPadding)
	{
		case 0: mask = 0x80; numBits = 8; break;
		case 1: mask = 0x40; numBits = 7; break;
		case 2: mask = 0x20; numBits = 6; break;
		case 3: mask = 0x10; numBits = 5; break;
		case 4: mask = 0x08; numBits = 4; break;
		case 5: mask = 0x04; numBits = 3; break;
		case 6: mask = 0x02; numBits = 2; break;
		case 7: mask = 0x01; numBits = 1; break;
		default: return;
	}
	
	for (int n = 0; n < numBits; n++)
	   {				   
		   // set line low for 100us if bit is 1, low for 200us if bit is 0
		   if (data & mask) // consider leftmost bit (MSB out first)
		   {
			digitalWrite(TCI_TX, LOW);
			delayMicroseconds(100);
			digitalWrite(TCI_TX, HIGH);
			delayMicroseconds(200);
		   }
		   else
		   {
			digitalWrite(TCI_TX, LOW);
			delayMicroseconds(200);
			digitalWrite(TCI_TX, HIGH);
			delayMicroseconds(100);				
		   }							   
		   data <<= 1; // shift uint8_t  left so next bit will be leftmost
	   }	   
	   return;
	   #endif
}

/*------------------------------------------------- tciRecord -----
 |  Function tciRecord
 |
 |  Purpose:  Record TCI_RX toggle burst and/or low activity to time stamp high-to-low transitions representing
		time-of-flight measurements. The time-of-flight is captures in microseconds, and saved to the ultrasonic
		measurement results array to later convert time-of-flight to distance in meters.
 |
 |  Parameters:
 |		data (IN) -- uint8_t  value to bit parse.
 |		numObj (IN) -- number of objects/toggles to monitor the TCI_RX line for (limited to 8 for this library)
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::tciRecord(uint8_t  numObj)
{
	#ifdef EnTCI
	bool recvState = false;
	bool lastState = false;
	tciToggle = micros();
	uint8_t  objCount = 0;
	starttime = millis();
	delayMicroseconds(300); //wait until after STAT bits are toggled by PGA460
	
	while(((millis() - starttime) < 100) && (objCount < numObj)) // timeout after 100ms, or after set number of objects are registered
	{
		recvState = digitalRead(TCI_RX);
		if (((recvState != lastState) && (recvState == 0))) // check for high-to-low toggle of TCI_RX line
		{
			objTime[objCount] = (int)(micros() - tciToggle); // capture time-of-flight
			objCount++;
		}
		lastState = recvState;
		delayMicroseconds(10); // master implemented deglitcher //8cm resolution due to micros timer
	}
	
	if (objCount == (numObj-1)) // if number of objects fills before timer expires
	{
		delay(100-(millis() - starttime)); //wait a total time of 100ms regardless
	}	

	// save each TCI time-of-flight to ultrasonic measurement results array (16 bit parsed into two 8 uint8_t  elements)
	for (int i = 0; i < objCount; i++)
	{	
		ultraMeasResult[(i*4)+1] = (objTime[i] >> 8) & 0x00FF; // MSB
		ultraMeasResult[(i*4)+2] = 0x00FF; //LSB
	}
	#endif
}

/*------------------------------------------------- tciCommand -----
 |  Function tciCommand
 |
 |  Purpose:  Toggle TCI_TX low for micro second duration based on nominal requirement of TCI command.
 |
 |  Parameters:
 |		cmd (IN) -- which TCI command to issue
 |			• 0 = BURST/LISTEN (Preset1)
 |			• 1 = BURST/LISTEN (Preset2)
 |			• 2 = LISTEN only (Preset1)
 |			• 3 = LISTEN only (Preset2)
 |			• 4 = Device configuration
 |			• 5 = Temperature measurement
 |			• 6 = Noise level
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::tciCommand(uint8_t  cmd)
{
	#ifdef EnTCI
	digitalWrite(TCI_TX, LOW);
	
	switch (cmd)
	{	
		case 0: delayMicroseconds(400); break; //send P1BL_TCI low pulse
		case 1: delayMicroseconds(1010); break; //send P2BL_TCI low pulse
		case 2: delayMicroseconds(780); break; //send P1LO_TCI low pulse
		case 3: delayMicroseconds(580); break; //send P2LO_TCI low pulse
		case 4: delayMicroseconds(1270); break; //send CFG_TCI low pulse
		case 5: delayMicroseconds(1550); break; //send TEMP_TCI low pulse
		case 6: delayMicroseconds(2200); break; //send NOISE_TCI low pulse
		default: break;	 
	}

	digitalWrite(TCI_TX, HIGH);
	delayMicroseconds(100); //TCI deadtime
	#endif
}

/*------------------------------------------------- spiTransfer -----
 |  Function spiTransfer
 |
 |  Purpose:  Transfers one uint8_t  over the SPI bus, both sending and receiving. 
 |			Captures MISO data in global uint8_t -array.
 |
 |  Parameters:
 |		mosi (IN) -- MOSI data uint8_t  array to transmit over SPI
 |		size (IN) -- size of MOSI data uint8_t  array
 |
 |  Returns: uint8_t  representation of calculated checksum value
 *-------------------------------------------------------------------*/
// void pga460::spiTransfer(uint8_t * mosi, uint8_t  size )
// {
// 	#ifdef EnSPI	
// 	memset(misoBuf, 0x00, sizeof(misoBuf)); // idle-low receive buffer data
	
// 	for (int i = 0; i<size; i++)
// 	{
// 		digitalWrite(SPI_CS, LOW);
// 		misoBuf[i] = usscSPI.transfer(mosi[i]);
// 		digitalWrite(SPI_CS, HIGH);
// 	}
// 	return;
// 	#endif
// }

/*------------------------------------------------- spiMosiIdle-----
 |  Function spiMosiIdle
 |
 |  Purpose:  Forces MOSI of 0xFF to idle high master output, while
 |			MISO pin returns valid data.
 |
 |  Parameters:
 |		size (IN) -- number of MISO data uint8_t s expected from slave
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
// void pga460::spiMosiIdle(uint8_t  size)
// {		
// 	#ifdef EnSPI	
// 	//memset(misoBuf, 0x00, sizeof(misoBuf)); // idle-low receive buffer data
	
// 	digitalWrite(SPI_CS, LOW);
// 	for (int i = 0; i<size; i++)
// 	{
// 		digitalWrite(SPI_CS, LOW);
// 		misoBuf[i] = usscSPI.transfer(0xFE);
// 		digitalWrite(SPI_CS, HIGH);
// 	}
// 	digitalWrite(SPI_CS, HIGH);
// 	return;
// 	#endif
// }


/*------------------------------------------------- pga460SerialFlush -----
 |  Function pga460SerialFlush
 |
 |  Purpose:  Clears the MSP430's UART receiver buffer
 |
 |  Parameters:
 |		none
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::pga460SerialFlush()
{	
	delay(10);
	serialFlush(serial_fd);
	while((serialDataAvail(serial_fd) > 0))// || (Serial1.read() < 0xFF)) 
	{
		char temp = serialGetchar(serial_fd);
		//Serial1.flush();
	}
	
	//redundant clear
	for (int i = 0; i < 10; i++)
	 {
	   while (serialDataAvail(serial_fd) > 0)
	   {
		 char k = serialGetchar(serial_fd);
		 delay(1);
	   }
	   delay(1);
	 }
	 
	serialFlush(serial_fd);
	return;
}

/*------------------------------------------------- toggleLEDs -----
 |  Function toggleLEDs
 |
 |  Purpose:  Set the BOOSTXL-PGA460 diagnostic LED state to ON or OFF.
 |
 |  Parameters:
 |		ds1State (IN) -- state of BOOSTXL-PGA460 RED LED populated at D9
 |		fdiagState (IN) -- state of BOOSTXL-PGA460 RED LED populated at D8
 |		vdiagate (IN) -- state of BOOSTXL-PGA460 RED LED populated at D7
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::toggleLEDs(bool ds1State, bool fdiagState, bool vdiagState)
{
	digitalWrite(DS1_LED, ds1State); digitalWrite(F_DIAG_LED, fdiagState); digitalWrite(V_DIAG_LED, vdiagState);
	return;
}

/*------------------------------------------------- triangulation -----
 |  Function triangulation
 |
 |  Purpose:  Uses the law of cosines to compute the position of the
 |			targeted object from transceiver S1.
 |
 |  Parameters:
 |		distanceA (IN) -- distance (m) from sensor module 1 (S1) to the targeted object based on UMR result
 |		distanceB (IN) -- distance (m) between sensor module 1 (S1) and sensor module 2 (S2)
 |		distanceC (IN) -- distance (m) from sensor module 2 (S2) to the targeted object based on UMR result
 |
 |  Returns:  angle (degrees) from transceiver element S1 to the targeted object
 *-------------------------------------------------------------------*/
double pga460::triangulation(double a, double b, double c)
{
	#ifdef EnTri
	// LAW OF COSINES
	double inAngle;
	if (a+b>c)
	{
		return inAngle =(acos(((a*a)+(b*b)-(c*c))/(2*a*b))) * 57.3; //Radian to Degree = Rad * (180/PI)
	}
	else
	{
		return 360;
	}
	
	// COORDINATE
	// TODO
	#endif
}

/*------------------------------------------------- registerRead -----
 |  Function registerRead
 |
 |  Purpose:  Read single register data from PGA460
 |
 |  Parameters:
 |		addr (IN) -- PGA460 register address to read data from
 |
 |  Returns:  8-bit data read from register
 *-------------------------------------------------------------------*/
uint8_t  pga460::registerRead(uint8_t  addr)
{
	uint8_t  data = 0x00;
	uint8_t  temp = 0;
	
	if (comm == 2)
	{
		owuShift = 1; // OWU receive buffer offset to ignore transmitted data
	}
	else
	{
		owuShift = 0;
	}	
	
	pga460SerialFlush();
	
	regAddr = addr;
	uint8_t  buf9[4] = {syncByte , SRR, regAddr, calcChecksum(SRR)};
	if (comm == 0 || comm == 2) // UART or OWU mode
	{
		serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
		for (int i = 1; i < sizeof(buf9); ++i) {
			serialPutchar(serial_fd, buf9[i]);
		}
	}
	// if (comm == 3) // SPI mode
	// {
	// 	spiTransfer(buf9, sizeof(buf9));
	// }
	delay(10);
	if (comm == 0 || comm == 2) // UART or OWU mode
	{
		for(int n=0; n<3; n++)
		{
		   if(n==1-owuShift)
		   {
				data = serialGetchar(serial_fd); // store read data
		   }
		   else
		   {
			   temp = serialGetchar(serial_fd);
		   }
		}
	}
	// if (comm == 3) // SPI mode
	// {
	// 	spiMosiIdle(3);
	// 	data = misoBuf[1];
	// }	
	
	return data;
}
	
/*------------------------------------------------- registerWrite -----
 |  Function registerWrite
 |
 |  Purpose:  Write single register data to PGA460
 |
 |  Parameters:
 |		addr (IN) -- PGA460 register address to write data to
 |		data (IN) -- 8-bit data value to write into register
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
uint8_t  pga460::registerWrite(uint8_t  addr, uint8_t  data)
{
	regAddr = addr;
	regData = data;	
	uint8_t  buf9[4] = {syncByte , SRR, regAddr, calcChecksum(SRR)};
	if (comm == 0 || comm == 2) // UART or OWU mode
	{
		serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
		for (int i = 1; i < sizeof(buf9); ++i) 
		{
			serialPutchar(serial_fd, buf9[i]);
		}
	}
	delay(10);
	return 0;
}

/*------------------------------------------------- autoThreshold -----
 |  Function autoThreshold
 |
 |  Purpose:  Automatically assigns threshold time and level values
 |  			based on a no-object burst/listen command
 |
 |  Parameters:
 |		cmd (IN) -- preset 1 or 2 burst and/or listen command to run
 |		noiseMargin (IN) -- margin between maximum downsampled noise
 |						value and the threshold level in intervals
 |						of 8.
 |		windowIndex (IN) -- spacing between each threshold time as an
 |						index (refer to datasheet for microsecond
 |						equivalent). To use the existing threshold
 |						times, enter a value of '16'.
 |		autoMax (IN) -- automatically set threshold levels up to this
 |					threshold point (maximum is 12). Remaining levels
 |					will not change.
 |		loops (IN) -- number of command loops to run to collect a
 |					running average of the echo data dump points.
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::autoThreshold(uint8_t  cmd, uint8_t  noiseMargin, uint8_t  windowIndex, uint8_t  autoMax, uint8_t  avgLoops)
{
	#ifdef EnAutoThr
	// local variables
	uint8_t  thrTime[6]; // threshold time values for selected preset
	uint8_t  thrLevel[10]; //threshold level values for selected preset
	uint8_t  thrMax[12]; // maximum echo data dump values per partition
	uint8_t  presetOffset = 0; // determines if regsiter threshold address space is initialized at P1 or P2
	uint8_t  thrOffset = 0; // -6 to +7 where MSB is sign value
	bool 	 thrOffsetFlag = 0; //when high, the level offset value is updated
	
	//read existing threhsold values into thrTime array
	switch (cmd)
	{
		//Preset 1 command
		case 0:
		case 2:
			pga460::thresholdBulkRead(1);			
			break;		
		//Preset 2 command
		case 1:
		case 3:
			pga460::thresholdBulkRead(2);
			presetOffset = 16;
			break;		
		//Invalid command
		default:
			return;
			break;
	}
	
	// set thrTime and thrLevel to existing threshold time and level values respectively
	for (uint8_t  h = 0; h<6; h++)
	{
		thrTime[h] = bulkThr[h + presetOffset];
	}
	for (uint8_t  g = 0; g<10; g++)
	{
		thrLevel[g] = bulkThr[g + 6 + presetOffset];
	}
	
	// replace each preset time with windowIndex for the number of points to auto-calc
	if (windowIndex >= 16)
	{
		//skip threshold-time configuration
	}
	else
	{
		for (uint8_t  i = 0; i < 12; i+=2)
		{	
	
			if (autoMax > i)
			{
				thrTime[i/2] = thrTime[i/2] & 0x0F;
				thrTime[i/2] = (windowIndex << 4) | thrTime[i/2];
				if (autoMax > i+1)
				{
					thrTime[i/2] = thrTime[i/2] & 0xF0;
					thrTime[i/2] = (windowIndex & 0x0F) | thrTime[i/2];
				}
			}
		}
	}

	// run burst-and-listen to collect EDD data
	pga460::runEchoDataDump(cmd);
	
	// read the record length value for the preset
	uint8_t  recLength = pga460::registerRead(0x22); // read REC_LENGTH Register
	switch(cmd)
	{
		//Preset 1 command
		case 0:
		case 2:
			recLength = (recLength >> 4) & 0x0F;			
			break;		
		//Preset 2 command
		case 1:
		case 3:
			recLength = recLength & 0x0F;
			break;		
		//Invalid command
		default:
			return;
			break;
	}
	
	// convert record length value to time equivalent in microseconds
	unsigned int recTime = (recLength + 1) * 4096;
	
	//determine the number of threshold points that are within the record length time
	uint8_t  numPoints = 0;
	uint8_t  thrTimeReg = 0;
	unsigned int thrMicro = 0; // threhsold total time in microseconds
	unsigned int eddMarker[12]; // echo data dump time marker between each threhsold point
	for (thrTimeReg = 0; thrTimeReg < 6; thrTimeReg++)
	{		
		// check threshold 1 of 2 in single register
		switch ((thrTime[thrTimeReg] >> 4) & 0x0F)
		{			
			case 0: thrMicro += 100; break;
			case 1: thrMicro += 200; break;
			case 2: thrMicro += 300; break;
			case 3: thrMicro += 400; break;
			case 4: thrMicro += 600; break;
			case 5: thrMicro += 800; break;
			case 6: thrMicro += 1000; break;
			case 7: thrMicro += 1200; break;
			case 8: thrMicro += 1400; break;
			case 9: thrMicro += 2000; break;
			case 10: thrMicro += 2400; break;
			case 11: thrMicro += 3200; break;
			case 12: thrMicro += 4000; break;
			case 13: thrMicro += 5200; break;
			case 14: thrMicro += 6400; break;
			case 15: thrMicro += 8000; break;
			default: break;
		}
		eddMarker[thrTimeReg*2] = thrMicro;
		if (thrMicro >= recTime)
		{			
			numPoints = thrTimeReg * 2;
			thrTimeReg = 6; //exit
		}
		else
		{
			// check threshold 2 of 2 in single register
			switch (thrTime[thrTimeReg] & 0x0F)
			{
				case 0: thrMicro += 100; break;
				case 1: thrMicro += 200; break;
				case 2: thrMicro += 300; break;
				case 3: thrMicro += 400; break;
				case 4: thrMicro += 600; break;
				case 5: thrMicro += 800; break;
				case 6: thrMicro += 1000; break;
				case 7: thrMicro += 1200; break;
				case 8: thrMicro += 1400; break;
				case 9: thrMicro += 2000; break;
				case 10: thrMicro += 2400; break;
				case 11: thrMicro += 3200; break;
				case 12: thrMicro += 4000; break;
				case 13: thrMicro += 5200; break;
				case 14: thrMicro += 6400; break;
				case 15: thrMicro += 8000; break;
				default: break;
			}
			eddMarker[thrTimeReg*2+1] = thrMicro;
			if (thrMicro >= recTime)
			{
				numPoints = (thrTimeReg * 2) + 1;
				thrTimeReg = 6; //exit
			}
		}	
	}
	if (numPoints == 0) //if all points fall within the record length
	{
		numPoints = 11;
	}
	
	//convert up to 12 echo data dump markers from microseconds to index
	uint8_t  eddIndex[13];
	eddIndex[0] = 0;
	for (uint8_t  l = 0; l < 12; l++)
	{
		eddIndex[l+1] = ((eddMarker[l]/100)*128)/(recTime/100); // divide by 100 for best accuracy in MSP430
	}	
	
	// downsample the echo data dump based on the number of partitions
	memset(thrMax, 0x00, 12); // zero thrMax array
	uint8_t  eddLevel = 0;
	for (uint8_t  j = 0; j < numPoints+1; j++)
	{	
		eddLevel = 0;
		for (uint8_t  k = eddIndex[j]; k < eddIndex[j+1]; k++)
		{
			eddLevel = pga460::pullEchoDataDump(k);
			if (thrMax[j] < eddLevel)
			{
				thrMax[j] = eddLevel;
			}		
		}	
	}
	//set threhsold points which exceed the record length to same value as last valid value
	if (numPoints < autoMax)
	{
		for (int o = numPoints; o < autoMax; o++)
		{
			if (numPoints ==0)
			{
				thrMax[o] = 128;
			}
			else
			{
				thrMax[o] = thrMax[numPoints-1];
			}
		}
	}

	// filter y-max for level compatibility of first eight points
	for (int m = 0; m < 8; m++)
	{
		//first eight levels must be mutliples of eight
		while ((thrMax[m] % 8 != 0) && (thrMax[m] < 248)) 
		{
			thrMax[m] += 1;
		}
	}
	
	// apply noise floor offset
	for (int n = 0; n < 12; n++)
	{
		if (thrMax[n] + noiseMargin >= 248 && thrMax[n] + noiseMargin < 255)
		{
			thrMax[n] = 248;
			thrOffset = 0b0110; //+6
			thrOffsetFlag = true;
		}
		else if (thrMax[n] + noiseMargin >= 255)
		{
			thrMax[n] = 248;
			thrOffset = 0b0111; // +7
			thrOffsetFlag = true;
		}
		else
		{
			thrMax[n] += noiseMargin;
		}
	}
	
	//convert first eight auto calibrated levels to five-bit equivalents
	uint8_t  rounding = 0;
	if (autoMax >= 8)
	{
		rounding = 8;
	}
	else
	{
		rounding = autoMax;
	}
	for(uint8_t  p = 0; p < rounding; p++)
	{
		thrMax[p] = thrMax[p] / 8;
	}
	
	// concatenate and merge threshold level register values
	if (autoMax > 0) //Px_THR_6 L1,L2
	{
		thrLevel[0] = (thrLevel[0] & ~0xF8) | (thrMax[0] << 3);
	}
	if (autoMax > 1) //Px_THR_6 L1,L2
	{
		thrLevel[0] = (thrLevel[0] & ~0x07) | (thrMax[1] >> 2);
	}
	
	if (autoMax > 1) //Px_THR_7 L2,L3,L4
	{
		thrLevel[1] = (thrLevel[1] & ~0xC0) | (thrMax[1] << 6);
	}
	if (autoMax > 2) //Px_THR_7 L2,L3,L4
	{
		thrLevel[1] = (thrLevel[1] & ~0x3E) | (thrMax[2] << 1);
	}
	if (autoMax > 3) //Px_THR_7 L2,L3,L4
	{
		thrLevel[1] = (thrLevel[1] & ~0x01) | (thrMax[3] >> 4 );
	}
	
	if (autoMax > 3) //Px_THR_8 L4,L5
	{
		thrLevel[2] = (thrLevel[2] & ~0xF0) | (thrMax[3] << 4 );
	}
	if (autoMax > 4) //Px_THR_8 L4,L5
	{
		thrLevel[2] = (thrLevel[2] & ~0x0F) | (thrMax[4] >> 1 );
	}
	
	if (autoMax > 4) //Px_THR_9 L5,L6,L7
	{
		thrLevel[3] = (thrLevel[3] & ~0x80) | (thrMax[4] << 7 );
	}
	if (autoMax > 5) //Px_THR_9 L5,L6,L7
	{
		thrLevel[3] = (thrLevel[3] & ~0x7C) | (thrMax[5] << 2 );
	}
	if (autoMax > 6) //Px_THR_9 L5,L6,L7
	{
		thrLevel[3] = (thrLevel[3] & ~0x03) | (thrMax[6] >> 3 );
	}
	
	if (autoMax > 6) //Px_THR_10 L7,L8
	{
		thrLevel[4] = (thrLevel[4] & ~0xE0) | (thrMax[6] << 5 );
	}
	if (autoMax > 7) //Px_THR_10 L7,L8
	{
		thrLevel[4] = (thrLevel[4] & ~0x1F) | (thrMax[7]);
	}
	
	if (autoMax > 8) //Px_THR_11 L9 
	{
		thrLevel[5] = thrMax[8];
	}
	if (autoMax > 9) //Px_THR_12 L10
	{
		thrLevel[6] = thrMax[9];
	}
	if (autoMax > 10) //Px_THR_13 L11 
	{
		thrLevel[7] = thrMax[10];
	}
	if (autoMax > 11) //Px_THR_14 L12
	{
		thrLevel[8] = thrMax[11];
	}
	if (thrOffsetFlag == true) //Px_THR_15 LOff
	{
		thrLevel[9] = thrOffset & 0x0F;
	}
	
	// update threshold register values
	switch(cmd)
	{
		//Preset 1 command
		case 0:
		case 2:				
			P1_THR_0 = thrTime[0];
			P1_THR_1 = thrTime[1];
			P1_THR_2 = thrTime[2];
			P1_THR_3 = thrTime[3];
			P1_THR_4 = thrTime[4];
			P1_THR_5 = thrTime[5];
			P1_THR_6 = thrLevel[0];
			P1_THR_7 = thrLevel[1];
			P1_THR_8 = thrLevel[2];
			P1_THR_9 = thrLevel[3];
			P1_THR_10 = thrLevel[4];
			P1_THR_11 = thrLevel[5];
			P1_THR_12 = thrLevel[6];
			P1_THR_13 = thrLevel[7];
			P1_THR_14 = thrLevel[8];
			P1_THR_15 = thrLevel[9];
			
			pga460::thresholdBulkRead(2);
			presetOffset = 16;
			
			P2_THR_0 = bulkThr[0 + presetOffset];
			P2_THR_1 = bulkThr[1 + presetOffset];
			P2_THR_2 = bulkThr[2 + presetOffset];
			P2_THR_3 = bulkThr[3 + presetOffset];
			P2_THR_4 = bulkThr[4 + presetOffset];
			P2_THR_5 = bulkThr[5 + presetOffset];
			P2_THR_6 = bulkThr[6 + presetOffset];
			P2_THR_7 = bulkThr[7 + presetOffset];
			P2_THR_8 = bulkThr[8 + presetOffset];
			P2_THR_9 = bulkThr[9 + presetOffset];
			P2_THR_10 = bulkThr[10 + presetOffset];
			P2_THR_11 = bulkThr[11 + presetOffset];
			P2_THR_12 = bulkThr[12 + presetOffset];
			P2_THR_13 = bulkThr[13 + presetOffset];
			P2_THR_14 = bulkThr[14 + presetOffset];
			P2_THR_15 = bulkThr[15 + presetOffset];		
			break;		
		//Preset 2 command
		case 1:
		case 3:
			P2_THR_0 = thrTime[0];
			P2_THR_1 = thrTime[1];
			P2_THR_2 = thrTime[2];
			P2_THR_3 = thrTime[3];
			P2_THR_4 = thrTime[4];
			P2_THR_5 = thrTime[5];
			P2_THR_6 = thrLevel[0];
			P2_THR_7 = thrLevel[1];
			P2_THR_8 = thrLevel[2];
			P2_THR_9 = thrLevel[3];
			P2_THR_10 = thrLevel[4];
			P2_THR_11 = thrLevel[5];
			P2_THR_12 = thrLevel[6];
			P2_THR_13 = thrLevel[7];
			P2_THR_14 = thrLevel[8];
			P2_THR_15 = thrLevel[9];
			
			pga460::thresholdBulkRead(1);
			presetOffset = 0;
			
			P1_THR_0 = bulkThr[0 + presetOffset];
			P1_THR_1 = bulkThr[1 + presetOffset];
			P1_THR_2 = bulkThr[2 + presetOffset];
			P1_THR_3 = bulkThr[3 + presetOffset];
			P1_THR_4 = bulkThr[4 + presetOffset];
			P1_THR_5 = bulkThr[5 + presetOffset];
			P1_THR_6 = bulkThr[6 + presetOffset];
			P1_THR_7 = bulkThr[7 + presetOffset];
			P1_THR_8 = bulkThr[8 + presetOffset];
			P1_THR_9 = bulkThr[9 + presetOffset];
			P1_THR_10 = bulkThr[10 + presetOffset];
			P1_THR_11 = bulkThr[11 + presetOffset];
			P1_THR_12 = bulkThr[12 + presetOffset];
			P1_THR_13 = bulkThr[13 + presetOffset];
			P1_THR_14 = bulkThr[14 + presetOffset];
			P1_THR_15 = bulkThr[15 + presetOffset];	
			break;		
		//Invalid command
		default:
			return;
			break;
	}
	
	uint8_t  p1ThrMap[16] = {P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5,
							P1_THR_6, P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11,
							P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15};
	uint8_t  p2ThrMap[16] = {P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5,
							P2_THR_6, P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11,
							P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15};
							
	pga460::thresholdBulkWrite(p1ThrMap, p2ThrMap);
	
	#endif
}

/*------------------------------------------------- thresholdBulkRead -----
 |  Function thresholdBulkRead
 |
 |  Purpose:  Bulk read all threshold times and levels 
 |
 |  Parameters:
 |		preset (IN) -- which preset's threshold data to read
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::thresholdBulkRead(uint8_t  preset)
{
	#ifdef EnAutoThr
	uint8_t  n = 0;
	uint8_t  buf15[2] = {syncByte , THRBR};
	uint8_t  presetOffset = 0;
	uint8_t  addr = 0x5F; // beginning of threshold register space
	
	switch (comm)
	{
		case 0:
		case 2:
			if (preset == 2) //Preset 2 advances 16 address uint8_t s
			{
				presetOffset = 16;
			}
			
			for (int n = 0; n<16; n++)
			{
				bulkThr[n + presetOffset] = registerRead(addr + presetOffset);
				addr++;
			}		
			
			// Threshold Bulk Read Command 15 too large for Serial1 receive buffer
			/*Serial1.write(buf15, sizeof(buf15));
			delay (300);
			while (Serial1.available() > 0)
			{
				bulkThr[n] = Serial1.read();
				n++;
			}*/
			
			break;
		
		case 1: //TCI
			//TODO
			break;
			
		case 3: //SPI
			//TODO
			break;
		
		default:
			break;
	}
	#endif
}

/*------------------------------------------------- thresholdBulkWrite -----
 |  Function thresholdBulkWrite
 |
 |  Purpose:  Bulk write to all threshold registers
 |
 |  Parameters:
 |		p1ThrMap (IN) -- data uint8_t  array for 16 uint8_t s of Preset 1 threhsold data
  |		p2ThrMap (IN) -- data uint8_t  array for 16 uint8_t s of Preset 2 threhsold data
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::thresholdBulkWrite(uint8_t  *p1ThrMap, uint8_t  *p2ThrMap)
{
	#ifdef EnAutoThr
	//bulk write new threshold values
	if ((comm == 0 || comm == 2 || comm==3) && (comm !=6)) 	// USART or OWU mode and not busDemo6
	{
		uint8_t  buf16[35] = {syncByte ,	THRBW, p1ThrMap[0], p1ThrMap[1], p1ThrMap[2], p1ThrMap[3], p1ThrMap[4], p1ThrMap[5],
			p1ThrMap[6], p1ThrMap[7], p1ThrMap[8], p1ThrMap[9], p1ThrMap[10], p1ThrMap[11], p1ThrMap[12],
			p1ThrMap[13], p1ThrMap[14], p1ThrMap[15],
			p2ThrMap[0], p2ThrMap[1], p2ThrMap[2], p2ThrMap[3], p2ThrMap[4], p2ThrMap[5],
			p2ThrMap[6], p2ThrMap[7], p2ThrMap[8], p2ThrMap[9], p2ThrMap[10], p2ThrMap[11], p2ThrMap[12],
			p2ThrMap[13], p2ThrMap[14], p2ThrMap[15],
			calcChecksum(THRBW)};
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			serialPutchar(serial_fd, syncByte); // Assuming serial_fd is the file descriptor for serial communication
			for (int i = 1; i < sizeof(buf16); ++i) {
				serialPutchar(serial_fd, buf16[i]);
			} // serial transmit master data for bulk threhsold
		}
		// if (comm == 3) // SPI mode
		// {
		// 	spiTransfer(buf16, sizeof(buf16));
		// }
		
	}
	else if(comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		tciIndexRW(5, true); //TCI Threshold Preset 1 write
		tciIndexRW(6, true); //TCI Threshold Preset 2 write
	}
	else
	{
		//do nothing
	}
	
	delay(100);
	return;
	#endif
}

/*------------------------------------------------- eepromThreshold -----
 |  Function eepromThreshold
 |
 |  Purpose:  Copy a single preset's threshold times and levels 
 |  			to USER_DATA1-16 in EEPROM
 |
 |  Parameters:
 |		preset (IN) -- preset's threshold to copy
 |		saveLoad (IN) -- when false, copy threshold to EEPROM;
 |					when true, copy threshold from EEPROM
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::eepromThreshold(uint8_t  preset, bool saveLoad)
{
	#ifdef EnAutoThr
	uint8_t  presetOffset = 0;
	uint8_t  addr = 0x5F; // beginning of threshold memory space
	
	if (saveLoad == false) // save thr
	{
		//Preset 2 advances 16 address uint8_t s
		if (preset == 2 || preset == 4) 
		{
			presetOffset = 16;
		}
		
		for (int n = 0; n<16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(addr + presetOffset);
			// write threshold values into USER_DATA1-16
			registerWrite(n, bulkThr[n + presetOffset]);
			addr++;
		}
	}
	else // load thr
	{
		//Preset 2 advances 16 address uint8_t s
		if (preset == 2 || preset == 4) //Preset 2 advances 16 address uint8_t s
		{
			presetOffset = 16;
		}
		
		// copy USER_DATA1-16 into selected preset threhsold space
		for (int n = 0; n<16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(n);
			// bulk write to threshold
			registerWrite(addr + presetOffset, bulkThr[n + presetOffset]);
			addr++;
		}
	}
	#endif
}

/*------------------------------------------------- FUNCTION_NAME -----
 |  Function FUNCTION_NAME
 |
 |  Purpose:  EXPLAIN WHAT THIS FUNCTION DOES TO SUPPORT THE CORRECT
 |      OPERATION OF THE PROGRAM, AND HOW IT DOES IT.
 |
 |  Parameters:
 |      parameter_name (IN, OUT, or IN/OUT) -- EXPLANATION OF THE
 |              PURPOSE OF THIS PARAMETER TO THE FUNCTION.
 |                      (REPEAT THIS FOR ALL FORMAL PARAMETERS OF
 |                       THIS FUNCTION.
 |                       IN = USED TO PASS DATA INTO THIS FUNCTION,
 |                       OUT = USED TO PASS DATA OUT OF THIS FUNCTION
 |                       IN/OUT = USED FOR BOTH PURPOSES.)
 |
 |  Returns:  IF THIS FUNCTION SENDS BACK A VALUE VIA THE RETURN
 |      MECHANISM, DESCRIBE THE PURPOSE OF THAT VALUE HERE.
 *-------------------------------------------------------------------*/
