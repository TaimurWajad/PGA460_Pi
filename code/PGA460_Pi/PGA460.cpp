#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  // Include this header for strerror
#include <stdint.h>  // Include this header for uint8_t and uint32_t
#include <termios.h>
#include "PGA460.h"





/*+++++++++++++++++ Define UART commands by name ++++++++++++++++*/
	// Single Address
		uint8_t P1BL = 0x00;
		uint8_t P2BL = 0x01;
		uint8_t P1LO = 0x02;
		uint8_t P2LO = 0x03;
		uint8_t TNLM = 0x04;
		uint8_t UMR = 0x05;
		uint8_t TNLR = 0x06;
		uint8_t TEDD = 0x07;
		uint8_t SD = 0x08;
		uint8_t SRR = 0x09; 
		uint8_t SRW = 0x0A;
		uint8_t EEBR = 0x0B;
		uint8_t EEBW = 0x0C;
		uint8_t TVGBR = 0x0D;
		uint8_t TVGBW = 0x0E;
		uint8_t THRBR = 0x0F;
		uint8_t THRBW = 0x10; 
	//Broadcast
		uint8_t BC_P1BL = 0x11;
		uint8_t BC_P2BL = 0x12;
		uint8_t BC_P1LO = 0x13;
		uint8_t BC_P2LO = 0x14;
		uint8_t BC_TNLM = 0x15;
		uint8_t BC_RW = 0x16;
		uint8_t BC_EEBW = 0x17;
		uint8_t BC_TVGBW = 0x18;
		uint8_t BC_THRBW = 0x19;
		//CMDs 26-31 are reserved

// List user registers by name with default settings from TI factory
	uint8_t USER_DATA1 = 0x00;
	uint8_t USER_DATA2 = 0x00;
	uint8_t USER_DATA3 = 0x00;
	uint8_t USER_DATA4 = 0x00;
	uint8_t USER_DATA5 = 0x00;
	uint8_t USER_DATA6 = 0x00;
	uint8_t USER_DATA7 = 0x00;
	uint8_t USER_DATA8 = 0x00;
	uint8_t USER_DATA9 = 0x00;
	uint8_t USER_DATA10 = 0x00;
	uint8_t USER_DATA11 = 0x00;
	uint8_t USER_DATA12 = 0x00;
	uint8_t USER_DATA13 = 0x00;
	uint8_t USER_DATA14 = 0x00;
	uint8_t USER_DATA15 = 0x00;
	uint8_t USER_DATA16 = 0x00;
	uint8_t USER_DATA17 = 0x00;
	uint8_t USER_DATA18 = 0x00;
	uint8_t USER_DATA19 = 0x00;
	uint8_t USER_DATA20 = 0x00;
	uint8_t TVGAIN0 = 0xAF;
	uint8_t TVGAIN1 = 0xFF;
	uint8_t TVGAIN2 = 0xFF;
	uint8_t TVGAIN3 = 0x2D;
	uint8_t TVGAIN4 = 0x68;
	uint8_t TVGAIN5 = 0x36;
	uint8_t TVGAIN6 = 0xFC;
	uint8_t INIT_GAIN = 0xC0;
	uint8_t FREQUENCY  = 0x8C;
	uint8_t DEADTIME = 0x00;
	uint8_t PULSE_P1 = 0x01;
	uint8_t PULSE_P2 = 0x12;
	uint8_t CURR_LIM_P1 = 0x47;
	uint8_t CURR_LIM_P2 = 0xFF;
	uint8_t REC_LENGTH = 0x1C;
	uint8_t FREQ_DIAG = 0x00;
	uint8_t SAT_FDIAG_TH = 0xEE;
	uint8_t FVOLT_DEC = 0x7C;
	uint8_t DECPL_TEMP = 0x0A;
	uint8_t DSP_SCALE = 0x00;
	uint8_t TEMP_TRIM = 0x00;
	uint8_t P1_GAIN_CTRL = 0x00;
	uint8_t P2_GAIN_CTRL = 0x00;
	uint8_t EE_CRC = 0x82;//0xFF;
	uint8_t EE_CNTRL = 0x00;
	uint8_t P1_THR_0 = 0x88;
	uint8_t P1_THR_1 = 0x88;
	uint8_t P1_THR_2 = 0x88;
	uint8_t P1_THR_3 = 0x88;
	uint8_t P1_THR_4 = 0x88;
	uint8_t P1_THR_5 = 0x88;
	uint8_t P1_THR_6 = 0x84;
	uint8_t P1_THR_7 = 0x21;
	uint8_t P1_THR_8 = 0x08;
	uint8_t P1_THR_9 = 0x42;
	uint8_t P1_THR_10 = 0x10;
	uint8_t P1_THR_11 = 0x80;
	uint8_t P1_THR_12 = 0x80;
	uint8_t P1_THR_13 = 0x80;
	uint8_t P1_THR_14 = 0x80;
	uint8_t P1_THR_15 = 0x80;
	uint8_t P2_THR_0 = 0x88;
	uint8_t P2_THR_1 = 0x88;
	uint8_t P2_THR_2 = 0x88;
	uint8_t P2_THR_3 = 0x88;
	uint8_t P2_THR_4 = 0x88;
	uint8_t P2_THR_5 = 0x88;
	uint8_t P2_THR_6 = 0x84;
	uint8_t P2_THR_7 = 0x21;
	uint8_t P2_THR_8 = 0x08;
	uint8_t P2_THR_9 = 0x42;
	uint8_t P2_THR_10 = 0x10;
	uint8_t P2_THR_11 = 0x80;
	uint8_t P2_THR_12 = 0x80;
	uint8_t P2_THR_13 = 0x80;
	uint8_t P2_THR_14 = 0x80;
	uint8_t P2_THR_15 = 0x80;

// Miscellaneous variables; (+) indicates OWU transmitted byte offset
	uint8_t checksum = 0x00; 			// UART checksum value	
	uint8_t ChecksumInput[44] = {0}; 		// data byte array for checksum calculator
	uint8_t ultraMeasResult[34+3] = {0}; 	// data byte array for cmd5 and tciB+L return
	uint8_t diagMeasResult[5+3] = {0}; 		// data byte array for cmd8 and index1 return
	uint8_t tempNoiseMeasResult[4+3] = {0}; 	// data byte array for cmd6 and index0&1 return
	uint8_t echoDataDump[130+3] = {0}; 		// data byte array for cmd7 and index12 return
	uint8_t tmpRst[3] = {0};
	uint8_t regRead[3] = {0};
	uint8_t tempOrNoise = 0; 			// data byte to determine if temp or noise measurement is to be performed
	uint8_t comm = 0; 					// indicates UART (0), TCI (1), OWU (2) communication mode	
	unsigned long starttime; 		// used for function time out
	uint8_t bulkThr[34+3] = {0};				// data byte array for bulk threhsold commands
//UART & OWU exclusive variables
	uint8_t syncByte = 0x55; 		// data byte for Sync field set UART baud rate of PGA460
	uint8_t regAddr = 0x00; 		// data byte for Register Address
	uint8_t regData = 0x00; 		// data byte for Register Data
	uint8_t uartAddr = 0; 			// PGA460 UART device address (0-7). '0' is factory default address
	uint8_t numObj = 1; 			// number of objects to detect
	
	
void initVariables() 
{

}
			
void sendBytes(int fd, unsigned char *data, int length) 
{
	//printf("Tx data: ");
	for (int i = 0; i < length; i++) 
	{
		serialPutchar(fd, data[i]);
		//printf("0x%02X ", data[i]);
	}
}
int receiveBytes(int fd, unsigned char *buffer, int maxLength) 
{
    int received = 0;

    while (serialDataAvail(fd) > 0 && received < maxLength) 
	{
        buffer[received++] = serialGetchar(fd);
        usleep(1000);  // Small delay to allow data to be received
    }

    return received;
}
// Function to receive a specified number of bytes from a serial port
bool receiveBytesFromSerial(int serial_port, unsigned char* buffer, int numBytesToReceive) 
{
    unsigned long starttime = millis(); // Start time in milliseconds

    // Wait until either all expected bytes are received or timeout occurs
    while ((serialDataAvail(serial_port) < numBytesToReceive) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT)) 
	{
        // Wait in this loop until we either get the expected bytes or time out
    }

    // Check if all expected bytes were received
    if (serialDataAvail(serial_port)<0)//TODO; Check here(serialDataAvail(serial_port) < numBytesToReceive) 
	{
        // Print an error message if data didn't come in within timeout
        printf("ERROR - Did not receive expected bytes from serial port!\n");
        return false; // Return false indicating failure
    } 
	else 
	{
        // Read the expected number of bytes into the buffer
        for (int n = 0; n < numBytesToReceive; n++) 
		{
            buffer[n] = serialGetchar(serial_port);
			//printf("0x%02X ", buffer[n]);
        }
        return true; // Return true indicating success
    }
}

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
void initBoostXLPGA460(uint8_t mode, uint32_t baud, uint8_t uartAddrUpdate)
{
	// check for valid UART address
	if (uartAddrUpdate > 7)
	{
		uartAddrUpdate = 0; // default to '0'
		printf("ERROR - Invalid UART Address!");
	}
	// globally update target PGA460 UART address and commands	
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
	

	// set communication mode flag
	if (mode < 4) // 0=UART, 1=TCI, 2=OWU, 3=SPI
	{
		comm = mode;
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
void initThresholds(uint8_t thr, int serial_port)
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
		
	case 3: //Custom: TI
			P1_THR_0 = 0x77; // Ignore initial ringing noise
			P1_THR_1 = 0x77; // Ignore ringing (close to transmit pulse)
			P1_THR_2 = 0x77; // Reduce sensitivity (ringing ends)
			P1_THR_3 = 0x77; // Start detecting echoes
			P1_THR_4 = 0x78; // Lower threshold for mid-range targets
			P1_THR_5 = 0x88; // Continue lowering threshold
			P1_THR_6 = 0x9C; // Sensitive to distant echoes
			P1_THR_7 = 0xD0;
			P1_THR_8 = 0x72;
			P1_THR_9 = 0x10;
			P1_THR_10 = 0x63; // Detect weak echoes
			P1_THR_11 = 0x28;
			P1_THR_12 = 0x30;
			P1_THR_13 = 0x34;
			P1_THR_14 = 0x3C;
			P1_THR_15 = 0x00; // End of detection window

			P2_THR_0 = 0x77; // Ignore initial ringing noise
			P2_THR_1 = 0x77; // Ignore ringing (close to transmit pulse)
			P2_THR_2 = 0x77; // Reduce sensitivity (ringing ends)
			P2_THR_3 = 0x77; // Start detecting echoes
			P2_THR_4 = 0x78; // Lower threshold for mid-range targets
			P2_THR_5 = 0x88; // Continue lowering threshold
			P2_THR_6 = 0x9C; // Sensitive to distant echoes
			P2_THR_7 = 0xD0;
			P2_THR_8 = 0x72;
			P2_THR_9 = 0x10;
			P2_THR_10 = 0x63; // Detect weak echoes
			P2_THR_11 = 0x28;
			P2_THR_12 = 0x30;
			P2_THR_13 = 0x34;
			P2_THR_14 = 0x3C;
			P2_THR_15 = 0x00; // End of detection window
		break;
	case 4: //Custom
		P1_THR_0 = 0xFF; // Ignore initial ringing noise
		P1_THR_1 = 0xFF; // Ignore ringing (close to transmit pulse)
		P1_THR_2 = 0xC8; // Reduce sensitivity (ringing ends)
		P1_THR_3 = 0x88; // Start detecting echoes
		P1_THR_4 = 0x68; // Lower threshold for mid-range targets
		P1_THR_5 = 0x58; // Continue lowering threshold
		P1_THR_6 = 0x48; // Sensitive to distant echoes
		P1_THR_7 = 0x38;
		P1_THR_8 = 0x28;
		P1_THR_9 = 0x20;
		P1_THR_10 = 0x18; // Detect weak echoes
		P1_THR_11 = 0x10;
		P1_THR_12 = 0x10;
		P1_THR_13 = 0x10;
		P1_THR_14 = 0x08;
		P1_THR_15 = 0x00; // End of detection window

		P2_THR_0 = 0xFF; // Ignore initial ringing noise
		P2_THR_1 = 0xFF; // Ignore ringing (close to transmit pulse)
		P2_THR_2 = 0xC8; // Reduce sensitivity (ringing ends)
		P2_THR_3 = 0x88; // Start detecting echoes
		P2_THR_4 = 0x68; // Lower threshold for mid-range targets
		P2_THR_5 = 0x58; // Continue lowering threshold
		P2_THR_6 = 0x48; // Sensitive to distant echoes
		P2_THR_7 = 0x38;
		P2_THR_8 = 0x28;
		P2_THR_9 = 0x20;
		P2_THR_10 = 0x18; // Detect weak echoes
		P2_THR_11 = 0x10;
		P2_THR_12 = 0x10;
		P2_THR_13 = 0x10;
		P2_THR_14 = 0x08;
		P2_THR_15 = 0x00; // End of detection window
		break;
		
		default: break;
	}
	
	uint8_t buf16[35] = {syncByte, THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
						 P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
						 P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6, 
						 P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
						calcChecksum(THRBW)};

	sendBytes(serial_port, buf16, sizeof(buf16)); // serial transmit master data for bulk threhsold	
	usleep(100*1000); // 100 ms
	return;
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
void defaultPGA460(uint8_t xdcr, int serial_port)
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
		   FREQUENCY  = 0x40;//0x32;
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
			TVGAIN0 = 0x88; // Lower initial gain
			TVGAIN1 = 0x88;
			TVGAIN2 = 0x88;
			TVGAIN3 = 0x72;
			TVGAIN4 = 0x50;
			TVGAIN5 = 0x30;
			TVGAIN6 = 0x20;
			INIT_GAIN = 0x60; // Initial gain (unchanged)
			FREQUENCY  = 0x94; // Set to ~40 kHz
			DEADTIME = 0xA0;
			PULSE_P1 = 0x08; // Shorter pulse train
			PULSE_P2 = 0x10;
			CURR_LIM_P1 = 0x88; // Current limit (unchanged)
			CURR_LIM_P2 = 0x88;
			REC_LENGTH = 0x25; // Extend receive window
			FREQ_DIAG = 0x33; // Diagnostic frequency
			SAT_FDIAG_TH = 0xEE; // Diagnostic threshold
			FVOLT_DEC = 0x7C; // Voltage decay
			DECPL_TEMP = 0x4F; // Decoupling cap and temp trim
			DSP_SCALE = 0x00; // No scaling
			TEMP_TRIM = 0x00; // Temp trim (unchanged)
			P1_GAIN_CTRL = 0x09; // P1 gain control
			P2_GAIN_CTRL = 0x09; // P2 gain control
		   break;
		}	
		case 3: // user custom //DEBUG
		{// TI
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
		   TVGAIN3 = 0x00;
		   TVGAIN4 = 0x00;
		   TVGAIN5 = 0x00;
		   TVGAIN6 = 0x00;
		   INIT_GAIN = 0x40;
		   FREQUENCY  = 0x32; // 0x32 for 40KHz
		   DEADTIME = 0x80;  // Optimized for transducer decay period (4080 �s)
		   PULSE_P1 = 0x0A;		   
		   PULSE_P2 = 0x10; //UART_ADDR=3
		   CURR_LIM_P1 = 0x40;
		   CURR_LIM_P2 = 0x40;
		   REC_LENGTH = 0x89;
		   FREQ_DIAG = 0x00;
		   SAT_FDIAG_TH = 0xEE;
		   FVOLT_DEC = 0x7C;
		   DECPL_TEMP = 0x4F;
		   DSP_SCALE = 0x00;
		   TEMP_TRIM = 0x00;
		   P1_GAIN_CTRL = 0x08;
		   P2_GAIN_CTRL = 0x18;
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
		   TVGAIN0 = 0x88;  // TVG at 0�1ms
		   TVGAIN1 = 0xAA;  // TVG at 1�2ms
		   TVGAIN2 = 0xCC;  // TVG at 2�3ms (decay ends ~2.7ms)
		   TVGAIN3 = 0xDD;  // TVG at 3�4ms
		   TVGAIN4 = 0xEE;  // TVG at 4�5ms
		   TVGAIN5 = 0xFF;  // TVG at 5�6ms
		   TVGAIN6 = 0xFF;  // TVG at 6�7ms (max gain)

		   INIT_GAIN = 0x60;  // Initial gain: Medium-high gain for better echo detection
		   FREQUENCY = 0x94;  // 40.7 kHz center frequency (adjusted for transducer specs)
		   DEADTIME = 0xA0;   // Adjust dead time for signal transition and ringing suppression

		   PULSE_P1 = 0x04;    // 4 pulses for P1 (ensure enough power for long range)
		   PULSE_P2 = 0x04;    // 4 pulses for P2
		   CURR_LIM_P1 = 0x55; // Medium drive current for P1
		   CURR_LIM_P2 = 0x55; // Medium drive current for P2

		   REC_LENGTH = 0x19;  // 25 ms record length for mid-range detection
		   FREQ_DIAG = 0x33;   // Enable frequency diagnostics for monitoring

		   SAT_FDIAG_TH = 0xEE; // Saturation threshold for diagnostics
		   FVOLT_DEC = 0x7C;    // Maximum drive voltage for transducer
		   DECPL_TEMP = 0x4F;   // Decoupling capacitor and temperature diagnostics

		   DSP_SCALE = 0x00;    // Default DSP scale
		   TEMP_TRIM = 0x00;    // Default temperature trim
		   P1_GAIN_CTRL = 0x09; // Enable gain control for P1
		   P2_GAIN_CTRL = 0x09; // Enable gain control for P2
		   break;
		}			
		default: break;
	}
		
		if ((comm == 0 || comm == 2 || comm ==3) && (comm !=6)) // USART or OWU mode and not busDemo6
		{
			uint8_t buf12[46] = {syncByte, EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
				USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, 
				USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
				TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
				PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
				DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(EEBW)};
		
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				sendBytes(serial_port, buf12, sizeof(buf12)); // serial transmit master data for bulk EEPROM
			}

			usleep(50000);  // Wait for 100 msecond before sending data again delay(50);
			
			// Update targeted UART_ADDR to address defined in EEPROM bulk switch-case
			uint8_t uartAddrUpdate = (PULSE_P2 >> 5) & 0x07;
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
		else
		{
			//do nothing
		}

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
void initTVG(uint8_t agr, uint8_t tvg, int serial_port)
{
	uint8_t gain_range = 0x4F;
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
	
	regAddr = 0x26;
	regData = gain_range;	
	uint8_t buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};
	sendBytes(serial_port, buf10, sizeof(buf10));

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
		
	case 3: //TI Levels
		TVGAIN0 = 0xAA;
		TVGAIN1 = 0xAA;
		TVGAIN2 = 0xAA;
		TVGAIN3 = 0x00;
		TVGAIN4 = 0x00;
		TVGAIN5 = 0x00;
		TVGAIN6 = 0x00;	
		break;
		
		default: break;
	}
	
	uint8_t buf14[10] = {syncByte, TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(TVGBW)};
	sendBytes(serial_port, buf14, sizeof(buf14)); // serial transmit master data for bulk TVG
	
	return;
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
double runDiagnostics(uint8_t run, uint8_t diag, int serial_port)
{
	double diagReturn = 0;
	//pga460SerialFlush(serial_port); // 
	// Flush UART buffers
    tcflush(serial_port, TCIOFLUSH);
	int elementOffset = 0;   //Only non-zero for OWU mode.
	int owuShiftSysDiag = 0; // Only non-zero for OWU mode.
	
	if (comm != 1) // USART and OWU
	{
			
		if (run == 1) // issue  P1 burst+listen, and run system diagnostics command to get latest results
		{
			// run burst+listen command at least once for proper diagnostic analysis
			ultrasonicCmd(0, 1, serial_port);	// always run preset 1 (short distance) burst+listen for 1 object for system diagnostic
			
			
			usleep(1000);  // Wait for 100 msecond before sending data again, :: delay(100); // record time length maximum of 65ms, so add margin
			//pga460SerialFlush(serial_port);
			// Flush UART buffers
			tcflush(serial_port, TCIOFLUSH);
			
			uint8_t buf8[3] = {syncByte, SD, calcChecksum(SD)};
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				sendBytes(serial_port, buf8, sizeof(buf8)); //serial transmit master data to read system diagnostic results
				
				receiveBytesFromSerial(serial_port, diagMeasResult, 4);
				printf("%02X ", diagMeasResult[0]);  // Print each byte in hexadecimal
				printf("%02X ", diagMeasResult[1]);  // Print each byte in hexadecimal
				printf("%02X ", diagMeasResult[2]);  // Print each byte in hexadecimal
#if 1				
				starttime = millis(); // 
				size_t bytes_read = 0;

				while ((bytes_read < 4 ) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT)) 
				{
            		bytes_read += read(serial_port, diagMeasResult + bytes_read, 4 - bytes_read);
        		}

				if (bytes_read < 4 ) 
				{
            		printf("ERROR - Did not receive system diagnostics!\n");
        		}
				else 
				{
            		for (int n = 0; n < 4; n++) 
					{
                		printf("Received byte[%d]: 0x%02X\n", n, diagMeasResult[n]);
            		}
        		}
#endif
			}
		}
		
		if (diag == 2) //run temperature measurement
		{
			tempOrNoise = 0; // temp meas
			uint8_t buf4[4] = {syncByte, TNLM, tempOrNoise, calcChecksum(TNLM)}; 
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				sendBytes(serial_port, buf4, sizeof(buf4)); //serial transmit master data to run temp measurement
				usleep(100);  // Wait for 10 ms before sending data againdelay(10);
				//pga460SerialFlush(serial_port);
				// Flush UART buffers
				tcflush(serial_port, TCIOFLUSH);
				usleep(100); 
			}
			
			uint8_t buf6[3] = {syncByte, TNLR, calcChecksum(TNLR)};
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				sendBytes(serial_port, buf6, sizeof(buf6)); //serial transmit master data to read temperature and noise results
			}
			
			usleep(1000); //delay(100);		
		}
			
		if (diag == 3) // run noise level meas
		{
			tempOrNoise = 1; // noise meas
			uint8_t buf4[4] = {syncByte, TNLM, tempOrNoise, calcChecksum(TNLM)};
			
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				sendBytes(serial_port, buf4, sizeof(buf4)); //serial transmit master data to run noise level measurement (requires at least 8.2ms of post-delay)
			}
			
			usleep(100);  // Wait for 10 ms delay(10);
			//pga460SerialFlush(serial_port);
			// Flush UART buffers
			tcflush(serial_port, TCIOFLUSH);
			usleep(100);  // Wait for 10 ms delay(10);
			
			uint8_t buf6[3] = {syncByte, TNLR, calcChecksum(TNLR)}; //serial transmit master data to read temperature and noise results
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				sendBytes(serial_port, buf6, sizeof(buf6));
			}
			
			usleep(1000);  // Wait for 100 ms delay(100);
		}
			
		if (comm == 0 || comm == 2) // UART or OWU mode
		{	
			if (diag == 2 || diag == 3) // pull temp and noise level results
			{
				receiveBytesFromSerial(serial_port, tempNoiseMeasResult, 4);
#if 0
				starttime = millis();
				while ( (Serial1.available()<(4)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) )
				{      
					// wait in this loop until we either get +4 bytes of data or 0.25 seconds have gone by
				}
				
				if(Serial1.available() < (4))
				{
					// the data didn't come in - handle the problem here
					Serial.println("ERROR - Did not receive temp/noise!");
				}
				else
				{
					for(int n=0; n<(4); n++)
					{
					   tempNoiseMeasResult[n] = Serial1.read();
					}
							
				}
#endif
			}
		}
			
	}
	else
	{
		//do nothing
	}
	
	usleep(1000); //delay(100);
		
	switch (diag)
	{
		case 0: // convert to transducer frequency in kHz
			{
				if(diagMeasResult[1+elementOffset] != 0)
				{
					diagReturn = (1 / (diagMeasResult[1+elementOffset] * 0.0000005)) / 1000;
				}
				else
				{
					diagReturn = 0;
				}
				
			}
			break;
		case 1: // convert to decay period time in us
			{
				if(diagMeasResult[1+elementOffset] != 0)
				{
					diagReturn = diagMeasResult[2+elementOffset] * 16;
				}
				else
				{
					diagReturn = 0;
				}			
			}
			break;
		case 2: //convert to temperature in degC
			{				
				if(tempNoiseMeasResult[1+elementOffset] != 0)
				{
					diagReturn = (tempNoiseMeasResult[1+elementOffset] - 64) / 1.5;
				}
				else
				{
					diagReturn = 0;
				}	
			}
			break;
		case 3: //noise floor level
			{				
				if(tempNoiseMeasResult[1+elementOffset] != 0)
				{
					diagReturn = tempNoiseMeasResult[2+elementOffset];
				}
				else
				{
					diagReturn = 0;
				}
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
#if 0
bool burnEEPROM(int serial_port)
{
    uint8_t burnStat = 0;
    bool burnSuccess = false;

    // Step 1: Write unlock pattern (0x68) to EE_CNTRL register
    regAddr = 0x40; // EE_CNTRL
    regData = 0x68; // Unlock pattern
    uint8_t buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};
    sendBytes(serial_port, buf10, sizeof(buf10));

    usleep(20); // Wait for 20us to allow device to process unlock command

    // Step 2: Read back EE_CNTRL register to verify unlock operation
    tcflush(serial_port, TCIOFLUSH); // Flush UART buffers
    uint8_t buf9[4] = {syncByte, SRR, regAddr, calcChecksum(SRR)};
    sendBytes(serial_port, buf9, sizeof(buf9));
    usleep(5);
    printf("Reading back EE_CNTRL after unlock pattern...\n");
    if (receiveBytesFromSerial(serial_port, tmpRst, 3)) 
    {
        printf("ReadBack Data (Unlock Check):\n");
        for (int i = 0; i < 3; i++) 
        {
            printf("0x%02X ", tmpRst[i]);
        }
        printf("\n");

        if ((tmpRst[1] & 0x08) == 0x08) 
        {
            printf("Unlock successful.\n");
        } 
        else 
        {
            printf("Unlock failed, aborting.\n");
            return false;
        }
    } 
    else 
    {
        printf("Failed to read data after unlock.\n");
        return false;
    }

    usleep(20000); // Short delay before next operation

    // Step 3: Write program bit (0x69) to EE_CNTRL register
    regData = 0x69; // Set program bit
    buf10[3] = regData;
    buf10[4] = calcChecksum(SRW);
    sendBytes(serial_port, buf10, sizeof(buf10));

    usleep(20000); // Wait for 2ms to allow the command to take effect

    // Step 4: Read back EE_CNTRL register to verify programming status
    sendBytes(serial_port, buf9, sizeof(buf9));
    printf("Reading back EE_CNTRL after programming...\n");
    if (receiveBytesFromSerial(serial_port, tmpRst, 3)) 
    {
        printf("ReadBack Data (Programming Check):\n");
        for (int i = 0; i < 3; i++) 
        {
            printf("0x%02X ", tmpRst[i]);
        }
        printf("\n");

        // Check if EE_PGRM_OK bit (bit 2) is set
        burnStat = tmpRst[1];
        if ((burnStat & 0x04) == 0x04) 
        {
            burnSuccess = true;
            printf("EEPROM programming successful.\n");
        } 
        else 
        {
            printf("EEPROM programming failed. Status: 0x%02X\n", burnStat);
        }
    } 
    else 
    {
        printf("Failed to read data after programming.\n");
    }

    return burnSuccess;
}
#endif



#if 1
bool burnEEPROM(int serial_port)
{
	uint8_t burnStat = 0;
	uint8_t temp = 0;
	
	bool burnSuccess = false;
	
	// Write "0xD" to EE_UNLCK to unlock EEPROM, and '0' to EEPRGM bit at EE_CNTRL register
	regAddr = 0x40; //EE_CNTRL
	regData = 0x68;
	uint8_t buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};

	sendBytes(serial_port, buf10, sizeof(buf10));
	
	usleep(1000);  // Wait for 1 msecond before sending data again delay(1);
	
	// Write "0xD" to EE_UNLCK to unlock EEPROM, and '1' to EEPRGM bit at EE_CNTRL register
	regAddr = 0x40; //EE_CNTRL
	regData = 0x01;
	buf10[2] = regAddr;
	buf10[3] = regData;
	buf10[4] = calcChecksum(SRW);
	
	//uint8_t buf11[5] = {syncByte, SRW, regAddr, 0x01, calcChecksum(SRW)}; // This
	sendBytes(serial_port, buf10, sizeof(buf10));
	
	usleep(50); //delay(1000);
		
	// Read back EEPROM program status	
	//pga460SerialFlush(serial_port);
	// Flush UART buffers
	tcflush(serial_port, TCIOFLUSH);
	regAddr = 0x1C; // FREQ //0x40; //EE_CNTRL
	uint8_t buf9[4] = {syncByte, SRR, regAddr, calcChecksum(SRR)};

	sendBytes(serial_port, buf9, sizeof(buf9));
	printf("EEPROM Sent Data:\n");
	for (int i = 0; i < 4; i++) 
	{
		printf("0x%02X ", buf9[i]);
	}

	usleep(1000); //delay(10);
	
	if (receiveBytesFromSerial(serial_port, tmpRst, 3)) 
    {
        printf("EEPROM ReadBack Data:\n");
        for (int i = 0; i < 3; i++) 
        {
            printf("0x%02X ", tmpRst[i]);
        }
        printf("\n");
    } 
    else 
    {
        printf("Failed to read data\n");
    }
	
#if 0	
	//Test
	usleep(100000); //delay(10);
	uint8_t buf29[4] = {syncByte, EEBR, regAddr, calcChecksum(EEBR)};
	sendBytes(serial_port, buf29, sizeof(buf29));
	usleep(20000); //delay(10);
	
	if (receiveBytesFromSerial(serial_port, tmpRst, 3)) 
    {
        printf("EEPROM ReadBack Data Try 2:\n");
        for (int i = 0; i < 3; i++) 
        {
            printf("0x%02X ", tmpRst[i]);
        }
        printf("\n");
    } 
    else 
    {
        printf("Failed to read data\n");
    }
#endif	
	
	burnStat = tmpRst[1];
	
	if((burnStat & 0x04) == 0x04){burnSuccess = true;} // check if EE_PGRM_OK bit is '1'
	
	return burnSuccess;
}
#endif
void readReg(int serial_port, int REG_ADDRR)
{
	regAddr = REG_ADDRR; 
	
	uint8_t buf9[4] = {syncByte, SRR, regAddr, calcChecksum(SRR)};
	
	sendBytes(serial_port, buf9, sizeof(buf9));
	//printf("Sent Data:\n");
	//for (int i = 0; i < 4; i++) 
	//{
	//	printf("0x%02X ", buf9[i]);
	//}
	//printf("\n");

	usleep(1000); //delay(10);
	
	if (receiveBytesFromSerial(serial_port, regRead, 3)) 
    {
        printf("Reg ReadBack Data:\n");
        for (int i = 0; i < 3; i++) 
        {
            printf("0x%02X ", regRead[i]);
        }
        printf("\n");
    } 
    else 
    {
        printf("Failed to read data\n");
    }
	
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
void ultrasonicCmd(uint8_t cmd, uint8_t numObjUpdate, int serial_port)
{	
	numObj = numObjUpdate; // number of objects to detect
	uint8_t bufCmd[4] = {syncByte, 0xFF, numObj, 0xFF}; // prepare bufCmd with 0xFF placeholders
		
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
	sendBytes(serial_port, bufCmd, sizeof(bufCmd)); // serial transmit master data to initiate burst and/or listen command

	usleep(70000);  // Wait for 10 milliseconds :: delay(70); // maximum record length is 65ms, so delay with margin
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
uint8_t pullUltrasonicMeasResult(bool busDemo, int serial_port)
{

	//pga460SerialFlush(serial_port);
	// Flush UART buffers
    tcflush(serial_port, TCIOFLUSH);
	//memset(ultraMeasResult, 0, sizeof(ultraMeasResult));
		
	uint8_t buf5[3] = {syncByte, UMR, calcChecksum(UMR)};

	sendBytes(serial_port, buf5, sizeof(buf5)); //serial transmit master data to read ultrasonic measurement results
	
	receiveBytesFromSerial(serial_port, ultraMeasResult, (2+(numObj*4)));
#if 0		
	starttime = millis();
	while ( (Serial1.available()<(5)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) )
	{      
		// wait in this loop until we either get +5 bytes of data, or 0.25 seconds have gone by
	}
			
	if((Serial1.available() < (5)))
	{
		if (busDemo == false)
		{
			// the data didn't come in - handle the problem here
			Serial.println("ERROR - Did not receive measurement results!");
		}
		return false;
	}
	else
	{
		for(int n=0; n<((2+(numObj*4))); n++)
		{			
			ultraMeasResult[n] = Serial1.read();
			delay(1);		   
		}			
	}
#endif
	
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
double printUltrasonicMeasResult(uint8_t umr)
{
	int speedSound = 343; // speed of sound in air at room temperature
	uint16_t objDist = 0;
	double objReturn = 0;
	double tmp_spd = 2*0.000001*343;
	
	objDist = (ultraMeasResult[1]<<8) + ultraMeasResult[2];
	
	objReturn = (objDist/tmp_spd);
	return objReturn;
	//printUltrasonicMeasResultExt(umr, speedSound);
}
uint8_t printUltrasonicMeasResultRaw(uint8_t umr)
{
	return ultraMeasResult[umr];
}
double printUltrasonicMeasResultExt(uint8_t umr, int speedSound)
{
	double objReturn = 0;
	double digitalDelay = 0; // TODO: compensates the burst time calculated as number_of_pulses/frequency.
	uint16_t objDist = 0;
	uint16_t objWidth = 0;
	uint16_t objAmp = 0;
	double tmp_spd = 2*0.000001*speedSound;
	
	switch (umr)
	{
		case 0: //Obj1 Distance (m)
		{
			objDist = (ultraMeasResult[1]<<8) + ultraMeasResult[2];
			if(tmp_spd != 0)
			{
				objReturn = (objDist/tmp_spd) - digitalDelay;
			}
			else
			{
				printf("ERROR - Invalid object result!");printf("\n");
				objReturn = 0xFFFF;
			}
			
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
		default:
		{
			printf("ERROR - Invalid object result!");printf("\n"); 
			break;
		}
	}	
	return objReturn;
}

/*------------------------------------------------- runEchoDataDump -----
 |  Function runEchoDataDump
 |
 |  Purpose:  Runs a preset 1 or 2 burst and or listen command to capture 128 bytes of echo data dump.
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
void runEchoDataDump(uint8_t preset, int serial_port)
{
	// enable Echo Data Dump bit
	regAddr = 0x40;
	regData = 0x80;
	uint8_t writeType = SRW; // default to single address register write (cmd10)
	if(preset>16) // update to broadcast register write if broadcast TOF preset command given
	{
		writeType = BC_RW; // cmd22
	}
	
	uint8_t buf10[5] = {syncByte, writeType, regAddr, regData, calcChecksum(writeType)};

	tcflush(serial_port, TCIOFLUSH);
	sendBytes(serial_port, buf10, sizeof(buf10));
	usleep(10000);
	
	
	// run preset 1 or 2 burst and or listen command
	ultrasonicCmd(preset, 1, serial_port);	

	// disbale Echo Data Dump bit
	regData = 0x00;
	buf10[3] = regData;
	buf10[4] = calcChecksum(writeType);

	sendBytes(serial_port, buf10, sizeof(buf10));
	return;
}

/*------------------------------------------------- pullEchoDataDump -----
 |  Function pullEchoDataDump
 |
 |  Purpose:  Read out 128 bytes of echo data dump (EDD) from latest burst and or listen command. 
 |		For UART and OWU, readout individual echo data dump register values, instead in bulk.
 |		For TCI, perform index 12 read of all echo data dump values in bulk.
 |
 |  Parameters:
 |		element (IN) -- element from the 128 byte EDD memory
 |
 |  Returns:  byte representation of EDD element value
 *-------------------------------------------------------------------*/
uint8_t pullEchoDataDump(uint8_t element, int serial_port)
{
	if (element == 0)
	{
		uint8_t temp = 0;
		tcflush(serial_port, TCIOFLUSH);			
		
		regAddr = 0x80; // start of EDD memory
		uint8_t buf9[4] = {syncByte, SRR, regAddr, calcChecksum(SRR)}; 
		sendBytes(serial_port, buf9, sizeof(buf9)); // read first byte of EDD memory						
		for(int m=0; m<129; m++) // loop readout by iterating through EDD address range
		{
			buf9[2] = regAddr;
			buf9[3] = calcChecksum(SRR);
			sendBytes(serial_port, buf9, sizeof(buf9));
			//delay(30);
			usleep(30000);
			for (int n = 0; n < 128; n++)
			{
				if (n == 1)
				{
					echoDataDump[m] = serialGetchar(serial_port); // Store the second byte in echoDataDump[m]
				}
				else
				{
					temp = serialGetchar(serial_port); // Read and discard other bytes
				}
			}
			regAddr++;
		}			
	}
	return echoDataDump[element];
}


void pga460SerialFlush(int serial_port) 
{
    // Initial delay
    usleep(10000); // 10 milliseconds

    // Flush serial_port
    serialFlush(serial_port);

    // Clear incoming data on serial_port
    while (serialDataAvail(serial_port) > 0) 
	{
        char temp = serialGetchar(serial_port);
    }

    // Final flush serial_port
    serialFlush(serial_port);
}

/*------------------------------------------------- pullEchoDataDumpBulk -----
 |  Function pullEchoDataDumpBulk
 |
 |  Purpose:  Bulk read out 128 bytes of echo data dump (EDD) from latest burst and or listen command. 
 |		For UART and OWU, readout bulk echo data dump register values.
 |		For TCI, perform index 12 read of all echo data dump values in bulk.
 |
 |  Parameters:
 |		none
 |
 |  Returns:  comma delimited string of all EDD values
 *-------------------------------------------------------------------*/
bool pullEchoDataDumpBulk(int serial_port)
{
	bool echoDataDump = false; // String to hold the result
	
	uint8_t temp = 0;
	tcflush(serial_port, TCIOFLUSH);			
		
	uint8_t buf7[3] = {syncByte, TEDD, calcChecksum(TEDD)};  
	sendBytes(serial_port, buf7, sizeof(buf7));	// respond bulk EDD command
	uint8_t eddBulk[130];
	
	// Read 130 bytes into eddBulk
    if (receiveBytesFromSerial(serial_port, eddBulk, 130)) 
    {
        printf("Rxd Data:\n");
        for (int i = 0; i < 130; i++) 
        {
            printf("%d ", eddBulk[i]);
        }
        printf("\n");
		
    } 
    else 
    {
        printf("Failed to read data\n");
		
    }
	
	
	if(eddBulk[0] != 0) // if diagnostic field is non-zero
	{			
		echoDataDump = true;
	}
	else
	{
		echoDataDump = false;
		// the data didn't come in - handle the problem here
		printf("ERROR - Did not receive echo data dump! ");			
	}
	return echoDataDump;	
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
 |  Returns: byte representation of calculated checksum value
 *-------------------------------------------------------------------*/
uint8_t calcChecksum(uint8_t cmd)
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

/*
	This function is to perform UART loop-back test
*/

void uartLoopBackTest(int serial_port) 
{
    uint8_t data_to_write = 50;
    uint8_t data_read;

    // Flush UART buffers
    tcflush(serial_port, TCIOFLUSH);

    // Write a single byte to the UART
    ssize_t bytes_written = write(serial_port, &data_to_write, 1);
    if (bytes_written != 1) {
        perror("Failed to write to UART");
        return;
    }

    usleep(20000); // 20ms delay for data to loop back

    printf("Write: %d\n", data_to_write);

    // Read data from the UART
    ssize_t bytes_read = read(serial_port, &data_read, 1);
    if (bytes_read < 0) {
        perror("Error reading from UART");
        return;
    } else if (bytes_read == 0) {
        printf("No data available to read.\n");
    } else {
        printf("Read: %d\n", data_read);
    }

    printf("\n");
    usleep(100000); // 100ms delay
}











