#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  // Include this header for strerror

#define UART_DEVICE "/dev/ttyS0"  // Default UART device on Raspberry Pi 4
#define BAUD_RATE 115200              // Set baud rate to match PGA450
#define UART_RX_PIN 15              // GPIO15 corresponds to UART RX
unsigned char RX_DATA[2] = {0};

//note: bare bones raw UART demo used to send a burst/listen command to PGA460

//cmd 0 - p1 burst listen
unsigned char buf0[4] = {0x55, 0x00, 0x01, 0xFE};
//cmd 1 - p2 burst listen
unsigned char buf1[4] = {0x55, 0x01, 0x01, 0xFD};
//cmd 5 - ultrasonic measurement (assume UART_ADDR=0)
unsigned char buf5[3] = {0x55, 0x05, 0xFA};
//cmd 10 - register write decple to time of 4.096ms
unsigned char buf10[5] = {0x55, 0x0A, 0x26, 0x00, 0xCF};
//cmd 17 - broadcast p1 burst listen
unsigned char buf17[4] = {0x55, 0x11, 0x01, 0xED};
//cmd 19 - broadcast p1 listen only
unsigned char buf19[4] = {0x55, 0x13, 0x01, 0xEB};
//cmd 25 - broadcast bulk threshold write
unsigned char buf25[35] =  {0x55, 0x19, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00, 0x7C};

unsigned char buf6[3] = {0x55, 0x06};//{0x55, 0x06, 0xF9};

void sendBytes(int fd, unsigned char *data, int length) {
	printf("Tx data: ");
    for (int i = 0; i < length; i++) {
        serialPutchar(fd, data[i]);
		printf("0x%02X ", data[i]);
    }
}

int receiveBytes(int fd, unsigned char *buffer, int maxLength) {
    int received = 0;

    while (serialDataAvail(fd) > 0 && received < maxLength) 
	{
        buffer[received++] = serialGetchar(fd);
        usleep(1000);  // Small delay to allow data to be received
    }

    return received;
}
#if 0
void SesnorMeasurement(int fd)
{
	int rx_length;
	sendBytes(fd, UART_CMD1_S, 5); 			// Short burst
	usleep(1000);  							// Small delay to allow data to be received
	sendBytes(fd, UART_CMD2, 4); 			// To read data
	rx_length = receiveBytes(fd, RX_DATA, 2);		
	if (rx_length > 0) 
	{
		printf("Received data Short: ");
		for (int i = 0; i < rx_length; i++) 
		{
			printf("0x%02X ", RX_DATA[i]);
		}
		printf("\n");
	} 
	else 
	{
		printf("No data received\n");
	}
	
	usleep(500); 

	sendBytes(fd, UART_CMD1_L, 5); 			// Short burst
	usleep(1000);  							// Small delay to allow data to be received
	sendBytes(fd, UART_CMD2, 4); 			// To read data
	rx_length = receiveBytes(fd, RX_DATA, 2);
	
	if (rx_length > 0) 
	{
		printf("Received data Long: ");
		for (int i = 0; i < rx_length; i++) 
		{
			printf("0x%02X ", RX_DATA[i]);
		}
		printf("\n");
	} 
	else 
	{
		printf("No data received\n");
	}

}
#endif

int main() {
    int fd;
    unsigned char txData[4] = {0x00, 0x55, 0x01, 0x00}; // Example data to send
    unsigned char rxData[6] = {0x00};  // Buffer to store received data
    unsigned char receivedLength;
	// Initialize WiringPi and GPIO
	wiringPiSetup();  // Use WiringPi's own pin numbering
	//wiringPiSetupGpio();  // Use BCM GPIO numbering
	//wiringPiSetupPhys();  // Use the physical pin numbers on the P1 connector

    wiringPiSetupGpio();
	//pinMode(UART_RX_PIN, INPUT);
	pullUpDnControl(UART_RX_PIN, PUD_UP);

    // Initialize WiringPi and UART
    if (wiringPiSetup() == -1) 
	{
        fprintf(stderr, "Failed to initialize WiringPi\n");
        return 1;
    }

    if ((fd = serialOpen(UART_DEVICE, BAUD_RATE)) < 0) 
	{
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }
	
	//
	usleep(1000000);  // Wait for 1 second before sending data again
	//sendBytes(fd, buf25, 35);
	usleep(1000);  // Wait for 100 msecond before sending data again
	
	// set UART_ADDR=0's time decouple to 4.096ms
	//sendBytes(fd, buf10, 5);
	usleep(1000);  // Wait for 100 msecond before sending data again
	delay(100);

    // Endless loop to send and receive data
    while (1) 
	{
//		SesnorMeasurement(fd);
#if 1
        // Send 4 bytes
		// broadcast p1 burst+listen (non-dependent on UART_ADDR)
		
		sendBytes(fd, buf17, sizeof(buf17));
		usleep(10000);  // Wait for 10 milliseconds
		//sendBytes(fd, buf6, 3);

        // Wait for data to be available (this is an example, you might want to implement a better waiting mechanism)
        //usleep(10000);  // Wait for 10 milliseconds
		// read back ultrasonic meas results from UART_ADDR=0
		sendBytes(fd, buf6, sizeof(buf6));

        // Receive data (up to 256 bytes in this example)
        receivedLength = receiveBytes(fd, rxData, sizeof(rxData));

        // Print received data in hexadecimal format
        if (receivedLength > 0) 
		{
            printf("Rx data: ");
            for (int i = 0; i < receivedLength; i++) 
			{
                printf("0x%02X ", rxData[i]);
            }
            printf("\n");
        } 
		else 
		{
            printf("No data received\n");
        }
#endif

        // Sleep for a while before the next iteration
        usleep(1000000);  // Wait for 1 second before sending data again
    }

    // Close the UART connection (this part will never be reached in the endless loop)
    serialClose(fd);

    return 0;
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
#if 0
unsigned char calcChecksum(unsigned char cmd)
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
#endif
