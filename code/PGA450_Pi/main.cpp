#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  // Include this header for strerror

#define UART_DEVICE "/dev/ttyS0"  // Default UART device on Raspberry Pi 4
#define BAUD_RATE 19200              // Set baud rate to match PGA450
#define UART_RX_PIN 10              // GPIO15 corresponds to UART RX
unsigned char UART_CMD1_S[5] = {0x00, 0x55, 0x11, 0x01, 0x00};			// Command 1: To initiate/triggere short burst (Uses Fixed Register settings stored in FIFO)
unsigned char UART_CMD1_L[5] = {0x00, 0x55, 0x11, 0x02, 0x00};			// Command 1: To initiate/triggere long burst  (Uses Fixed Register settings stored in FIFO)
unsigned char UART_CMD2[4] = {0x00, 0x55, 0x21, 0x00};					// Command 2: Used to read TOF (Once short/long burst is initiated, Cmd2 is used to read TOF)
unsigned char RX_DATA[2] = {0};

void sendBytes(int fd, unsigned char *data, int length) {
    for (int i = 0; i < length; i++) {
        serialPutchar(fd, data[i]);
		printf("0x%02X ", data[i]);
    }
}

int receiveBytes(int fd, unsigned char *buffer, int maxLength) {
    int received = 0;

    while (serialDataAvail(fd) > 0 && received < maxLength) {
        buffer[received++] = serialGetchar(fd);
        usleep(1000);  // Small delay to allow data to be received
    }

    return received;
}

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

int main() {
    int fd;
    unsigned char txData[4] = {0x00, 0x55, 0x01, 0x00}; // Example data to send
    unsigned char rxData[4];  // Buffer to store received data
    int receivedLength;
	// Initialize WiringPi and GPIO
	wiringPiSetup();  // Use WiringPi's own pin numbering
	//wiringPiSetupGpio();  // Use BCM GPIO numbering
	//wiringPiSetupPhys();  // Use the physical pin numbers on the P1 connector

    wiringPiSetupGpio();
	//pinMode(UART_RX_PIN, INPUT);
	//pullUpDnControl(UART_RX_PIN, PUD_UP);

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

    // Endless loop to send and receive data
    while (1) 
	{
		SesnorMeasurement(fd);
#if 0
        // Send 4 bytes
        sendBytes(fd, txData, 4);

        // Wait for data to be available (this is an example, you might want to implement a better waiting mechanism)
        usleep(5000);  // Wait for 5 milliseconds

        // Receive data (up to 256 bytes in this example)
        receivedLength = receiveBytes(fd, rxData, sizeof(rxData));

        // Print received data in hexadecimal format
        if (receivedLength > 0) 
		{
            printf("Received data: ");
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


