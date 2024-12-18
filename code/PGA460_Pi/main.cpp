#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  // Include this header for strerror

#define UART_DEVICE "/dev/ttyAMA5"  // Default UART device on Raspberry Pi 4
#define BAUD_RATE 115200              // Set baud rate to match PGA450
#define UART_RX_PIN 13              // GPIO15 corresponds to UART RX

// Pin definitions
#define UART_SEL0 22
#define UART_SEL1 17
#define ULTRASONIC_PWR_EN 6 	//Rev 1 and below is GPIO6, Rev 2 and up boards is GPIO21

#define SELECT_SENSOR_1() \
    digitalWrite(UART_SEL1, LOW); \
    digitalWrite(UART_SEL0, LOW)

#define SELECT_SENSOR_2() \
    digitalWrite(UART_SEL1, HIGH); \
    digitalWrite(UART_SEL0, LOW)


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

unsigned char buf6[3] = {0x55, 0x06, 0xF9};
unsigned char buf_t1[4] = {0x55, 0x09, 0x4C, 0xAA};

void sendBytes(int fd, unsigned char *data, int length) 
{
	printf("Tx data: ");
    for (int i = 0; i < length; i++) {
        serialPutchar(fd, data[i]);
		printf("0x%02X ", data[i]);
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
	

    // Initialize WiringPi and UART
    if (wiringPiSetup() == -1) 
	{
        fprintf(stderr, "Failed to initialize WiringPi\n");
        return 1;
    }
    pullUpDnControl(UART_RX_PIN, PUD_UP);

    pinMode(ULTRASONIC_PWR_EN, OUTPUT);
    pinMode(UART_SEL0, OUTPUT);
    pinMode(UART_SEL1, OUTPUT);
    usleep(100);

    // Enable power to ultrasonic sensors
    digitalWrite(ULTRASONIC_PWR_EN, HIGH);
    SELECT_SENSOR_1();

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

        // Send 4 bytes
		// broadcast p1 burst+listen (non-dependent on UART_ADDR)
#if 1		
		sendBytes(fd, buf17, sizeof(buf17));
		usleep(2000);  // Wait for 10 milliseconds
		

        // Wait for data to be available (this is an example, you might want to implement a better waiting mechanism)
        //usleep(10000);  // Wait for 10 milliseconds
		// read back ultrasonic meas results from UART_ADDR=0
		sendBytes(fd, buf5, sizeof(buf5));
#endif
		//sendBytes(fd, buf6, sizeof(buf6));
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


        // Sleep for a while before the next iteration
        usleep(1000000);  // Wait for 1 second before sending data again
    }

    // Disable power to ultrasonic sensors
	digitalWrite(ULTRASONIC_PWR_EN, LOW);

    // Close the UART connection (this part will never be reached in the endless loop)
    serialClose(fd);

    return 0;
}


/********************/



#if 0

int main() {
    int serial_port;
    unsigned char diagMeasResult[4]; // Array to store diagnostic measurements
    int owuShift = 0; // Example value for owuShift
    int owuShiftSysDiag = 0; // Example value for owuShiftSysDiag

    // Initialize wiringPi library
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Unable to start wiringPi: %s\n", strerror(errno));
        return 1;
    }

    // Open Serial1 port (adjust path and baud rate as needed)
    serial_port = serialOpen("/dev/ttyS0", 9600);
    if (serial_port < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    // Call the function to receive system diagnostics
    if (receiveBytesFromSerial(serial_port, diagMeasResult, 4 + owuShift - owuShiftSysDiag)) {
        // Print received data for verification
        printf("Received system diagnostics: ");
        for (int i = 0; i < (4 + owuShift - owuShiftSysDiag); i++) {
            printf("%d ", diagMeasResult[i]);
        }
        printf("\n");
    }

    // Close the serial port
    serialClose(serial_port);

    return 0;
}
#endif



