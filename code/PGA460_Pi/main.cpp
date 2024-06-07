#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  // Include this header for strerror

#define UART_DEVICE "/dev/ttyACM0"  // Default UART device on Raspberry Pi 4
#define BAUD_RATE 19200              // Set baud rate to match PGA450
#define UART_RX_PIN 16//10              // GPIO15 corresponds to UART RX
unsigned char UART_CMD1_S[5] = {0x00, 0x55, 0x11, 0x01, 0x00};			// Command 1: To initiate/triggere short burst (Uses Fixed Register settings stored in FIFO)
unsigned char UART_CMD1_L[5] = {0x00, 0x55, 0x11, 0x02, 0x00};			// Command 1: To initiate/triggere long burst  (Uses Fixed Register settings stored in FIFO)
unsigned char UART_CMD2[4] = {0x00, 0x55, 0x21, 0x00};					// Command 2: Used to read TOF (Once short/long burst is initiated, Cmd2 is used to read TOF)
unsigned char RX_DATA[2] = {0};

//note: bare bones raw UART demo used to send a burst/listen command to PGA460

//cmd 0 - p1 burst listen
unsigned char buf0[4] = {0x55, 0x00, 0x01, 0xFE};
//cmd 1 - p2 burst listen
unsigned char buf1[4] = {0x55, 0x01, 0x01, 0xFD};
//cmd 5 - ultrasonic measurement (assume UART_ADDR=0)
unsigned char buf5[3] = {0x00, 0x00, 0x00};//{0x55, 0x05, 0xFA};
//cmd 10 - register write decple to time of 4.096ms
unsigned char buf10[5] = {0x55, 0x0A, 0x26, 0x00, 0xCF};
//cmd 17 - broadcast p1 burst listen
unsigned char buf17[4] = {0x55, 0x11, 0x01, 0xED};
//cmd 19 - broadcast p1 listen only
unsigned char buf19[4] = {0x55, 0x13, 0x01, 0xEB};
//cmd 25 - broadcast bulk threshold write
unsigned char buf25[35] =  {0x55, 0x19, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00, 0x7C};


void sendBytes(int fd, unsigned char *data, int length) {
	printf("Tx data: ");
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
    unsigned char rxData[8];  // Buffer to store received data
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
	sendBytes(fd, buf25, 35);
	usleep(1000);  // Wait for 100 msecond before sending data again
	
	// set UART_ADDR=0's time decouple to 4.096ms
	sendBytes(fd, buf10, 5);
	usleep(1000);  // Wait for 100 msecond before sending data again
	delay(100);

    // Endless loop to send and receive data
    while (1) 
	{
//		SesnorMeasurement(fd);
#if 1
        // Send 4 bytes
		// broadcast p1 burst+listen (non-dependent on UART_ADDR)
		sendBytes(fd, buf17, 4);

        // Wait for data to be available (this is an example, you might want to implement a better waiting mechanism)
        usleep(100000);  // Wait for 100 milliseconds
		// read back ultrasonic meas results from UART_ADDR=0
		sendBytes(fd, buf5, sizeof(buf5));

        // Receive data (up to 256 bytes in this example)
        receivedLength = receiveBytes(fd, rxData, sizeof(rxData));
		printf("Rx Length: "); printf(receivedLength);printf(" : ");

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




/* ##############################################  */
#if 0
const int buttonPin = PUSH2;     // the number of the pushbutton pin
const int ledPin = RED_LED;      // the number of the LED pin
int buttonState = 0;         // variable for reading the pushbutton status

void setup() { 

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  delay(1000);
  
  // put your setup code here, to run once:
  Serial1.begin(19200);  // initialize PGA460 UART serial channel
  delay(1000);

  //assume UART_ADDR=0
  //bulk threshold write mid-code values to clear THR_CRC_ERR
  Serial1.write(buf25, sizeof(buf25));
  delay(100);

  // [TODO] set p1 rec length to 4.096ms
  
  // set UART_ADDR=0's time decouple to 4.096ms
  Serial1.write(buf10, sizeof(buf10));
  delay(100);
}

void loop() {
    // put your main code here, to run repeatedly: 
  
    // check if the pushbutton is pressed.
    while (digitalRead(buttonPin) == HIGH){}
    
    // broadcast p1 burst+listen (non-dependent on UART_ADDR)
    Serial1.write(buf17, sizeof(buf17));
    
    // delay by 100ms
    delay(100);

    //[TODO] print ultrasonic measurement results on terminal
    // read back ultrasonic meas results from UART_ADDR=0
    Serial1.write(buf5, sizeof(buf5));
  
    // toggle red LED
    digitalWrite(ledPin, !(digitalRead(ledPin)));   // turn the LED on (HIGH is the voltage level)

    // repeat loop every second
    delay (1000);      
}
#endif


