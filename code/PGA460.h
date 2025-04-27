#ifndef PGA460_H
#define PGA460_H
#include <stdint.h>  // Include this header for uint8_t and uint32_t

#define MAX_MILLIS_TO_WAIT 250 // Maximum time to wait in milliseconds
#define UART_DEVICE "/dev/ttyAMA2"  		 // Default UART device on Raspberry Pi 4
#define BAUD_RATE 115200              	 // // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200 
#define UART_RX_PIN 28              	 // GPIO15 corresponds to UART RX



void sendBytes(int fd, unsigned char *data, int length);
int receiveBytes(int fd, unsigned char *buffer, int maxLength);
void initBoostXLPGA460(uint8_t mode, uint32_t baud, uint8_t uartAddrUpdate);
void initThresholds(uint8_t thr, int serial_port);
void defaultPGA460(uint8_t xdcr, int serial_port);
void initTVG(uint8_t agr, uint8_t tvg, int serial_port);
double runDiagnostics(uint8_t run, uint8_t diag, int serial_port);
bool burnEEPROM(int serial_port);
void ultrasonicCmd(uint8_t cmd, uint8_t numObjUpdate, int serial_port);
uint8_t pullUltrasonicMeasResult(bool busDemo, int serial_port);
double printUltrasonicMeasResult(uint8_t umr);
uint8_t printUltrasonicMeasResultRaw(uint8_t umr);
double printUltrasonicMeasResultExt(uint8_t umr, int speedSound);
bool receiveBytesFromSerial(int serial_port, unsigned char* buffer, int numBytesToReceive);
uint8_t calcChecksum(uint8_t cmd);
void pga460SerialFlush(int serial_port);
void initVariables();
void runEchoDataDump(uint8_t preset, int serial_port);
uint8_t pullEchoDataDump(uint8_t element, int serial_port);
char* pullEchoDataDumpBulk(int serial_port);


#endif