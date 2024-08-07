#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  					 // Include this header for strerror
#include <stdint.h>  					 // Include this header for uint8_t and uint32_t

#include "PGA460.h"

uint8_t commMode = 0;            // Communication mode: 0=UART, 1=TCI, 2=OneWireUART
uint8_t fixedThr = 0;            // set P1 and P2 thresholds to 0=%25, 1=50%, or 2=75% of max; initial minDistLim (i.e. 20cm) ignored
uint8_t xdcr = 1;                // set PGA460 to recommended settings for 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R
uint8_t agrTVG = 2;              // set TVG's analog front end gain range to 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB
uint8_t fixedTVG = 1;            // set fixed TVG level at 0=%25, 1=50%, or 1=75% of max
uint8_t runDiag = 1;             // run system diagnostics and temp/noise level before looping burst+listen command
uint8_t edd = 1;                 // echo data dump of preset 1, 2, or neither TODO: Import this Fn.
uint8_t burn = 1;                // trigger EE_CNTRL to burn and program user EEPROM memory
uint8_t cdMultiplier = 1;        // multiplier for command cycle delay
uint8_t numOfObj = 4;            // number of object to detect set to 1-8


uint8_t uartAddrUpdate = 0;      // PGA460 UART address to interface to; default is 0, possible address 0-7
bool objectDetected = false;  		 // object detected flag to break burst+listen cycle when true
bool demoMode = false;        		 // only true when running UART/OWU multi device demo mode
bool alwaysLong = true;      		 // always run preset 2, regardless of preset 1 result (hard-coded only)
double minDistLim = 0.1;      		 // minimum distance as limited by ringing decay of single transducer and threshold masking
uint16_t commandDelay = 0;    		 // Delay between each P1 and Preset 2 command


// Result variables
double distance = 0;          	// one-way object distance in meters
double width = 0;             	// object width in microseconds
double peak = 0;              	// object peak in 8-bit
double diagnostics = 0;       	// diagnostic selector
int Serial_Port;

bool stringComplete = false;	// Global flag to indicate if the string is complete

  
  
  /*------------------------------------------------- initPGA460 -----
|  function initPGA460
|
|  Purpose: One-time setup of PGA460-Q1 EVM hardware and software 
|      in the following steps: 
|    1) Configure the master to operate in UART, TCI, or OWU 
|      communication mode.
|    2) Confgiure the EVM for compatiblity based on the selected 
|      communicaton mode.
|    3) Option to update user EEPROM and threhsold registers with 
|      pre-defined values.
|    4) Option to burn the EEPROM settings (not required unless 
|      values are to be preserved after power cycling device).
|    5) Option to report echo data dump and/or system diagnostics.
|  
|  In userInput mode, the user is prompted to enter values through 
|   the Serial COM terminal to configure the device.
|
|  In standAlone mode, the user must hard-code the configuration 
|   variables in the globals section for the device to 
|   auto-configure in the background.
*-------------------------------------------------------------------*/
void initPGA460() 
{
	
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
    }

    if ((Serial_Port = serialOpen(UART_DEVICE, BAUD_RATE)) < 0) 
	{
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    }
	
	initVariables();


/*------------------------------------------------- userInput & standAlone mode initialization -----
  Configure the EVM in the following order:
  1) Select PGA460 interface, device baud, and COM terminal baud up to 115.2k for targeted address.
  2) Bulk write all threshold values to clear the THR_CRC_ERR.
  3) Bulk write user EEPROM with pre-define values in PGA460_USSC.c. 
  4) Update analog front end gain range, and bulk write TVG.
  5) Run system diagnostics for frequency, decay, temperature, and noise measurements
  6) Program (burn) EEPROM memory to save user EEPROM values
  7) Run a preset 1 or 2 burst and/or listen command to capture the echo data dump  
  
  if the input is 'x' (72d), then skip that configuration
*-------------------------------------------------------------------*/
  // -+-+-+-+-+-+-+-+-+-+- 1 : interface setup   -+-+-+-+-+-+-+-+-+-+- //
    initBoostXLPGA460(commMode, BAUD_RATE, uartAddrUpdate); 
    
  // -+-+-+-+-+-+-+-+-+-+- 2 : bulk threshold write   -+-+-+-+-+-+-+-+-+-+- //
    if (fixedThr != 72){initThresholds(fixedThr, Serial_Port);} 
  // -+-+-+-+-+-+-+-+-+-+- 3 : bulk user EEPROM write   -+-+-+-+-+-+-+-+-+-+- //
    if (xdcr != 72){defaultPGA460(xdcr, Serial_Port);}
  // -+-+-+-+-+-+-+-+-+-+- 4 : bulk TVG write   -+-+-+-+-+-+-+-+-+-+- //
    if (agrTVG != 72 && fixedTVG != 72){initTVG(agrTVG,fixedTVG, Serial_Port);}
  // -+-+-+-+-+-+-+-+-+-+- 5 : run system diagnostics   -+-+-+-+-+-+-+-+-+-+- //
    if (runDiag == 1)
    {      
      diagnostics = runDiagnostics(1,0, Serial_Port);       // run and capture system diagnostics, and print freq diag result
      printf("System Diagnostics - Frequency (kHz): %f\n", diagnostics);
      diagnostics = runDiagnostics(0,1, Serial_Port);       // do not re-run system diagnostic, but print decay diag result
      printf("System Diagnostics - Decay Period (us): %f\n", diagnostics);
      diagnostics = runDiagnostics(0,2, Serial_Port);       // do not re-run system diagnostic, but print temperature measurement
      printf("System Diagnostics - Die Temperature (C): %f\n", diagnostics);
      diagnostics = runDiagnostics(0,3, Serial_Port);       // do not re-run system diagnostic, but print noise level measurement
	  printf("System Diagnostics - Noise Level: %f\n", diagnostics);
    }
  // -+-+-+-+-+-+-+-+-+-+- 6 : burn EEPROM   -+-+-+-+-+-+-+-+-+-+- //
    if(burn == 1)
    {
      unsigned char burnStat = burnEEPROM(Serial_Port); // TODO
      if(burnStat == true){printf("EEPROM programmed successfully.");}
      else{printf("EEPROM program failed.");}
    }
  // -+-+-+-+-+-+-+-+-+-+- 7 : capture echo data dump   -+-+-+-+-+-+-+-+-+-+- //	
    if (edd != 0)                                   // run or skip echo data dump
    {
      printf("Retrieving echo data dump profile. Wait...");
      runEchoDataDump(edd-1, Serial_Port);                  // run preset 1 or 2 burst and/or listen command
      printf(pullEchoDataDumpBulk(Serial_Port));
      printf("");
    }

  // -+-+-+-+-+-+-+-+-+-+-  others   -+-+-+-+-+-+-+-+-+-+- //
  commandDelay = 100 * cdMultiplier;                   // command cycle delay result in ms
  if (numOfObj == 0 || numOfObj >8) { numOfObj = 1; } // sets number of objects to detect to 1 if invalid input                      

}

/*------------------------------------------------- main loop -----
|  main loop  GetDistance
|
|   The PGA460 is initiated with a Preset 1 Burst-and-Listen 
|     Time-of-Flight measurement. Preset 1 is ideally configured for 
|     short-range measurements (sub-1m range) when using the pre-defined 
|     user EEPROM configurations.
|
|   If no object is detected, the PGA460 will then be issued a 
|     Preset 2 Burst-and-Listen Time-of-Flight measurement. 
|     Preset 2 is configured for long-range measurements (beyond 
|     1m range). 
|
|   Depending on the resulting distance, the diagnostics LEDs will 
|     illuminate to represent a short, mid, or long range value.
|   
|   In userInput mode, the distance, width, and/or amplitude value 
|     of each object is serial printed on the COM terminal.
|
|   In standAlone mode, only distance can be represented visually 
|     on the LEDs. The resulting values are still serial printed 
|     on a COM terminal for debug, and to view the numerical values 
|     of the data captured.
|
*-------------------------------------------------------------------*/

void Cyclic_Task() 
{                 // put your main code here, to run repeatedly
	
	
    // -+-+-+-+-+-+-+-+-+-+-  PRESET 1 (SHORT RANGE) MEASUREMENT   -+-+-+-+-+-+-+-+-+-+- //
      objectDetected = false;                       // Initialize object detected flag to false
      ultrasonicCmd(0, numOfObj, Serial_Port);               // run preset 1 (short distance) burst+listen for 1 object
      pullUltrasonicMeasResult(demoMode, Serial_Port);      // Pull Ultrasonic Measurement Result
      for (uint8_t i=0; i<numOfObj; i++)
      { 
        // Log uUltrasonic Measurement Result: Obj1: 0=Distance(m), 1=Width, 2=Amplitude; Obj2: 3=Distance(m), 4=Width, 5=Amplitude; etc.;
          distance = printUltrasonicMeasResult(0+(i*3));      
          //width = ussc.printUltrasonicMeasResult(1+(i*3));  // only available for UART, OWU, and SPI
          //peak = ussc.printUltrasonicMeasResult(2+(i*3));   // only available for UART, OWU, and SPI
  
        usleep(commandDelay*10);  // Wait for 100 msecond before sending data again delay(commandDelay);
    
        if (distance > minDistLim && distance < 11.2)  // turn on DS1_LED if object is above minDistLim
        {
            printf("P1 Obj %d Distance (m): %.2f\n", i + 1, distance);

            //Serial.print("P1 Obj"); Serial.print(i+1); Serial.print(" Width (us): "); Serial.println(width);
            //Serial.print("P1 Obj"); Serial.print(i+1); Serial.print(" Amplitude (dec): "); Serial.println(peak);
            objectDetected = true;
        }
      }
    
    // -+-+-+-+-+-+-+-+-+-+-  PRESET 2 (LONG RANGE) MEASUREMENT   -+-+-+-+-+-+-+-+-+-+- //
      if(objectDetected == false || alwaysLong == true)                       // If no preset 1 (short distance) measurement result, switch to Preset 2 B+L command
      {   
        ultrasonicCmd(1, numOfObj, Serial_Port);                // run preset 2 (long distance) burst+listen for 1 object
        pullUltrasonicMeasResult(demoMode, Serial_Port);                // Get Ultrasonic Measurement Result
        for (uint8_t i=0; i<numOfObj; i++)
        {  
          distance = printUltrasonicMeasResult(0+(i*3));   // Print Ultrasonic Measurement Result i.e. Obj1: 0=Distance(m), 1=Width, 2=Amplitude; Obj2: 3=Distance(m), 4=Width, 5=Amplitude;
          //width = ussc.printUltrasonicMeasResult(1+(i*3));    // only available for UART, OWU, and SPI
          //peak = ussc.printUltrasonicMeasResult(2+(i*3));     // only available for UART, OWU, and SPI
  
          usleep(commandDelay*10);  //delay(commandDelay);
    
          if (distance < 1 && distance > minDistLim)    // turn on DS1_LED and F_DIAG_LED if object is within 1m
          {
              //printf("P2 Obj"); printf(i+1); printf(" Distance (m): "); printf(distance);printf("\n");
			  printf("P2 Obj %d Distance (m): %.2f\n", i + 1, distance);

              //Serial.print("P2 Obj"); Serial.print(i+1); Serial.print(" Width (us): "); Serial.println(width);
              //Serial.print("P2 Obj"); Serial.print(i+1); Serial.print(" Amplitude (dec): "); Serial.println(peak);
              objectDetected = true;
          } 
          else if (distance < 3 && distance >= 1)      // turn on DS1_LED and F_DIAG_LED if object is within 3m
          {
              //printf("P2 Obj"); printf(i+1); printf(" Distance (m): "); printf(distance);printf("\n");
			  printf("P2 Obj %d Distance (m): %.2f\n", i + 1, distance);
              objectDetected = true;
          }    
          else if (distance >= 3 && distance < 11.2)     // turn on DS1_LED, F_DIAG_LED, and V_DIAG_LED if object is greater than 3m
          {
			  printf("P2 Obj %d Distance (m): %.2f\n", i + 1, distance);
              //printf("P2 Obj"); printf(i+1); printf(" Distance (m): "); printf(distance);printf("\n");
              objectDetected = true;
          }    
          else if (distance == 0 && commMode!=1)                       
          {
              //Serial.print("Error reading measurement results..."); //Serial.println(distance);
          }
          else //(distance > 11.2 && distance < minDistLim)         
          {
              if (i == numOfObj-1 && objectDetected == false)
              {         
                printf("No object...");printf("\n");
              }
          }
        }  
      }
}

// Function to handle serial events
// TODO: Check if this function is required!!! Call in loop
void serialEvent(int serial_port) {
    while (serialDataAvail(serial_port)) {
        char inChar = serialGetchar(serial_port); // Get the new byte

        // If the incoming character is a 'q', set a flag, stop the main loop, and re-run initialization
        if (inChar == 'q') {
            stringComplete = true;
            initPGA460();
        }
        // If the incoming character is a 'p', set a flag, pause the loop, and resume loop upon receiving another 'p' character
        else if (inChar == 'p') {
            printf("PAUSE\n");
            stringComplete = false;

            while (!stringComplete) {
                while (serialDataAvail(serial_port)) {
                    inChar = serialGetchar(serial_port); // Get the new byte

                    if (inChar == 'p') {
                        stringComplete = true;
                    } else if (inChar == 'q') {
                        stringComplete = true;
                        initPGA460();
                    }
                }
                usleep(100000); // Short delay to prevent CPU overload (100 ms)
            }

            stringComplete = false;
            printf("\n");
        }
    }
}

int main()
{
	initPGA460();
	
	while(1)
	{         
		Cyclic_Task();
		//usleep(250000); // Sleep for 250 milliseconds)
		usleep(100000); // Sleep for 100 milliseconds)
		//serialEvent(Serial_Port);
	}	
	// Close the serial port
	serialClose(Serial_Port);

	return 0;
	
}


