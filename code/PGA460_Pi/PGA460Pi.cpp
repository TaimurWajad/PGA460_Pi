#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>  // Include this header for strerror

#include <PGA460_USSC.h>

  unsigned char commMode = 0;            // Communication mode: 0=UART, 1=TCI, 2=OneWireUART
  unsigned char fixedThr = 1;            // set P1 and P2 thresholds to 0=%25, 1=50%, or 2=75% of max; initial minDistLim (i.e. 20cm) ignored
  unsigned char xdcr = 1;                // set PGA460 to recommended settings for 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R
  unsigned char agrTVG = 2;              // set TVG's analog front end gain range to 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB
  unsigned char fixedTVG = 1;            // set fixed TVG level at 0=%25, 1=50%, or 1=75% of max
  unsigned char runDiag = 0;             // run system diagnostics and temp/noise level before looping burst+listen command
  unsigned char edd = 0;                 // echo data dump of preset 1, 2, or neither
  unsigned char burn = 0;                // trigger EE_CNTRL to burn and program user EEPROM memory
  unsigned char cdMultiplier = 1;        // multiplier for command cycle delay
  unsigned char numOfObj = 1;            // number of object to detect set to 1-8
  unsigned char uartAddrUpdate = 0;      // PGA460 UART address to interface to; default is 0, possible address 0-7
  bool objectDetected = false;  // object detected flag to break burst+listen cycle when true
  bool demoMode = false;        // only true when running UART/OWU multi device demo mode
  bool alwaysLong = false;      // always run preset 2, regardless of preset 1 result (hard-coded only)
  double minDistLim = 0.1;      // minimum distance as limited by ringing decay of single transducer and threshold masking
  uint16_t commandDelay = 0;    // Delay between each P1 and Preset 2 command
  uint32_t baudRate = 9600;     // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200 

//PUSH BUTTON used for standAlone mode
  const int buttonPin = 10;  // the number of the pushbutton pin
  int buttonState = 0;          // variable for reading the pushbutton status

// Result variables
  double distance = 0;          // one-way object distance in meters
  double width = 0;             // object width in microseconds
  double peak = 0;              // object peak in 8-bit
  double diagnostics = 0;       // diagnostic selector
  byte echoDataDumpElement = 0; // echo data dump element 0 to 127
  String interruptString = "";  // a string to hold incoming data
  boolean stringComplete = false; // whether the string is complete
  
  
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
    initBoostXLPGA460(commMode, baudRate, uartAddrUpdate); 
    
  // -+-+-+-+-+-+-+-+-+-+- 2 : bulk threshold write   -+-+-+-+-+-+-+-+-+-+- //
    if (fixedThr != 72){initThresholds(fixedThr);} 
  // -+-+-+-+-+-+-+-+-+-+- 3 : bulk user EEPROM write   -+-+-+-+-+-+-+-+-+-+- //
    if (xdcr != 72){defaultPGA460(xdcr);}
  // -+-+-+-+-+-+-+-+-+-+- 4 : bulk TVG write   -+-+-+-+-+-+-+-+-+-+- //
    if (agrTVG != 72 && fixedTVG != 72){initTVG(agrTVG,fixedTVG);}
  // -+-+-+-+-+-+-+-+-+-+- 5 : run system diagnostics   -+-+-+-+-+-+-+-+-+-+- //
    if (runDiag == 1)
    {      
      diagnostics = ussc.runDiagnostics(1,0);       // run and capture system diagnostics, and print freq diag result
      printf("System Diagnostics - Frequency (kHz): "); printf(diagnostics);
      diagnostics = runDiagnostics(0,1);       // do not re-run system diagnostic, but print decay diag result
      printf("System Diagnostics - Decay Period (us): "); printf(diagnostics);
      diagnostics = ussc.runDiagnostics(0,2);       // do not re-run system diagnostic, but print temperature measurement
      printf("System Diagnostics - Die Temperature (C): "); printf(diagnostics);
      diagnostics = runDiagnostics(0,3);       // do not re-run system diagnostic, but print noise level measurement
      printf("System Diagnostics - Noise Level: "); printf(diagnostics);
    }
  // -+-+-+-+-+-+-+-+-+-+- 6 : burn EEPROM   -+-+-+-+-+-+-+-+-+-+- //
    if(burn == 1)
    {
      unsigned char burnStat = burnEEPROM();
      if(burnStat == true){printf("EEPROM programmed successfully.");}
      else{printf("EEPROM program failed.");}
    }
  // -+-+-+-+-+-+-+-+-+-+- 7 : capture echo data dump   -+-+-+-+-+-+-+-+-+-+- //
    if (edd != 0)                                   // run or skip echo data dump
    {
      printf("Retrieving echo data dump profile. Wait...");
      runEchoDataDump(edd-1);                  // run preset 1 or 2 burst and/or listen command
      printf(ussc.pullEchoDataDumpBulk());
      printf("");
    }
  // -+-+-+-+-+-+-+-+-+-+-  others   -+-+-+-+-+-+-+-+-+-+- //
  commandDelay = 100 * cdMultiplier;                   // command cycle delay result in ms
  if (numOfObj == 0 || numOfObj >8) { numOfObj = 1; } // sets number of objects to detect to 1 if invalid input                      

}
