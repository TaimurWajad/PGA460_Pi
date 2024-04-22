/*------------------------------------------------- GetDistance -----
 PROJECT:     PGA460 UART, TCI, & OWU Ultrasonic Time-of-Flight
 DESCRIPTION: Transmits and receives ultrasonic echo data to measure 
              time-of-flight distance, width, and/or amplitude. 
 CREATED:     22 February 2017
 UPDATED:     31 July 2019
 REVISION:    D
 AUTHOR:      A. Whitehead
 NOTES:       This example code is in the public domain.
*-------------------------------------------------------------------*/

#include "PGA460_USSC.h"

/*------------------------------------------------- run mode -----
|  userInputMode
|
|  Purpose:  This code can be operated in two run modes: 
|    • userInputMode = allows the user to configure the device using
|      the COM serial terminal. Resulting data is printed in the
|      terminal view. Recommended run mode.
|    • standAloneMode = waits for the user to press the
|      LaucnhPad's PUSH2 button to automatically execute the
|      initializaiton routine, and begin the burst-and-listen captures.
|      The device is configured based on the hard-coded global 
|      variables. LEDs are illumanted to represent approximate
|      object distance. Results also printed on serial COM terminal.
|      Comment out the run mode to use standAloneMode.
*-------------------------------------------------------------------*/

/*------------------------------------------------- Global Variables -----
|  Global Variables
|
|  Purpose:  Variables shared throughout the GetDistance sketch for 
|    both userInput and standAlone modes. Hard-code these values to
|    the desired conditions when automatically updating the device
|    in standAlone mode.
*-------------------------------------------------------------------*/

// Configuration variables
  uint8_t commMode = 0;            // Communication mode: 0=UART, 1=TCI, 2=OneWireUART
  uint8_t fixedThr = 1;            // set P1 and P2 thresholds to 0=%25, 1=50%, or 2=75% of max; initial minDistLim (i.e. 20cm) ignored
  uint8_t xdcr = 1;                // set PGA460 to recommended settings for 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R
  uint8_t agrTVG = 2;              // set TVG's analog front end gain range to 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB
  uint8_t fixedTVG = 1;            // set fixed TVG level at 0=%25, 1=50%, or 1=75% of max
  uint8_t runDiag = 1;             // run system diagnostics and temp/noise level before looping burst+listen command
  uint8_t edd = 1;                 // echo data dump of preset 1, 2, or neither
  uint8_t burn = 0;                // trigger EE_CNTRL to burn and program user EEPROM memory
  uint8_t cdMultiplier = 1;        // multiplier for command cycle delay
  uint8_t numOfObj = 1;            // number of object to detect set to 1-8
  uint8_t uartAddrUpdate = 0;      // PGA460 UART address to interface to; default is 0, possible address 0-7
  bool objectDetected = false;  // object detected flag to break burst+listen cycle when true
  bool demoMode = false;        // only true when running UART/OWU multi device demo mode
  bool alwaysLong = false;      // always run preset 2, regardless of preset 1 result (hard-coded only)
  double minDistLim = 0.1;      // minimum distance as limited by ringing decay of single transducer and threshold masking
  uint16_t commandDelay = 0;    // Delay between each P1 and Preset 2 command
  uint32_t baudRate = 115200;     // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200 

//PUSH BUTTON used for standAlone mode
  const int buttonPin = 10;  // the number of the pushbutton pin
  int buttonState = 0;          // variable for reading the pushbutton status

// Result variables
  double distance = 0;          // one-way object distance in meters
  double width = 0;             // object width in microseconds
  double peak = 0;              // object peak in 8-bit
  double diagnostics = 0;       // diagnostic selector
  uint8_t echoDataDumpElement = 0; // echo data dump element 0 to 127
//  String interruptString = "";  // a string to hold incoming data
  uint8_t stringComplete = false; // whether the string is complete

// PGA460_USSC library class
  pga460 ussc;

  void initPGA460() {

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
    ussc.initBoostXLPGA460(commMode, baudRate, uartAddrUpdate);

  // -+-+-+-+-+-+-+-+-+-+- 2 : bulk threshold write   -+-+-+-+-+-+-+-+-+-+- //
    if (fixedThr != 72){ussc.initThresholds(fixedThr);} 
  // -+-+-+-+-+-+-+-+-+-+- 3 : bulk user EEPROM write   -+-+-+-+-+-+-+-+-+-+- //
    if (xdcr != 72){ussc.defaultPGA460(xdcr);}
  // -+-+-+-+-+-+-+-+-+-+- 4 : bulk TVG write   -+-+-+-+-+-+-+-+-+-+- //
    if (agrTVG != 72 && fixedTVG != 72){ussc.initTVG(agrTVG,fixedTVG);}
  // -+-+-+-+-+-+-+-+-+-+- 5 : run system diagnostics   -+-+-+-+-+-+-+-+-+-+- //
    if (runDiag == 1)
    {      
      diagnostics = ussc.runDiagnostics(1,0);       // run and capture system diagnostics, and print freq diag result
      std::cout << "System Diagnostics - Frequency (kHz): "<< std::endl; std::cout << diagnostics << std::endl;
      diagnostics = ussc.runDiagnostics(0,1);       // do not re-run system diagnostic, but print decay diag result
      std::cout << "System Diagnostics - Decay Period (us): "<< std::endl; std::cout << diagnostics << std::endl;
      diagnostics = ussc.runDiagnostics(0,2);       // do not re-run system diagnostic, but print temperature measurement
      std::cout << "System Diagnostics - Die Temperature (C): "<< std::endl; std::cout << diagnostics << std::endl;
      diagnostics = ussc.runDiagnostics(0,3);       // do not re-run system diagnostic, but print noise level measurement
      std::cout << "System Diagnostics - Noise Level: "<< std::endl; std::cout << diagnostics << std::endl;
    }
  // -+-+-+-+-+-+-+-+-+-+- 6 : burn EEPROM   -+-+-+-+-+-+-+-+-+-+- //
    if(burn == 1)
    {
      uint8_t burnStat = ussc.burnEEPROM();
      if(burnStat == true){std::cout << "EEPROM programmed successfully." << std::endl;}
      else{std::cout << "EEPROM program failed." << std::endl;}
    }
  // -+-+-+-+-+-+-+-+-+-+- 7 : capture echo data dump   -+-+-+-+-+-+-+-+-+-+- //
    if (edd != 0)                                   // run or skip echo data dump
    {
      std::cout << "Retrieving echo data dump profile. Wait..." << std::endl;
      ussc.runEchoDataDump(edd-1);                  // run preset 1 or 2 burst and/or listen command
      std::cout << ussc.pullEchoDataDumpBulk()<< std::endl;
      std::cout << "" << std::endl;
    }
  // -+-+-+-+-+-+-+-+-+-+-  others   -+-+-+-+-+-+-+-+-+-+- //
  commandDelay = 100 * cdMultiplier;                   // command cycle delay result in ms
  if (numOfObj == 0 || numOfObj >8) { numOfObj = 1; } // sets number of objects to detect to 1 if invalid input                      

}

// -+-+-+-+-+-+-+-+-+-+-  SERIAL MONITORING   -+-+-+-+-+-+-+-+-+-+- //
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
    int fd = serialOpen(UART_PORT, baudRate); // Open serial port ("/dev/ttyS0" for primary UART on Raspberry Pi)
    if (fd < 0) {
        std::cerr << "Error opening serial port" << std::endl;
        return;
    }

    while (serialDataAvail(fd)) {
        char inChar = (char)serialGetchar(fd); // Read the new byte
      
        // if the incoming character is a 'q', set a flag, stop the main loop, and re-run initialization
        if (inChar == 'q'){
            stringComplete = true;
            initPGA460();
        }
      
        // if the incoming character is a 'p', set a flag, pause the loop, and resume loop upon receiving another 'p' character
        else if (inChar == 'p') 
		{
            std::cout << "PAUSE" << std::endl;
            stringComplete = false; 
            while(stringComplete == false && serialDataAvail(fd)) 
			{
                inChar = (char)serialGetchar(fd); // Read the new byte
                if (inChar == 'p') {
                    stringComplete = true;
                }
                if (inChar == 'q') {
                    stringComplete = true; 
                    initPGA460();
                }
                delay(1000);
            }
            stringComplete=false;
            std::cout << "" << std::endl;
        }    
    }
    serialClose(fd); // Close serial port
}


void loop() {                 // put your main code here, to run repeatedly
    // -+-+-+-+-+-+-+-+-+-+-  PRESET 1 (SHORT RANGE) MEASUREMENT   -+-+-+-+-+-+-+-+-+-+- //
      objectDetected = false;                       // Initialize object detected flag to false
      std::cout << "TEST0" << std::endl;
      ussc.ultrasonicCmd(0,numOfObj);               // run preset 1 (short distance) burst+listen for 1 object
      std::cout << "TEST1" << std::endl;
      ussc.pullUltrasonicMeasResult(demoMode);      // Pull Ultrasonic Measurement Result
      std::cout << "TEST2" << std::endl;
      for (uint8_t i=0; i<numOfObj; i++)
      {  
        std::cout << "TEST3" << std::endl;    
        // Log uUltrasonic Measurement Result: Obj1: 0=Distance(m), 1=Width, 2=Amplitude; Obj2: 3=Distance(m), 4=Width, 5=Amplitude; etc.;
          distance = ussc.printUltrasonicMeasResult(0+(i*3));      
          //width = ussc.printUltrasonicMeasResult(1+(i*3));  // only available for UART, OWU, and SPI
          //peak = ussc.printUltrasonicMeasResult(2+(i*3));   // only available for UART, OWU, and SPI
          std::cout << distance << std::endl;
          std::cout << "TEST4" << std::endl;
  
        delay(commandDelay);
    
        if (distance > minDistLim && distance < 11.2)  // turn on DS1_LED if object is above minDistLim
        {
            //ussc.toggleLEDs(HIGH,LOW,LOW);
            std::cout << "P1 Obj"<< std::endl; std::cout << i+1<< std::endl; std::cout << " Distance (m): "<< std::endl; std::cout << distance << std::endl;
            //Serial.print("P1 Obj"); Serial.print(i+1); Serial.print(" Width (us): "); Serial.println(width);
            //Serial.print("P1 Obj"); Serial.print(i+1); Serial.print(" Amplitude (dec): "); Serial.println(peak);
            objectDetected = true;
        }
      }
      std::cout << "TEST5" << std::endl;
    
    // -+-+-+-+-+-+-+-+-+-+-  PRESET 2 (LONG RANGE) MEASUREMENT   -+-+-+-+-+-+-+-+-+-+- //
      if(objectDetected == false || alwaysLong == true)                       // If no preset 1 (short distance) measurement result, switch to Preset 2 B+L command
      {   
        ussc.ultrasonicCmd(1,numOfObj);                // run preset 2 (long distance) burst+listen for 1 object
        ussc.pullUltrasonicMeasResult(demoMode);                // Get Ultrasonic Measurement Result
        for (uint8_t i=0; i<numOfObj; i++)
        {  
          distance = ussc.printUltrasonicMeasResult(0+(i*3));   // Print Ultrasonic Measurement Result i.e. Obj1: 0=Distance(m), 1=Width, 2=Amplitude; Obj2: 3=Distance(m), 4=Width, 5=Amplitude;
          //width = ussc.printUltrasonicMeasResult(1+(i*3));    // only available for UART, OWU, and SPI
          //peak = ussc.printUltrasonicMeasResult(2+(i*3));     // only available for UART, OWU, and SPI
  
          delay(commandDelay);
    
          if (distance < 1 && distance > minDistLim)    // turn on DS1_LED and F_DIAG_LED if object is within 1m
          {
              //ussc.toggleLEDs(HIGH,LOW,LOW);
              std::cout << "P2 Obj"<< std::endl;  std::cout << distance << i+1 << std::endl; std::cout << distance <<" Distance (m): " << std::endl; std::cout << distance << std::endl;
              //Serial.print("P2 Obj"); Serial.print(i+1); Serial.print(" Width (us): "); Serial.println(width);
              //Serial.print("P2 Obj"); `i+1); Serial.print(" Amplitude (dec): "); Serial.println(peak);
              objectDetected = true;
              
          } 
          else if (distance < 3 && distance >= 1)      // turn on DS1_LED and F_DIAG_LED if object is within 3m
          {
              ussc.toggleLEDs(HIGH,HIGH,LOW);
              std::cout << "P2 Obj"<< std::endl; std::cout << i+1<< std::endl; std::cout << " Distance (m): "<< std::endl; std::cout << distance << std::endl;
              objectDetected = true;
          }    
          else if (distance >= 3 && distance < 11.2)     // turn on DS1_LED, F_DIAG_LED, and V_DIAG_LED if object is greater than 3m
          {
              ussc.toggleLEDs(HIGH,HIGH,HIGH);
              std::cout << "P2 Obj"<< std::endl; std::cout << i+1<< std::endl; std::cout << " Distance (m): "<< std::endl; std::cout << distance << std::endl;
              objectDetected = true;
          }    
          else if (distance == 0 && commMode!=1)                         // turn off all LEDs if no object detected
          {
              ussc.toggleLEDs(LOW,LOW,LOW);
              //std::cout << "Error reading measurement results..."); //Serial.println(distance);
              std::cout << "TEST6" << std::endl;
          }
          else //(distance > 11.2 && distance < minDistLim)         // turn off all LEDs if no object detected or below minimum distance limit
          {
            std::cout << "TEST7" << std::endl;
              if (i == numOfObj-1 && objectDetected == false)
              {
                ussc.toggleLEDs(LOW,LOW,LOW);            
                std::cout << "No object..." << std::endl;
              }
          }
        }  
      }
    
    // -+-+-+-+-+-+-+-+-+-+-  STATUS   -+-+-+-+-+-+-+-+-+-+- //
      //digitalWrite(11, !digitalRead(11));   //toggle green LED after each sequence
      //digitalWrite(12, !digitalRead(12));     //toggle red LED after each sequence

      //serialEvent();
}

int main(){
    // Initialize WiringPi and GPIO
    wiringPiSetup();
    wiringPiSetupGpio();
    initPGA460();
    while (true)
    {
        loop();
    }

    return 0;
}
