#include <iostream>
#include <cstring>
#include <string>
#include <bitset>
#include <cmath>


// Include WiringPi library for GPIO access
#include <wiringPi.h> ///home/twajad/PGA460_Pi4/wiringPi.h
#include <wiringSerial.h>
#include <wiringPiSPI.h>
#define UART_PORT  "/dev/ttyAMA0" // Replace with correct path
//#define SERIAL_PORT  "/dev/ttyUSB0"  // Replace with correct path 
// TODO Port Interface
#define BAUD_RATE 19200
class pga460
{
public:
    pga460();
    uint8_t pullEchoDataDump(uint8_t element);
    uint8_t registerRead(uint8_t addr);
    uint8_t registerWrite(uint8_t addr, uint8_t data);
    void initBoostXLPGA460(uint8_t mode, uint32_t baud, uint8_t uartAddrUpdate);
    void defaultPGA460(uint8_t xdcr);
    void initThresholds(uint8_t thr);
    void initTVG(uint8_t agr, uint8_t tvg);
    void ultrasonicCmd(uint8_t cmd, uint8_t numObjUpdate);
    void runEchoDataDump(uint8_t preset);
    void broadcast(bool eeBulk, bool tvgBulk, bool thrBulk);
    void toggleLEDs(bool ds1State, bool fdiagState, bool vdiagState);
    void autoThreshold(uint8_t cmd, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t avgLoops);
    void eepromThreshold(uint8_t preset, bool saveLoad);
    void thresholdBulkRead(uint8_t preset);
    void thresholdBulkWrite(uint8_t p1ThrMap[], uint8_t p2ThrMap[]);
    bool burnEEPROM();
    bool pullUltrasonicMeasResult(bool busDemo);
    double printUltrasonicMeasResult(uint8_t umr);
    uint8_t printUltrasonicMeasResultRaw(uint8_t umr);
    double printUltrasonicMeasResultExt(uint8_t umr, int speedSound);
    double runDiagnostics(uint8_t run, uint8_t diag);
    double triangulation(double a, double b, double c);
    std::string pullEchoDataDumpBulk();

private:
    
    uint8_t calcChecksum(uint8_t cmd);
    void pga460SerialFlush();
    void tciRecord(uint8_t numObj);
    void tciByteToggle(uint8_t data, uint8_t zeroPadding);
    void tciIndexRW(uint8_t index, bool write);
    void tciCommand(uint8_t cmd);
    void spiTransfer(uint8_t *mosi, uint8_t size);
    void spiMosiIdle(uint8_t size);
};
