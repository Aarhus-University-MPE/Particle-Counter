/*****************
This program was written by WS on 13/11/2018

This program is intended for Arduino UNO R3 or Leonardo R3 (or Olimexino, which
is a Leonardo clone). Edit #defines to set which one. If using Arduino UNO R3 or
Leonardo R3, the different communication logic levels should be respected. Arduino
is 5v, OPC-N3 is 3.3v. The OPC-N3 PIC is 5v tolerant on most of the SPI pins, but
not the Slave Select pin. A simple solution is to use a resistor pair to divide the
voltage on the SS pin appropriately. If an Olimexino device is used, it has a
jumper switch to set it to either 5v or 3.3v. Set to 3.3v to connect to OPC-N3
without need for logic level conversion.
The program is for use with OPC-N3 devices with standard v1.17a firmware.

This program configures the Arduino to put an OPC-N3 through 1 minute cycles
taking one set of measurements per second during the 10s active part of the cycle
and turning the fan and laser off for the remainder of the cycle.
The data taken from the OPC-N3 via the SPI interface is processed
and the processed data fed to the serial interface (the USB
port) so that it can be displayed on a computer using suitable terminal software.
Usual serial port settings should be set in the terminal software (8 data bits,
1 stop bit, no parity, no flow control). Set to baud rate to match the value
defined in this program.

Adding more Slave Select pins will allow multiple OPC-N3 devices on one SPI bus.
Only one Slave Select pin should be active at any time.
*****************/

#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSPI.h>
#include <avr/wdt.h>

#define FirmwareVer   "OPC-N3-02(UNOr3)(res divider on SS)"

#define BaudRate      9600

// OPC-N3 SPI responces
#define SPI_OPC_busy  0x31
#define SPI_OPC_ready 0xF3

// Bitbang-SPI pins for OPC
#define OPC_CS        4
#define OPC_SCK       3
#define OPC_MOSI      5
#define OPC_MISO      6

// SPI pins for SD
#define SD_CS         10

// LED's to indicate states
#define green         14
#define red           15

// Declare global SPI communicasion for OPC.
SoftwareSPI OPC_SPI(OPC_SCK, OPC_MOSI, OPC_MISO, OPC_CS);

// RTC functions.
RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

unsigned long currentTime;
unsigned long cloopTime;
unsigned char SPI_in[86], SPI_in_index, ssPin_OPC;

// Define custome functions:
void InitRTC();
void InitSD();
void OpenSysData(String filename);
void OpenStorage(String filename);
void CloseSD(Stream &Port, File &FileName);
void PrintFirmwareVer(Stream &port, File &Filename);
void InitDevice(void);
void Timestamp(Stream &port, File &Filename);
void ReadOPCstring(unsigned char SPIcommand);
void PrintOPCstring(Stream &port, File &Filename);
void TimestampShort(Stream &port, File &Filename);
void ReadOPChist(void);
void DiscardSPIbytes(byte NumToDiscard);
void ReadOPCconfig(Stream &port, File &Filename);
void StartOPC(void);
void StartLaser(void);
void StopLaser(void);
void StopOPC(void);
void GetReadyResponse(byte SPIcommand);
void MODBUS_CalcCRC();
void ConvSTtoTemperature();
void ConvSRHtoRelativeHumidity();
void PrintData(Stream &port, File &Filename);
void PrintDataLabels(Stream &port, File &Filename);
void AddDelimiter(Stream &port, File &Filename);
void BlinkLED(int blinks, int LEDPin);

File Storage;
File SysData;

DateTime now;
DateTime Time;

char TimeShort;
char SystemData[]  = "SDyymmdd.txt";
char StorageFile[] = "SFyymmdd.csv";

void setup() {
  wdt_reset();          // Reset watchdog timer
  wdt_enable(WDTO_8S);  // Enable watchdog timer, countdown 8s (max)

  delay(6000);  // delay in case of noise on power connection. Also allows OPC to boot up.

  wdt_reset();  // Reset watchdog timer

  // Start serial port
  Serial.begin(BaudRate);

  // Define Led pins
  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);
  digitalWrite(green, LOW);
  digitalWrite(red, LOW);

  // Start RTC and configure current time and date.
  InitRTC();

  // If the seial port is connected the RTC will reconfigure.
  if (Serial) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialice the SD card stogare.
  InitSD();

  // Creat filenames for SD storage.
  DateTime Time = rtc.now();
  sprintf(SystemData, "SD%02d%02d%02d.txt", Time.day(), Time.month(), Time.year() % 100);
  sprintf(StorageFile, "SF%02d%02d%02d.csv", Time.day(), Time.month(), Time.year() % 100);

  wdt_reset();  // Reset watchdog timer

  OpenSysData(SystemData);

  // Initiate the SPI.Bitbang lib.
  OPC_SPI.beginSPI();

  PrintFirmwareVer(Serial, SysData);  // Print firmware version to serial port

  wdt_reset();  // Reset watchdog timer

  // Start bitbanging the sensor
  InitDevice();

  wdt_reset();  // Reset watchdog timer

  Timestamp(Serial, SysData);
  delay(10);

  CloseSD(Serial, SysData);
  delay(1000);

  OpenStorage(StorageFile);

  PrintDataLabels(Serial, Storage);  // Print labels to serial port

  CloseSD(Serial, Storage);

  cloopTime = millis();  // cloopTime, for the mail loop, initiates at the millis cnt. at the point were the initiation has compleated, this wey the first two samplings whon't be close togehter.
}

void InitRTC() {
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    BlinkLED(2, red);
    while (1) delay(10);
  }
  Serial.println("initialization of RTC done.");
  BlinkLED(2, green);
  wdt_reset();  // Reset watchdog timer
  delay(1000);
  wdt_reset();  // Reset watchdog timer
}

void InitSD() {  // SD storage
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization of SD failed!");
    BlinkLED(3, red);
    while (1);
  }
  Serial.println("initialization of SD done.");
  BlinkLED(3, green);
  wdt_reset();  // Reset watchdog timer
  delay(1000);
  wdt_reset();  // Reset watchdog timer
}

void OpenSysData(String filename) {
  SysData = SD.open(filename, FILE_WRITE | O_CREAT | O_APPEND);
  if (!SysData) {
    Serial.println("Failed to open or create Storage file");
    BlinkLED(4, red);
    while (1);  // Kan blive kommenteret ud for at ignorere at den tror filen ikke er oprettet
  } else {
    Serial.println("SysData opend");
    BlinkLED(4, green);
    wdt_reset();  // Reset watchdog timer
    delay(1000);
    wdt_reset();  // Reset watchdog timer
  }
}

void OpenStorage(String filename) {
  Storage = SD.open(filename, FILE_WRITE | O_CREAT | O_APPEND);
  if (!Storage) {
    Serial.println("Failed to open or create Storage file");
    BlinkLED(5, red);
    while (1);  // Kan blive kommenteret ud for at ignorere at den tror filen ikke er oprettet
  } else {
    Serial.println("Storage opend");
    BlinkLED(5, green);
    wdt_reset();  // Reset watchdog timer
    delay(1000);
    wdt_reset();  // Reset watchdog timer
  }
}

void CloseSD(Stream &Port, File &FileName) {
  FileName.close();
  digitalWrite(SD_CS, HIGH);
  Port.println("SD Closed");
  BlinkLED(1, green);
  BlinkLED(1, red);
  wdt_reset();  // Reset watchdog timer
  delay(1000);
  wdt_reset();  // Reset watchdog timer
}

void PrintFirmwareVer(Stream &port, File &Filename) {
  port.print(F("Datalogger firmware ver "));
  port.println(FirmwareVer);
  Filename.print(F("Datalogger firmware ver "));
  Filename.println(FirmwareVer);
}

void InitDevice(void) {
  wdt_reset();  // Reset watchdog timer

  ReadOPCstring(0x10);  // Get serialstr from OPC device
  ReadOPCstring(0x3F);  // Get infostr from OPC device

  StartOPC();                      // Switch on power to fan and laser
  wdt_reset();                     // Reset watchdog timer
  ReadOPCconfig(Serial, SysData);  // Get Config data (bin boundaries etc.) from OPC device
}

void Timestamp(Stream &port, File &Filename) {
  DateTime now = rtc.now();

  // Serial printing:
  port.print(now.year(), DEC);
  port.print('/');
  port.print(now.month(), DEC);
  port.print('/');
  port.print(now.day(), DEC);
  port.print(" (");
  port.print(daysOfTheWeek[now.dayOfTheWeek()]);
  port.print(") ");
  port.print(now.hour(), DEC);
  port.print(':');
  port.print(now.minute(), DEC);
  port.print(':');
  port.print(now.second(), DEC);
  port.println();
  // UTC
  port.print("Since midnight 1/1/1970 (UTC) = ");
  port.print(now.unixtime());
  port.print("s = ");
  port.print(now.unixtime() / 86400L);
  port.println("d");

  // File printing:
  Filename.print(now.year(), DEC);
  Filename.print('/');
  Filename.print(now.month(), DEC);
  Filename.print('/');
  Filename.print(now.day(), DEC);
  Filename.print(" (");
  Filename.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Filename.print(") ");
  Filename.print(now.hour(), DEC);
  Filename.print(':');
  Filename.print(now.minute(), DEC);
  Filename.print(':');
  Filename.print(now.second(), DEC);
  Filename.println();
  // UTC
  Filename.print("Since midnight 1/1/1970 (UTC) = ");
  Filename.print(now.unixtime());
  Filename.print("s = ");
  Filename.print(now.unixtime() / 86400L);
  Filename.println("d");

  /*
  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");
  */
}

// Main Loop
void loop() {
  wdt_reset();  // Reset watchdog timer

  BlinkLED(1, green);  // Blinks green Led to indicate that the system can be powerd off "safly", the Led will need to be blinking rytmich, and the fan will still be on.

  // This is the main loop which should do the following:
  // Switch ON fan and laser
  // Get 10 histogram data sets, one per second (don't record the first one)
  // Switch OFF fan and laser
  // Repeat every 60s

  // The timings mey be off given we bitbang the sensor insted of using real SPI.

  currentTime = millis();  // millis count will reset on sketch restart
  if (currentTime >= cloopTime) {
    cloopTime += 60000;  // Updates cloopTime

    // Creats the storage file name, new file if the previus is a day old.
    DateTime Time = rtc.now();
    sprintf(StorageFile, "SF%02d%02d%02d.csv", Time.day(), Time.month(), Time.year() % 100);
    delay(5);

    OpenStorage(StorageFile);
    delay(50);

    wdt_reset();  // Reset watchdog timer

    // Switch power ON to fan and laser
    // StartOPC(); The Fan should not be stopped, so we only need to turn the laser back on
    StartLaser();

    wdt_reset();  // Reset watchdog timer

    // Get 10 histogram data sets (don't record the first one as it will contain invalid data)
    // Not in use; unsigned long GetHistTime = millis();  // Set initial GetHistTime
    for (byte i = 0; i < 10; i++) {
      delay(1000);
      ReadOPChist();  // Read OPC histogram data
      if (i != 0) {
        // Print time since start (millis() returns an unsigned long of number of ms since program started. It wraps around in ~50 days.)
        // Serial.print(millis()); We no longer whant the millis time stamp.
        // Storage.print(millis());
        TimestampShort(Serial, Storage);

        PrintData(Serial, Storage);  // Print data to serial
      }
      wdt_reset();  // Reset watchdog timer
    }

    // Switch power OFF to fan and laser
    // StopOPC(); We dont wish to stop the fan.
    StopLaser();
    // END Device #1
    CloseSD(Serial, Storage);

    Serial.println("Waiting until next cycle");
    Storage.println("Waiting until next cycle");
  }
}

// Get string (serialstr or infostr) from OPC device
void ReadOPCstring(unsigned char SPIcommand) {
  GetReadyResponse(SPIcommand);
  for (SPI_in_index = 0; SPI_in_index < 60; SPI_in_index++) {
    delayMicroseconds(10);
    // OLD SPI: SPI_in[SPI_in_index] = SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);
  }

  OPC_SPI.deselect();

  PrintOPCstring(Serial, SysData);
}

void PrintOPCstring(Stream &port, File &Filename) {
  port.write(SPI_in, 60);  // print 60 characters from SPI_in[] array
  port.println("");
  port.flush();
  Filename.write(SPI_in, 60);  // print 60 characters from SPI_in[] array
  Filename.println("");
  Filename.flush();
}

void TimestampShort(Stream &port, File &Filename) {
  DateTime now = rtc.now();

  // Serial printing:
  port.print(now.year() % 100 < 10 ? "0" : "");  // Get the last two digits of the year
  port.print(now.year() % 100, DEC);
  // port.print('/'); The CSV file cant handle the "/"
  port.print(now.month() < 10 ? "0" : "");  // Format month with leading zero if needed
  port.print(now.month(), DEC);
  // port.print('/'); The CSV file cant handle the "/"
  port.print(now.day() < 10 ? "0" : "");  // Format day with leading zero if needed
  port.print(now.day(), DEC);
  port.print(";");
  port.print(now.hour(), DEC);
  port.print(':');
  port.print(now.minute(), DEC);
  port.print(':');
  port.print(now.second(), DEC);
  // UTC
  /*
  port.print(" since midnight 1/1/1970 = ");
  port.print(now.unixtime());
  port.print("s = ");
  port.print(now.unixtime() / 86400L);
  port.println("d");
  */

  // File printing:
  // Serial printing:
  Filename.print(now.year() % 100 < 10 ? "0" : "");  // Get the last two digits of the year
  Filename.print(now.year() % 100, DEC);
  // port.print('/'); The CSV file cant handle the "/"
  Filename.print(now.month() < 10 ? "0" : "");  // Format month with leading zero if needed
  Filename.print(now.month(), DEC);
  // port.print('/'); The CSV file cant handle the "/"
  Filename.print(now.day() < 10 ? "0" : "");  // Format day with leading zero if needed
  Filename.print(now.day(), DEC);
  Filename.print(";");
  Filename.print(now.hour(), DEC);
  Filename.print(':');
  Filename.print(now.minute(), DEC);
  Filename.print(':');
  Filename.print(now.second(), DEC);
  // UTC
  /*
  Filename.print(" since midnight 1/1/1970 = ");
  Filename.print(now.unixtime());
  Filename.print("s = ");
  Filename.print(now.unixtime() / 86400L);
  Filename.println("d");
  */
}

void ReadOPChist(void) {
  GetReadyResponse(0x30);
  for (SPI_in_index = 0; SPI_in_index < 86; SPI_in_index++) {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
  OPC_SPI.deselect();
  delay(10);
}

void DiscardSPIbytes(byte NumToDiscard) {
  for (SPI_in_index = 0; SPI_in_index < NumToDiscard; SPI_in_index++) {
    delayMicroseconds(10);
    OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
}

// Get Config data (bin boundaries etc.) from OPC device
void ReadOPCconfig(Stream &port, File &Filename) {
  unsigned int *pUInt16;
  // Not in use; float *pFloat;

  // Have to read config from OPC device in this 'chunks' manner as Arduino buffer isn't big enough to hold all config data at once and OPC could timeout if Arduino took time to print/save data from the buffer during the SPI transfer sequence.
  // Instead, config data is read several times, and each time a different chunk is saved to the Arduino buffer and printed. This way there is no delay during each individual SPI transfer sequence.

  // Get config from OPC device (Bin Boundaries ADC)
  GetReadyResponse(0x3C);
  for (SPI_in_index = 0; SPI_in_index < 50; SPI_in_index++) {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
  // Throw away any remaining bytes OPC expects to transfer. Although not putting it in buffer yet, must complete SPI transfer of all config bytes as OPC is expecting this
  DiscardSPIbytes(118);
  OPC_SPI.deselect();
  delay(10);

  port.print(F("BinBoundaries(ADC)"));
  Filename.print(F("BinBoundaries(ADC)"));
  for (SPI_in_index = 0; SPI_in_index < 50; SPI_in_index += 2) {
    AddDelimiter(port, Filename);
    pUInt16 = (unsigned int *)&SPI_in[SPI_in_index];
    port.print(*pUInt16, DEC);
    Filename.print(*pUInt16, DEC);
  }
  port.println("");
  port.flush();
  Filename.println("");
  Filename.flush();
  // END Get config from OPC device (Bin Boundaries ADC)

  // Get config from OPC device (Bin Boundaries um)
  GetReadyResponse(0x3C);
  // Throw away bytes until reaching desired bytes in config
  DiscardSPIbytes(50);
  // Put required config bytes in buffer
  for (SPI_in_index = 0; SPI_in_index < 50; SPI_in_index++) {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
  // Throw away any remaining bytes OPC expects to transfer
  DiscardSPIbytes(68);
  OPC_SPI.deselect();
  delay(10);

  port.print(F("BinBoundaries(um)"));
  Filename.print(F("BinBoundaries(um)"));
  for (SPI_in_index = 0; SPI_in_index < 50; SPI_in_index += 2) {
    AddDelimiter(port, Filename);
    pUInt16 = (unsigned int *)&SPI_in[SPI_in_index];
    port.print((float)*pUInt16 / 100, 2);      // print to 2dp
    Filename.print((float)*pUInt16 / 100, 2);  // print to 2dp
  }
  port.println("");
  port.flush();
  Filename.println("");
  Filename.flush();
  // END Get config from OPC device (Bin Boundaries um)

  // Get config from OPC device (Bin Weightings)
  GetReadyResponse(0x3C);
  // Throw away bytes until reaching desired bytes in config
  DiscardSPIbytes(100);
  // Put required config bytes in buffer
  for (SPI_in_index = 0; SPI_in_index < 48; SPI_in_index++) {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
  // Throw away any remaining bytes OPC expects to transfer
  DiscardSPIbytes(20);
  OPC_SPI.deselect();
  delay(10);

  port.print(F("BinWeightings"));
  Filename.print(F("BinWeightings"));
  for (SPI_in_index = 0; SPI_in_index < 48; SPI_in_index += 2) {
    AddDelimiter(port, Filename);
    pUInt16 = (unsigned int *)&SPI_in[SPI_in_index];
    port.print((float)*pUInt16 / 100, 2);      // print to 2dp
    Filename.print((float)*pUInt16 / 100, 2);  // print to 2dp
  }
  port.println("");
  port.flush();
  Filename.println("");
  Filename.flush();
  // END Get config from OPC device (Bin Weightings)

  // Get config from OPC device (Misc)
  GetReadyResponse(0x3C);
  // Throw away bytes until reaching desired bytes in config
  DiscardSPIbytes(148);
  // Put required config bytes in buffer
  for (SPI_in_index = 0; SPI_in_index < 20; SPI_in_index++) {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
  OPC_SPI.deselect();
  delay(10);

  port.print(F("M_A(um);"));
  Filename.print(F("M_A(um);"));
  pUInt16 = (unsigned int *)&SPI_in[0];
  port.println((float)*pUInt16 / 100, 2);      // print to 2dp
  Filename.println((float)*pUInt16 / 100, 2);  // print to 2dp

  port.print(F("M_B(um);"));
  Filename.print(F("M_B(um);"));
  pUInt16 = (unsigned int *)&SPI_in[2];
  port.println((float)*pUInt16 / 100, 2);      // print to 2dp
  Filename.println((float)*pUInt16 / 100, 2);  // print to 2dp

  port.print(F("M_C(um);"));
  Filename.print(F("M_C(um);"));
  pUInt16 = (unsigned int *)&SPI_in[4];
  port.println((float)*pUInt16 / 100, 2);      // print to 2dp
  Filename.println((float)*pUInt16 / 100, 2);  // print to 2dp

  port.print(F("MaxTOF(us);"));
  Filename.print(F("MaxTOF(us);"));
  pUInt16 = (unsigned int *)&SPI_in[6];
  port.println((float)*pUInt16 / 48, 2);      // print to 2dp
  Filename.println((float)*pUInt16 / 48, 2);  // print to 2dp

  port.print(F("AMSamplingIntervalCount;"));
  Filename.print(F("AMSamplingIntervalCount;"));
  pUInt16 = (unsigned int *)&SPI_in[8];
  port.println(*pUInt16, DEC);      // print value
  Filename.println(*pUInt16, DEC);  // print value

  port.print(F("AMIdleIntervalCount;"));
  Filename.print(F("AMIdleIntervalCount;"));
  pUInt16 = (unsigned int *)&SPI_in[10];
  port.println(*pUInt16, DEC);      // print value
  Filename.println(*pUInt16, DEC);  // print value

  port.print(F("AMMaxDataArraysInFile;"));
  Filename.print(F("AMMaxDataArraysInFile;"));
  pUInt16 = (unsigned int *)&SPI_in[12];
  port.println(*pUInt16, DEC);      // print value
  Filename.println(*pUInt16, DEC);  // print value

  port.print(F("AMOnlySavePMData;"));
  port.println(SPI_in[14], DEC);  // print value
  Filename.print(F("AMOnlySavePMData;"));
  Filename.println(SPI_in[14], DEC);  // print value

  port.print(F("AMFanOnInIdle;"));
  port.println(SPI_in[15], DEC);  // print value
  Filename.print(F("AMFanOnInIdle;"));
  Filename.println(SPI_in[15], DEC);  // print value

  port.print(F("AMLaserOnInIdle;"));
  port.println(SPI_in[16], DEC);  // print value
  Filename.print(F("AMLaserOnInIdle;"));
  Filename.println(SPI_in[16], DEC);  // print value

  port.print(F("TOFtoSFRfactor;"));
  port.println(SPI_in[17], DEC);
  Filename.print(F("TOFtoSFRfactor;"));
  Filename.println(SPI_in[17], DEC);

  port.print(F("PVP(us);"));
  port.println((float)SPI_in[18] / 48, 2);  // print to 2dp
  Filename.print(F("PVP(us);"));
  Filename.println((float)SPI_in[18] / 48, 2);  // print to 2dp

  port.print(F("BinWeightingIndex;"));
  port.println(SPI_in[19], DEC);
  port.flush();
  Filename.print(F("BinWeightingIndex;"));
  Filename.println(SPI_in[19], DEC);
  Filename.flush();
  // END Get config from OPC device (Misc)

  // Get DAC and power status from OPC device (this is a different command to the 'get config' one)
  GetReadyResponse(0x13);

  // Put required status bytes in buffer
  for (SPI_in_index = 0; SPI_in_index < 6; SPI_in_index++) {
    delayMicroseconds(10);
    SPI_in[SPI_in_index] = OPC_SPI.transfer(0x01);  // Value of outgoing byte doesn't matter
  }
  OPC_SPI.deselect();
  delay(10);

  port.print(F("FanDAC_ON;"));
  port.println(SPI_in[0], DEC);  // print value
  Filename.print(F("FanDAC_ON;"));
  Filename.println(SPI_in[0], DEC);  // print value

  port.print(F("LaserDAC_ON;"));
  port.println(SPI_in[1], DEC);  // print value
  Filename.print(F("LaserDAC_ON;"));
  Filename.println(SPI_in[1], DEC);  // print value

  port.print(F("FanDACval;"));
  port.println(SPI_in[2], DEC);  // print value
  Filename.print(F("FanDACval;"));
  Filename.println(SPI_in[2], DEC);  // print value

  port.print(F("LaserDACval;"));
  port.println(SPI_in[3], DEC);  // print value
  Filename.print(F("LaserDACval;"));
  Filename.println(SPI_in[3], DEC);  // print value

  port.print(F("LaserSwitch;"));
  port.println(SPI_in[4], DEC);  // print value
  Filename.print(F("LaserSwitch;"));
  Filename.println(SPI_in[4], DEC);  // print value

  port.print(F("AutoGainToggle;"));
  port.println(SPI_in[5] >> 1, DEC);  // print value
  port.flush();
  Filename.print(F("AutoGainToggle;"));
  Filename.println(SPI_in[5] >> 1, DEC);  // print value
  Filename.flush();
  // END Get DAC and power status from OPC device (this is a different command to the 'get config' one)
}

void StartOPC(void) {
  // Turn ON fan and laser

  // Laser power ON
  GetReadyResponse(0x03);
  // OLD SPI: SPI.transfer(0x07);  // Turn ON laser power
  OPC_SPI.transfer(0x07);
  OPC_SPI.deselect();
  delay(10);

  delay(1000);  // Delay may be necessary to seperate power ON of fan and laser

  // Fan DAC ON
  GetReadyResponse(0x03);
  // OLD SPI: SPI.transfer(0x03);  // Turn ON fan DAC
  OPC_SPI.transfer(0x03);
  OPC_SPI.deselect();
  delay(10);

  // Wait for fan to reach full speed (and for multiple attempts by OPC firmware to turn on fan)
  for (int i = 0; i < 5; i++) {
    wdt_reset();  // Reset watchdog timer
    delay(1000);
  }
}

void StartLaser(void) {
  // Turn ON fan and laser

  // Laser power ON
  GetReadyResponse(0x03);
  // OLD SPI: SPI.transfer(0x07);  // Turn ON laser power
  OPC_SPI.transfer(0x07);
  OPC_SPI.deselect();
  delay(10);

  delay(1000);  // Delay may be necessary to seperate power ON of fan and laser
  // If there are issues with this, it might be due to the whaiting loop missing see bottom of "StartOPC"
}

void StopLaser(void) {
  // Turn OFF laser.

  // Laser power OFF
  GetReadyResponse(0x03);
  OPC_SPI.transfer(0x06);  // Turn OFF laser power
  OPC_SPI.deselect();
  delay(10);
}

void StopOPC(void) {
  // Turn OFF fan and laser.

  // Laser power OFF
  GetReadyResponse(0x03);
  OPC_SPI.transfer(0x06);  // Turn OFF laser power
  OPC_SPI.deselect();
  delay(10);

  // Fan DAC OFF
  GetReadyResponse(0x03);
  OPC_SPI.transfer(0x02);  // Turn OFF fan DAC
  OPC_SPI.deselect();
  delay(10);
}

void GetReadyResponse(byte SPIcommand) {
  unsigned char Response;

  // OLD SPI function
  // SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));

  OPC_SPI.select();
  delay(20);

  // Try reading a byte here to clear out anything remnant of SD card SPI activity (WORKS!)
  // OLD SPI: Response = SPI.transfer(SPIcommand);
  Response = OPC_SPI.transfer(SPIcommand);
  delay(10);  // wait 1ms maby 10 is better

  // OLD SPI: SetSSpin(LOW);
  unsigned char Tries = 0;
  do {
    // OLD SPI: Response = SPI.transfer(SPIcommand);
    Response = OPC_SPI.transfer(SPIcommand);
    if (Response != SPI_OPC_ready) 
    delay(1);  // wait 1ms
  } while ((Tries++ < 20) && (Response != SPI_OPC_ready));

  if (Response != SPI_OPC_ready) {
    if (Response == SPI_OPC_busy) {
      Serial.println(F("ERROR Waiting 2s (for OPC comms timeout)"));  // signal user
      Serial.flush();
      wdt_reset();
      delay(2000);  // wait 2s
    } else {
      // End SPI and wait a few seconds for it to be cleared
      OPC_SPI.deselect();
      Serial.println(F("ERROR connecting to OPC"));  // signal user
      Serial.flush();
      BlinkLED(2, red);
      // Wait 6s here for buffer to be cleared
      wdt_reset();
      delay(6000);
      wdt_reset();
    }
  }
  delay(10);

  wdt_reset();
}

unsigned int MODBUS_CalcCRC(unsigned char data[], unsigned char nbrOfBytes) {
#define POLYNOMIAL_MODBUS 0xA001  // Generator polynomial for MODBUS crc
#define InitCRCval_MODBUS 0xFFFF  // Initial CRC value

  unsigned char _bit;                    // bit mask
  unsigned int crc = InitCRCval_MODBUS;  // initialise calculated checksum
  unsigned char byteCtr;                 // byte counter

  // calculates 16-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (unsigned int)data[byteCtr];
    for (_bit = 0; _bit < 8; _bit++) {
      if (crc & 1)  // if bit0 of crc is 1
      {
        crc >>= 1;
        crc ^= POLYNOMIAL_MODBUS;
      } else
        crc >>= 1;
    }
  }
  return crc;
}

// Convert SHT31 ST output to Temperature (C)
float ConvSTtoTemperature(unsigned int ST) {
  return -45 + 175 * (float)ST / 65535;
}

// Convert SHT31 SRH output to Relative Humidity (%)
float ConvSRHtoRelativeHumidity(unsigned int SRH) {
  return 100 * (float)SRH / 65535;
}

// Process OPC data and print
void PrintData(Stream &port, File &Filename) {
  unsigned char i;
  unsigned int *pUInt16;
  float *pFloat;
  float Afloat;

  // Histogram bins (UInt16) x16
  for (i = 0; i < 48; i += 2) {
    AddDelimiter(port, Filename);
    pUInt16 = (unsigned int *)&SPI_in[i];
    port.print(*pUInt16, DEC);
    Filename.print(*pUInt16, DEC);
  }

  // MToF bytes (UInt8) x4
  for (i = 48; i < 52; i++) {
    AddDelimiter(port, Filename);
    Afloat = (float)SPI_in[i];
    Afloat /= 3;  // convert to us
    port.print(Afloat, 2);
    Filename.print(Afloat, 2);
  }

  // Sampling period(s) (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[52];
  port.print((float)*pUInt16 / 100, 3);      // print to 3dp
  Filename.print((float)*pUInt16 / 100, 3);  // print to 3dp

  // SFR (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[54];
  port.print((float)*pUInt16 / 100, 3);      // print to 3dp
  Filename.print((float)*pUInt16 / 100, 3);  // print to 3dp

  // Temperature (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[56];
  port.print(ConvSTtoTemperature(*pUInt16), 1);      // print to 1dp
  Filename.print(ConvSTtoTemperature(*pUInt16), 1);  // print to 1dp

  // Relative humidity (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[58];
  port.print(ConvSRHtoRelativeHumidity(*pUInt16), 1);      // print to 1dp
  Filename.print(ConvSRHtoRelativeHumidity(*pUInt16), 1);  // print to

  // PM values(ug/m^3) (4-byte float) x3
  for (i = 60; i < 72; i += 4) {
    AddDelimiter(port, Filename);
    pFloat = (float *)&SPI_in[i];
    port.print(*pFloat, 3);      // print to 3dp
    Filename.print(*pFloat, 3);  // print to 3dp
  }

  // Reject count Glitch (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[72];
  port.print(*pUInt16, DEC);
  Filename.print(*pUInt16, DEC);

  // Reject count LongTOF (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[74];
  port.print(*pUInt16, DEC);
  Filename.print(*pUInt16, DEC);

  // Reject count Ratio (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[76];
  port.print(*pUInt16, DEC);
  Filename.print(*pUInt16, DEC);

  // Reject count OutOfRange (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[78];
  port.print(*pUInt16, DEC);
  Filename.print(*pUInt16, DEC);

  // Fan rev count (UInt16) x1 (Not using)
  // AddDelimiter(port);
  // pUInt16 = (unsigned int *)&SPI_in[80];
  // port.print(*pUInt16, DEC);

  // Laser status (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[82];
  port.print(*pUInt16, DEC);
  Filename.print(*pUInt16, DEC);

  // Checksum (UInt16) x1
  AddDelimiter(port, Filename);
  pUInt16 = (unsigned int *)&SPI_in[84];
  port.println(*pUInt16, DEC);
  Filename.println(*pUInt16, DEC);

  // Compare recalculated Checksum with one sent
  if (*pUInt16 != MODBUS_CalcCRC(SPI_in, 84)) {  // if checksums aren't equal
    port.println(F("Checksum error in line above!"));
    Filename.println(F("Checksum error in line above!"));
  }
  port.flush();
  Filename.flush();
}

// Print data labels
void PrintDataLabels(Stream &port, File &Filename) {
  unsigned char i;

  port.print(F("Date;Time"));
  Filename.print(F("Date;Time"));

  for (i = 0; i < 24; i++) {
    port.print(F(";Bin"));
    Filename.print(F(";Bin"));
    if (i < 10) port.print(F("0"));      // leading 0 for single digit bin numbers
    if (i < 10) Filename.print(F("0"));  // leading 0 for single digit bin numbers
    port.print(i, DEC);
    Filename.print(i, DEC);
  }

  for (i = 1; i < 9; i += 2) {
    port.print(F(";MToFBin"));
    Filename.print(F(";MToFBin"));
    port.print(i, DEC);
    Filename.print(i, DEC);
    if (i == 1) port.print(F("(us)"));      // print units for first value of this type
    if (i == 1) Filename.print(F("(us)"));  // print units for first value of this type
  }

  port.println(F(";SampPrd(s);SFR(ml/s);T(C);RH(%);PM_A(ug/m^3);PM_B;PM_C;#RejectGlitch;#RejectLongTOF;#RejectRatio;#RejectCountOutOfRange;LaserStatus;Checksum"));      //(Not using FanRevCount)
  Filename.println(F(";SampPrd(s);SFR(ml/s);T(C);RH(%);PM_A(ug/m^3);PM_B;PM_C;#RejectGlitch;#RejectLongTOF;#RejectRatio;#RejectCountOutOfRange;LaserStatus;Checksum"));  //(Not using FanRevCount) PM_A var slettet tror bestemt ikke det var meningen.

  port.flush();
  Filename.flush();
}

void AddDelimiter(Stream &port, File &Filename) {
  port.print(F(";"));      // delimiter
  Filename.print(F(";"));  // delimiter
}

void BlinkLED(int blinks, int LEDPin) {  // The timing and the number of blinks needs to be within the watchdog timer.
  wdt_reset();                           // Reset watchdog timer
  unsigned char i;
  for (i = 0; i < blinks; i++) {
    digitalWrite(LEDPin, HIGH);
    delay(200);
    digitalWrite(LEDPin, LOW);
    delay(200);
  }
}