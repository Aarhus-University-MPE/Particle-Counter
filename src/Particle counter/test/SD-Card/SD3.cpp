/*
  SD card read/write

  This example shows how to read and write data to and from an SD card file
  The circuit:
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created   Nov 2010
  by David A. Mellis
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/
#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSPI.h>
#include <avr/wdt.h>

#define FirmwareVer   "SD3 Test code for Date and file generation -0001"

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
void BlinkLED(int blinks, int LEDPin);

File Storage;
File SysData;

DateTime now;

char TimeShort;
char SystemData[]  = "SDyymmdd.txt";
char StorageFile[] = "SFyymmdd.csv";

void setup() {
  wdt_reset();          // Reset watchdog timer
  wdt_enable(WDTO_8S);  // Enable watchdog timer, countdown 8s (max)

  delay(1000);  // delay in case of noise on power connection. Also allows OPC to boot up.

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

  PrintFirmwareVer(Serial, SysData);  // Print firmware version to serial port

  wdt_reset();  // Reset watchdog timer

  // Start bitbanging the sensor
 
  wdt_reset();  // Reset watchdog timer

   CloseSD(Serial, SysData);
  delay(1000);

  OpenStorage(StorageFile);

  CloseSD(Serial, Storage);
}

void loop() {
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
}

void InitSD() {  // SD storage
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization of SD failed!");
    BlinkLED(3, red);
    while (1)
      ;
  }
  Serial.println("initialization of SD done.");
  BlinkLED(3, green);
}

void OpenSysData(String filename) {
  SysData = SD.open(filename, FILE_WRITE | O_CREAT | O_APPEND);
  if (!SysData) {
    Serial.println("Failed to Create SysData file");
    BlinkLED(4, red);
    // while (1)  (Kommenteret ud for at ignorere at den tror filen ikke er oprettet)
    //   ;
  } else {
    Serial.println("SysData opend");
    BlinkLED(4, green);
  }
}

void OpenStorage(String filename) {
  Storage = SD.open(filename, FILE_WRITE | O_CREAT | O_APPEND);
  if (!Storage) {
    Serial.println("Failed to open or Create Storage file");
    BlinkLED(5, red);
    // while (1) (Kommenteret ud for at ignorere at den tror filen ikke er oprettet)
    //   ;
  } else {
    Serial.println("Storage Opend");
    BlinkLED(5, green);
  }
}

void CloseSD(Stream &Port, File &FileName) {
  FileName.close();
  digitalWrite(SD_CS, HIGH);
  Port.println("SD Closed");
  BlinkLED(1, red);
}

void PrintFirmwareVer(Stream &port, File &Filename) {
  port.print(F("Datalogger firmware ver "));
  port.println(FirmwareVer);
  Filename.print(F("Datalogger firmware ver "));
  Filename.println(FirmwareVer);
}

void BlinkLED(int blinks, int LEDPin) {  // The timing and the number of blinks needs to be within the watchdog timer.
  wdt_reset();                           // Reset watchdog timer
  unsigned char i;
  for (i = 1; i <= blinks; i++) {
    digitalWrite(LEDPin, HIGH);
    delay(200);
    digitalWrite(LEDPin, LOW);
    delay(200);
  }
  wdt_reset();  // Reset watchdog timer
  delay(1000);
  wdt_reset();  // Reset watchdog timer
}