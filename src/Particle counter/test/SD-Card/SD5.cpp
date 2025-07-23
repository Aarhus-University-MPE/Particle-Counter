#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <SoftwareSPI.h>

File SystemData;
File Storage;

//#define SPI_OPC_busy  0x31
//#define SPI_OPC_ready 0xF3

// OPC_bitbang connections.
//#define OPC_CS        4
//#define OPC_SCK       3
//#define OPC_MOSI      5
//#define OPC_MISO      6

// SS pin for SD SPI.
#define SD_CS         10




void setup() {
  
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!-26");
    while (1)
      ;
  }
  Serial.println("done.-26");
  delay(10);
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  SystemData = SD.open("systemdata.txt", FILE_WRITE | O_CREAT);
  delay(10);
  if (SystemData.available()) {
    Serial.println("SystemData ready");
  }
  delay(100);
  Serial.print("SystemData available: ");
  Serial.println(SystemData.available());

  delay(1000);

  //SystemData.close();
  //digitalWrite(SD_CS, HIGH);
}

void loop() {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  digitalWrite(SD_CS, LOW); //high? ingen pinMode

  SystemData = SD.open("systemdata.txt", FILE_WRITE);

  wdt_reset();  // Reset watchdog timer

  // if the file opened okay, write to it:
  if (SystemData) {
    Serial.print("Writing to SystemData.txt...");
    SystemData.println("testing 1, 2, 3.");
    // close the file:
    SystemData.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening SystemData.txt");
  }

  wdt_reset();  // Reset watchdog timer

  // re-open the file for reading:
  SystemData = SD.open("systemdata.txt");
  if (SystemData) {
    Serial.println("SystemData.txt:");

    // read from the file until there's nothing else in it:
    while (SystemData.available()) {
      Serial.write(SystemData.read());
    }
    // close the file:
    SystemData.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening SystemData.txt");
  }

  wdt_reset();  // Reset watchdog timer

  digitalWrite(SD_CS, HIGH);
  Serial.println("Waiting 10sec...");
  delay(10000);
}
