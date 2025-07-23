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

File SD_spi;

#define CS_OPC  10
#define SD_CS   4
#define SD_SCK  3
#define SD_MISO 5
#define SD_MOSI 6

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(CS_OPC, OUTPUT);
  digitalWrite(CS_OPC, HIGH);

  SoftwareSPI SD_spi(SD_SCK, SD_MOSI, SD_MISO, SD_CS);

  // start the SPI-bitbang library:
  SD_spi.beginSPI();  // Enable SPI for SD comms

  /*
    Serial.print("Initializing SD card...");
    if (!SD_spi.beginSDSPI()) {
      Serial.println("initialization failed!");
      while (1)
        ;
    }
    Serial.println("initialization done.");
  */

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  int i = 5;
  SD_spi.select();
  i = SD_spi.transfer(5);
  SD_spi.deselect();
}

void loop() {
  /*
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  digitalWrite(CS_SD, LOW);

  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  digitalWrite(CS_SD, HIGH);
  delay(10000);
  */
}