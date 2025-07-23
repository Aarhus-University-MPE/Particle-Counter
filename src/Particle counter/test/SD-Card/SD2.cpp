#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <avr/wdt.h>
#include <SoftwareSPI.h>

File datadada;

#define CS_OPC 4
#define CS_SD  10

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");
  if (!SD.begin(CS_SD)) {
    Serial.println("initialization failed!-20");
    while (1)
      ;
  }
  Serial.println("initialization done.-20");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  datadada = SD.open("datadada.txt", FILE_WRITE | O_TRUNC);
  //Serial.println("Cleared file");
}

void loop() {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  digitalWrite(CS_SD, LOW);

  datadada = SD.open("datadada.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (datadada) {
    Serial.print("Writing to datadada.txt...");
    datadada.println("testing 1, 2, 3, 4.");
    // close the file:
    datadada.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening datadada.txt");
  }

  // re-open the file for reading:
  datadada = SD.open("datadada.txt");
  if (datadada) {
    Serial.println("datadada.txt:");

    // read from the file until there's nothing else in it:
    while (datadada.available()) {
      Serial.write(datadada.read());
    }
    // close the file:
    datadada.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening datadada.txt");
  }

  digitalWrite(CS_SD, HIGH);
  delay(10000);
}