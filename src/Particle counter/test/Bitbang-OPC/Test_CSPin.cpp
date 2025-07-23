#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <SoftwareSPI.h>
#include <avr/wdt.h>

#define FirmwareVer "OPC-N3-02(UNOr3)(res divider on SS)"
#define ArduinoUNO

#define BaudRate      9600

#define SPI_OPC_busy  0x31
#define SPI_OPC_ready 0xF3

#define OPC_CS        4
#define OPC_SCK       3
#define OPC_MOSI      5
#define OPC_MISO      6

#define SD_CS         10

// Declare global SPI communicasion for OPC.
SoftwareSPI OPC_SPI(OPC_SCK, OPC_MOSI, OPC_MISO, OPC_CS);

void setup () {
// Start serial port
  Serial.begin(BaudRate);

  OPC_SPI.beginSPI();

}

void loop (){

OPC_SPI.select();
Serial.println ("LOW");
delay(5000);
OPC_SPI.deselect();
Serial.println ("HIGH");
delay(5000);
}