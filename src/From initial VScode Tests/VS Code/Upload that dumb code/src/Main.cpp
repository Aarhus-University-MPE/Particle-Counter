#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSPI.h>
#include <avr/wdt.h>

#define BaudRate 9600

// LED's to indicate states
#define green    14
#define red      15

void setup() {
  // Start serial port
  Serial.begin(BaudRate);

  // Define Led pins
  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);
  digitalWrite(green, LOW);
  digitalWrite(red, LOW);
}

void loop(){
unsigned char i;
  for (i = 1; i <= 10; i++) {
    digitalWrite(red, HIGH);
    delay(200);
    digitalWrite(red, LOW);
    delay(200);
    digitalWrite(green, HIGH);
    delay(200);
    digitalWrite(green, LOW);
    delay(200);
  }
}
