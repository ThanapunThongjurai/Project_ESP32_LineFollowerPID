#include <Arduino.h>
#include 

SoftwareSerial mySerial(10, 11); // RX, TX

void setup()
{
  Serial.begin(9600);
  while (!Serial) ;
  mySerial.begin(9600);
}

void loop()
{
  if (mySerial.available())
    Serial.write(mySerial.read());
  if (Serial.available())
    mySerial.write(Serial.read());
}