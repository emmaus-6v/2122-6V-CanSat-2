/*
 Wiring
  
  BMP280:
  VCC = 3.3V
  GND = GND
  SCL = A5
  SDA = A4
  CSB = na
  SDO = na

  SD-module:
  CS = D4
  SCK = D13
  MOSI = D11
  MISO = D12
  VCC = 5V
  GND = GND
*/

#include <SD.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
File myFile;
const int chipSelect = 4;
float QNH = 1022.67; //Change the "1022.67" to your current barrometric pressure
const int BMP_address = 0x76;

float pressure;
float temperature;
float altimeter;
char charRead;
char runMode;
byte i = 0; //counter
char dataStr[100] = "";
char buffer[7];

void setup(){
  Serial.begin(9600);
  Serial.println("BMP280/SD Card Demo");
  bmp.begin(BMP_address);
  if (SD.begin(chipSelect))
  {
    Serial.println("SD card is present & ready");
  }
  else
  {
    Serial.println("SD card missing or failure");
    while (1); //halt program
  }
}

void loop(){
  dataStr[0] = 0;
  pressure = bmp.readPressure() / 100; //and conv Pa to hPa
  temperature = bmp.readTemperature();
  altimeter = bmp.readAltitude (QNH); //QNH is your current barrometric pressure
 
  dtostrf(temperature, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
  strcat( dataStr, buffer); //append the coverted float
  strcat( dataStr, " "); //append space

  dtostrf(pressure, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
  strcat( dataStr, buffer); //append the coverted float
  strcat( dataStr, " "); //append the space

  dtostrf(altimeter, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
  strcat( dataStr, buffer); //append the coverted float
  strcat( dataStr, 0); //terminate correctly
  
    Serial.print(temperature); 
    Serial.print(" ");
    Serial.print(pressure);
    Serial.print(" ");
    Serial.print(altimeter); 
    Serial.println();
    
    // open the file. note that only one file can be open at a time,
    myFile = SD.open("csv.txt", FILE_WRITE);
    // if the file opened okay, write to it:
    if (myFile)
    {
      myFile.println(dataStr);
      myFile.close();
    }
    else
    {
      Serial.println("error opening csv.txt");
    }
    delay(1000);
  }
 
