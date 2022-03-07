/***************************************************************************
   Wiring BMP280:
   VCC = 3.3V
   GND = GND
   SCL = A5
   SDA = A4
   CSB = na
   SDO = na
  ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

float grondHoogte = 0;

Adafruit_BMP280 bmp; // I2C

const int BMP_address = 0x76;

const unsigned long eventInterval = 1000;

class Zoomer {

  public:

    int zoomerPin = 10;
    unsigned long currentTime;
    unsigned long previousTime = 0;

    Zoomer() {}

    void begin(int _zoomerPin) {
      zoomerPin = _zoomerPin;
      pinMode(zoomerPin, OUTPUT);
    }

    void gaPiepen() {

      if (bmp.readAltitude(grondHoogte) < 10 ) {
        unsigned long currentTime = millis();

        if (currentTime - previousTime >= eventInterval) {
          tone(zoomerPin, 2000, 500);
          previousTime = currentTime;
          Serial.print(currentTime);
        }

      }

    }
};

Zoomer buzzer = Zoomer();

void setup() {
  Serial.begin(9600);
  const int zoomerPin = 10;

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println(F("BMP280 test"));
  bmp.begin(BMP_address);


  if (!bmp.begin(BMP_address)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  grondHoogte = bmp.readPressure() / 100 ;
}

void loop() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(grondHoogte)); // this should be adjusted to your local forcase
  Serial.println(" m");

  Serial.println();

  Serial1.print(F("Temperature = "));
  Serial1.print(bmp.readTemperature());
  Serial1.println(" *C");

  Serial1.print(F("Pressure = "));
  Serial1.print(bmp.readPressure());
  Serial1.println(" Pa");

  Serial1.print(F("Approx altitude = "));
  Serial1.print(bmp.readAltitude(grondHoogte)); // this should be adjusted to your local forcase
  Serial1.println(" m");

  buzzer.gaPiepen();
  delay(2000);
}
