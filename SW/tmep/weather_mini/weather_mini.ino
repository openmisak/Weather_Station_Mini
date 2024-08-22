// připojení knihoven
#include "config.h"  // change to config.h and fill the file.
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// nastavení adresy senzoru
#define BME280_ADRESA (0x77)
// inicializace senzoru BME z knihovny
Adafruit_BME280 bme;

void setup() {
  // komunikace po sériové lince rychlostí 9600 baud
  Serial.begin(115200);
  // zahájení komunikace se senzorem BME280,
  // v případě chyby je vypsána hláška po sériové lince
  // a zastaven program
  if (!bme.begin(BME280_ADRESA)) {
    Serial.println("BME280 senzor nenalezen, zkontrolujte zapojeni!");
    while (1);
  }
}

void loop() {
  // výpis všech dostupných informací ze senzoru BMP
  // výpis teploty
  Serial.print("Teplota: ");
  Serial.print(bme.readTemperature());
  Serial.println(" stupnu Celsia.");
  // výpis relativní vlhkosti
  Serial.print("Relativni vlhkost: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  // výpis tlaku s přepočtem na hektoPascaly
  Serial.print("Tlak: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa.");
  // vytištění prázdného řádku a pauza po dobu 2 vteřin
  Serial.println();
  delay(2000);
}