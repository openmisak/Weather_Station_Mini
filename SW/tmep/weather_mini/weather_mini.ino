/* 
 */

// připojení knihoven
#include "config.h"  // základní konfigurace čidla

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>     // BME280 by Adafruit - senzor teploty
#include <Adafruit_TSL2561_U.h>  // TSL2561 by Adafruit - senzor osvitu
#include <WiFiManager.h>         // WiFi manager by tzapu https://github.com/tzapu/WiFiManager

//instance pro teplotní čidlo
Adafruit_BME280 bme;

// Vytvoření instance pro čidlo osvitu, 24351 je vymyšlené unikátní ID
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 24351);

float temperature;
float pressure;
float humidity;
float bat_voltage;
int32_t rssi;
float lux;
bool tempSensor;
bool luxSensor;

void postData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    //načtení síly signálu
    rssi = WiFi.RSSI();

    if (tempSensor) {
      //GUID, nasleduje hodnota teploty, pro vlhkost "humV", pro CO2 "CO2", pro napeti baterie "v"
      String tempServerPath = String(TEMP_SENSOR_NAME) + GUID + "=" + temperature + "&humV=" + humidity + "&pressV=" + pressure + "&v=" + bat_voltage + "&rssi=" + rssi;
      Serial.println(tempServerPath);
      // zacatek http spojeni
      http.begin(tempServerPath.c_str());

      // http get request
      int tempHttpResponseCode = http.GET();

      if (tempHttpResponseCode > 0) {
        Serial.print("HTTP odpoved: ");
        Serial.println(tempHttpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      } else {
        Serial.print("Error kod: ");
        Serial.println(tempHttpResponseCode);
      }
      // Free resources
      http.end();
    }

    if (luxSensor) {
      //GUID je lux
      String luxServerPath = String(LUX_SENSOR_NAME) + luxGUID + "=" + lux;

      // zacatek http spojeni
      http.begin(luxServerPath.c_str());

      // http get request
      int luxHttpResponseCode = http.GET();

      if (luxHttpResponseCode > 0) {
        Serial.print("HTTP odpoved: ");
        Serial.println(luxHttpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      } else {
        Serial.print("Error kod: ");
        Serial.println(luxHttpResponseCode);
      }
      // Free resources
      http.end();
    }

  } else
    Serial.println("Wi-Fi odpojeno");
}

void GoToSleep() {
  delay(1);
  // ESP Deep Sleep
  digitalWrite(PIN_ON, LOW);  // Turn off the uSUP power
  Serial.println("ESP in sleep mode");
  esp_sleep_enable_timer_wakeup(SLEEP_SEC * 1000000);
  esp_deep_sleep_start();
}

// pripojeni k WiFi | WiFi Connection
void WiFiConnection() {
  // Probudit WiFi | Wake up WiFi Modem
  WiFi.mode(WIFI_STA);

  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result

  wm.setConfigPortalTimeout(180);  // set portal time to 3 min, then sleep.
  bool res;
  res = wm.autoConnect("LaskaKitMeteo", "meteostation");  // password protected ap

  if (!res) {
    Serial.println("Failed to connect");
    // ESP.restart();
  } else {
    //if you get here you have connected to the WiFi
    Serial.println("Wi-Fi connected successfully");
  }
}

// Přečíst data z BME280 | Read data from BME280
void readBME() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.println("°C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("% RH");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println("hPa");
}

// Přečíst data z čidla osvitu TSL2561
void readTSL() {
  sensors_event_t event;
  tsl.getEvent(&event);

  /* Display the results (light is measured in lux) */
  if (event.light) {
    lux = event.light;
    Serial.print(String(lux));
    Serial.println(" lux");
  } else {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
  delay(250);
}

void displaySensorDetails(void) {
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("-- LUX Sensor setup --");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" lux");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" lux");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void configureSensor(void) {
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true); /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS); /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print("Gain:         ");
  Serial.println("Auto");
  Serial.print("Timing:       ");
  Serial.println("13 ms");
  Serial.println("------------------------------------");
}

// Měření napětí baterie | Battery voltage measurement
void readBat() {
  bat_voltage = ((analogReadMilliVolts(ADC_PIN) * deviderRatio) / 1000) * REF_VOLTAGE;
  Serial.println("Battery voltage " + String(bat_voltage) + "V");
}

void setup() {
  Serial.println(BOARD_TYPE);
  // Hned vypneme WiFi | disable WiFi, coming from DeepSleep, as we do not need it right away
  WiFi.mode(WIFI_OFF);
  delay(1);

  //Serial.begin(115200);
  Serial.begin(9600);
  delay(100);
  while (!Serial) {}  // Wait

  // for board version over 3.5 need to turn uSUP ON
  pinMode(PIN_ON, OUTPUT);     // Set EN pin for uSUP stabilisator as output
  digitalWrite(PIN_ON, HIGH);  // Turn on the uSUP power

  // initilizace BME280 | BME280 Initialization
  Wire.begin(SDA, SCL);

  //check BME connection
  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    tempSensor = false;
    //while (1)
    //  ;
  } else {
    tempSensor = true;
    Serial.println("-- Weather Station setup --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,  // temperature
                    Adafruit_BME280::SAMPLING_X1,  // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_OFF);
    delay(10);
  }

  //check TSL connection
  if (!tsl.begin()) {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.println("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    luxSensor = false;
    //while (1)
    //  ;
  } else {
    luxSensor = true;
    /* Display some basic information on this sensor */
    //displaySensorDetails();
    /* Setup the sensor gain and integration time */
    configureSensor();
  }
}

void loop() {
  //scénář pro senzor osvitu
  if (luxSensor) {
    readTSL();
  }

  //BME read
  if (tempSensor) {
    readBME();
  }

  //BATTERY read
  readBat();

  // Pripojeni k WiFi | Connect to WiFi
  WiFiConnection();

  postData();

  WiFi.disconnect(true);

  //sleep desky
  GoToSleep();
}